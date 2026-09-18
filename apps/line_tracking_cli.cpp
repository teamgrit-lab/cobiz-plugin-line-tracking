#include "line_tracking/config.hpp"
#include "line_tracking/local_path.hpp"
#include "line_tracking/segmenter.hpp"
#include "line_tracking/visualization.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

class Arguments {
public:
  Arguments(const int argc, char **argv) {
    for (int index = 0; index < argc; ++index) {
      values_.emplace_back(argv[index]);
    }
  }

  [[nodiscard]] bool has(const std::string_view option) const {
    return std::ranges::find(values_, option) != values_.end();
  }

  [[nodiscard]] std::optional<std::string>
  optional(const std::string_view option) const {
    const auto found = std::ranges::find(values_, option);
    if (found == values_.end()) {
      return std::nullopt;
    }
    const auto next = found + 1;
    if (next == values_.end() || next->starts_with("--")) {
      throw std::invalid_argument(std::string{option} + " requires a value");
    }
    return *next;
  }

  [[nodiscard]] std::string required(const std::string_view option) const {
    const auto value = optional(option);
    if (!value) {
      throw std::invalid_argument("missing required argument " +
                                  std::string{option});
    }
    return *value;
  }

  [[nodiscard]] int integer(const std::string_view option,
                            const int fallback) const {
    const auto value = optional(option);
    return value ? std::stoi(*value) : fallback;
  }

private:
  std::vector<std::string> values_;
};

void print_help() {
  std::cout
      << "cobiz line tracking C++ tools\n\n"
      << "Usage:\n"
      << "  line_tracking_cli profiles\n"
      << "  line_tracking_cli segment-video --input VIDEO --output MP4 --model "
         "MODEL.onnx [--overwrite]\n"
      << "  line_tracking_cli local-path-video --input VIDEO --output MP4 "
         "--model MODEL.onnx [--path-mask-class 1|2] "
         "[--search-roi BLx,BLy,BRx,BRy,TRx,TRy,TLx,TLy] [--overwrite]\n"
      << "  line_tracking_cli roi-preview --input IMAGE_OR_VIDEO --output "
         "IMAGE "
         "[--path-mask-class 1|2] [--overwrite]\n"
      << "  line_tracking_cli benchmark --input VIDEO --model MODEL.onnx "
         "[--max-frames N]\n"
      << "  line_tracking_cli evaluate --candidate MASK --reference MASK\n\n"
      << "Common options: --profile NAME --cpu --report FILE.json\n";
}

line_tracking::SegmenterConfig segmenter_config(const Arguments &arguments) {
  line_tracking::SegmenterConfig config;
  config.model_path = arguments.required("--model");
  config.profile = arguments.optional("--profile")
                       .value_or(std::string{line_tracking::kDefaultProfile});
  config.prefer_cuda = !arguments.has("--cpu");
  return config;
}

std::array<double, 8> search_roi(const Arguments &arguments,
                                 const int path_class) {
  const auto &preset = line_tracking::offline_search_roi_polygon(path_class);
  std::array<double, 8> polygon = preset;
  if (const auto override = arguments.optional("--search-roi")) {
    polygon = line_tracking::parse_roi_polygon(*override);
  }
  for (const double value : polygon) {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      throw std::invalid_argument(
          "search ROI must use normalized coordinates in [0, 1]");
    }
  }
  return polygon;
}

void draw_search_roi(cv::Mat &image, const std::array<double, 8> &polygon) {
  const auto vertices =
      line_tracking::normalized_polygon_pixels(polygon, image.size());
  std::vector<cv::Point> pixels;
  pixels.reserve(vertices.size());
  for (const auto &vertex : vertices) {
    pixels.emplace_back(cvRound(vertex.x), cvRound(vertex.y));
  }
  cv::polylines(image, pixels, true, cv::Scalar{255, 255, 0}, 3, cv::LINE_AA);
  cv::putText(image, "OFFLINE SEARCH ROI", {20, std::max(30, image.rows - 20)},
              cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar{255, 255, 0}, 2,
              cv::LINE_AA);
}

void write_json(const std::filesystem::path &path,
                const nlohmann::json &value) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  const auto temporary = path.string() + ".tmp";
  {
    std::ofstream stream{temporary, std::ios::binary | std::ios::trunc};
    if (!stream) {
      throw std::runtime_error("could not write report: " + path.string());
    }
    stream << std::setw(2) << value << '\n';
  }
  std::filesystem::rename(temporary, path);
}

int profiles() {
  for (const auto &profile : line_tracking::profiles()) {
    std::cout << profile.name << " | " << profile.model_family << " | "
              << profile.input_height << 'x' << profile.input_width << " | "
              << profile.precision << '\n';
  }
  return 0;
}

struct VideoRun {
  std::uint64_t frames{};
  double total_inference_seconds{};
  double total_seconds{};
};

VideoRun run_video(const Arguments &arguments, const bool local_path) {
  const std::filesystem::path input = arguments.required("--input");
  const std::filesystem::path output = arguments.required("--output");
  if (!std::filesystem::is_regular_file(input)) {
    throw std::invalid_argument("input video does not exist: " +
                                input.string());
  }
  if (std::filesystem::exists(output) && !arguments.has("--overwrite")) {
    throw std::invalid_argument(
        "output exists; pass --overwrite to replace it");
  }
  const int path_class = arguments.integer("--path-mask-class", 2);
  const auto search_polygon =
      local_path ? std::optional{search_roi(arguments, path_class)}
                 : std::nullopt;
  line_tracking::SurfaceSegmenter segmenter{segmenter_config(arguments)};
  if (output.has_parent_path()) {
    std::filesystem::create_directories(output.parent_path());
  }
  cv::VideoCapture capture{input.string()};
  if (!capture.isOpened()) {
    throw std::runtime_error("could not open input video: " + input.string());
  }
  const double fps = std::max(1.0, capture.get(cv::CAP_PROP_FPS));
  const int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
  const int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
  cv::VideoWriter writer{output.string(),
                         cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                         fps,
                         {width, height}};
  if (!writer.isOpened()) {
    throw std::runtime_error("could not open output video: " + output.string());
  }

  line_tracking::LocalPathConfig path_config;
  line_tracking::LocalPathSmoother smoother{path_config};
  const auto started = std::chrono::steady_clock::now();
  VideoRun run;
  cv::Mat frame;
  while (capture.read(frame)) {
    const auto result = segmenter.segment(frame);
    run.total_inference_seconds += result.total_seconds;
    cv::Mat rendered;
    if (local_path) {
      const double timestamp = run.frames / fps;
      const auto estimate = line_tracking::extract_surface_centerline(
          line_tracking::apply_search_roi(line_tracking::select_path_region(
                                              result.selected_mask, path_class),
                                          *search_polygon),
          path_config);
      const auto path = smoother.update(estimate, timestamp);
      const line_tracking::LidarSafetyResult safety{
          true,
          false,
          false,
          0,
          std::nullopt,
          std::nullopt,
          "offline_video_has_no_lidar"};
      rendered = line_tracking::render_local_path_overlay(
          frame, result.selected_mask, estimate, path, safety, path_config,
          run.frames, run.frames + 1,
          result.total_seconds > 0.0 ? 1.0 / result.total_seconds : 0.0,
          path_class);
      draw_search_roi(rendered, *search_polygon);
    } else {
      rendered = line_tracking::render_segmentation_overlay(
          frame, result.selected_mask, run.frames, fps);
    }
    writer.write(rendered);
    ++run.frames;
    if (run.frames % 100 == 0) {
      std::cerr << "VIDEO_PROGRESS frames=" << run.frames << '\n';
    }
  }
  run.total_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
          .count();
  if (run.frames == 0) {
    throw std::runtime_error("input video contained no frames");
  }
  if (const auto report = arguments.optional("--report")) {
    write_json(*report,
               {{"schema_version", 2},
                {"runtime", "cpp-opencv-dnn"},
                {"input", std::filesystem::absolute(input).string()},
                {"output", std::filesystem::absolute(output).string()},
                {"mode", local_path ? "local-path-video" : "segment-video"},
                {"frames", run.frames},
                {"source_fps", fps},
                {"total_seconds", run.total_seconds},
                {"inference_seconds", run.total_inference_seconds},
                {"mean_inference_seconds",
                 run.total_inference_seconds / static_cast<double>(run.frames)},
                {"offline_search_roi_polygon",
                 search_polygon ? nlohmann::json{*search_polygon}
                                : nlohmann::json{nullptr}},
                {"ground_projection_roi_polygon",
                 local_path ? nlohmann::json{path_config.roi_polygon}
                            : nlohmann::json{nullptr}}});
  }
  std::cout << std::filesystem::absolute(output).string() << '\n';
  return run;
}

int roi_preview(const Arguments &arguments) {
  const std::filesystem::path input = arguments.required("--input");
  const std::filesystem::path output = arguments.required("--output");
  if (!std::filesystem::is_regular_file(input)) {
    throw std::invalid_argument("input image/video does not exist: " +
                                input.string());
  }
  if (std::filesystem::exists(output) && !arguments.has("--overwrite")) {
    throw std::invalid_argument(
        "output exists; pass --overwrite to replace it");
  }
  const int path_class = arguments.integer("--path-mask-class", 2);
  const auto polygon = search_roi(arguments, path_class);
  cv::Mat frame = cv::imread(input.string(), cv::IMREAD_COLOR);
  if (frame.empty()) {
    cv::VideoCapture capture{input.string()};
    if (!capture.isOpened() || !capture.read(frame)) {
      throw std::runtime_error("could not read first frame: " + input.string());
    }
  }
  // Reuse the same coordinate validation as the offline mask gate.
  (void)line_tracking::apply_search_roi(
      cv::Mat(frame.size(), CV_8UC1, cv::Scalar{255}), polygon);
  draw_search_roi(frame, polygon);
  if (output.has_parent_path()) {
    std::filesystem::create_directories(output.parent_path());
  }
  if (!cv::imwrite(output.string(), frame)) {
    throw std::runtime_error("could not write ROI preview: " + output.string());
  }
  std::cout << std::filesystem::absolute(output).string() << '\n';
  return 0;
}

int benchmark(const Arguments &arguments) {
  const std::filesystem::path input = arguments.required("--input");
  cv::VideoCapture capture{input.string()};
  if (!capture.isOpened()) {
    throw std::runtime_error("could not open input video: " + input.string());
  }
  line_tracking::SurfaceSegmenter segmenter{segmenter_config(arguments)};
  const int maximum = arguments.integer("--max-frames", 0);
  std::vector<double> times;
  cv::Mat frame;
  while (capture.read(frame) && (maximum <= 0 || std::ssize(times) < maximum)) {
    times.push_back(segmenter.segment(frame).total_seconds);
  }
  if (times.empty()) {
    throw std::runtime_error("input video contained no frames");
  }
  std::ranges::sort(times);
  const double mean = std::accumulate(times.begin(), times.end(), 0.0) /
                      static_cast<double>(times.size());
  const auto percentile = [&times](const double fraction) {
    const auto index = static_cast<std::size_t>(
        std::clamp(fraction * (times.size() - 1), 0.0,
                   static_cast<double>(times.size() - 1)));
    return times[index];
  };
  const nlohmann::json result{{"runtime", "cpp-opencv-dnn"},
                              {"frames", times.size()},
                              {"mean_seconds", mean},
                              {"p50_seconds", percentile(0.50)},
                              {"p95_seconds", percentile(0.95)},
                              {"throughput_fps", 1.0 / mean},
                              {"cuda", segmenter.using_cuda()}};
  std::cout << std::setw(2) << result << '\n';
  if (const auto report = arguments.optional("--report")) {
    write_json(*report, result);
  }
  return 0;
}

int evaluate(const Arguments &arguments) {
  const auto candidate_path = arguments.required("--candidate");
  const auto reference_path = arguments.required("--reference");
  cv::Mat candidate = cv::imread(candidate_path, cv::IMREAD_GRAYSCALE);
  cv::Mat reference = cv::imread(reference_path, cv::IMREAD_GRAYSCALE);
  if (candidate.empty() || reference.empty()) {
    throw std::runtime_error(
        "candidate and reference must be readable mask images");
  }
  if (candidate.size() != reference.size()) {
    cv::resize(candidate, candidate, reference.size(), 0.0, 0.0,
               cv::INTER_NEAREST);
  }
  nlohmann::json metrics;
  for (const auto [label, name] :
       {std::pair{1, "road"}, std::pair{2, "sidewalk"}}) {
    cv::Mat left;
    cv::Mat right;
    cv::compare(candidate, label, left, cv::CMP_EQ);
    cv::compare(reference, label, right, cv::CMP_EQ);
    cv::Mat intersection;
    cv::Mat union_mask;
    cv::bitwise_and(left, right, intersection);
    cv::bitwise_or(left, right, union_mask);
    const int union_count = cv::countNonZero(union_mask);
    metrics[name] = {
        {"iou", union_count > 0 ? cv::countNonZero(intersection) /
                                      static_cast<double>(union_count)
                                : 1.0},
        {"candidate_area_ratio",
         cv::countNonZero(left) / static_cast<double>(candidate.total())},
        {"reference_area_ratio",
         cv::countNonZero(right) / static_cast<double>(reference.total())}};
  }
  std::cout << std::setw(2) << metrics << '\n';
  return 0;
}

} // namespace

int main(const int argc, char **argv) {
  try {
    if (argc < 2) {
      print_help();
      return 2;
    }
    const std::string command{argv[1]};
    const Arguments arguments{argc - 1, argv + 1};
    if (command == "profiles") {
      return profiles();
    }
    if (command == "segment-video") {
      (void)run_video(arguments, false);
      return 0;
    }
    if (command == "local-path-video") {
      (void)run_video(arguments, true);
      return 0;
    }
    if (command == "roi-preview") {
      return roi_preview(arguments);
    }
    if (command == "benchmark") {
      return benchmark(arguments);
    }
    if (command == "evaluate") {
      return evaluate(arguments);
    }
    if (command == "--help" || command == "help") {
      print_help();
      return 0;
    }
    throw std::invalid_argument("unknown command: " + command);
  } catch (const std::exception &error) {
    std::cerr << "line_tracking_cli: " << error.what() << '\n';
    return 1;
  }
}
