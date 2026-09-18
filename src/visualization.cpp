#include "line_tracking/visualization.hpp"

#include "line_tracking/local_path.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace line_tracking {
namespace {

cv::Mat colorized_overlay(const cv::Mat &frame_bgr,
                          const cv::Mat &selected_mask) {
  cv::Mat mask;
  cv::resize(selected_mask, mask, frame_bgr.size(), 0.0, 0.0,
             cv::INTER_NEAREST);
  cv::Mat colored = frame_bgr.clone();
  colored.setTo(cv::Scalar{40, 190, 40}, mask == 1);
  colored.setTo(cv::Scalar{220, 60, 220}, mask == 2);
  cv::Mat result;
  cv::addWeighted(frame_bgr, 0.68, colored, 0.32, 0.0, result);
  return result;
}

void draw_path(cv::Mat &image, const PathPoints &points,
               const cv::Scalar &color, const int width) {
  std::vector<cv::Point> pixels;
  for (const auto &point : points) {
    if (std::isfinite(point.x) && std::isfinite(point.y)) {
      pixels.emplace_back(cvRound(point.x), cvRound(point.y));
    }
  }
  if (pixels.size() >= 2) {
    cv::polylines(image, pixels, false, color, width, cv::LINE_AA);
  }
}

std::string fixed(const double value, const int precision = 2) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

void draw_panel(cv::Mat &image, const std::vector<std::string> &lines,
                const cv::Scalar &status_color, const std::size_t status_line) {
  const int width = std::min(image.cols - 20, 540);
  const int height = static_cast<int>(lines.size()) * 22 + 12;
  cv::rectangle(image, {10, 10}, {10 + width, 10 + height}, cv::Scalar{0, 0, 0},
                cv::FILLED);
  for (std::size_t index = 0; index < lines.size(); ++index) {
    cv::putText(image, lines[index], {20, 31 + static_cast<int>(index) * 22},
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                index == status_line ? status_color : cv::Scalar{255, 255, 255},
                1, cv::LINE_AA);
  }
}

} // namespace

cv::Mat render_segmentation_overlay(const cv::Mat &frame_bgr,
                                    const cv::Mat &selected_mask,
                                    const std::uint64_t frame_index,
                                    const double fps) {
  cv::Mat result = colorized_overlay(frame_bgr, selected_mask);
  const std::vector<std::string> lines{
      "SWIN-L MAPILLARY SURFACES", "green=road magenta=sidewalk",
      "frame=" + std::to_string(frame_index) +
          " time=" + fixed(frame_index / std::max(fps, 0.001), 1) + "s"};
  draw_panel(result, lines, cv::Scalar{255, 255, 255}, lines.size());
  return result;
}

cv::Mat render_local_path_overlay(
    const cv::Mat &frame_bgr, const cv::Mat &selected_mask,
    const std::optional<LocalPathEstimate> &estimate,
    const std::optional<SmoothedPath> &path, const LidarSafetyResult &safety,
    const LocalPathConfig &config, const std::uint64_t frame_index,
    const std::uint64_t inference_count, const double inference_hz,
    const int path_mask_class) {
  cv::Mat result = colorized_overlay(frame_bgr, selected_mask);
  const auto roi_float =
      normalized_polygon_pixels(config.roi_polygon, frame_bgr.size());
  std::vector<cv::Point> roi;
  for (const auto &point : roi_float) {
    roi.emplace_back(cvRound(point.x), cvRound(point.y));
  }
  cv::polylines(result, roi, true, cv::Scalar{255, 180, 0}, 2, cv::LINE_AA);
  const cv::Mat homography =
      pixel_to_ground_homography(frame_bgr.size(), config);
  if (estimate) {
    draw_path(result, ground_to_pixel(estimate->points_xy, homography),
              cv::Scalar{0, 165, 255}, 3);
  }
  if (path) {
    const auto pixels = ground_to_pixel(path->points_xy, homography);
    draw_path(result, pixels, cv::Scalar{255, 255, 255}, 5);
    const std::size_t step = std::max<std::size_t>(1, pixels.size() / 6);
    for (std::size_t index = 0; index < pixels.size(); index += step) {
      const auto &point = pixels[index];
      if (std::isfinite(point.x) && std::isfinite(point.y)) {
        cv::circle(result, {cvRound(point.x), cvRound(point.y)}, 4,
                   cv::Scalar{255, 255, 255}, cv::FILLED, cv::LINE_AA);
      }
    }
  }

  std::string safety_text;
  cv::Scalar safety_color;
  if (safety.stop) {
    safety_text = "LIDAR STOP: " + safety.reason;
    safety_color = {0, 0, 255};
  } else if (safety.lidar_available) {
    safety_text =
        "LIDAR CLEAR: " +
        (safety.clearance_m ? fixed(*safety.clearance_m, 1) + "m" : "--");
    safety_color = {0, 220, 0};
  } else {
    safety_text = "LIDAR: " + safety.reason;
    safety_color = {0, 165, 255};
  }
  const std::string surface = path_mask_class == 1 ? "ROAD" : "SIDEWALK";
  std::vector<std::string> lines{
      "SWIN-L " + surface + " LOCAL PATH",
      estimate ? "raw=" + fixed(estimate->confidence) +
                     " valid=" + fixed(estimate->valid_ratio)
               : "raw=--",
      path ? "path=TRACKED hold=" + fixed(path->age_sec) + "s" : "path=LOST",
      "inference=" + fixed(inference_hz) +
          "Hz updates=" + std::to_string(inference_count) +
          " frame=" + std::to_string(frame_index),
      safety_text,
      "white=smoothed path orange=raw | magenta=sidewalk"};
  draw_panel(result, lines, safety_color, 4);
  return result;
}

} // namespace line_tracking
