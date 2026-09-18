#include "line_tracking/local_path.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace line_tracking {
namespace {

float interpolate(const std::vector<float> &xs, const std::vector<float> &ys,
                  const float value) {
  if (xs.empty() || xs.size() != ys.size()) {
    throw std::invalid_argument("interpolation inputs are invalid");
  }
  if (value <= xs.front()) {
    return ys.front();
  }
  if (value >= xs.back()) {
    return ys.back();
  }
  const auto upper = std::ranges::upper_bound(xs, value);
  const auto right = static_cast<std::size_t>(upper - xs.begin());
  const auto left = right - 1;
  const float scale = (value - xs[left]) / (xs[right] - xs[left]);
  return ys[left] + scale * (ys[right] - ys[left]);
}

std::vector<std::pair<int, int>> runs(const cv::Mat &row) {
  std::vector<std::pair<int, int>> result;
  int start = -1;
  for (int column = 0; column < row.cols; ++column) {
    const bool active = row.at<std::uint8_t>(0, column) != 0;
    if (active && start < 0) {
      start = column;
    }
    if (start >= 0 && (!active || column + 1 == row.cols)) {
      result.emplace_back(start, active ? column : column - 1);
      start = -1;
    }
  }
  return result;
}

float read_float(const std::uint8_t *bytes, const bool big_endian) {
  std::array<std::uint8_t, 4> copy{};
  std::memcpy(copy.data(), bytes, copy.size());
  if (big_endian == (std::endian::native == std::endian::little)) {
    std::ranges::reverse(copy);
  }
  float value{};
  std::memcpy(&value, copy.data(), sizeof(value));
  return value;
}

} // namespace

std::array<cv::Point2f, 4>
normalized_polygon_pixels(const std::array<double, 8> &polygon,
                          const cv::Size frame_size) {
  if (frame_size.width <= 0 || frame_size.height <= 0) {
    throw std::invalid_argument("frame size must be positive");
  }
  std::array<cv::Point2f, 4> points{};
  for (std::size_t index = 0; index < points.size(); ++index) {
    points[index] = {static_cast<float>(polygon[index * 2] *
                                        std::max(frame_size.width - 1, 1)),
                     static_cast<float>(polygon[index * 2 + 1] *
                                        std::max(frame_size.height - 1, 1))};
  }
  return points;
}

cv::Mat pixel_to_ground_homography(const cv::Size frame_size,
                                   const LocalPathConfig &config) {
  config.validate();
  const auto source = normalized_polygon_pixels(config.roi_polygon, frame_size);
  const std::array<cv::Point2f, 4> destination{
      cv::Point2f{static_cast<float>(config.near_distance_m),
                  static_cast<float>(config.ground_half_width_m)},
      cv::Point2f{static_cast<float>(config.near_distance_m),
                  static_cast<float>(-config.ground_half_width_m)},
      cv::Point2f{static_cast<float>(config.far_distance_m),
                  static_cast<float>(-config.ground_half_width_m)},
      cv::Point2f{static_cast<float>(config.far_distance_m),
                  static_cast<float>(config.ground_half_width_m)}};
  return cv::getPerspectiveTransform(source.data(), destination.data());
}

PathPoints ground_to_pixel(const PathPoints &points_xy,
                           const cv::Mat &pixel_to_ground) {
  if (points_xy.empty()) {
    return {};
  }
  cv::Mat inverse;
  if (cv::invert(pixel_to_ground, inverse) == 0.0) {
    throw std::runtime_error("camera-to-ground homography is singular");
  }
  PathPoints projected;
  cv::perspectiveTransform(points_xy, projected, inverse);
  return projected;
}

std::optional<LocalPathEstimate>
extract_surface_centerline(const cv::Mat &surface_mask,
                           const LocalPathConfig &config) {
  config.validate();
  if (surface_mask.empty() || surface_mask.channels() != 1) {
    throw std::invalid_argument(
        "surface_mask must be a non-empty single-channel image");
  }
  cv::Mat binary;
  cv::compare(surface_mask, 0, binary, cv::CMP_GT);

  const cv::Mat homography =
      pixel_to_ground_homography(surface_mask.size(), config);
  cv::Mat inverse;
  if (cv::invert(homography, inverse) == 0.0) {
    throw std::runtime_error("camera-to-ground homography is singular");
  }

  std::vector<float> x_values(static_cast<std::size_t>(config.bev_height_px));
  std::vector<float> y_values(static_cast<std::size_t>(config.bev_width_px));
  for (int row = 0; row < config.bev_height_px; ++row) {
    x_values[static_cast<std::size_t>(row)] =
        static_cast<float>(config.near_distance_m +
                           (config.far_distance_m - config.near_distance_m) *
                               row / std::max(config.bev_height_px - 1, 1));
  }
  for (int column = 0; column < config.bev_width_px; ++column) {
    y_values[static_cast<std::size_t>(column)] = static_cast<float>(
        config.search_half_width_m - 2.0 * config.search_half_width_m * column /
                                         std::max(config.bev_width_px - 1, 1));
  }

  PathPoints ground_points;
  ground_points.reserve(static_cast<std::size_t>(config.bev_height_px) *
                        static_cast<std::size_t>(config.bev_width_px));
  for (const float x : x_values) {
    for (const float y : y_values) {
      ground_points.emplace_back(x, y);
    }
  }
  PathPoints image_points;
  cv::perspectiveTransform(ground_points, image_points, inverse);
  cv::Mat map_x(config.bev_height_px, config.bev_width_px, CV_32F);
  cv::Mat map_y(config.bev_height_px, config.bev_width_px, CV_32F);
  for (int row = 0; row < config.bev_height_px; ++row) {
    for (int column = 0; column < config.bev_width_px; ++column) {
      const auto index =
          static_cast<std::size_t>(row * config.bev_width_px + column);
      map_x.at<float>(row, column) = image_points[index].x;
      map_y.at<float>(row, column) = image_points[index].y;
    }
  }
  cv::Mat birdseye;
  cv::remap(binary, birdseye, map_x, map_y, cv::INTER_NEAREST,
            cv::BORDER_CONSTANT, cv::Scalar{0});
  if (config.close_kernel_px > 0) {
    const cv::Mat kernel =
        cv::Mat::ones(config.close_kernel_px, config.close_kernel_px, CV_8U);
    cv::morphologyEx(birdseye, birdseye, cv::MORPH_CLOSE, kernel);
  }

  const double meters_per_column =
      2.0 * config.search_half_width_m / std::max(config.bev_width_px - 1, 1);
  const int minimum_width_px =
      std::max(1, static_cast<int>(std::ceil(config.min_surface_width_m /
                                             meters_per_column)));
  PathPoints raw_points;
  std::vector<float> widths;
  std::optional<float> previous_y;
  for (int row = 0; row < birdseye.rows; ++row) {
    struct Candidate {
      double score;
      float center_y;
      float width_m;
    };
    std::optional<Candidate> best;
    for (const auto &[start, end] : runs(birdseye.row(row))) {
      const int width_px = end - start + 1;
      if (width_px < minimum_width_px) {
        continue;
      }
      const float center_column = (start + end) / 2.0F;
      const float center_y = y_values[static_cast<std::size_t>(
          std::clamp(static_cast<int>(std::lround(center_column)), 0,
                     config.bev_width_px - 1))];
      double score = width_px;
      if (previous_y) {
        score -= 0.20 * std::abs(center_y - *previous_y) / meters_per_column;
      }
      Candidate candidate{score, center_y,
                          static_cast<float>(width_px * meters_per_column)};
      if (!best || candidate.score > best->score) {
        best = candidate;
      }
    }
    if (best) {
      previous_y = best->center_y;
      raw_points.emplace_back(x_values[static_cast<std::size_t>(row)],
                              best->center_y);
      widths.push_back(best->width_m);
    }
  }

  const double valid_ratio = static_cast<double>(raw_points.size()) /
                             std::max(config.bev_height_px, 1);
  const auto minimum_rows = static_cast<std::size_t>(
      std::max(3, static_cast<int>(std::ceil(config.min_valid_ratio *
                                             config.bev_height_px))));
  if (raw_points.size() < minimum_rows) {
    return std::nullopt;
  }

  const int degree =
      std::min(config.fit_degree, static_cast<int>(raw_points.size()) - 1);
  cv::Mat design(static_cast<int>(raw_points.size()), degree + 1, CV_64F);
  cv::Mat observed(static_cast<int>(raw_points.size()), 1, CV_64F);
  for (int row = 0; row < design.rows; ++row) {
    const double x = raw_points[static_cast<std::size_t>(row)].x;
    double power = 1.0;
    for (int column = 0; column <= degree; ++column) {
      design.at<double>(row, column) = power;
      power *= x;
    }
    observed.at<double>(row, 0) = raw_points[static_cast<std::size_t>(row)].y;
  }
  cv::Mat coefficients;
  const bool fitted = cv::solve(design, observed, coefficients, cv::DECOMP_SVD);

  PathPoints points;
  points.reserve(static_cast<std::size_t>(config.path_points));
  std::vector<float> raw_x;
  std::vector<float> raw_y;
  raw_x.reserve(raw_points.size());
  raw_y.reserve(raw_points.size());
  for (const auto &point : raw_points) {
    raw_x.push_back(point.x);
    raw_y.push_back(point.y);
  }
  for (int index = 0; index < config.path_points; ++index) {
    const float x =
        static_cast<float>(config.near_distance_m +
                           (config.far_distance_m - config.near_distance_m) *
                               index / std::max(config.path_points - 1, 1));
    double y = interpolate(raw_x, raw_y, x);
    if (fitted) {
      y = 0.0;
      double power = 1.0;
      for (int coefficient = 0; coefficient <= degree; ++coefficient) {
        y += coefficients.at<double>(coefficient, 0) * power;
        power *= x;
      }
    }
    points.emplace_back(
        x, static_cast<float>(std::clamp(y, -config.max_path_lateral_m,
                                         config.max_path_lateral_m)));
  }

  const double mean_width = std::accumulate(widths.begin(), widths.end(), 0.0) /
                            std::max<std::size_t>(widths.size(), 1);
  const double width_support = std::min(
      1.0, mean_width / std::max(config.ground_half_width_m * 0.5, 1e-6));
  const double confidence =
      std::clamp(0.75 * valid_ratio + 0.25 * width_support, 0.0, 1.0);
  return LocalPathEstimate{std::move(points), confidence, valid_ratio,
                           mean_width, std::move(raw_points)};
}

LocalPathSmoother::LocalPathSmoother(LocalPathConfig config)
    : config_(std::move(config)) {
  config_.validate();
}

void LocalPathSmoother::reset() {
  std::scoped_lock lock{mutex_};
  points_.clear();
  confidence_ = 0.0;
  last_update_.reset();
}

std::optional<SmoothedPath>
LocalPathSmoother::update(const std::optional<LocalPathEstimate> &estimate,
                          const double timestamp_sec) {
  std::scoped_lock lock{mutex_};
  if (estimate && estimate->valid()) {
    if (points_.empty() || points_.size() != estimate->points_xy.size()) {
      points_ = estimate->points_xy;
      confidence_ = estimate->confidence;
    } else {
      const double previous = last_update_.value_or(timestamp_sec);
      const double elapsed = std::max(timestamp_sec - previous, 0.0);
      const double alpha = 1.0 - std::exp(-std::max(elapsed, 1.0 / 30.0) /
                                          config_.smoothing_time_constant_sec);
      for (std::size_t index = 0; index < points_.size(); ++index) {
        const float dx =
            std::clamp(estimate->points_xy[index].x - points_[index].x,
                       static_cast<float>(-config_.max_lateral_update_m),
                       static_cast<float>(config_.max_lateral_update_m));
        const float dy =
            std::clamp(estimate->points_xy[index].y - points_[index].y,
                       static_cast<float>(-config_.max_lateral_update_m),
                       static_cast<float>(config_.max_lateral_update_m));
        points_[index].x += static_cast<float>(alpha * dx);
        points_[index].y += static_cast<float>(alpha * dy);
      }
      confidence_ = (1.0 - alpha) * confidence_ + alpha * estimate->confidence;
    }
    last_update_ = timestamp_sec;
  }
  return current_unlocked(timestamp_sec);
}

std::optional<SmoothedPath>
LocalPathSmoother::current(const double timestamp_sec) const {
  std::scoped_lock lock{mutex_};
  return current_unlocked(timestamp_sec);
}

std::optional<SmoothedPath>
LocalPathSmoother::current_unlocked(const double timestamp_sec) const {
  if (points_.empty() || !last_update_) {
    return std::nullopt;
  }
  const double age = std::max(timestamp_sec - *last_update_, 0.0);
  if (age > config_.path_hold_sec) {
    return std::nullopt;
  }
  return SmoothedPath{points_, confidence_, age,
                      age > 0.02 ? "smoothed_hold" : "smoothed_update"};
}

std::vector<cv::Point3f> decode_pointcloud_xyz(const PointCloud2View &cloud) {
  if (cloud.width == 0 || cloud.height == 0 || cloud.point_step < 12 ||
      cloud.row_step < cloud.width * cloud.point_step) {
    throw std::invalid_argument("invalid PointCloud2 dimensions or strides");
  }
  std::unordered_map<std::string, std::size_t> offsets;
  for (const auto &field : cloud.fields) {
    offsets[field.name] = field.offset;
  }
  for (const auto *name : {"x", "y", "z"}) {
    if (!offsets.contains(name) ||
        offsets[name] + sizeof(float) > cloud.point_step) {
      throw std::invalid_argument(
          "PointCloud2 does not contain valid x/y/z fields");
    }
  }
  if (cloud.data.size() < cloud.row_step * cloud.height) {
    throw std::invalid_argument(
        "PointCloud2 data is shorter than row_step * height");
  }
  std::vector<cv::Point3f> result;
  result.reserve(cloud.width * cloud.height);
  for (std::size_t row = 0; row < cloud.height; ++row) {
    const auto *row_data = cloud.data.data() + row * cloud.row_step;
    for (std::size_t column = 0; column < cloud.width; ++column) {
      const auto *point = row_data + column * cloud.point_step;
      result.emplace_back(read_float(point + offsets["x"], cloud.big_endian),
                          read_float(point + offsets["y"], cloud.big_endian),
                          read_float(point + offsets["z"], cloud.big_endian));
    }
  }
  return result;
}

LidarSafetyMonitor::LidarSafetyMonitor(LidarSafetyConfig config)
    : config_(std::move(config)) {
  config_.validate();
}

void LidarSafetyMonitor::update(const std::span<const cv::Point3f> points_xyz,
                                const double timestamp_sec) {
  std::vector<cv::Point3f> finite;
  finite.reserve(points_xyz.size());
  for (const auto &point : points_xyz) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      finite.push_back(point);
    }
  }
  std::scoped_lock lock{mutex_};
  points_ = std::move(finite);
  timestamp_ = timestamp_sec;
}

void LidarSafetyMonitor::invalidate() {
  std::scoped_lock lock{mutex_};
  points_.clear();
  timestamp_.reset();
}

LidarSafetyResult
LidarSafetyMonitor::evaluate(const std::optional<SmoothedPath> &path,
                             const double timestamp_sec) const {
  std::vector<cv::Point3f> points;
  std::optional<double> scan_timestamp;
  {
    std::scoped_lock lock{mutex_};
    points = points_;
    scan_timestamp = timestamp_;
  }
  if (!scan_timestamp) {
    return {
        true, false, false, 0, std::nullopt, std::nullopt, "lidar_unavailable"};
  }
  const double age = std::max(timestamp_sec - *scan_timestamp, 0.0);
  if (age > config_.timeout_sec) {
    return {true, false, false, 0, std::nullopt, age, "lidar_timeout"};
  }
  if (!path) {
    return {false, true, false, 0, std::nullopt, age, "path_unavailable"};
  }
  std::vector<float> path_x;
  std::vector<float> path_y;
  path_x.reserve(path->points_xy.size());
  path_y.reserve(path->points_xy.size());
  for (const auto &point : path->points_xy) {
    path_x.push_back(point.x);
    path_y.push_back(point.y);
  }
  std::size_t obstacle_count = 0;
  std::optional<double> clearance;
  for (const auto &point : points) {
    if (point.x <= 0.05F || point.x > config_.obstacle_distance_m ||
        point.z < config_.z_min_m || point.z > config_.z_max_m) {
      continue;
    }
    const float center_y = interpolate(path_x, path_y, point.x);
    if (std::abs(point.y - center_y) <= config_.corridor_half_width_m) {
      ++obstacle_count;
      clearance =
          std::min(clearance.value_or(point.x), static_cast<double>(point.x));
    }
  }
  const bool enough_points =
      obstacle_count >= static_cast<std::size_t>(config_.min_obstacle_points);
  const bool close = clearance && *clearance <= config_.stop_distance_m;
  const bool stop = enough_points && close;
  return {stop,
          true,
          obstacle_count > 0,
          obstacle_count,
          clearance,
          age,
          stop ? "obstacle_in_path"
               : (obstacle_count ? "obstacle_far" : "clear")};
}

} // namespace line_tracking
