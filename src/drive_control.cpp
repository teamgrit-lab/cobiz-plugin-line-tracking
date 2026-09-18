#include "line_tracking/drive_control.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace line_tracking {
namespace {

bool invalid_age(const std::optional<double> age, const double maximum) {
  return !age || !std::isfinite(*age) || *age < 0.0 || *age > maximum;
}

std::string normalized_frame(std::string_view frame) {
  std::size_t begin = 0;
  while (begin < frame.size() &&
         (frame[begin] == '/' ||
          std::isspace(static_cast<unsigned char>(frame[begin])))) {
    ++begin;
  }
  std::size_t end = frame.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(frame[end - 1]))) {
    --end;
  }
  return std::string{frame.substr(begin, end - begin)};
}

} // namespace

DriveDecision DriveDecision::stopped(std::string reason) {
  return {0.0, 0.0, 0.0, std::move(reason)};
}

std::array<float, 3> DriveDecision::joy_axes() const {
  return {static_cast<float>(vy), static_cast<float>(-vx),
          static_cast<float>(yaw_rate)};
}

bool lidar_frame_matches_base(const std::string_view message_frame,
                              const std::string_view path_frame) {
  return normalized_frame(message_frame) == path_frame &&
         path_frame == "base_link";
}

DriveDecision decide_drive(const std::optional<SmoothedPath> &path,
                           const LidarSafetyResult &safety,
                           const std::optional<double> camera_age_sec,
                           const std::optional<double> inference_age_sec,
                           const bool other_control_publishers,
                           const bool enabled, const bool calibrated,
                           const DriveConfig &config) {
  config.validate();
  if (!enabled || !calibrated) {
    return DriveDecision::stopped("drive_not_armed");
  }
  if (other_control_publishers) {
    return DriveDecision::stopped("multiple_control_publishers");
  }
  if (invalid_age(camera_age_sec, config.max_camera_age_sec)) {
    return DriveDecision::stopped("camera_stale");
  }
  if (invalid_age(inference_age_sec, config.max_inference_age_sec)) {
    return DriveDecision::stopped("inference_stale");
  }
  if (!path) {
    return DriveDecision::stopped("path_unavailable");
  }
  if (!std::isfinite(path->age_sec) || path->age_sec < 0.0 ||
      path->age_sec > config.max_path_age_sec) {
    return DriveDecision::stopped("path_stale");
  }
  if (!std::isfinite(path->confidence) ||
      path->confidence < config.min_confidence) {
    return DriveDecision::stopped("path_low_confidence");
  }
  if (!safety.lidar_available) {
    return DriveDecision::stopped("lidar_unavailable");
  }
  if (invalid_age(safety.age_sec, config.max_lidar_age_sec)) {
    return DriveDecision::stopped("lidar_stale");
  }
  if (safety.stop) {
    return DriveDecision::stopped("lidar_" + safety.reason);
  }
  if (safety.clearance_m && (!std::isfinite(*safety.clearance_m) ||
                             *safety.clearance_m <= config.min_clearance_m)) {
    return DriveDecision::stopped("lidar_clearance_low");
  }

  if (path->points_xy.size() < 2) {
    return DriveDecision::stopped("path_geometry_invalid");
  }
  for (std::size_t index = 0; index < path->points_xy.size(); ++index) {
    const auto &point = path->points_xy[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        (index > 0 && point.x <= path->points_xy[index - 1].x)) {
      return DriveDecision::stopped("path_geometry_invalid");
    }
  }
  if (path->points_xy.front().x <= 0.0 ||
      path->points_xy.front().x > config.lookahead_m ||
      path->points_xy.back().x < config.lookahead_m) {
    return DriveDecision::stopped("path_geometry_invalid");
  }

  auto upper = std::ranges::upper_bound(path->points_xy,
                                        static_cast<float>(config.lookahead_m),
                                        {}, &cv::Point2f::x);
  const auto right = static_cast<std::size_t>(upper - path->points_xy.begin());
  const auto left = right - 1;
  const auto &first = path->points_xy[left];
  const auto &second = path->points_xy[right];
  const double fraction = (config.lookahead_m - first.x) / (second.x - first.x);
  const double lateral = first.y + fraction * (second.y - first.y);
  if (std::abs(lateral) > config.max_lateral_target_m) {
    return DriveDecision::stopped("path_lateral_target_large");
  }
  const double heading = std::atan2(lateral, config.lookahead_m);
  const double yaw_rate = std::clamp(config.heading_gain * heading,
                                     -config.max_yaw_rps, config.max_yaw_rps);
  return {config.max_forward_mps, 0.0, yaw_rate, "tracking"};
}

} // namespace line_tracking
