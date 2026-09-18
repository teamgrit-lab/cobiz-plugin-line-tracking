#pragma once

#include "line_tracking/config.hpp"
#include "line_tracking/types.hpp"

#include <array>
#include <optional>
#include <string>
#include <string_view>

namespace line_tracking {

struct DriveDecision {
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  std::string reason;

  [[nodiscard]] static DriveDecision stopped(std::string reason);
  [[nodiscard]] std::array<float, 3> joy_axes() const;
};

[[nodiscard]] bool lidar_frame_matches_base(std::string_view message_frame,
                                            std::string_view path_frame);
[[nodiscard]] DriveDecision decide_drive(
    const std::optional<SmoothedPath> &path, const LidarSafetyResult &safety,
    std::optional<double> camera_age_sec,
    std::optional<double> inference_age_sec, bool other_control_publishers,
    bool enabled, bool calibrated, const DriveConfig &config);

} // namespace line_tracking
