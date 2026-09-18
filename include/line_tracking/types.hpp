#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace line_tracking {

using PathPoints = std::vector<cv::Point2f>;

struct LocalPathEstimate {
  PathPoints points_xy;
  double confidence{0.0};
  double valid_ratio{0.0};
  double mean_surface_width_m{0.0};
  PathPoints raw_points_xy;

  [[nodiscard]] bool valid() const noexcept {
    return points_xy.size() >= 2 && confidence > 0.0;
  }
};

struct SmoothedPath {
  PathPoints points_xy;
  double confidence{0.0};
  double age_sec{0.0};
  std::string source;
};

struct LidarSafetyResult {
  bool stop{true};
  bool lidar_available{false};
  bool obstacle_in_path{false};
  std::size_t obstacle_count{0};
  std::optional<double> clearance_m;
  std::optional<double> age_sec;
  std::string reason{"lidar_unavailable"};
};

} // namespace line_tracking
