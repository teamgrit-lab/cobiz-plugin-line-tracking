#pragma once

#include "line_tracking/config.hpp"
#include "line_tracking/types.hpp"

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace line_tracking {

[[nodiscard]] std::array<cv::Point2f, 4>
normalized_polygon_pixels(const std::array<double, 8> &polygon,
                          cv::Size frame_size);
[[nodiscard]] cv::Mat pixel_to_ground_homography(cv::Size frame_size,
                                                 const LocalPathConfig &config);
[[nodiscard]] PathPoints ground_to_pixel(const PathPoints &points_xy,
                                         const cv::Mat &pixel_to_ground);
[[nodiscard]] std::optional<LocalPathEstimate>
extract_surface_centerline(const cv::Mat &surface_mask,
                           const LocalPathConfig &config);

class LocalPathSmoother {
public:
  explicit LocalPathSmoother(LocalPathConfig config);
  void reset();
  [[nodiscard]] std::optional<SmoothedPath>
  update(const std::optional<LocalPathEstimate> &estimate,
         double timestamp_sec);
  [[nodiscard]] std::optional<SmoothedPath> current(double timestamp_sec) const;

private:
  [[nodiscard]] std::optional<SmoothedPath>
  current_unlocked(double timestamp_sec) const;

  LocalPathConfig config_;
  mutable std::mutex mutex_;
  PathPoints points_;
  double confidence_{0.0};
  std::optional<double> last_update_;
};

struct PointFieldView {
  std::string name;
  std::size_t offset{};
};

struct PointCloud2View {
  std::size_t width{};
  std::size_t height{};
  std::size_t point_step{};
  std::size_t row_step{};
  bool big_endian{false};
  std::vector<PointFieldView> fields;
  std::span<const std::uint8_t> data;
};

[[nodiscard]] std::vector<cv::Point3f>
decode_pointcloud_xyz(const PointCloud2View &cloud);

class LidarSafetyMonitor {
public:
  explicit LidarSafetyMonitor(LidarSafetyConfig config);
  void update(std::span<const cv::Point3f> points_xyz, double timestamp_sec);
  void invalidate();
  [[nodiscard]] LidarSafetyResult
  evaluate(const std::optional<SmoothedPath> &path, double timestamp_sec) const;

private:
  LidarSafetyConfig config_;
  mutable std::mutex mutex_;
  std::vector<cv::Point3f> points_;
  std::optional<double> timestamp_;
};

} // namespace line_tracking
