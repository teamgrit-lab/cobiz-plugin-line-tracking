#include "test_framework.hpp"

#include "line_tracking/local_path.hpp"

#include <opencv2/imgproc.hpp>

#include <array>
#include <cstring>
#include <vector>

using namespace line_tracking;

LT_TEST("extracts a metric centerline from a straight surface") {
  LocalPathConfig config;
  config.min_valid_ratio = 0.20;
  cv::Mat mask = cv::Mat::zeros(360, 640, CV_8U);
  const std::vector<cv::Point> polygon{
      {70, 359}, {260, 359}, {275, 80}, {205, 80}};
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{polygon},
               cv::Scalar{255});

  const auto estimate = extract_surface_centerline(mask, config);
  LT_REQUIRE(estimate.has_value());
  LT_REQUIRE(estimate->points_xy.size() ==
             static_cast<std::size_t>(config.path_points));
  LT_REQUIRE(estimate->valid_ratio >= config.min_valid_ratio);
  for (std::size_t index = 1; index < estimate->points_xy.size(); ++index) {
    LT_REQUIRE(estimate->points_xy[index].x > estimate->points_xy[index - 1].x);
  }
  double mean = 0.0;
  for (const auto &point : estimate->points_xy) {
    mean += point.y;
  }
  mean /= estimate->points_xy.size();
  LT_REQUIRE(mean > 1.0);
  LT_REQUIRE(mean < config.max_path_lateral_m);
}

LT_TEST("smoother bounds updates and expires held paths") {
  LocalPathConfig config;
  config.max_lateral_update_m = 0.2;
  config.path_hold_sec = 1.0;
  LocalPathSmoother smoother{config};
  LocalPathEstimate first;
  LocalPathEstimate second;
  first.confidence = second.confidence = 0.8;
  first.valid_ratio = second.valid_ratio = 1.0;
  for (int index = 0; index < 4; ++index) {
    const float x = 3.0F + 5.0F * index / 3.0F;
    first.points_xy.emplace_back(x, 0.0F);
    second.points_xy.emplace_back(x, 2.0F);
  }
  LT_REQUIRE(smoother.update(first, 0.0).has_value());
  const auto updated = smoother.update(second, 0.25);
  LT_REQUIRE(updated.has_value());
  for (const auto &point : updated->points_xy) {
    LT_REQUIRE(point.y < 0.2F);
  }
  LT_REQUIRE(smoother.current(0.5).has_value());
  LT_REQUIRE(!smoother.current(1.51).has_value());
}

LT_TEST("lidar gate stops for multiple close corridor points") {
  LidarSafetyConfig config;
  config.min_obstacle_points = 2;
  LidarSafetyMonitor monitor{config};
  SmoothedPath path;
  path.confidence = 0.8;
  for (int index = 0; index < 10; ++index) {
    path.points_xy.emplace_back(3.0F + 5.0F * index / 9.0F, 0.0F);
  }
  const std::vector<cv::Point3f> points{
      {2.5F, 0.1F, 0.2F}, {2.6F, -0.1F, 0.3F}, {5.0F, 2.0F, 0.2F}};
  monitor.update(points, 1.0);
  const auto result = monitor.evaluate(path, 1.1);
  LT_REQUIRE(result.stop);
  LT_REQUIRE(result.obstacle_in_path);
  LT_REQUIRE(result.obstacle_count == 2);
  LT_NEAR(*result.clearance_m, 2.5, 1e-6);
}

LT_TEST("point cloud decoder supports padded Hesai points") {
  std::vector<std::uint8_t> payload(52, 0);
  const auto put = [&payload](const std::size_t offset, const float value) {
    std::memcpy(payload.data() + offset, &value, sizeof(value));
  };
  put(0, 1.0F);
  put(4, 2.0F);
  put(8, 3.0F);
  put(26, 4.0F);
  put(30, 5.0F);
  put(34, 6.0F);
  const PointCloud2View view{
      2, 1, 26, 52, false, {{"x", 0}, {"y", 4}, {"z", 8}}, payload};
  const auto points = decode_pointcloud_xyz(view);
  LT_REQUIRE(points.size() == 2);
  LT_NEAR(points[0].x, 1.0, 1e-6);
  LT_NEAR(points[1].z, 6.0, 1e-6);
}
