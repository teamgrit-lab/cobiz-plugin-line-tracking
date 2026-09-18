#include "test_framework.hpp"

#include "line_tracking/drive_control.hpp"

using namespace line_tracking;

namespace {

SmoothedPath path(double lateral = 0.2, double confidence = 0.9,
                  double age = 0.1) {
  SmoothedPath value;
  value.confidence = confidence;
  value.age_sec = age;
  value.source = "test";
  for (int index = 0; index < 20; ++index) {
    value.points_xy.emplace_back(3.0F + 5.0F * index / 19.0F,
                                 static_cast<float>(lateral));
  }
  return value;
}

LidarSafetyResult safety(bool stop = false) {
  return {stop,
          true,
          stop,
          stop ? 3U : 0U,
          stop ? std::optional<double>{2.5} : std::nullopt,
          0.1,
          stop ? "obstacle_in_path" : "clear"};
}

DriveDecision decide(const std::optional<SmoothedPath> &candidate = path(),
                     const LidarSafetyResult &lidar = safety(),
                     bool enabled = true, bool calibrated = true,
                     bool other_publishers = false) {
  return decide_drive(candidate, lidar, 0.1, 0.1, other_publishers, enabled,
                      calibrated, DriveConfig{});
}

} // namespace

LT_TEST("fresh calibrated path creates capped A2 command") {
  const auto command = decide();
  LT_REQUIRE(command.reason == "tracking");
  LT_NEAR(command.vx, 0.10, 1e-9);
  LT_REQUIRE(command.yaw_rate > 0.0 && command.yaw_rate <= 0.18);
  const auto axes = command.joy_axes();
  LT_NEAR(axes[0], 0.0, 1e-9);
  LT_NEAR(axes[1], -0.10, 1e-6);
  LT_NEAR(axes[2], command.yaw_rate, 1e-6);
}

LT_TEST("unsafe drive inputs fail closed") {
  LT_REQUIRE(decide(path(), safety(), false).reason == "drive_not_armed");
  LT_REQUIRE(decide(path(), safety(), true, false).reason == "drive_not_armed");
  LT_REQUIRE(decide(path(), safety(), true, true, true).reason ==
             "multiple_control_publishers");
  LT_REQUIRE(decide(std::nullopt).reason == "path_unavailable");
  LT_REQUIRE(decide(path(0.2, 0.5)).reason == "path_low_confidence");
  LT_REQUIRE(decide(path(1.0)).reason == "path_lateral_target_large");
  LT_REQUIRE(decide(path(), safety(true)).reason == "lidar_obstacle_in_path");
}

LT_TEST("right-side path commands right yaw and no lateral velocity") {
  const auto command = decide(path(-0.2));
  LT_REQUIRE(command.reason == "tracking");
  LT_REQUIRE(command.vy == 0.0);
  LT_REQUIRE(command.yaw_rate < 0.0);
}

LT_TEST("lidar frame must already match base_link") {
  LT_REQUIRE(lidar_frame_matches_base("base_link", "base_link"));
  LT_REQUIRE(lidar_frame_matches_base("/base_link", "base_link"));
  LT_REQUIRE(!lidar_frame_matches_base("hesai_lidar", "base_link"));
  LT_REQUIRE(!lidar_frame_matches_base("base_link", "map"));
}
