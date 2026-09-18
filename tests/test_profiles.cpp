#include "test_framework.hpp"

#include "line_tracking/config.hpp"
#include "line_tracking/segmenter.hpp"

#include <opencv2/core.hpp>

using namespace line_tracking;

LT_TEST("selected default profile keeps pinned Swin-L contract") {
  const auto &profile = resolve_profile(kDefaultProfile);
  LT_REQUIRE(profile.name == "swin-l-aspect-224x384");
  LT_REQUIRE(profile.model_id == kPinnedModelId);
  LT_REQUIRE(profile.model_revision == kPinnedModelRevision);
  LT_REQUIRE(profile.input_height == 224);
  LT_REQUIRE(profile.input_width == 384);
  LT_NEAR(profile.temporal_alpha, 0.62, 1e-9);
  LT_NEAR(profile.temporal_hysteresis_margin, 0.07, 1e-9);
}

LT_TEST("unknown profile and invalid path class are rejected") {
  LT_THROWS(resolve_profile("unknown"));
  const cv::Mat selected = cv::Mat::zeros(2, 2, CV_8U);
  LT_THROWS(select_path_region(selected, 0));
}

LT_TEST("path-region selection never treats background as drivable") {
  cv::Mat selected = (cv::Mat_<std::uint8_t>(2, 3) << 0, 1, 2, 2, 0, 1);
  const auto road = select_path_region(selected, 1);
  const auto sidewalk = select_path_region(selected, 2);
  LT_REQUIRE(cv::countNonZero(road) == 2);
  LT_REQUIRE(cv::countNonZero(sidewalk) == 2);
  cv::Mat overlap;
  cv::bitwise_and(road, sidewalk, overlap);
  LT_REQUIRE(cv::countNonZero(overlap) == 0);
}
