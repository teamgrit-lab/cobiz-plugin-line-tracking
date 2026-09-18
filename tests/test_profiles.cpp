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

LT_TEST("offline road and sidewalk ROI presets match the requested image "
        "positions") {
  const auto &road = offline_search_roi_polygon(1);
  const auto &sidewalk = offline_search_roi_polygon(2);
  LT_REQUIRE((road == std::array<double, 8>{0.12, 0.95, 0.88, 0.95, 0.68, 0.42,
                                            0.32, 0.42}));
  LT_REQUIRE((sidewalk == std::array<double, 8>{0.03, 0.95, 0.62, 0.95, 0.58,
                                                0.42, 0.30, 0.42}));
  LT_REQUIRE(LocalPathConfig{}.roi_polygon == kDefaultRoiPolygon);
  LT_THROWS(offline_search_roi_polygon(0));
}

LT_TEST("offline search ROI gates masks without changing projection geometry") {
  const cv::Mat all_selected(100, 100, CV_8UC1, cv::Scalar{255});
  const auto road =
      apply_search_roi(all_selected, offline_search_roi_polygon(1));
  const auto sidewalk =
      apply_search_roi(all_selected, offline_search_roi_polygon(2));
  LT_REQUIRE(road.at<std::uint8_t>(70, 50) == 255);
  LT_REQUIRE(road.at<std::uint8_t>(70, 20) == 0);
  LT_REQUIRE(sidewalk.at<std::uint8_t>(70, 20) == 255);
  LT_REQUIRE(sidewalk.at<std::uint8_t>(70, 80) == 0);
  LT_REQUIRE(road.at<std::uint8_t>(10, 50) == 0);
  LT_REQUIRE(sidewalk.at<std::uint8_t>(99, 50) == 0);
  LT_REQUIRE(cv::countNonZero(all_selected) == 10000);
  LT_THROWS(apply_search_roi(
      all_selected,
      std::array<double, 8>{-0.1, 0.95, 0.88, 0.95, 0.68, 0.42, 0.32, 0.42}));
}
