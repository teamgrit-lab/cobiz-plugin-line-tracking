#pragma once

#include "line_tracking/config.hpp"

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <cstdint>
#include <vector>

namespace line_tracking {

struct SegmentationResult {
  cv::Mat selected_mask;
  double inference_seconds{};
  double postprocess_seconds{};
  double total_seconds{};
  double hysteresis_hold_ratio{};
  double road_area_ratio{};
  double sidewalk_area_ratio{};
};

class SurfaceSegmenter {
public:
  explicit SurfaceSegmenter(SegmenterConfig config);
  void reset();
  [[nodiscard]] SegmentationResult segment(const cv::Mat &frame_bgr);
  [[nodiscard]] const SegmenterConfig &config() const noexcept {
    return config_;
  }
  [[nodiscard]] const ProfileSpec &profile() const noexcept { return profile_; }
  [[nodiscard]] bool using_cuda() const noexcept { return using_cuda_; }

private:
  [[nodiscard]] std::vector<cv::Mat> infer_scores(const cv::Mat &frame_bgr);
  [[nodiscard]] std::vector<cv::Mat>
  decode_outputs(const std::vector<cv::Mat> &outputs) const;

  SegmenterConfig config_;
  ProfileSpec profile_;
  cv::dnn::Net network_;
  bool using_cuda_{false};
  double temporal_alpha_{};
  double hysteresis_margin_{};
  std::vector<cv::Mat> previous_scores_;
  cv::Mat previous_selected_;
};

[[nodiscard]] cv::Mat select_path_region(const cv::Mat &selected_mask,
                                         int path_mask_class);
[[nodiscard]] cv::Mat remove_small_components(const cv::Mat &binary_mask,
                                              int minimum_area);

} // namespace line_tracking
