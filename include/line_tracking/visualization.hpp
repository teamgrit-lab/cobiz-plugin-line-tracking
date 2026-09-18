#pragma once

#include "line_tracking/config.hpp"
#include "line_tracking/types.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <optional>

namespace line_tracking {

[[nodiscard]] cv::Mat render_segmentation_overlay(const cv::Mat &frame_bgr,
                                                  const cv::Mat &selected_mask,
                                                  std::uint64_t frame_index,
                                                  double fps);
[[nodiscard]] cv::Mat render_local_path_overlay(
    const cv::Mat &frame_bgr, const cv::Mat &selected_mask,
    const std::optional<LocalPathEstimate> &estimate,
    const std::optional<SmoothedPath> &path, const LidarSafetyResult &safety,
    const LocalPathConfig &config, std::uint64_t frame_index,
    std::uint64_t inference_count, double inference_hz, int path_mask_class);

} // namespace line_tracking
