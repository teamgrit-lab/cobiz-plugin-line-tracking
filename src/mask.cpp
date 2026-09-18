#include "line_tracking/segmenter.hpp"

#include <opencv2/imgproc.hpp>

#include <stdexcept>

namespace line_tracking {

cv::Mat select_path_region(const cv::Mat &selected_mask,
                           const int path_mask_class) {
  if (path_mask_class != 1 && path_mask_class != 2) {
    throw std::invalid_argument(
        "SWIN_L_PATH_MASK_CLASS must be 1 (road) or 2 (sidewalk)");
  }
  cv::Mat result;
  cv::compare(selected_mask, path_mask_class, result, cv::CMP_EQ);
  return result;
}

cv::Mat remove_small_components(const cv::Mat &binary_mask,
                                const int minimum_area) {
  if (binary_mask.empty() || binary_mask.channels() != 1 || minimum_area < 1) {
    throw std::invalid_argument("component filtering arguments are invalid");
  }
  cv::Mat normalized;
  cv::compare(binary_mask, 0, normalized, cv::CMP_GT);
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int count = cv::connectedComponentsWithStats(normalized, labels, stats,
                                                     centroids, 8, CV_32S);
  cv::Mat retained = cv::Mat::zeros(binary_mask.size(), CV_8U);
  for (int component = 1; component < count; ++component) {
    if (stats.at<int>(component, cv::CC_STAT_AREA) >= minimum_area) {
      retained.setTo(255, labels == component);
    }
  }
  return retained;
}

} // namespace line_tracking
