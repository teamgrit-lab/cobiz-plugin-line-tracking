#include "line_tracking/segmenter.hpp"

#include "line_tracking/local_path.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>

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

cv::Mat apply_search_roi(const cv::Mat &binary_mask,
                         const std::array<double, 8> &roi_polygon) {
  if (binary_mask.empty() || binary_mask.type() != CV_8UC1) {
    throw std::invalid_argument(
        "search mask must be a non-empty CV_8UC1 image");
  }
  for (const double value : roi_polygon) {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      throw std::invalid_argument("search ROI must use normalized coordinates");
    }
  }
  const auto vertices =
      normalized_polygon_pixels(roi_polygon, binary_mask.size());
  std::vector<cv::Point> polygon;
  polygon.reserve(vertices.size());
  for (const auto &vertex : vertices) {
    polygon.emplace_back(cvRound(vertex.x), cvRound(vertex.y));
  }
  cv::Mat search_mask = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
  cv::fillPoly(search_mask, std::vector<std::vector<cv::Point>>{polygon},
               cv::Scalar{255});
  cv::Mat result;
  cv::bitwise_and(binary_mask, search_mask, result);
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
