#include "line_tracking/config.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <ranges>
#include <sstream>
#include <stdexcept>

namespace line_tracking {
namespace {

void require_positive(double value, std::string_view name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string{name} +
                                " must be finite and positive");
  }
}

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

} // namespace

void LocalPathConfig::validate() const {
  require_positive(near_distance_m, "near_distance_m");
  if (!std::isfinite(far_distance_m) || far_distance_m <= near_distance_m) {
    throw std::invalid_argument(
        "far_distance_m must be finite and greater than near_distance_m");
  }
  require_positive(ground_half_width_m, "ground_half_width_m");
  require_positive(search_half_width_m, "search_half_width_m");
  if (search_half_width_m > ground_half_width_m) {
    throw std::invalid_argument(
        "search_half_width_m must not exceed ground_half_width_m");
  }
  if (std::ranges::any_of(roi_polygon, [](const double value) {
        return !std::isfinite(value) || value < 0.0 || value > 1.0;
      })) {
    throw std::invalid_argument(
        "roi_polygon values must be normalized to [0, 1]");
  }
  if (path_points < 2) {
    throw std::invalid_argument("path_points must be at least 2");
  }
  if (bev_width_px < 8 || bev_height_px < 8) {
    throw std::invalid_argument("bird's-eye dimensions are too small");
  }
  if (!std::isfinite(min_valid_ratio) || min_valid_ratio <= 0.0 ||
      min_valid_ratio > 1.0) {
    throw std::invalid_argument("min_valid_ratio must be in (0, 1]");
  }
  require_positive(min_surface_width_m, "min_surface_width_m");
  if (close_kernel_px < 0 ||
      (close_kernel_px > 0 && close_kernel_px % 2 == 0)) {
    throw std::invalid_argument(
        "close_kernel_px must be zero or a positive odd number");
  }
  if (fit_degree != 1 && fit_degree != 2) {
    throw std::invalid_argument("fit_degree must be 1 or 2");
  }
  require_positive(max_path_lateral_m, "max_path_lateral_m");
  require_positive(smoothing_time_constant_sec, "smoothing_time_constant_sec");
  require_positive(max_lateral_update_m, "max_lateral_update_m");
  if (!std::isfinite(path_hold_sec) || path_hold_sec < 0.0) {
    throw std::invalid_argument(
        "path_hold_sec must be finite and non-negative");
  }
  require_positive(path_duration_sec, "path_duration_sec");
}

void LidarSafetyConfig::validate() const {
  require_positive(timeout_sec, "LiDAR timeout");
  require_positive(obstacle_distance_m, "obstacle_distance_m");
  if (!std::isfinite(stop_distance_m) || stop_distance_m <= 0.0 ||
      stop_distance_m > obstacle_distance_m) {
    throw std::invalid_argument(
        "stop_distance_m must be within obstacle_distance_m");
  }
  require_positive(corridor_half_width_m, "corridor_half_width_m");
  if (!std::isfinite(z_min_m) || !std::isfinite(z_max_m) ||
      z_min_m >= z_max_m) {
    throw std::invalid_argument("LiDAR z bounds are invalid");
  }
  if (min_obstacle_points < 1) {
    throw std::invalid_argument("min_obstacle_points must be positive");
  }
}

void DriveConfig::validate() const {
  for (const auto &[value, name] :
       std::array{std::pair{max_forward_mps, "max_forward_mps"},
                  std::pair{max_yaw_rps, "max_yaw_rps"},
                  std::pair{heading_gain, "heading_gain"},
                  std::pair{lookahead_m, "lookahead_m"},
                  std::pair{max_lateral_target_m, "max_lateral_target_m"},
                  std::pair{max_camera_age_sec, "max_camera_age_sec"},
                  std::pair{max_inference_age_sec, "max_inference_age_sec"},
                  std::pair{max_path_age_sec, "max_path_age_sec"},
                  std::pair{max_lidar_age_sec, "max_lidar_age_sec"},
                  std::pair{min_clearance_m, "min_clearance_m"}}) {
    require_positive(value, name);
  }
  if (!std::isfinite(min_confidence) || min_confidence <= 0.0 ||
      min_confidence > 1.0) {
    throw std::invalid_argument("min_confidence must be in (0, 1]");
  }
}

const std::vector<ProfileSpec> &profiles() {
  static const std::vector<ProfileSpec> value{
      {"swin-l-best-so-far", "mask2former", std::string{kPinnedModelId},
       std::string{kPinnedModelRevision}, 384, 384, "fp32", 0.62, 0.07},
      {"swin-l-aspect-224x384", "mask2former", std::string{kPinnedModelId},
       std::string{kPinnedModelRevision}, 224, 384, "fp32", 0.62, 0.07},
      {"swin-l-aspect-448x768", "mask2former", std::string{kPinnedModelId},
       std::string{kPinnedModelRevision}, 448, 768, "fp32", 0.62, 0.07},
      {"r50-fp16-640x360", "maskformer", "facebook/maskformer-resnet50-vistas",
       "ae4b8c2590c0a090fc32d5c217d78738a2dd4b19", 360, 640, "fp16", 0.62, 0.0},
  };
  return value;
}

const ProfileSpec &resolve_profile(const std::string_view name) {
  const auto &values = profiles();
  const auto found = std::ranges::find(values, name, &ProfileSpec::name);
  if (found == values.end()) {
    throw std::invalid_argument("unsupported profile: " + std::string{name});
  }
  return *found;
}

void SegmenterConfig::validate() const {
  (void)resolve_profile(profile);
  if (model_path.empty()) {
    throw std::invalid_argument(
        "model_path must point to an exported ONNX model");
  }
  if (evaluation_height <= 0 || evaluation_width <= 0) {
    throw std::invalid_argument("evaluation dimensions must be positive");
  }
  if (temporal_alpha && (!std::isfinite(*temporal_alpha) ||
                         *temporal_alpha < 0.5 || *temporal_alpha > 1.0)) {
    throw std::invalid_argument("temporal_alpha must be in [0.5, 1.0]");
  }
  if (temporal_hysteresis_margin &&
      (!std::isfinite(*temporal_hysteresis_margin) ||
       *temporal_hysteresis_margin < 0.0 ||
       *temporal_hysteresis_margin > 1.0)) {
    throw std::invalid_argument("temporal_hysteresis_margin must be in [0, 1]");
  }
}

std::optional<std::string> environment(const std::string_view name) {
  const std::string key{name};
  const char *value = std::getenv(key.c_str());
  if (value == nullptr) {
    return std::nullopt;
  }
  return trim(value);
}

std::string environment_or(const std::string_view name, std::string fallback) {
  auto value = environment(name);
  return value && !value->empty() ? *value : std::move(fallback);
}

bool environment_flag(const std::string_view name, const bool fallback) {
  auto value = environment(name);
  if (!value) {
    return fallback;
  }
  std::ranges::transform(*value, value->begin(), [](unsigned char character) {
    return std::tolower(character);
  });
  if (*value == "1" || *value == "true" || *value == "yes" || *value == "on") {
    return true;
  }
  if (*value == "0" || *value == "false" || *value == "no" || *value == "off") {
    return false;
  }
  throw std::invalid_argument(std::string{name} + " must be a boolean value");
}

double environment_double(const std::string_view name, const double fallback) {
  auto value = environment(name);
  if (!value || value->empty()) {
    return fallback;
  }
  std::size_t consumed = 0;
  const double parsed = std::stod(*value, &consumed);
  if (consumed != value->size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(std::string{name} + " must be a finite number");
  }
  return parsed;
}

int environment_int(const std::string_view name, const int fallback) {
  auto value = environment(name);
  if (!value || value->empty()) {
    return fallback;
  }
  std::size_t consumed = 0;
  const int parsed = std::stoi(*value, &consumed);
  if (consumed != value->size()) {
    throw std::invalid_argument(std::string{name} + " must be an integer");
  }
  return parsed;
}

std::array<double, 8> parse_roi_polygon(const std::string_view value) {
  std::string normalized{value};
  std::ranges::replace(normalized, ';', ',');
  std::istringstream stream{normalized};
  std::array<double, 8> result{};
  std::string part;
  std::size_t index = 0;
  while (std::getline(stream, part, ',')) {
    if (index >= result.size()) {
      throw std::invalid_argument(
          "SWIN_L_ROI_POLYGON must contain eight values");
    }
    std::size_t consumed = 0;
    result[index] = std::stod(trim(part), &consumed);
    if (consumed != trim(part).size()) {
      throw std::invalid_argument(
          "SWIN_L_ROI_POLYGON must be comma-separated floats");
    }
    ++index;
  }
  if (index != result.size()) {
    throw std::invalid_argument("SWIN_L_ROI_POLYGON must contain eight values");
  }
  return result;
}

} // namespace line_tracking
