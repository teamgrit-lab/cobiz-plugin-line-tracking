#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace line_tracking {

inline constexpr std::array<double, 8> kDefaultRoiPolygon{
    0.08, 1.00, 0.92, 1.00, 0.62, 0.22, 0.38, 0.22};

struct LocalPathConfig {
  double near_distance_m{3.0};
  double far_distance_m{8.0};
  double ground_half_width_m{4.0};
  double search_half_width_m{3.5};
  std::array<double, 8> roi_polygon{kDefaultRoiPolygon};
  int path_points{20};
  int bev_width_px{280};
  int bev_height_px{160};
  double min_valid_ratio{0.35};
  double min_surface_width_m{0.12};
  int close_kernel_px{5};
  int fit_degree{2};
  double max_path_lateral_m{3.5};
  double smoothing_time_constant_sec{0.80};
  double max_lateral_update_m{0.35};
  double path_hold_sec{0.90};
  double path_duration_sec{1.50};

  void validate() const;
};

struct LidarSafetyConfig {
  std::string topic{"/unitree/slam_lidar/points2"};
  double timeout_sec{0.35};
  double obstacle_distance_m{8.0};
  double stop_distance_m{3.0};
  double corridor_half_width_m{0.55};
  double z_min_m{-0.40};
  double z_max_m{0.80};
  int min_obstacle_points{3};

  void validate() const;
};

struct DriveConfig {
  double max_forward_mps{0.10};
  double max_yaw_rps{0.18};
  double heading_gain{1.0};
  double lookahead_m{4.0};
  double min_confidence{0.70};
  double max_lateral_target_m{0.75};
  double max_camera_age_sec{0.50};
  double max_inference_age_sec{0.50};
  double max_path_age_sec{0.45};
  double max_lidar_age_sec{0.35};
  double min_clearance_m{3.0};

  void validate() const;
};

struct ProfileSpec {
  std::string name;
  std::string model_family;
  std::string model_id;
  std::string model_revision;
  int input_height{};
  int input_width{};
  std::string precision;
  double temporal_alpha{};
  double temporal_hysteresis_margin{};
};

inline constexpr std::string_view kDefaultProfile = "swin-l-aspect-224x384";
inline constexpr std::string_view kPinnedModelId =
    "facebook/mask2former-swin-large-mapillary-vistas-semantic";
inline constexpr std::string_view kPinnedModelRevision =
    "4772b6bf101d91f2534c106dc524d906aeb3c68a";

[[nodiscard]] const std::vector<ProfileSpec> &profiles();
[[nodiscard]] const ProfileSpec &resolve_profile(std::string_view name);

struct SegmenterConfig {
  std::string profile{std::string{kDefaultProfile}};
  std::filesystem::path model_path;
  int evaluation_height{360};
  int evaluation_width{640};
  std::optional<double> temporal_alpha;
  std::optional<double> temporal_hysteresis_margin;
  bool prefer_cuda{true};

  void validate() const;
};

[[nodiscard]] std::optional<std::string> environment(std::string_view name);
[[nodiscard]] std::string environment_or(std::string_view name,
                                         std::string fallback);
[[nodiscard]] bool environment_flag(std::string_view name,
                                    bool fallback = false);
[[nodiscard]] double environment_double(std::string_view name, double fallback);
[[nodiscard]] int environment_int(std::string_view name, int fallback);
[[nodiscard]] std::array<double, 8> parse_roi_polygon(std::string_view value);

} // namespace line_tracking
