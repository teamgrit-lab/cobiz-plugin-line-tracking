#include "line_tracking/config.hpp"
#include "line_tracking/drive_control.hpp"
#include "line_tracking/local_path.hpp"
#include "line_tracking/segmenter.hpp"
#include "line_tracking/task_manager.hpp"
#include "line_tracking/visualization.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace line_tracking {
namespace {

using namespace std::chrono_literals;

double steady_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

bool live_source_stamp(const std::int64_t source, const std::int64_t now,
                       const double maximum_age_sec) {
  if (source <= 0 || source > now) {
    return false;
  }
  return (now - source) / 1e9 <= maximum_age_sec;
}

LocalPathConfig local_path_config() {
  LocalPathConfig config;
  config.near_distance_m = environment_double("SWIN_L_NEAR_DISTANCE_M", 3.0);
  config.far_distance_m = environment_double("SWIN_L_FAR_DISTANCE_M", 8.0);
  config.ground_half_width_m =
      environment_double("SWIN_L_GROUND_HALF_WIDTH_M", 4.0);
  config.search_half_width_m =
      environment_double("SWIN_L_SEARCH_HALF_WIDTH_M", 3.5);
  config.roi_polygon = parse_roi_polygon(environment_or(
      "SWIN_L_ROI_POLYGON", "0.08,1.00,0.92,1.00,0.62,0.22,0.38,0.22"));
  config.path_points = environment_int("SWIN_L_PATH_POINTS", 20);
  config.bev_width_px = environment_int("SWIN_L_BEV_WIDTH_PX", 280);
  config.bev_height_px = environment_int("SWIN_L_BEV_HEIGHT_PX", 160);
  config.min_valid_ratio = environment_double("SWIN_L_MIN_VALID_RATIO", 0.35);
  config.min_surface_width_m =
      environment_double("SWIN_L_MIN_SIDEWALK_WIDTH_M", 0.12);
  config.close_kernel_px = environment_int("SWIN_L_CLOSE_KERNEL_PX", 5);
  config.smoothing_time_constant_sec =
      environment_double("SWIN_L_SMOOTHING_TIME_CONSTANT_SEC", 0.80);
  config.max_lateral_update_m =
      environment_double("SWIN_L_MAX_LATERAL_UPDATE_M", 0.35);
  config.path_hold_sec = environment_double("SWIN_L_PATH_HOLD_SEC", 0.90);
  config.path_duration_sec =
      environment_double("SWIN_L_PATH_DURATION_SEC", 1.50);
  config.validate();
  return config;
}

LidarSafetyConfig lidar_config() {
  LidarSafetyConfig config;
  config.topic =
      environment_or("SWIN_L_LIDAR_TOPIC", "/unitree/slam_lidar/points1");
  config.timeout_sec = environment_double("SWIN_L_LIDAR_TIMEOUT_SEC", 0.35);
  config.obstacle_distance_m =
      environment_double("SWIN_L_OBSTACLE_DISTANCE_M", 8.0);
  config.stop_distance_m = environment_double("SWIN_L_STOP_DISTANCE_M", 3.0);
  config.corridor_half_width_m =
      environment_double("SWIN_L_CORRIDOR_HALF_WIDTH_M", 0.55);
  config.z_min_m = environment_double("SWIN_L_LIDAR_Z_MIN_M", -0.40);
  config.z_max_m = environment_double("SWIN_L_LIDAR_Z_MAX_M", 0.80);
  config.min_obstacle_points = environment_int("SWIN_L_MIN_OBSTACLE_POINTS", 3);
  config.validate();
  return config;
}

SegmenterConfig segmenter_config() {
  SegmenterConfig config;
  config.profile =
      environment_or("SWIN_L_PROFILE", std::string{kDefaultProfile});
  config.model_path = environment_or(
      "SWIN_L_MODEL_PATH", "/models/mask2former-swin-l-mapillary-224x384.onnx");
  config.evaluation_height = environment_int("SWIN_L_EVALUATION_HEIGHT", 360);
  config.evaluation_width = environment_int("SWIN_L_EVALUATION_WIDTH", 640);
  config.prefer_cuda = environment_or("SWIN_L_DEVICE", "cuda") != "cpu";
  config.validate();
  return config;
}

TaskPolicy task_policy(const int default_selected_mask) {
  TaskPolicy policy;
  policy.default_duration_sec =
      environment_double("LINE_TRACKING_DEFAULT_DURATION_SEC", 60.0);
  policy.max_duration_sec =
      environment_double("LINE_TRACKING_MAX_DURATION_SEC", 300.0);
  policy.unsafe_timeout_sec =
      environment_double("LINE_TRACKING_UNSAFE_TIMEOUT_SEC", 2.0);
  policy.default_selected_mask = default_selected_mask;
  policy.validate();
  return policy;
}

nlohmann::json lidar_json(const LidarSafetyResult &value) {
  return {{"stop", value.stop},
          {"lidar_available", value.lidar_available},
          {"obstacle_in_path", value.obstacle_in_path},
          {"obstacle_count", value.obstacle_count},
          {"clearance_m", value.clearance_m ? nlohmann::json{*value.clearance_m}
                                            : nlohmann::json{nullptr}},
          {"age_sec", value.age_sec ? nlohmann::json{*value.age_sec}
                                    : nlohmann::json{nullptr}},
          {"reason", value.reason}};
}

} // namespace

class LineTrackingNode final : public rclcpp::Node {
public:
  LineTrackingNode()
      : Node("swin_l_line_tracking_cpp"),
        task_mode_(environment_or("SWIN_L_MODE", "ros2") == "task-drive"),
        path_mask_class_(environment_int("SWIN_L_PATH_MASK_CLASS", 2)),
        output_hz_(environment_double("SWIN_L_OUTPUT_HZ", 10.0)),
        inference_hz_(environment_double("SWIN_L_INFERENCE_HZ", 4.0)),
        image_topic_(
            environment_or("SWIN_L_IMAGE_TOPIC", "/a2/front_camera/image_raw")),
        lidar_topic_(environment_or("SWIN_L_LIDAR_TOPIC",
                                    "/unitree/slam_lidar/points1")),
        joy_topic_(environment_or("JOY_TOPIC", "/a2_control")),
        path_frame_id_(environment_or("SWIN_L_PATH_FRAME_ID", "base_link")),
        local_config_(local_path_config()), lidar_config_(lidar_config()),
        segmenter_config_(segmenter_config()), segmenter_(segmenter_config_),
        lidar_(lidar_config_), tasks_(task_policy(path_mask_class_)) {
    validate_runtime();
    smoothers_[1] = std::make_unique<LocalPathSmoother>(local_config_);
    smoothers_[2] = std::make_unique<LocalPathSmoother>(local_config_);
    drive_config_.validate();
    drive_armed_ = environment_flag("SWIN_L_DRIVE_ENABLED") &&
                   environment_flag("SWIN_L_CALIBRATION_CONFIRMED");

    const auto reliability =
        environment_or("SWIN_L_INPUT_RELIABILITY", "best_effort") == "reliable"
            ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
            : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rclcpp::QoS input_qos{rclcpp::KeepLast{1}};
    input_qos.reliability(reliability);
    rclcpp::QoS output_qos{rclcpp::KeepLast{5}};
    output_qos.reliable();

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, input_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          on_image(std::move(message));
        });
    lidar_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, input_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          on_lidar(std::move(message));
        });
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
        environment_or("SWIN_L_LOCAL_PATH_TOPIC",
                       "/line_tracking/swin_l/local_path"),
        output_qos);
    safety_publisher_ = create_publisher<std_msgs::msg::Bool>(
        environment_or("SWIN_L_SAFETY_STOP_TOPIC",
                       "/line_tracking/swin_l/safety_stop"),
        output_qos);
    metrics_publisher_ = create_publisher<std_msgs::msg::String>(
        environment_or("SWIN_L_METRICS_TOPIC", "/line_tracking/swin_l/metrics"),
        output_qos);
    if (task_mode_) {
      overlay_publisher_ = create_publisher<sensor_msgs::msg::Image>(
          environment_or("SWIN_L_OVERLAY_TOPIC",
                         "/line_tracking/swin_l/overlay"),
          input_qos);
      clearance_publisher_ = create_publisher<std_msgs::msg::Float32>(
          environment_or("SWIN_L_CLEARANCE_TOPIC",
                         "/line_tracking/swin_l/clearance_m"),
          output_qos);
      task_state_publisher_ = create_publisher<std_msgs::msg::String>(
          environment_or("LINE_TRACKING_TASK_STATE_TOPIC", "/task_state"),
          output_qos);
      task_event_subscription_ = create_subscription<std_msgs::msg::String>(
          environment_or("LINE_TRACKING_TASK_EVENT_TOPIC", "/task_event"),
          output_qos, [this](std_msgs::msg::String::ConstSharedPtr message) {
            on_task_event(std::move(message));
          });
    }
    output_timer_ =
        create_wall_timer(std::chrono::duration<double>{1.0 / output_hz_},
                          [this] { publish_state(); });
    running_ = true;
    worker_ = std::thread([this] { inference_loop(); });

    RCLCPP_INFO(
        get_logger(),
        "C++ line tracking started | mode=%s image=%s lidar=%s profile=%s "
        "surface=%s inference=%.2fHz output=%.2fHz backend=%s",
        task_mode_ ? "task-drive" : "ros2", image_topic_.c_str(),
        lidar_topic_.c_str(), segmenter_config_.profile.c_str(),
        path_mask_class_ == 1 ? "ROAD" : "SIDEWALK", inference_hz_, output_hz_,
        segmenter_.using_cuda() ? "cuda" : "cpu");
  }

  ~LineTrackingNode() override {
    publish_drive(DriveDecision::stopped("shutdown"));
    if (auto body = tasks_.finish("TASK_ABORTED", "shutdown")) {
      publish_task_state(*body);
    }
    running_ = false;
    frame_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct FramePacket {
    cv::Mat frame;
    std_msgs::msg::Header header;
    double arrival_sec{};
    std::int64_t source_stamp_ns{};
    std::uint64_t sequence{};
  };

  struct RuntimeState {
    cv::Mat frame;
    cv::Mat selected_mask;
    std::array<std::optional<LocalPathEstimate>, 3> estimates;
    std::optional<std_msgs::msg::Header> header;
    std::optional<double> last_image_at;
    std::optional<double> last_inference_at;
    std::optional<std::int64_t> last_image_stamp_ns;
    std::optional<std::int64_t> last_inference_stamp_ns;
    std::optional<std::int64_t> last_lidar_stamp_ns;
    std::uint64_t sequence{};
    std::uint64_t inference_count{};
    std::uint64_t overwritten{};
    std::optional<std::string> worker_error;
    double last_inference_seconds{};
  };

  void validate_runtime() const {
    if (path_mask_class_ != 1 && path_mask_class_ != 2) {
      throw std::invalid_argument(
          "SWIN_L_PATH_MASK_CLASS must be 1 (road) or 2 (sidewalk)");
    }
    if (output_hz_ <= 0.0 || inference_hz_ <= 0.0) {
      throw std::invalid_argument("inference/output rates must be positive");
    }
    if (task_mode_) {
      if (segmenter_config_.profile != kDefaultProfile ||
          segmenter_config_.evaluation_height != 360 ||
          segmenter_config_.evaluation_width != 640) {
        throw std::invalid_argument(
            "task-drive is pinned to swin-l-aspect-224x384 with a 360x640 "
            "score map");
      }
      if (path_frame_id_ != "base_link") {
        throw std::invalid_argument(
            "task-drive requires a calibrated base_link path");
      }
      if (output_hz_ < 10.0) {
        throw std::invalid_argument(
            "task-drive requires at least 10 Hz zero-command updates");
      }
      if (environment_flag("SWIN_L_REQUIRE_CUDA", true) &&
          !segmenter_.using_cuda()) {
        throw std::runtime_error(
            "task-drive requires an OpenCV DNN CUDA backend; set "
            "SWIN_L_REQUIRE_CUDA=false only for bench testing");
      }
    }
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr message) {
    try {
      const auto source_stamp = stamp_ns(message->header.stamp);
      if (task_mode_) {
        const auto now_ns = now().nanoseconds();
        std::scoped_lock state_lock{state_mutex_};
        if (!live_source_stamp(source_stamp, now_ns,
                               drive_config_.max_camera_age_sec) ||
            (state_.last_image_stamp_ns &&
             source_stamp <= *state_.last_image_stamp_ns)) {
          state_.last_image_at.reset();
          publish_drive(DriveDecision::stopped("camera_timestamp_invalid"));
          return;
        }
      }
      const auto converted = cv_bridge::toCvCopy(message, "bgr8");
      FramePacket packet{converted->image.clone(), message->header,
                         steady_seconds(), source_stamp, 0};
      {
        std::scoped_lock state_lock{state_mutex_};
        packet.sequence = state_.sequence++;
        state_.last_image_at = packet.arrival_sec;
        state_.last_image_stamp_ns = source_stamp;
      }
      {
        std::scoped_lock frame_lock{frame_mutex_};
        if (latest_frame_) {
          std::scoped_lock state_lock{state_mutex_};
          ++state_.overwritten;
        }
        latest_frame_ = std::move(packet);
      }
      frame_condition_.notify_one();
    } catch (const std::exception &error) {
      std::scoped_lock state_lock{state_mutex_};
      state_.last_image_at.reset();
      publish_drive(DriveDecision::stopped("camera_conversion_error"));
      RCLCPP_ERROR(get_logger(), "camera conversion failed: %s", error.what());
    }
  }

  void on_lidar(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    try {
      const auto source_stamp = stamp_ns(message->header.stamp);
      if (task_mode_) {
        const auto now_ns = now().nanoseconds();
        std::scoped_lock state_lock{state_mutex_};
        if (!live_source_stamp(source_stamp, now_ns,
                               drive_config_.max_lidar_age_sec) ||
            (state_.last_lidar_stamp_ns &&
             source_stamp <= *state_.last_lidar_stamp_ns)) {
          lidar_.invalidate();
          publish_drive(DriveDecision::stopped("lidar_timestamp_invalid"));
          return;
        }
        if (!lidar_frame_matches_base(message->header.frame_id,
                                      path_frame_id_)) {
          lidar_.invalidate();
          publish_drive(DriveDecision::stopped("lidar_frame_invalid"));
          RCLCPP_WARN(get_logger(),
                      "LiDAR is not in base_link; no transform is applied");
          return;
        }
      }
      std::vector<PointFieldView> fields;
      fields.reserve(message->fields.size());
      for (const auto &field : message->fields) {
        fields.push_back({field.name, field.offset});
      }
      const PointCloud2View view{message->width,        message->height,
                                 message->point_step,   message->row_step,
                                 message->is_bigendian, std::move(fields),
                                 message->data};
      const auto points = decode_pointcloud_xyz(view);
      const bool has_finite_point =
          std::ranges::any_of(points, [](const cv::Point3f &point) {
            return std::isfinite(point.x) && std::isfinite(point.y) &&
                   std::isfinite(point.z);
          });
      if (task_mode_ && !has_finite_point) {
        throw std::runtime_error("LiDAR scan contains no finite points");
      }
      lidar_.update(points, steady_seconds());
      if (task_mode_) {
        std::scoped_lock state_lock{state_mutex_};
        state_.last_lidar_stamp_ns = source_stamp;
      }
    } catch (const std::exception &error) {
      if (task_mode_) {
        lidar_.invalidate();
        publish_drive(DriveDecision::stopped("lidar_decode_error"));
      }
      RCLCPP_WARN(get_logger(), "LiDAR decode failed: %s", error.what());
    }
  }

  void inference_loop() {
    auto next_allowed = std::chrono::steady_clock::now();
    while (running_) {
      std::optional<FramePacket> packet;
      {
        std::unique_lock lock{frame_mutex_};
        frame_condition_.wait(
            lock, [this] { return !running_ || latest_frame_.has_value(); });
        if (!running_) {
          return;
        }
        packet = std::move(latest_frame_);
        latest_frame_.reset();
      }
      if (std::chrono::steady_clock::now() < next_allowed) {
        std::this_thread::sleep_until(next_allowed);
      }
      try {
        auto result = segmenter_.segment(packet->frame);
        std::array<std::optional<LocalPathEstimate>, 3> estimates;
        for (const int mask_class :
             task_mode_ ? std::array{1, 2} : std::array{path_mask_class_, 0}) {
          if (mask_class == 0) {
            continue;
          }
          estimates[mask_class] = extract_surface_centerline(
              select_path_region(result.selected_mask, mask_class),
              local_config_);
          (void)smoothers_[mask_class]->update(estimates[mask_class],
                                               packet->arrival_sec);
        }
        std::scoped_lock state_lock{state_mutex_};
        state_.frame = packet->frame;
        state_.selected_mask = std::move(result.selected_mask);
        state_.estimates = std::move(estimates);
        state_.header = packet->header;
        ++state_.inference_count;
        state_.last_inference_at = steady_seconds();
        state_.last_inference_stamp_ns = packet->source_stamp_ns;
        state_.last_inference_seconds = result.total_seconds;
      } catch (const std::exception &error) {
        std::scoped_lock state_lock{state_mutex_};
        state_.worker_error = error.what();
        publish_drive(DriveDecision::stopped("inference_error"));
      }
      next_allowed =
          std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>{1.0 / inference_hz_});
    }
  }

  std::tuple<std::optional<SmoothedPath>, LidarSafetyResult, DriveDecision>
  drive_readiness(const int mask_class, const double current) {
    std::optional<double> camera_age;
    std::optional<double> inference_age;
    std::optional<std::int64_t> lidar_source_stamp;
    {
      std::scoped_lock state_lock{state_mutex_};
      if (state_.last_image_at) {
        camera_age = std::max(current - *state_.last_image_at, 0.0);
      }
      if (state_.last_inference_at) {
        inference_age = std::max(current - *state_.last_inference_at, 0.0);
      }
      lidar_source_stamp = state_.last_lidar_stamp_ns;
    }
    auto path = smoothers_[mask_class]->current(current);
    auto safety = lidar_.evaluate(path, current);
    if (task_mode_ && safety.lidar_available && lidar_source_stamp) {
      const double source_age =
          std::max((now().nanoseconds() - *lidar_source_stamp) / 1e9, 0.0);
      safety.age_sec = std::max(safety.age_sec.value_or(0.0), source_age);
    }
    const auto publishers = count_publishers(joy_topic_);
    const bool other_publishers =
        publishers > (command_publisher_ ? std::size_t{1} : std::size_t{0});
    auto decision =
        decide_drive(path, safety, camera_age, inference_age, other_publishers,
                     true, task_mode_ ? drive_armed_ : true, drive_config_);
    return {std::move(path), std::move(safety), std::move(decision)};
  }

  void on_task_event(std_msgs::msg::String::ConstSharedPtr message) {
    try {
      const auto event = nlohmann::json::parse(message->data);
      const double current = steady_seconds();
      std::string ready_reason = "inputs_not_ready";
      if (event.is_object() && event.value("type", "") == "TASK_REGISTERED" &&
          event.value("action_name", "") == kActionName && !tasks_.active()) {
        try {
          const int selected = requested_selected_mask(
              event, tasks_.policy().default_selected_mask);
          ready_reason = std::get<2>(drive_readiness(selected, current)).reason;
        } catch (const std::exception &) {
          // The lifecycle reports the precise invalid payload reason.
        }
      }
      if (command_publisher_ && !tasks_.active()) {
        ready_reason = "control_release_pending";
      }
      const bool was_active = tasks_.active().has_value();
      auto body = tasks_.handle_event(event, current, ready_reason);
      if (!body) {
        return;
      }
      if ((*body)["type"] == "TASK_STARTED") {
        try {
          rclcpp::QoS qos{rclcpp::KeepLast{10}};
          qos.best_effort();
          command_publisher_ =
              create_publisher<sensor_msgs::msg::Joy>(joy_topic_, qos);
          publish_drive(DriveDecision::stopped("startup_hold"));
        } catch (const std::exception &) {
          body = tasks_.finish("TASK_REJECTED", "control_publisher_error");
          command_publisher_.reset();
        }
      } else if (was_active && !tasks_.active()) {
        release_task_control("task_aborted_by_server");
      }
      if (body) {
        publish_task_state(*body);
      }
    } catch (const nlohmann::json::exception &) {
      RCLCPP_WARN(get_logger(), "ignored malformed /task_event JSON");
    }
  }

  void publish_state() {
    try {
      publish_state_checked();
    } catch (const std::exception &error) {
      publish_drive(DriveDecision::stopped("publish_error"));
      if (auto body = tasks_.finish("TASK_ABORTED", "publish_error")) {
        publish_task_state(*body);
      }
      RCLCPP_ERROR(get_logger(), "output failed: %s", error.what());
    }
  }

  void publish_state_checked() {
    const int mask_class =
        tasks_.active() ? tasks_.active()->selected_mask : path_mask_class_;
    const double current = steady_seconds();
    auto [path, safety, ready_decision] = drive_readiness(mask_class, current);
    std::optional<DriveDecision> decision;
    if (task_mode_) {
      last_ready_reason_ = ready_decision.reason;
      decision = ready_decision;
      if (tasks_.active() && current - tasks_.active()->started_at <
                                 tasks_.policy().startup_hold_sec) {
        decision = DriveDecision::stopped("startup_hold");
      }
      if (tasks_.active()) {
        if (auto terminal = tasks_.tick(current, decision->reason)) {
          release_task_control(terminal->value("reason", "task_complete"));
          publish_task_state(*terminal);
        } else {
          publish_drive(*decision);
        }
      } else if (command_publisher_) {
        decision = DriveDecision::stopped("task_idle");
        publish_drive(*decision);
        if (current >= stop_until_) {
          command_publisher_.reset();
        }
      } else {
        decision = DriveDecision::stopped("task_idle");
      }
    }

    std_msgs::msg::Bool safety_message;
    safety_message.data = safety.stop;
    safety_publisher_->publish(safety_message);
    if (clearance_publisher_) {
      std_msgs::msg::Float32 message;
      message.data = static_cast<float>(safety.clearance_m.value_or(0.0));
      clearance_publisher_->publish(message);
    }

    RuntimeState snapshot;
    {
      std::scoped_lock state_lock{state_mutex_};
      snapshot = state_;
    }
    if (snapshot.worker_error) {
      throw std::runtime_error(*snapshot.worker_error);
    }
    nlohmann::json metrics{
        {"runtime", "cpp-opencv-dnn"},
        {"profile", segmenter_config_.profile},
        {"camera_topic", image_topic_},
        {"lidar_topic", lidar_topic_},
        {"path_mask_class", mask_class},
        {"path_surface", mask_class == 1 ? "ROAD" : "SIDEWALK"},
        {"path_tracked", path.has_value()},
        {"path_confidence", path ? path->confidence : 0.0},
        {"path_age_sec",
         path ? nlohmann::json{path->age_sec} : nlohmann::json{nullptr}},
        {"path_duration_sec", local_config_.path_duration_sec},
        {"near_distance_m", local_config_.near_distance_m},
        {"far_distance_m", local_config_.far_distance_m},
        {"lidar", lidar_json(safety)},
        {"queue_overwritten", snapshot.overwritten},
        {"inference_count", snapshot.inference_count},
        {"inference_seconds", snapshot.last_inference_seconds},
        {"drive_reason",
         decision ? nlohmann::json{decision->reason} : nlohmann::json{nullptr}},
        {"ready_reason", task_mode_ ? nlohmann::json{last_ready_reason_}
                                    : nlohmann::json{nullptr}},
        {"task_active", tasks_.active().has_value()}};
    std_msgs::msg::String metrics_message;
    metrics_message.data = metrics.dump();
    metrics_publisher_->publish(metrics_message);

    if (!snapshot.header) {
      return;
    }
    nav_msgs::msg::Path path_message;
    path_message.header = *snapshot.header;
    path_message.header.frame_id = path_frame_id_;
    if (path) {
      for (std::size_t index = 0; index < path->points_xy.size(); ++index) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_message.header;
        pose.pose.position.x = path->points_xy[index].x;
        pose.pose.position.y = path->points_xy[index].y;
        if (index + 1 < path->points_xy.size()) {
          const double yaw = std::atan2(
              path->points_xy[index + 1].y - path->points_xy[index].y,
              path->points_xy[index + 1].x - path->points_xy[index].x);
          pose.pose.orientation.z = std::sin(yaw / 2.0);
          pose.pose.orientation.w = std::cos(yaw / 2.0);
        } else {
          pose.pose.orientation.w = 1.0;
        }
        path_message.poses.push_back(std::move(pose));
      }
    }
    path_publisher_->publish(path_message);

    if (overlay_publisher_ && !snapshot.frame.empty() &&
        !snapshot.selected_mask.empty()) {
      const auto overlay = render_local_path_overlay(
          snapshot.frame, snapshot.selected_mask,
          snapshot.estimates[mask_class], path, safety, local_config_,
          snapshot.sequence, snapshot.inference_count,
          snapshot.last_inference_seconds > 0.0
              ? 1.0 / snapshot.last_inference_seconds
              : 0.0,
          mask_class);
      auto message =
          cv_bridge::CvImage(*snapshot.header, "bgr8", overlay).toImageMsg();
      overlay_publisher_->publish(*message);
    }
  }

  void publish_drive(const DriveDecision &decision) {
    if (!command_publisher_) {
      return;
    }
    sensor_msgs::msg::Joy message;
    message.header.stamp = now();
    message.header.frame_id = "swin_l_drive_cpp";
    const auto axes = decision.joy_axes();
    message.axes.assign(axes.begin(), axes.end());
    message.buttons.assign(10, 0);
    command_publisher_->publish(message);
  }

  void release_task_control(const std::string &reason) {
    if (command_publisher_) {
      publish_drive(DriveDecision::stopped(reason));
      stop_until_ = steady_seconds() + 1.0;
    }
  }

  void publish_task_state(const nlohmann::json &body) {
    if (!task_state_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = body.dump();
    task_state_publisher_->publish(message);
  }

  bool task_mode_{};
  int path_mask_class_{};
  double output_hz_{};
  double inference_hz_{};
  std::string image_topic_;
  std::string lidar_topic_;
  std::string joy_topic_;
  std::string path_frame_id_;
  LocalPathConfig local_config_;
  LidarSafetyConfig lidar_config_;
  SegmenterConfig segmenter_config_;
  SurfaceSegmenter segmenter_;
  LidarSafetyMonitor lidar_;
  DriveConfig drive_config_;
  LineTrackingTasks tasks_;
  std::array<std::unique_ptr<LocalPathSmoother>, 3> smoothers_;
  bool drive_armed_{false};
  std::string last_ready_reason_{"inputs_not_ready"};
  double stop_until_{0.0};

  std::atomic<bool> running_{false};
  std::thread worker_;
  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  std::optional<FramePacket> latest_frame_;
  std::mutex state_mutex_;
  RuntimeState state_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      lidar_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      task_event_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr clearance_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr output_timer_;
};

} // namespace line_tracking

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<line_tracking::LineTrackingNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "line_tracking_node: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
