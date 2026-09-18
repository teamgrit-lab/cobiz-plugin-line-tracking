#pragma once

#include <nlohmann/json.hpp>

#include <deque>
#include <optional>
#include <string>
#include <string_view>

namespace line_tracking {

inline constexpr std::string_view kActionName = "LINE_TRACKING";

struct TaskPolicy {
  double default_duration_sec{60.0};
  double max_duration_sec{300.0};
  double unsafe_timeout_sec{2.0};
  double startup_hold_sec{2.0};
  int default_selected_mask{2};

  void validate() const;
};

struct ActiveTask {
  nlohmann::json task_id;
  std::string key;
  nlohmann::json device_id;
  std::optional<std::string> device_name;
  double started_at{};
  double duration_sec{};
  int selected_mask{2};
};

[[nodiscard]] int requested_selected_mask(const nlohmann::json &event,
                                          int default_value);
[[nodiscard]] nlohmann::json
task_state(const nlohmann::json &task_id, std::string_view state_type,
           const nlohmann::json &device_id = nullptr,
           const std::optional<std::string> &device_name = std::nullopt,
           const std::optional<std::string> &reason = std::nullopt);

class LineTrackingTasks {
public:
  explicit LineTrackingTasks(TaskPolicy policy = {});
  [[nodiscard]] const TaskPolicy &policy() const noexcept { return policy_; }
  [[nodiscard]] const std::optional<ActiveTask> &active() const noexcept {
    return active_;
  }
  [[nodiscard]] std::optional<nlohmann::json>
  handle_event(const nlohmann::json &event, double now,
               std::string_view ready_reason);
  [[nodiscard]] std::optional<nlohmann::json>
  tick(double now, std::string_view drive_reason);
  [[nodiscard]] std::optional<nlohmann::json>
  finish(std::string_view state_type,
         const std::optional<std::string> &reason = std::nullopt);

private:
  [[nodiscard]] nlohmann::json
  state(const ActiveTask &task, std::string_view state_type,
        const std::optional<std::string> &reason = std::nullopt) const;
  void remember(std::string key);

  TaskPolicy policy_;
  std::optional<ActiveTask> active_;
  bool tracking_seen_{false};
  std::optional<double> unsafe_since_;
  std::deque<std::string> recent_ids_;
};

} // namespace line_tracking
