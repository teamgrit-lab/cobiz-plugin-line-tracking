#include "line_tracking/task_manager.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <stdexcept>
#include <unordered_map>

namespace line_tracking {
namespace {

const std::regex kTaskIdPattern{"^[A-Za-z0-9_-]{1,128}$"};

struct TaskIdentity {
  nlohmann::json raw;
  std::string key;
};

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::optional<TaskIdentity> task_identity(const nlohmann::json &event) {
  if (!event.is_object() || !event.contains("task_id")) {
    return std::nullopt;
  }
  const auto &value = event["task_id"];
  std::string key;
  nlohmann::json raw;
  if (value.is_string()) {
    key = trim(value.get<std::string>());
    raw = key;
  } else if (value.is_number_integer() || value.is_number_unsigned()) {
    key = value.dump();
    raw = value;
  } else {
    return std::nullopt;
  }
  if (!std::regex_match(key, kTaskIdPattern)) {
    return std::nullopt;
  }
  return TaskIdentity{std::move(raw), std::move(key)};
}

nlohmann::json payload(const nlohmann::json &event) {
  if (!event.contains("payload") || event["payload"].is_null()) {
    return nlohmann::json::object();
  }
  nlohmann::json value = event["payload"];
  if (value.is_string()) {
    try {
      value = nlohmann::json::parse(value.get<std::string>());
    } catch (const nlohmann::json::parse_error &) {
      throw std::invalid_argument("invalid_payload_json");
    }
  }
  if (!value.is_object()) {
    throw std::invalid_argument("invalid_payload");
  }
  return value;
}

int selected_mask(const nlohmann::json &value, const int fallback) {
  if (!value.contains("selected_mask")) {
    return fallback;
  }
  const auto &selected = value["selected_mask"];
  if (!selected.is_number_integer() || selected.is_boolean()) {
    throw std::invalid_argument("invalid_selected_mask");
  }
  const int result = selected.get<int>();
  if (result != 1 && result != 2) {
    throw std::invalid_argument("invalid_selected_mask");
  }
  return result;
}

double duration(const nlohmann::json &value, const TaskPolicy &policy) {
  if (!value.contains("duration_sec")) {
    return policy.default_duration_sec;
  }
  const auto &raw = value["duration_sec"];
  if (!raw.is_number() || raw.is_boolean()) {
    throw std::invalid_argument("invalid_duration_sec");
  }
  const double result = raw.get<double>();
  if (!std::isfinite(result) || result < policy.startup_hold_sec + 1.0 ||
      result > policy.max_duration_sec) {
    throw std::invalid_argument("duration_sec_out_of_range");
  }
  return result;
}

std::optional<std::string> string_field(const nlohmann::json &object,
                                        const char *name) {
  if (!object.contains(name) || !object[name].is_string()) {
    return std::nullopt;
  }
  return object[name].get<std::string>();
}

} // namespace

void TaskPolicy::validate() const {
  for (const double value : {default_duration_sec, max_duration_sec,
                             unsafe_timeout_sec, startup_hold_sec}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
          "task timing limits must be positive and finite");
    }
  }
  if (default_duration_sec < startup_hold_sec + 1.0 ||
      default_duration_sec > max_duration_sec) {
    throw std::invalid_argument(
        "default duration must exceed startup hold by 1 second");
  }
  if (default_selected_mask != 1 && default_selected_mask != 2) {
    throw std::invalid_argument(
        "default selected_mask must be 1 (road) or 2 (sidewalk)");
  }
}

int requested_selected_mask(const nlohmann::json &event,
                            const int default_value) {
  return selected_mask(payload(event), default_value);
}

nlohmann::json task_state(const nlohmann::json &task_id,
                          const std::string_view state_type,
                          const nlohmann::json &device_id,
                          const std::optional<std::string> &device_name,
                          const std::optional<std::string> &reason) {
  static const std::unordered_map<std::string, std::string> routes{
      {"TASK_STARTED", "start"},
      {"TASK_COMPLETED", "complete"},
      {"TASK_REJECTED", "reject"},
      {"TASK_ABORTED", "abort"},
  };
  const auto route = routes.find(std::string{state_type});
  if (route == routes.end()) {
    throw std::invalid_argument("unsupported task state type");
  }
  nlohmann::json result{{"type", state_type},
                        {"task_id", task_id},
                        {"task_type", route->second},
                        {"action_name", kActionName}};
  if (!device_id.is_null()) {
    result["device_id"] = device_id;
  }
  if (device_name) {
    result["device_name"] = *device_name;
  }
  if (reason && !reason->empty()) {
    result["reason"] = *reason;
  }
  return result;
}

LineTrackingTasks::LineTrackingTasks(TaskPolicy policy) : policy_(policy) {
  policy_.validate();
}

void LineTrackingTasks::remember(std::string key) {
  if (recent_ids_.size() == 256) {
    recent_ids_.pop_front();
  }
  recent_ids_.push_back(std::move(key));
}

nlohmann::json
LineTrackingTasks::state(const ActiveTask &task,
                         const std::string_view state_type,
                         const std::optional<std::string> &reason) const {
  return task_state(task.task_id, state_type, task.device_id, task.device_name,
                    reason);
}

std::optional<nlohmann::json>
LineTrackingTasks::handle_event(const nlohmann::json &event, const double now,
                                const std::string_view ready_reason) {
  if (!event.is_object()) {
    return std::nullopt;
  }
  const std::string event_type = event.value("type", "");
  const std::string action = event.value("action_name", "");
  const auto identity = task_identity(event);
  if (event_type == "TASK_ABORTED") {
    if (active_ && identity && identity->key == active_->key &&
        (action.empty() || action == kActionName)) {
      return finish("TASK_ABORTED", "task_aborted_by_server");
    }
    return std::nullopt;
  }
  if (event_type != "TASK_REGISTERED" || action != kActionName || !identity) {
    return std::nullopt;
  }
  if (std::ranges::find(recent_ids_, identity->key) != recent_ids_.end()) {
    return std::nullopt;
  }
  remember(identity->key);

  nlohmann::json device_id = nullptr;
  if (event.contains("device_id") &&
      (event["device_id"].is_string() ||
       event["device_id"].is_number_integer() ||
       event["device_id"].is_number_unsigned())) {
    device_id = event["device_id"];
  }
  const auto device_name = string_field(event, "device_name");
  ActiveTask candidate{identity->raw,
                       identity->key,
                       device_id,
                       device_name,
                       now,
                       0.0,
                       policy_.default_selected_mask};
  if (active_) {
    return state(candidate, "TASK_REJECTED",
                 "another_line_tracking_task_active");
  }

  int requested_mask = policy_.default_selected_mask;
  double requested_duration = policy_.default_duration_sec;
  try {
    const auto body = payload(event);
    requested_mask = selected_mask(body, policy_.default_selected_mask);
    requested_duration = duration(body, policy_);
  } catch (const std::invalid_argument &error) {
    return state(candidate, "TASK_REJECTED", error.what());
  }
  if (ready_reason != "tracking") {
    return state(candidate, "TASK_REJECTED", std::string{ready_reason});
  }
  candidate.duration_sec = requested_duration;
  candidate.selected_mask = requested_mask;
  active_ = std::move(candidate);
  tracking_seen_ = false;
  unsafe_since_.reset();
  return state(*active_, "TASK_STARTED");
}

std::optional<nlohmann::json>
LineTrackingTasks::tick(const double now, const std::string_view drive_reason) {
  if (!active_) {
    return std::nullopt;
  }
  const double elapsed = now - active_->started_at;
  if (elapsed < policy_.startup_hold_sec) {
    return std::nullopt;
  }
  const bool tracked_before_this_tick = tracking_seen_;
  if (drive_reason == "tracking") {
    tracking_seen_ = true;
    unsafe_since_.reset();
  } else if (!unsafe_since_) {
    unsafe_since_ = now;
  } else if (now - *unsafe_since_ >= policy_.unsafe_timeout_sec) {
    return finish("TASK_ABORTED", "unsafe:" + std::string{drive_reason});
  }
  if (elapsed >= active_->duration_sec) {
    if (tracked_before_this_tick && drive_reason == "tracking") {
      return finish("TASK_COMPLETED");
    }
    return finish("TASK_ABORTED",
                  "tracking_unavailable:" + std::string{drive_reason});
  }
  return std::nullopt;
}

std::optional<nlohmann::json>
LineTrackingTasks::finish(const std::string_view state_type,
                          const std::optional<std::string> &reason) {
  if (!active_) {
    return std::nullopt;
  }
  auto result = state(*active_, state_type, reason);
  active_.reset();
  tracking_seen_ = false;
  unsafe_since_.reset();
  return result;
}

} // namespace line_tracking
