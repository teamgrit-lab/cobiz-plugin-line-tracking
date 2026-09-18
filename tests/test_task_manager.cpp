#include "test_framework.hpp"

#include "line_tracking/task_manager.hpp"

using nlohmann::json;
using namespace line_tracking;

namespace {

json event(json task_id = 123) {
  return {{"type", "TASK_REGISTERED"},
          {"task_id", std::move(task_id)},
          {"action_name", "LINE_TRACKING"},
          {"device_id", 45},
          {"device_name", "Dangjin-A2"}};
}

} // namespace

LT_TEST("unrelated and malformed tasks do not start") {
  LineTrackingTasks tasks;
  auto unrelated = event();
  unrelated["action_name"] = "ARM_RESET";
  LT_REQUIRE(!tasks.handle_event(unrelated, 0.0, "tracking"));
  auto malformed = event("../../bad");
  LT_REQUIRE(!tasks.handle_event(malformed, 0.0, "tracking"));
  LT_REQUIRE(!tasks.active());
}

LT_TEST("unsafe preflight rejects without activation") {
  LineTrackingTasks tasks;
  const auto result = tasks.handle_event(event(), 0.0, "drive_not_armed");
  LT_REQUIRE(result.has_value());
  LT_REQUIRE((*result)["type"] == "TASK_REJECTED");
  LT_REQUIRE((*result)["reason"] == "drive_not_armed");
  LT_REQUIRE(!tasks.active());
}

LT_TEST("finite task starts and completes only after tracking") {
  TaskPolicy policy;
  policy.default_duration_sec = 5.0;
  policy.max_duration_sec = 10.0;
  LineTrackingTasks tasks{policy};
  auto request = event();
  request["payload"] = {{"duration_sec", 4}};
  const auto started = tasks.handle_event(request, 10.0, "tracking");
  LT_REQUIRE(started && (*started)["type"] == "TASK_STARTED");
  LT_REQUIRE(!tasks.tick(11.0, "drive_not_armed"));
  LT_REQUIRE(!tasks.tick(12.1, "tracking"));
  const auto completed = tasks.tick(14.1, "tracking");
  LT_REQUIRE(completed && (*completed)["type"] == "TASK_COMPLETED");
  LT_REQUIRE(!tasks.active());
}

LT_TEST("task payload selects road and rejects invalid masks") {
  LineTrackingTasks road_tasks;
  auto request = event();
  request["payload"] = "{\"selected_mask\":1}";
  const auto started = road_tasks.handle_event(request, 0.0, "tracking");
  LT_REQUIRE(started.has_value());
  LT_REQUIRE(road_tasks.active()->selected_mask == 1);

  LineTrackingTasks invalid_tasks;
  auto invalid = event(124);
  invalid["payload"] = {{"selected_mask", 0}};
  const auto rejected =
      invalid_tasks.handle_event(invalid, 0.0, "path_unavailable");
  LT_REQUIRE(rejected && (*rejected)["reason"] == "invalid_selected_mask");
}

LT_TEST("sustained unsafe state aborts but transient blockage clears") {
  TaskPolicy policy;
  policy.default_duration_sec = 10.0;
  policy.max_duration_sec = 10.0;
  LineTrackingTasks tasks{policy};
  LT_REQUIRE(tasks.handle_event(event(), 0.0, "tracking").has_value());
  LT_REQUIRE(!tasks.tick(2.1, "tracking"));
  LT_REQUIRE(!tasks.tick(3.0, "lidar_clearance_low"));
  LT_REQUIRE(!tasks.tick(4.0, "tracking"));
  LT_REQUIRE(!tasks.tick(5.0, "lidar_unavailable"));
  const auto stopped = tasks.tick(7.1, "lidar_unavailable");
  LT_REQUIRE(stopped && (*stopped)["type"] == "TASK_ABORTED");
  LT_REQUIRE((*stopped)["reason"] == "unsafe:lidar_unavailable");
}

LT_TEST("server abort requires the active matching task") {
  LineTrackingTasks tasks;
  LT_REQUIRE(tasks.handle_event(event("a-1"), 0.0, "tracking").has_value());
  const json other{{"type", "TASK_ABORTED"}, {"task_id", "other"}};
  LT_REQUIRE(!tasks.handle_event(other, 1.0, "tracking"));
  const json matching{{"type", "TASK_ABORTED"}, {"task_id", "a-1"}};
  const auto stopped = tasks.handle_event(matching, 1.0, "tracking");
  LT_REQUIRE(stopped && (*stopped)["task_type"] == "abort");
}

LT_TEST("busy task is rejected and duplicate ids are ignored") {
  LineTrackingTasks tasks;
  LT_REQUIRE(tasks.handle_event(event(), 0.0, "tracking").has_value());
  const auto busy = tasks.handle_event(event(124), 1.0, "tracking");
  LT_REQUIRE(busy && (*busy)["reason"] == "another_line_tracking_task_active");
  LT_REQUIRE(!tasks.handle_event(event(124), 2.0, "tracking"));
  LT_REQUIRE(tasks.active()->key == "123");
}

LT_TEST("invalid duration and subsecond default policy are rejected") {
  for (const json value :
       {json{0}, json{-1}, json{2}, json{301}, json{true}, json{"20"}}) {
    LineTrackingTasks tasks;
    auto request = event();
    request["payload"] = {{"duration_sec", value}};
    const auto rejected = tasks.handle_event(request, 0.0, "tracking");
    LT_REQUIRE(rejected && (*rejected)["type"] == "TASK_REJECTED");
  }
  TaskPolicy policy;
  policy.default_duration_sec = 0.5;
  LT_THROWS(LineTrackingTasks{policy});
}

LT_TEST("first tracking tick at deadline cannot falsely complete") {
  TaskPolicy policy;
  policy.default_duration_sec = 3.0;
  policy.max_duration_sec = 10.0;
  LineTrackingTasks tasks{policy};
  LT_REQUIRE(tasks.handle_event(event(), 0.0, "tracking").has_value());
  const auto result = tasks.tick(3.0, "tracking");
  LT_REQUIRE(result && (*result)["type"] == "TASK_ABORTED");
}

LT_TEST("string task id is normalized for core reports") {
  LineTrackingTasks tasks;
  const auto started = tasks.handle_event(event(" 123 "), 0.0, "tracking");
  LT_REQUIRE(started && (*started)["task_id"] == "123");
  const json abort{{"type", "TASK_ABORTED"}, {"task_id", 123}};
  const auto stopped = tasks.handle_event(abort, 1.0, "tracking");
  LT_REQUIRE(stopped && (*stopped)["task_type"] == "abort");
}
