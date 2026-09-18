#include "test_framework.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string read(const std::filesystem::path &relative) {
  const auto path = std::filesystem::path{LINE_TRACKING_SOURCE_DIR} / relative;
  std::ifstream stream{path};
  if (!stream) {
    throw std::runtime_error("could not read deployment file: " +
                             path.string());
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

} // namespace

LT_TEST("compose keeps task-drive and host ROS contracts") {
  const auto compose = read("docker-compose.yml");
  LT_REQUIRE(compose.find("actual-activate:") != std::string::npos);
  LT_REQUIRE(compose.find("network_mode: host") != std::string::npos);
  LT_REQUIRE(compose.find("SWIN_L_MODE: task-drive") != std::string::npos);
  LT_REQUIRE(compose.find("JOY_TOPIC: ${JOY_TOPIC:-/a2_control}") !=
             std::string::npos);
  LT_REQUIRE(compose.find("SWIN_L_MODEL_PATH: /models/") != std::string::npos);
  LT_REQUIRE(compose.find("HF_HOME") == std::string::npos);
  LT_REQUIRE(compose.find("/opt/venv/bin/python") == std::string::npos);
}

LT_TEST("debug service cannot publish control by configuration") {
  const auto compose = read("docker-compose.yml");
  const auto debug_begin = compose.find("debugging-swin-l:");
  const auto actual_begin = compose.find("actual-activate:");
  LT_REQUIRE(debug_begin != std::string::npos);
  LT_REQUIRE(actual_begin != std::string::npos && actual_begin > debug_begin);
  const auto debug = compose.substr(debug_begin, actual_begin - debug_begin);
  LT_REQUIRE(debug.find("SWIN_L_MODE: task-drive") == std::string::npos);
  LT_REQUIRE(debug.find("JOY_TOPIC:") == std::string::npos);
}

LT_TEST("container builds and launches native binaries") {
  const auto dockerfile = read("Dockerfile.swin-l-debug");
  const auto entrypoint = read("docker/swin_l_debug_entrypoint.sh");
  LT_REQUIRE(dockerfile.find("cmake --build") != std::string::npos);
  LT_REQUIRE(dockerfile.find("line_tracking_node") != std::string::npos);
  LT_REQUIRE(dockerfile.find("pip install") == std::string::npos);
  LT_REQUIRE(dockerfile.find("COPY tools") == std::string::npos);
  LT_REQUIRE(entrypoint.find("line_tracking_node") != std::string::npos);
  LT_REQUIRE(entrypoint.find("python") == std::string::npos);
  LT_REQUIRE(entrypoint.find("TEAMGRIT_DDS_ENV") != std::string::npos);
}
