cmake_minimum_required(VERSION 3.22)

project(cobiz_line_tracking VERSION 2.0.0 LANGUAGES CXX)

option(LINE_TRACKING_BUILD_ROS2 "Build the ROS 2 runtime node" ON)
option(LINE_TRACKING_BUILD_TESTS "Build native unit tests" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio dnn)
find_package(Threads REQUIRED)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp REQUIRED)
  add_library(nlohmann_json INTERFACE)
  add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
  target_include_directories(nlohmann_json INTERFACE "${NLOHMANN_JSON_INCLUDE_DIR}")
endif()

add_library(line_tracking_core
  src/config.cpp
  src/drive_control.cpp
  src/local_path.cpp
  src/mask.cpp
  src/task_manager.cpp
  src/visualization.cpp
)
add_library(cobiz::line_tracking ALIAS line_tracking_core)
target_include_directories(line_tracking_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(line_tracking_core
  PUBLIC
    opencv_core
    opencv_imgproc
    nlohmann_json::nlohmann_json
    Threads::Threads
)
target_compile_options(line_tracking_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(line_tracking_inference src/segmenter.cpp)
add_library(cobiz::line_tracking_inference ALIAS line_tracking_inference)
target_link_libraries(line_tracking_inference
  PUBLIC cobiz::line_tracking
  PRIVATE opencv_dnn
)
target_compile_options(line_tracking_inference PRIVATE -Wall -Wextra -Wpedantic)

add_executable(line_tracking_cli apps/line_tracking_cli.cpp)
target_link_libraries(line_tracking_cli PRIVATE
  cobiz::line_tracking_inference opencv_imgcodecs opencv_videoio
)

if(LINE_TRACKING_BUILD_ROS2)
  find_package(ament_cmake QUIET)
  if(ament_cmake_FOUND)
    set(LINE_TRACKING_HAS_AMENT ON)
    find_package(cv_bridge REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)

    add_executable(line_tracking_node ros/line_tracking_node.cpp)
    target_link_libraries(line_tracking_node cobiz::line_tracking_inference)
    ament_target_dependencies(line_tracking_node
      cv_bridge geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs
    )
    install(TARGETS line_tracking_node DESTINATION lib/${PROJECT_NAME})
  else()
    message(STATUS "ament_cmake not found; ROS 2 node will not be built")
  endif()
endif()

include(GNUInstallDirs)
install(TARGETS line_tracking_core line_tracking_inference line_tracking_cli
  EXPORT line_trackingTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(LINE_TRACKING_BUILD_TESTS)
  enable_testing()
  add_executable(line_tracking_tests
    tests/test_deployment.cpp
    tests/test_main.cpp
    tests/test_drive_control.cpp
    tests/test_local_path.cpp
    tests/test_profiles.cpp
    tests/test_task_manager.cpp
  )
  target_link_libraries(line_tracking_tests PRIVATE cobiz::line_tracking)
  target_include_directories(line_tracking_tests PRIVATE tests)
  target_compile_definitions(line_tracking_tests PRIVATE
    LINE_TRACKING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(NAME line_tracking_tests COMMAND line_tracking_tests)
endif()

if(LINE_TRACKING_HAS_AMENT)
  ament_package()
endif()
