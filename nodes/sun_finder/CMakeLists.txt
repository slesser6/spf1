cmake_minimum_required(VERSION 3.8)
project(sun_finder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(sun_finder_node src/sun_finder_node.cpp)
target_link_libraries(sun_finder_node PUBLIC
      ${OpenCV_LIBS}
      ${sensor_msgs_TARGETS}
      ${geometry_msgs_TARGETS}
      rclcpp::rclcpp
      cv_bridge::cv_bridge
    )
include_directories(${OpenCV_INCLUDE_DIRS})

install(TARGETS
  sun_finder_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
