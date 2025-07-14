cmake_minimum_required(VERSION 3.8)
project(pr_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pr_node src/pr_node.cpp)
target_link_libraries(pr_node PUBLIC
      ${sensor_msgs_TARGETS}
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
      mosquitto
    )

install(TARGETS
  pr_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
