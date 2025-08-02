cmake_minimum_required(VERSION 3.8)
project(server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# include headers
include_directories(include)

add_executable(server_node src/server_node.cpp)
target_link_libraries(server_node PUBLIC
      rclcpp::rclcpp
      ${std_srvs_TARGETS}
      ${nav_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
    )

install(TARGETS
  server_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
