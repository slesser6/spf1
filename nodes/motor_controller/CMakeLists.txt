cmake_minimum_required(VERSION 3.8)
project(motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# include header files
include_directories(include)

add_executable(motor_node src/motor_node.cpp
                          src/motor_driver.cpp
                          src/pid.cpp)

target_link_libraries(motor_node PUBLIC
      ${std_srvs_TARGETS}
      ${nav_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      wiringPi
      rclcpp::rclcpp
    )

install(TARGETS
  motor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
