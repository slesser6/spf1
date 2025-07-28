cmake_minimum_required(VERSION 3.8)
project(stereo_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(stereo_node src/stereo_node.cpp)
target_link_libraries(stereo_node PUBLIC
      rclcpp::rclcpp
      ${OpenCV_LIBS}
    )
include_directories(${OpenCV_INCLUDE_DIRS})

install(TARGETS
  stereo_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
