cmake_minimum_required(VERSION 3.8)
project(state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# include header files
include_directories(include)

add_executable(state_node src/state_node.cpp
                          src/fsm.cpp)

target_link_libraries(state_node PUBLIC
      ${lifecycle_msgs_TARGETS}
      rclcpp::rclcpp
    )

install(TARGETS
  state_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
