source /opt/ros/kilted/setup.bash

clang-format -i ./nodes/aligner/src/aligner_node.cpp
clang-format -i ./nodes/image_capture/src/image_node.cpp
clang-format -i ./nodes/imu_sensor/src/imu_node.cpp
clang-format -i ./nodes/motor_controller/src/motor_driver.cpp
clang-format -i ./nodes/motor_controller/src/motor_node.cpp
clang-format -i ./nodes/motor_controller/src/pid.cpp
clang-format -i ./nodes/path_planner/src/path_node.cpp
clang-format -i ./nodes/pr_sensor/src/pr_node.cpp
clang-format -i ./nodes/server/src/server_node.cpp
clang-format -i ./nodes/state/src/state_node.cpp
clang-format -i ./nodes/state/src/fsm.cpp
clang-format -i ./nodes/stereo_vision/src/stereo_node.cpp
clang-format -i ./nodes/sun_finder/src/sun_finder_node.cpp

colcon build --packages-select aligner --paths ./nodes/aligner
colcon build --packages-select image_capture --paths ./nodes/image_capture
colcon build --packages-select imu_sensor --paths ./nodes/imu_sensor
colcon build --packages-select motor_controller --paths ./nodes/motor_controller
colcon build --packages-select path_planner --paths ./nodes/path_planner
colcon build --packages-select pr_sensor --paths ./nodes/pr_sensor
colcon build --packages-select server --paths ./nodes/server
colcon build --packages-select state --paths ./nodes/state
colcon build --packages-select stereo_vision --paths ./nodes/stereo_vision
colcon build --packages-select sun_finder --paths ./nodes/sun_finder
