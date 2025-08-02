source /opt/ros/kilted/setup.bash

. install/setup.bash

sudo chmod 666 /dev/i2c-1

ros2 run aligner aligner_node &
ros2 run image_capture image_node &
ros2 run imu_sensor imu_node &
ros2 run motor_controller motor_node &
ros2 run path_planner path_node &
ros2 run pr_sensor pr_node &
ros2 run server server_node &
ros2 run state state_node &
ros2 run stereo_vision stereo_node &
ros2 run sun_finder sun_finder_node &

wait