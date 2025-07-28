source /opt/ros/kilted/setup.bash

. install/setup.bash

sudo chmod 666 /dev/i2c-1

ros2 run pr_sensor pr_node &
ros2 run imu_sensor imu_node &
ros2 run path_planner path_node &
ros2 run motor_controller motor_node &
ros2 run status status_node &

wait