
. install/setup.bash

ros2 run pr_sensor pr_node &
ros2 run imu_sensor imu_node &
ros2 run path_planner path_node &
ros2 run motor_controller motor_node &
ros2 run status status_node &

wait