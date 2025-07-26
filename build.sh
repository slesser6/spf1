colcon build --packages-select pr_sensor --paths ./nodes/pr_sensor
colcon build --packages-select imu_sensor --paths ./nodes/imu_sensor
colcon build --packages-select path_planner --paths ./nodes/path_planner
colcon build --packages-select motor_controller --paths ./nodes/motor_controller
colcon build --packages-select status --paths ./nodes/status
