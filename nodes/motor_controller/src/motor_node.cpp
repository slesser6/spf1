#include "motor_driver.h"
#include "pid.h"
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @class MotorNode
 * @brief ROS 2 node that controls motors.
 *
 * Subscribes to a state, user control messaeges, a navigation path,
 * and IMU data, then uses the state to determine whether to control
 * the motors based on user input or the navigation path. Uses a PID
 * controller to adjust motor commands to follow the path.
 */
class MotorNode : public rclcpp::Node {
public:
  MotorNode()
      : Node("motor_node"), path_received_(false), imu_received_(false) {

    pid_controller_ = PIDController();
    pid_controller_.kp = 1.5f;
    pid_controller_.ki = 0.0f;
    pid_controller_.kd = 0.2f;
    pid_controller_.prev_error = 0.0f;
    pid_controller_.integral = 0.0f;

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/state/get", 10,
        std::bind(&MotorNode::onState, this, std::placeholders::_1));
    ctrl_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/motor/control", 10,
        std::bind(&MotorNode::onControl, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        std::bind(&MotorNode::onPath, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/sensors/imu", 10,
        std::bind(&MotorNode::onImu, this, std::placeholders::_1));
    status_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/status", std::bind(&MotorNode::onStatus, this, std::placeholders::_1,
                             std::placeholders::_2));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&MotorNode::controlLoop, this));

    last_time_ = now();
    initialized_ = false;
    RCLCPP_INFO(get_logger(), "MotorNode started");
  }

private:
  // ROS Topics
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctrl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool path_received_, imu_received_, ctrl_received_, initialized_;
  geometry_msgs::msg::Pose target_pose_;
  geometry_msgs::msg::Quaternion current_orientation_;
  rclcpp::Time last_time_;
  std::string state_, current_ctrl_;

  // PID Controller
  PIDController pid_controller_;

  /**
   * @brief Callback for receiving state change messages.
   *
   * Updates the current state.
   *
   * @param msg Shared pointer to the received String message.
   */
  void onState(const std_msgs::msg::String::SharedPtr msg) {
    state_ = msg->data;
  }

  /**
   * @brief Callback for receiving motor control messages.
   *
   * Updates the motor control command.
   *
   * @param msg Shared pointer to the received String message.
   */
  void onControl(const std_msgs::msg::String::SharedPtr msg) {
    current_ctrl_ = msg->data;
    ctrl_received_ = true;
  }

  /**
   * @brief Callback for receiving navigation path messages.
   *
   * Updates the target pose to the last pose in the path and marks path as
   * received if the state is FIND.
   *
   * @param msg Shared pointer to the received Path message.
   */
  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg->poses.empty() && state_ == "FIND") {
      target_pose_ = msg->poses.back().pose;
      path_received_ = true;
    }
  }

  /**
   * @brief Callback for receiving IMU messages.
   *
   * Updates the current orientation from the IMU and marks IMU data as
   * received.
   *
   * @param msg Shared pointer to the received IMU message.
   */
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    current_orientation_ = msg->orientation;
    imu_received_ = true;
  }

  /**
   * @brief Main control loop called periodically by the timer.
   *
   * If the state is INIT - initializes the motors
   * If the state is CTRL or ALIGN - uses the control messages to move
   * forwards, backwards, left, or right.
   * If the state is NAV - Checks for valid inputs, computes yaw error
   * using PID, clamps the speed, and sends motor commands accordingly.
   * If inputs are missing, motors are stopped.
   */
  void controlLoop() {

    if (state_ == "INIT") {
      if (!initialized_) {
        motorDriverInit();
        initialized_ = true;
        RCLCPP_INFO(get_logger(), "Motors initialized");
      }
    } else if (state_ == "CTRL" || state_ == "ALIGN") {
      if (!ctrl_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for inputs...");
        motorStop();
        return;
      }
      switch (current_ctrl_[0]) {
      case 'R':
        motorTurnRight(70);
        RCLCPP_DEBUG(get_logger(), "Turning right");
        break;
      case 'L':
        motorTurnLeft(70);
        RCLCPP_DEBUG(get_logger(), "Turning left");
        break;
      case 'F':
        motorDriveForward(70);
        RCLCPP_DEBUG(get_logger(), "Driving forwards");
        break;
      case 'B':
        RCLCPP_DEBUG(get_logger(), "Driving backwards");
        motorDriveBackward(70);
        break;
      default:
        motorStop();
      }
    } else if (state_ == "NAV") {
      if (!path_received_ || !imu_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for inputs...");
        motorStop();
        return;
      }

      // Compute yaw from quaternion
      float yaw = getYawFromQuaternion(current_orientation_);
      float target_yaw = computeTargetYaw(target_pose_);

      float dt = (now() - last_time_).seconds();
      last_time_ = now();

      float control_signal = pidCompute(&pid_controller_, target_yaw, yaw, dt);
      int speed = clampSpeed(std::abs(control_signal) * 70.0f);

      if (std::abs(control_signal) < 0.05f) {
        motorDriveForward(speed);
      } else if (control_signal > 0) {
        motorTurnLeft(speed);
      } else {
        motorTurnRight(speed);
      }
    }
  }

  /**
   * @brief Service callback to report system status.
   *
   * Sets success to true if both path and IMU data have been received.
   *
   * @param request Service request (unused).
   * @param response Service response containing status.
   */
  void onStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = path_received_ && imu_received_;
    res->message = res->success ? "System ready" : "Inputs missing";
  }

  /**
   * @brief Extract yaw angle from a quaternion orientation.
   *
   * Assumes flat ground and extracts yaw (rotation about Z axis).
   *
   * @param q Quaternion to extract yaw from.
   * @return float Yaw angle in radians.
   */
  float getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  /**
   * @brief Compute target yaw angle based on target pose position.
   *
   * Calculates heading angle from robot origin to target pose.
   *
   * @param target Target pose.
   * @return float Target yaw angle in radians.
   */
  float computeTargetYaw(const geometry_msgs::msg::Pose &target) {
    float dx = target.position.x;
    float dy = target.position.y;
    return std::atan2(dy, dx); // Assume robot starts at 0,0 facing x+
  }
};

/**
 * @brief Main entry point for MotorNode.
 *
 * Initializes ROS 2, spins the MotorNode instance, then shuts down.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}