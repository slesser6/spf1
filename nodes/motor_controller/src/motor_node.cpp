#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "pid.h"
#include "motor_driver.h"

/**
 * @class MotorNode
 * @brief ROS 2 node that controls motors based on path and IMU sensor inputs.
 *
 * Subscribes to a navigation path and IMU orientation data,
 * uses a PID controller to adjust motor commands to follow the path,
 * and provides a status service to report readiness.
 */
class MotorNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for MotorNode.
     *
     * Initializes motor driver, sets up PID controller,
     * subscribes to path and IMU topics, creates status service,
     * and starts a timer for the control loop.
     */
    MotorNode()
        : Node("motor_node"),
          path_received_(false), imu_received_(false)
    {
        motor_driver_init();
        pid_controller_ = PIDController();
        pid_controller_.kp = 1.5f;
        pid_controller_.ki = 0.0f;
        pid_controller_.kd = 0.2f;
        pid_controller_.prev_error = 0.0f;
        pid_controller_.integral = 0.0f;

        // Subscribers
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&MotorNode::on_path, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu", 10, std::bind(&MotorNode::on_imu, this, std::placeholders::_1));
        status_srv_ = create_service<std_srvs::srv::Trigger>(
            "/status", std::bind(&MotorNode::on_status, this,
                                 std::placeholders::_1, std::placeholders::_2));
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MotorNode::control_loop, this));

        last_time_ = now();
        RCLCPP_INFO(get_logger(), "MotorNode started");
    }

private:
    /**
     * @brief Callback for receiving navigation path messages.
     *
     * Updates the target pose to the last pose in the path and marks path as received.
     *
     * @param msg Shared pointer to the received Path message.
     */
    void on_path(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg->poses.empty())
        {
            target_pose_ = msg->poses.back().pose;
            path_received_ = true;
        }
    }

    /**
     * @brief Callback for receiving IMU messages.
     *
     * Updates the current orientation from the IMU and marks IMU data as received.
     *
     * @param msg Shared pointer to the received IMU message.
     */
    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        current_orientation_ = msg->orientation;
        imu_received_ = true;
    }

    /**
     * @brief Main control loop called periodically by the timer.
     *
     * Checks for valid inputs, computes yaw error using PID, clamps speed,
     * and sends motor commands accordingly.
     * If inputs are missing, motors are stopped.
     */
    void control_loop()
    {
        if (!path_received_ || !imu_received_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for inputs...");
            motor_stop();
            return;
        }

        // Compute yaw from quaternion
        float yaw = get_yaw_from_quaternion(current_orientation_);
        float target_yaw = compute_target_yaw(target_pose_);

        float dt = (now() - last_time_).seconds();
        last_time_ = now();

        float control_signal = pid_compute(&pid_controller_, target_yaw, yaw, dt);
        int speed = clamp_speed(std::abs(control_signal) * 100.0f);

        if (std::abs(control_signal) < 0.05f)
        {
            motor_drive_forward(speed);
        }
        else if (control_signal > 0)
        {
            motor_turn_left(speed);
        }
        else
        {
            motor_turn_right(speed);
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
    void on_status(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
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
    float get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        // Basic Euler extraction (assuming flat ground)
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
    float compute_target_yaw(const geometry_msgs::msg::Pose &target)
    {
        // Simple heading: use x/y from target pose
        float dx = target.position.x;
        float dy = target.position.y;
        return std::atan2(dy, dx); // Assume robot starts at 0,0 facing x+
    }

    // ROS Interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    bool path_received_, imu_received_;
    geometry_msgs::msg::Pose target_pose_;
    geometry_msgs::msg::Quaternion current_orientation_;
    rclcpp::Time last_time_;

    // PID Controller
    PIDController pid_controller_;
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
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}