#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "pid_controller.h"
#include "motor_driver.h"

class MotorController : public rclcpp::Node
{
public:
    MotorController()
        : Node("motor_controller"),
          path_received_(false), imu_received_(false)
    {
        motor_driver_init();
        pid_controller_ = {.kp = 1.5f, .ki = 0.0f, .kd = 0.2f, .prev_error = 0.0f, .integral = 0.0f};

        // Subscribers
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&MotorController::onPath, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&MotorController::onImu, this, std::placeholders::_1));
        status_srv_ = create_service<std_srvs::srv::Trigger>(
            "/status", std::bind(&MotorController::onStatus, this,
                                 std::placeholders::_1, std::placeholders::_2));
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MotorController::controlLoop, this));

        last_time_ = now();
        RCLCPP_INFO(get_logger(), "MotorController started");
    }

private:
    void onPath(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg->poses.empty())
        {
            target_pose_ = msg->poses.back().pose;
            path_received_ = true;
        }
    }

    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        current_orientation_ = msg->orientation;
        imu_received_ = true;
    }

    void controlLoop()
    {
        if (!path_received_ || !imu_received_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for inputs...");
            motor_stop();
            return;
        }

        // Compute yaw from quaternion
        float yaw = getYawFromQuaternion(current_orientation_);
        float target_yaw = computeTargetYaw(target_pose_);

        float dt = (now() - last_time_).seconds();
        last_time_ = now();

        float control_signal = pid_compute(&pid_angular_, target_yaw, yaw, dt);
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

    void onStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        res->success = path_received_ && imu_received_;
        res->message = res->success ? "System ready" : "Inputs missing";
    }

    float getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        // Basic Euler extraction (assuming flat ground)
        float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    float computeTargetYaw(const geometry_msgs::msg::Pose &target)
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}