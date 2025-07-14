#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node") {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&ImuNode::publish_imu_data, this));

        // Open I2C
        const char *device = "/dev/i2c-1";
        i2c_file_ = open(device, O_RDWR);
        if (i2c_file_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open I2C device");
            throw std::runtime_error("I2C device open failed");
        }
        if (ioctl(i2c_file_, I2C_SLAVE, MPU6050_ADDR) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to set I2C address");
            throw std::runtime_error("I2C ioctl failed");
        }
        if (i2c_file_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open I2C device");
            rclcpp::shutdown();
        }

        if (ioctl(i2c_file_, I2C_SLAVE, MPU6050_ADDR) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to set I2C address");
            rclcpp::shutdown();
        }

        // Wake up MPU6050
        char config[2] = {PWR_MGMT_1, 0};
        write(i2c_file_, config, 2);
    }

    ~ImuNode() {
        if (i2c_file_ >= 0) close(i2c_file_);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_file_;

    int16_t read_word(int reg) {
        char buf[2];
        buf[0] = reg;
        write(i2c_file_, buf, 1);
        read(i2c_file_, buf, 2);
        return (int16_t)((buf[0] << 8) | buf[1]);
    }

    void publish_imu_data() {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // Read raw values
        int16_t ax = read_word(ACCEL_XOUT_H);
        int16_t ay = read_word(ACCEL_XOUT_H + 2);
        int16_t az = read_word(ACCEL_XOUT_H + 4);
        int16_t gx = read_word(ACCEL_XOUT_H + 8);
        int16_t gy = read_word(ACCEL_XOUT_H + 10);
        int16_t gz = read_word(ACCEL_XOUT_H + 12);

        // Convert to m/s² (assuming default sensitivity 16384 LSB/g)
        imu_msg.linear_acceleration.x = ax * 9.81 / 16384.0;
        imu_msg.linear_acceleration.y = ay * 9.81 / 16384.0;
        imu_msg.linear_acceleration.z = az * 9.81 / 16384.0;

        // Convert to rad/s (assuming 131 LSB/(°/s))
        imu_msg.angular_velocity.x = gx * M_PI / (180.0 * 131.0);
        imu_msg.angular_velocity.y = gy * M_PI / (180.0 * 131.0);
        imu_msg.angular_velocity.z = gz * M_PI / (180.0 * 131.0);

        imu_msg.orientation_covariance[0] = -1; // Unknown orientation

        // Add estimated covariance values
        for (int i = 0; i < 9; i++) {
            imu_msg.linear_acceleration_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
        }

        imu_msg.linear_acceleration_covariance[0] = 0.04;  // X
        imu_msg.linear_acceleration_covariance[4] = 0.04;  // Y
        imu_msg.linear_acceleration_covariance[8] = 0.04;  // Z

        imu_msg.angular_velocity_covariance[0] = 0.02;
        imu_msg.angular_velocity_covariance[4] = 0.02;
        imu_msg.angular_velocity_covariance[8] = 0.02;
        
        imu_pub_->publish(imu_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
