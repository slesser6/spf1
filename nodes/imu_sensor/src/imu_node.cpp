#include "MadgwickAHRS.h"
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std::chrono_literals;

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define G 9.80665f
#define PI_DEG 180.0f
#define GYRO_SENSITIVITY 131.0f    // assuming 131 LSB/(°/s)
#define ACCEL_SENSITIVITY 16384.0f // assuming default sensitivity 16384 LSB/g
#define SAMPLE_FREQ 10.0f          // Hz

/**
 * @class ImuNode
 * @brief ROS 2 node that reads data from MPU6050 IMU sensor via I2C and
 * publishes it as a sensor_msgs::msg::Imu message.
 *
 * This node opens an I2C connection to the MPU6050 sensor, reads accelerometer
 * and gyroscope data every 100ms, converts the raw readings to SI units, and
 * publishes the data on the "/sensors/imu" topic.
 */
class ImuNode : public rclcpp::Node {
public:
  ImuNode() : Node("imu_node") {
    filter_.begin(SAMPLE_FREQ);

    imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", 10);
    timer_ = this->create_wall_timer(100ms,
                                     std::bind(&ImuNode::publishImuData, this));

    // Open the I2C connection
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

    RCLCPP_INFO(get_logger(), "ImuNode started");
  }

  ~ImuNode() {
    // close the I2C connection
    if (i2c_file_ >= 0)
      close(i2c_file_);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int i2c_file_;
  Madgwick filter_;

  /**
   * @brief Reads a 16-bit signed word from the specified MPU6050 register.
   *
   * @param reg The register address to read from.
   * @return int16_t The signed 16-bit value read from the sensor.
   */
  int16_t readWord(int reg) {
    char buf[2];
    buf[0] = reg;
    write(i2c_file_, buf, 1);
    read(i2c_file_, buf, 2);
    return (int16_t)((buf[0] << 8) | buf[1]);
  }

  /**
   * @brief Reads sensor data from MPU6050, converts it, and publishes as a ROS
   * IMU message.
   */
  void publishImuData() {
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";

    // Read raw values
    int16_t ax = readWord(ACCEL_XOUT_H);
    int16_t ay = readWord(ACCEL_XOUT_H + 2);
    int16_t az = readWord(ACCEL_XOUT_H + 4);
    int16_t gx = readWord(ACCEL_XOUT_H + 8);
    int16_t gy = readWord(ACCEL_XOUT_H + 10);
    int16_t gz = readWord(ACCEL_XOUT_H + 12);

    // Convert to m/s²
    imu_msg.linear_acceleration.x = ax * G / ACCEL_SENSITIVITY;
    imu_msg.linear_acceleration.y = ay * G / ACCEL_SENSITIVITY;
    imu_msg.linear_acceleration.z = az * G / ACCEL_SENSITIVITY;

    // Convert to rad/s
    imu_msg.angular_velocity.x = gx * M_PI / (PI_DEG * GYRO_SENSITIVITY);
    imu_msg.angular_velocity.y = gy * M_PI / (PI_DEG * GYRO_SENSITIVITY);
    imu_msg.angular_velocity.z = gz * M_PI / (PI_DEG * GYRO_SENSITIVITY);

    filter_.updateIMU(gx, gy, gz, ax, ay, az);
    float q0 = filter_.q0;
    float q1 = filter_.q1;
    float q2 = filter_.q2;
    float q3 = filter_.q3;

    imu_msg.orientation.x = q0;
    imu_msg.orientation.y = q1;
    imu_msg.orientation.z = q2;
    imu_msg.orientation.w = q3;

    imu_pub_->publish(imu_msg);
  }
};

/**
 * @brief Main function for IMU node.
 *
 * Initializes ROS 2, creates an ImuNode instance, spins it, and then shuts down
 * ROS 2.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit status code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
