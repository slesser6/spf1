#include "std_msgs/msg/string.hpp"
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>

#define FRONT_RIGHT 0
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3

/**
 * @class AlignerNode
 * @brief ROS 2 node that subscribes to photoresistor topics for for ROS
 * Illuminance messages to align to the sunniest point.
 *
 * This node subscribes to an Illuminance topic and publishes a Path topic.
 */
class AlignerNode : public rclcpp::Node {
public:
  AlignerNode() : Node("aligner_node") {
    ctrl_pub_ =
        this->create_publisher<std_msgs::msg::String>("/motor/control", 10);

    for (int i = 1; i <= 4; i++) {
      std::string topic = "/sensors/photo_" + std::to_string(i);
      pr_subs_[i - 1] =
          this->create_subscription<sensor_msgs::msg::Illuminance>(
              topic, 10,
              [this, i](const sensor_msgs::msg::Illuminance::SharedPtr msg) {
                this->align(msg, i - 1);
              });
    }

    pr_vals_ = {0, 0, 0, 0};
    received_ = 0;
  }

private:
  /**
   * @brief Create a path to align with the sunniest area
   *
   * Receives a photoresistor message and when a message from all 4 is received,
   * then creates a path.
   *
   * @param msg the Illuminance message.
   * @param num Which photoresistor.
   */
  void align(const sensor_msgs::msg::Illuminance::SharedPtr msg, int num) {

    received_ = received_ | (1 << num);
    pr_vals_[num] = msg->illuminance;

    if (received_ == 0x0F) {

      double lr = pr_vals_[FRONT_RIGHT] + pr_vals_[BACK_RIGHT] -
                  pr_vals_[FRONT_LEFT] - pr_vals_[BACK_LEFT];
      double fb = pr_vals_[FRONT_RIGHT] + pr_vals_[FRONT_LEFT] -
                  pr_vals_[BACK_RIGHT] - pr_vals_[BACK_LEFT];

      std_msgs::msg::String ctrl_msg;

      if (lr > fb) {
        if (lr < 0) {
          ctrl_msg.data = "L";
        } else {
          ctrl_msg.data = "R";
        }
      } else {
        if (fb < 0) {
          ctrl_msg.data = "B";
        } else {
          ctrl_msg.data = "F";
        }
      }
      ctrl_pub_->publish(ctrl_msg);
      received_ = 0;
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ctrl_pub_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::Illuminance>::SharedPtr, 4>
      pr_subs_;
  std::array<double, 4> pr_vals_;
  uint8_t received_;
};

/**
 * @brief Main entry point for StateNode.
 *
 * Initializes ROS 2, spins the StateNode instance, then shuts down.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlignerNode>());
  rclcpp::shutdown();
  return 0;
}