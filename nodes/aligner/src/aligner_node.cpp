#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

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
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    for (int i = 1; i <= 4; i++) {
      std::string topic = "/sensors/photo_" + std::to_string(i);
      pr_subs_[i] = this->create_subscription<sensor_msgs::msg::Illuminance>(
          topic, 10,
          std::bind(&AlignerNode::align, this, std::placeholders::_1, i));
    }

    pr_vals_ = new double[4]{0};

    timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),
                                     std::bind(&AlignerNode::align, this));

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
  void align(const sensor_msgs::msg::Imu::SharedPtr msg, int num) {

    received_ = received_ | (1 << num);
    pr_vals_[num - 1] = msg.illuminance

                        if received_ == 30 {
      auto path_msg = nav_msgs::msg::Path();

      path_msg.header.stamp = this->get_clock()->now();
      path_msg.header.frame_id = "map";

      double lr = pr_vals_[FRONT_RIGHT] + pr_vals_[BACK_RIGHT] -
                  pr_vals_[FRONT_LEFT] - pr_vals_[BACK_LEFT];
      double fb = pr_vals_[FRONT_RIGHT] + pr_vals_[FRONT_LEFT] -
                  pr_vals_[BACK_RIGHT] - pr_vals_[BACK_LEFT];

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = lr;
      pose.pose.position.y = fb;
      pose.pose.orientation.w = 1.0;

      path_msg.poses.push_back(pose);

      path_pub_->publish(path_msg);
      received_ = 0;
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscriber<sensor_msgs::msg::Illuminance>::SharedPtr pr_subs_[4];
  double pr_vals_[4];
  uint8_t received_;
  rclcpp::TimerBase::SharedPtr timer_;
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