#include "fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class StateNode
 * @brief A ROS 2 node that holds the state in an FSM.
 *
 * This node listens to a ROS 2 topic for a change state message,
 * updates its FSM, then publishes the current state.
 */
class StateNode : public rclcpp::Node {
public:
  StateNode() : rclcpp::Node("state_node") {
    state_pub_ =
        this->create_publisher<std_msgs::msg::String>("/state/get", 10);
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/state/set", 10,
        std::bind(&StateNode::onState, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "StateNode started");
  }

private:
  FSM fsm_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  /**
   * @brief Callback for when a state change message is received
   *
   * Updates the FSM.
   * @param msg the new state as a String message
   */
  void onState(const std_msgs::msg::String msg) {
    if (msg.data == "INIT") {
      fsm_.setState(FSM::State::INIT);
    } else if (msg.data == "IDLE") {
      fsm_.setState(FSM::State::IDLE);
    } else if (msg.data == "NAV") {
      fsm_.setState(FSM::State::NAV);
    } else if (msg.data == "ALIGN") {
      fsm_.setState(FSM::State::ALIGN);
    } else if (msg.data == "CTRL") {
      fsm_.setState(FSM::State::CTRL);
    } else {
      fsm_.setState(FSM::State::ERROR);
    }
    std_msgs::msg::String state_msg;
    state_msg.data = fsm_.toString();
    state_pub_->publish(state_msg);
  }
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
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateNode>());
  rclcpp::shutdown();
  return 0;
}