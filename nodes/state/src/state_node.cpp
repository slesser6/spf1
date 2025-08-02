#include "fsm.hpp"
#include <rclcpp/rclcpp.hpp>

class StateNode : public rclcpp::Node {
public:
  StateNode() : Node("state_node") {
    state_pub_ =
        this->create_publisher<std_msgs::msg::String>("/state/get", 10);
    state_srv_ = create_service<std_srvs::srv::Trigger>(
        "/state/set", std::bind(&StateNode::onState, this,
                                std::placeholders::_1, std::placeholders::_2));
  }

private:
  FSM fsm_;
  rclcpp::Service<std_srvs::srv::setString>::SharedPtr state_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  void onState(const std::shared_ptr<std_srvs::srv::setString::Request> req,
               std::shared_ptr<std_srvs::srv::setString::Response> res) {
    success = true;
    switch (req->data) {
    case "INIT":
      fsm_.setState(FSM::INIT);
      break;
    case "IDLE":
      fsm_.setState(FSM::IDLE);
      break;
    case "NAV":
      fsm_.setState(FSM::NAC);
      break;
    case "ALIGN":
      fsm_.setState(FSM::ALIGN);
      break;
    case "CTRL":
      fsm_.setState(FSM::CTRL);
      break;
    defualt:
      fsm_.setState(FSM::ERROR);
      success = false;
      break;
    }
    res->success = success;
    res->message = fsm_.toString();

    std_msgs::msg::String msg;
    msg.data = res->message;
    state_pub_->publish(msg);
  }

}

/**
 * @brief Main entry point for StateNode.
 *
 * Initializes ROS 2, spins the StateNode instance, then shuts down.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
int
main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateNode>());
  rclcpp::shutdown();
  return 0;
}