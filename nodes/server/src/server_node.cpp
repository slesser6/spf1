#include "httplib.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>

/**
 * @class ServerNode
 * @brief A ROS 2 node that subscribes to topics and serves their latest values
 * via an HTTP web server.
 *
 * This node listens to selected ROS 2 topics, stores the latest messages,
 * and runs an HTTP server that displays these values in a browser.
 */
class ServerNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new ServerNode node.
   *
   * Sets up ROS 2 subscriptions and starts the HTTP server on a separate
   * thread.
   */
  ServerNode() : Node("server_node") {
    // Subscribe to services
    server_client_ = this->create_client<std_srvs::srv::Trigger>("/server");
    if (!server_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(this->get_logger(),
                  "Service /server not available. Proceeding anyway.");
    }

    // Subscribe to topics
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/state/get", 10, [this](std_msgs::msg::String::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_values_["/state/get"] = msg->data;
        });
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/sensors/imu", 10, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          std::ostringstream ss;
          ss << "linear acceleration: [" << msg->linear_acceleration.x << ", "
             << msg->linear_acceleration.y << ", " << msg->linear_acceleration.z
             << "], " << "angular velocity: [" << msg->angular_velocity.x
             << ", " << msg->angular_velocity.y << ", "
             << msg->angular_velocity.z << "], " << "orientation: ["
             << msg->orientation.x << ", " << msg->orientation.y << ", "
             << msg->orientation.z << ", " << msg->orientation.w << "]";
          latest_values_["/imu/data"] = ss.str();
        });
    pr1_sub_ = this->create_subscription<sensor_msgs::msg::Illuminance>(
        "/sensors/photo_1", 10,
        [this](sensor_msgs::msg::Illuminance::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_values_["/sensors/photo_1"] = std::to_string(msg->illuminance);
        });
    pr2_sub_ = this->create_subscription<sensor_msgs::msg::Illuminance>(
        "/sensors/photo_2", 10,
        [this](sensor_msgs::msg::Illuminance::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_values_["/sensors/photo_2"] = std::to_string(msg->illuminance);
        });
    pr3_sub_ = this->create_subscription<sensor_msgs::msg::Illuminance>(
        "/sensors/photo_3", 10,
        [this](sensor_msgs::msg::Illuminance::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_values_["/sensors/photo_3"] = std::to_string(msg->illuminance);
        });
    pr4_sub_ = this->create_subscription<sensor_msgs::msg::Illuminance>(
        "/sensors/photo_4", 10,
        [this](sensor_msgs::msg::Illuminance::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_values_["/sensors/photo_4"] = std::to_string(msg->illuminance);
        });
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](nav_msgs::msg::Path::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          std::ostringstream ss;

          ss << "Path frame_id: " << msg->header.frame_id << "\n";
          ss << "Number of poses: " << msg->poses.size() << "\n";
          for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto &pose = msg->poses[i].pose;
            ss << "Pose[" << i << "]: " << "Position(x=" << pose.position.x
               << ", y=" << pose.position.y << ", z=" << pose.position.z
               << "), " << "Orientation(x=" << pose.orientation.x
               << ", y=" << pose.orientation.y << ", z=" << pose.orientation.z
               << ", w=" << pose.orientation.w << ")\n";
          }
          latest_values_["/path"] = ss.str();
        });

    // Publisher to topics
    state_pub_ =
        this->create_publisher<std_msgs::msg::String>("/state/set", 10);
    ctrl_pub_ =
        this->create_publisher<std_msgs::msg::String>("/motor/control", 10);

    // Launch the HTTP server in a background thread
    server_thread_ = std::thread([this]() { this->startServer(); });
  }

  /**
   * @brief Destructor. Stops the HTTP server and joins the server thread.
   */
  ~ServerNode() {
    server_.stop();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  std::unordered_map<std::string, std::string> latest_values_;
  std::mutex data_mutex_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr server_client_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> state_pub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> ctrl_pub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> state_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Illuminance>> pr1_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Illuminance>> pr2_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Illuminance>> pr3_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Illuminance>> pr4_sub_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path>> path_sub_;
  httplib::Server server_;
  std::thread server_thread_;

  /**
   * @brief Starts the HTTP server.
   *
   * This method runs in a background thread and serves the latest
   * topic values as an HTML page on port 8000.
   */
  void startServer() {
    server_.Get("/", [this](const httplib::Request &, httplib::Response &res) {
      RCLCPP_INFO(this->get_logger(), "Received HTTP request");
      std::lock_guard<std::mutex> lock(data_mutex_);
      std::string html = "<html><body><h1>ROS 2 Topic Monitor</h1><ul>";
      for (const auto &[topic, value] : latest_values_) {
        html += "<li><b>" + topic + ":</b> " + value + "</li>";
      }
      html += "</ul>";
      html += R"(
                <form action="/init" method="get">
                    <button type="submit">Initialize</button>
                </form>
                <form action="/navigate" method="get">
                    <button type="submit">Navigate</button>
                </form>
                <form action="/align" method="get">
                    <button type="submit">Align</button>
                </form>
                <form action="/control" method="get">
                    <button type="submit">User Control</button>
                </form>
            )";
      html += "</body></html>";
      res.set_content(html, "text/html");
    });
    server_.Get("/init",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String state_msg;
                  state_msg.data = "INIT";
                  state_pub_->publish(state_msg);
                  res.set_redirect("/");
                });
    server_.Get("/navigate",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String state_msg;
                  state_msg.data = "NAV";
                  state_pub_->publish(state_msg);
                  res.set_redirect("/");
                });
    server_.Get("/align",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String state_msg;
                  state_msg.data = "ALIGN";
                  state_pub_->publish(state_msg);
                  res.set_redirect("/");
                });
    server_.Get("/forwards",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String ctrl_msg;
                  ctrl_msg.data = "F";
                  ctrl_pub_->publish(ctrl_msg);
                });
    server_.Get("/backwards",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String ctrl_msg;
                  ctrl_msg.data = "B";
                  ctrl_pub_->publish(ctrl_msg);
                });
    server_.Get("/left",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String ctrl_msg;
                  ctrl_msg.data = "L";
                  ctrl_pub_->publish(ctrl_msg);
                });
    server_.Get("/right",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String ctrl_msg;
                  ctrl_msg.data = "R";
                  ctrl_pub_->publish(ctrl_msg);
                });
    server_.Get("/stop",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String ctrl_msg;
                  ctrl_msg.data = "S";
                  ctrl_pub_->publish(ctrl_msg);
                });
    server_.Get("/quit_control",
                [this](const httplib::Request &, httplib::Response &res) {
                  std_msgs::msg::String ctrl_msg;
                  ctrl_msg.data = "S";
                  ctrl_pub_->publish(ctrl_msg);
                });
    server_.Get("/control",
                [this](const httplib::Request &, httplib::Response &res) {
                  std::string html = "<html><body><h1>User Control</h1><ul>";
                  for (const auto &[topic, value] : latest_values_) {
                    html += "<li><b>" + topic + ":</b> " + value + "</li>";
                  }
                  html += "</ul>";
                  html += R"(
                <form action="/forwards" method="get">
                    <button type="submit">Forward</button>
                </form>
                <form action="/backwards" method="get">
                    <button type="submit">Backward</button>
                </form>
                <form action="/left" method="get">
                    <button type="submit">Left</button>
                </form>
                <form action="/right" method="get">
                    <button type="submit">Right</button>
                </form>
                <form action="/stop" method="get">
                    <button type="submit">Stop</button>
                </form>
                <form action="/quit_control" method="get">
                    <button type="submit">Quit User Control</button>
                </form>
            )";
                  html += "</body></html>";
                  res.set_content(html, "text/html");
                  std_msgs::msg::String state_msg;
                  state_msg.data = "CTRL";
                  state_pub_->publish(state_msg);
                });

    RCLCPP_INFO(this->get_logger(),
                "HTTP server listening on http://0.0.0.0:8080");
    try {
      server_.listen("0.0.0.0", 8080);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Server failed: %s", e.what());
    }
  }
};

/**
 * @brief Main entry point for ServerNode.
 *
 * Initializes ROS 2, spins the ServerNode instance, then shuts down.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}