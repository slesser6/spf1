#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "httplib.h"

#include <mutex>
#include <thread>
#include <unordered_map>
#include <iostream>

/**
 * @class StatusNode
 * @brief A ROS 2 node that subscribes to topics and serves their latest values via an HTTP web server.
 *
 * This node listens to selected ROS 2 topics, stores the latest messages,
 * and runs an HTTP server that displays these values in a browser.
 */
class StatusNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new StatusNode node.
     *
     * Sets up ROS 2 subscriptions and starts the HTTP server on a separate thread.
     */
    StatusNode() : Node("status_node") {
        // Subscribe to services
        status_client_ = this->create_client<std_srvs::srv::Trigger>("/status");
        if (!status_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "Service /status not available. Proceeding anyway.");
        }

        // Subscribe to topics            
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                std::ostringstream ss;
                ss << "linear acceleration: ["
                << msg->linear_acceleration.x << ", "
                << msg->linear_acceleration.y << ", "
                << msg->linear_acceleration.z << "], "
                << "angular velocity: ["
                << msg->angular_velocity.x << ", "
                << msg->angular_velocity.y << ", "
                << msg->angular_velocity.z << "], "
                << "orientation: ["
                << msg->orientation.x << ", "
                << msg->orientation.y << ", "
                << msg->orientation.z << ", "
                << msg->orientation.w << "]";
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
            "/path", 10,
            [this](nav_msgs::msg::Path::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                std::ostringstream ss;

                ss << "Path frame_id: " << msg->header.frame_id << "\n";
                ss << "Number of poses: " << msg->poses.size() << "\n";
                for (size_t i = 0; i < msg->poses.size(); ++i) {
                    const auto &pose = msg->poses[i].pose;
                    ss << "Pose[" << i << "]: "
                    << "Position(x=" << pose.position.x
                    << ", y=" << pose.position.y
                    << ", z=" << pose.position.z << "), "
                    << "Orientation(x=" << pose.orientation.x
                    << ", y=" << pose.orientation.y
                    << ", z=" << pose.orientation.z
                    << ", w=" << pose.orientation.w << ")\n";
                }
                latest_values_["/path"] = ss.str();
            });

        // Launch the HTTP server in a background thread
        server_thread_ = std::thread([this]() { this->start_http_server(); });
    }

    /**
     * @brief Destructor. Stops the HTTP server and joins the server thread.
     */
    ~StatusNode() {
        server_.stop();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

private:
    std::unordered_map<std::string, std::string> latest_values_;
    std::mutex data_mutex_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr status_client_;
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
    void start_http_server() {
        server_.Get("/", [this](const httplib::Request&, httplib::Response& res) {
            RCLCPP_INFO(this->get_logger(), "Received HTTP request");
            std::lock_guard<std::mutex> lock(data_mutex_);
            std::string html = "<html><body><h1>ROS 2 Topic Monitor</h1><ul>";
            for (const auto& [topic, value] : latest_values_) {
                html += "<li><b>" + topic + ":</b> " + value + "</li>";
            }
            html += "</ul></body></html>";
            res.set_content(html, "text/html");
        });

        RCLCPP_INFO(this->get_logger(), "HTTP server listening on http://0.0.0.0:8080");
        try {
            server_.listen("0.0.0.0", 8080);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Server failed: %s", e.what());
        }
    }
};

/**
 * @brief Main entry point for StatusNode.
 *
 * Initializes ROS 2, spins the StatusNode instance, then shuts down.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusNode>());
    rclcpp::shutdown();
    return 0;
}