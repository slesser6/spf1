#include <rclcpp/rclcpp.hpp>
#include "httplib.h"

/**
 * @class StatusNode
 * @brief A ROS 2 node that runs an HTTP server to provide status information.
 *
 * This node starts an HTTP server on port 8080 that serves a JSON status response
 * on the /status endpoint. The server runs in a separate thread to allow
 * concurrent ROS spinning.
 */
class StatusNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for StatusNode.
     *
     * Initializes the node with the name "status_node", sets up an HTTP GET handler
     * for the /status endpoint, and starts the HTTP server in a background thread.
     */
    StatusNode() : Node("status_node") {
        // Start server on port 8080
        server_.Get("/status", [this](const httplib::Request&, httplib::Response& res) {
            std::string status_json = R"({"status":"ok","uptime_sec":12345})";
            res.set_content(status_json, "application/json");
        });

        // Run server in a separate thread so it doesn't block ROS spinning
        server_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Starting HTTP server on port 8080");
            server_.listen("0.0.0.0", 8080);
        });
    }

    /**
     * @brief Destructor for StatusNode.
     *
     * Stops the HTTP server and joins the server thread if it is joinable.
     */
    ~StatusNode() {
        server_.stop();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

private:
    httplib::Server server_;
    std::thread server_thread_;
};

/**
 * @brief Main entry point for the program.
 *
 * Initializes the ROS 2 system, creates the StatusNode, spins it, then shuts down ROS 2.
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line argument strings
 * @return int Exit status code
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
