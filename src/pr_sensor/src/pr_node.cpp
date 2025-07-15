#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <mosquitto.h>
#include <unordered_map>
#include <string>
#include <cstring>
#include <memory>

class PRNode : public rclcpp::Node {
public:
    PRNode() : Node("pr_node") {
        mosquitto_lib_init();
        mqtt_ = mosquitto_new(nullptr, true, this);
        if (!mqtt_) {
            throw std::runtime_error("Failed to create Mosquitto client");
        }

        mosquitto_connect_callback_set(mqtt_, on_connect);
        mosquitto_message_callback_set(mqtt_, on_message);

        if (mosquitto_connect(mqtt_, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS) {
            throw std::runtime_error("Failed to connect to MQTT broker");
        }

        // Create ROS publishers
        for (int i = 1; i <= 4; i++) {
            std::string topic = "sensors/photo_" + std::to_string(i);
            std::string ros_topic = "/photo_" + std::to_string(i);
            publishers_[topic] = this->create_publisher<sensor_msgs::msg::Illuminance>(ros_topic, 10);
        }

        // Start MQTT loop in background thread
        mosquitto_loop_start(mqtt_);
    }

    ~PRNode() {
        mosquitto_loop_stop(mqtt_, true);
        mosquitto_destroy(mqtt_);
        mosquitto_lib_cleanup();
    }

    void handle_message(const std::string &topic, const std::string &payload) {
        if (publishers_.count(topic)) {
            try {
                double lux = std::stod(payload);
                auto msg = sensor_msgs::msg::Illuminance();
                msg.header.stamp = this->now();
                msg.header.frame_id = "photoresistor";
                msg.illuminance = lux;
                msg.variance = 0.5;
                publishers_[topic]->publish(msg);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Invalid float payload on topic %s: '%s'", topic.c_str(), payload.c_str());
            }
        }
    }

private:
    static void on_connect(struct mosquitto *mosq, void *obj, int rc) {
        if (rc != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("mosquitto"), "MQTT connect failed with code %d", rc);
            return;
        }
        auto *node = static_cast<PRNode *>(obj);
        for (const auto &pair : node->publishers_) {
            mosquitto_subscribe(mosq, nullptr, pair.first.c_str(), 1);
        }
    }

    static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
        if (!message || !message->payload) return;
        std::string topic = message->topic;
        std::string payload(static_cast<char *>(message->payload), message->payloadlen);
        static_cast<PRNode *>(obj)->handle_message(topic, payload);
    }

    struct mosquitto *mqtt_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr> publishers_;
};
