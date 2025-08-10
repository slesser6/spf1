#include <cstring>
#include <memory>
#include <mosquitto.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <string>
#include <unordered_map>

#define MQTT_HOST "192.168.0.42"

/**
 * @class PRNode
 * @brief ROS 2 node that subscribes to MQTT topics for photoresistor sensors
 * and publishes ROS Illuminance messages.
 *
 * This node connects to an MQTT broker with the mosquitto library, subscribes
 * to photoresistor sensor topics, and republishes the received illuminance
 * values as ROS messages on corresponding topics.
 */
class PRNode : public rclcpp::Node {
public:
  PRNode() : Node("pr_node") {

    // Set up mosquitto for mqtt
    mosquitto_lib_init();
    mqtt_ = mosquitto_new(nullptr, true, this);
    if (!mqtt_) {
      throw std::runtime_error("Failed to create Mosquitto client");
    }

    mosquitto_connect_callback_set(mqtt_, onConnect);
    mosquitto_message_callback_set(mqtt_, onMessage);

    if (mosquitto_connect(mqtt_, MQTT_HOST, 1883, 60) != MOSQ_ERR_SUCCESS) {
      throw std::runtime_error("Failed to connect to MQTT broker");
    }

    // Create ROS publishers
    for (int i = 1; i <= 4; i++) {
      std::string topic = "/sensors/photo_" + std::to_string(i);
      publishers_[topic] =
          this->create_publisher<sensor_msgs::msg::Illuminance>(topic, 10);
    }

    // Start MQTT loop in background thread
    mosquitto_loop_start(mqtt_);

    RCLCPP_INFO(this->get_logger(), "PRNode started");
  }

  ~PRNode() {
    mosquitto_loop_stop(mqtt_, true);
    mosquitto_destroy(mqtt_);
    mosquitto_lib_cleanup();
  }

private:
  struct mosquitto *mqtt_;
  std::unordered_map<
      std::string, rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr>
      publishers_;

  /**
   * @brief Handles incoming MQTT messages by parsing and publishing as ROS
   * Illuminance messages.
   *
   * Converts the MQTT payload to a double representing illuminance in lux,
   * creates a ROS Illuminance message, and publishes it on the corresponding
   * ROS topic. Logs a warning if the payload cannot be converted.
   *
   * @param topic The MQTT topic string of the received message.
   * @param payload The payload string containing the illuminance value.
   */
  void handleMessage(const std::string &topic, const std::string &payload) {
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
        RCLCPP_WARN(this->get_logger(),
                    "Invalid float payload on topic %s: '%s'", topic.c_str(),
                    payload.c_str());
      }
    }
  }

  /**
   * @brief Callback called when MQTT connection is established.
   *
   * Subscribes to all photoresistor topics on the MQTT broker.
   *
   * @param mosq Pointer to the Mosquitto client.
   * @param obj Pointer to this PRNode instance.
   * @param rc Connection result code.
   */
  static void onConnect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("mosquitto"),
                   "MQTT connect failed with code %d", rc);
      return;
    }
    auto *node = static_cast<PRNode *>(obj);
    for (const auto &pair : node->publishers_) {
      mosquitto_subscribe(mosq, nullptr, pair.first.c_str(), 1);
    }
  }

  /**
   * @brief Callback called when an MQTT message is received.
   *
   * Extracts the topic and payload, and passes them to the handleMessage
   * method.
   *
   * @param mosq Pointer to the Mosquitto client.
   * @param obj Pointer to this PRNode instance.
   * @param message Pointer to the received MQTT message structure.
   */
  static void onMessage(struct mosquitto *mosq, void *obj,
                        const struct mosquitto_message *message) {
    if (!message || !message->payload)
      return;
    std::string topic = message->topic;
    std::string payload(static_cast<char *>(message->payload),
                        message->payloadlen);
    static_cast<PRNode *>(obj)->handleMessage(topic, payload);
  }
};

/**
 * @brief Main function for the photoresistor node.
 *
 * Initializes ROS 2, creates the PRNode, spins it, and shuts down ROS 2.
 *
 * @param argc Argument count from command line.
 * @param argv Argument vector from command line.
 * @return int Exit status code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PRNode>());
  rclcpp::shutdown();
  return 0;
}
