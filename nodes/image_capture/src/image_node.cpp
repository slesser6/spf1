#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <thread>

#define FRAME_WIDTH 640 // smaller resolution to ease processing
#define FRAME_HEIGHT 360
#define SKIP_FRAMES 29 // 1 frame every second to ease processing

/**
 * @class ImageNode
 * @brief ROS 2 node that captures images from two USB cameras and
 * publishes them as sensor_msgs::msg::Images.
 *
 * This node opens two OpenCV captures, reads a frame every second,
 * and publishes to a topic.
 */
class ImageNode : public rclcpp::Node {
public:
  ImageNode() : Node("image_node") {
    cam_r_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/right/image_raw", 10);
    cam_l_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/left/image_raw", 10);
    cam_r_mono_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/right/image_mono", 10);
    cam_l_mono_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/left/image_mono", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&ImageNode::processFrames, this));

    // open camera connections
    cap_r_.open("/dev/video0", cv::CAP_V4L2);
    cap_r_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap_r_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    cap_l_.open("/dev/video2", cv::CAP_V4L2);
    cap_l_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap_l_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    std::this_thread::sleep_for(std::chrono::milliseconds(33));

    if (!cap_r_.isOpened() || !cap_l_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video streams");
    }

    RCLCPP_INFO(get_logger(), "ImageNode started");
  }

private:
  int skip_count_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_r_pub_, cam_l_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_r_mono_pub_,
      cam_l_mono_pub_;
  cv::VideoCapture cap_r_, cap_l_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Callback for the timer to process a frame
   *
   * Skips frames to read about 1 frame per second, then
   * converts the frame to a message and publishes it.
   */
  void processFrames() {

    if (skip_count_ < SKIP_FRAMES) {
      cv::Mat tmp;
      cap_r_ >> tmp;
      cap_l_ >> tmp;

      skip_count_ = (skip_count_ + 1) % 30;
      return;
    }
    cv::Mat frame_r, frame_l;
    cap_r_ >> frame_r;
    cap_l_ >> frame_l;

    if (frame_r.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty framesfrom right camera");
      return;
    } else {
      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();
      header.frame_id = "camera_right_frame";
      sensor_msgs::msg::Image::SharedPtr msg_r =
          cv_bridge::CvImage(header, "bgr8", frame_r).toImageMsg();
      cam_r_pub_->publish(*msg_r);
      cv::Mat gray_r;
      cv::cvtColor(frame_r, gray_r, cv::COLOR_BGR2GRAY);
      header.frame_id = "camera_right_mono_frame";
      sensor_msgs::msg::Image::SharedPtr msg_m_r =
          cv_bridge::CvImage(header, "mono8", gray_r).toImageMsg();
      cam_r_mono_pub_->publish(*msg_m_r);
    }

    if (frame_l.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty frames from left camera");
      return;
    } else {
      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();
      header.frame_id = "camera_left_frame";
      sensor_msgs::msg::Image::SharedPtr msg_l =
          cv_bridge::CvImage(header, "bgr8", frame_l).toImageMsg();
      cam_l_pub_->publish(*msg_l);
      cv::Mat gray_l;
      cv::cvtColor(frame_l, gray_l, cv::COLOR_BGR2GRAY);
      header.frame_id = "camera_left_mono_frame";
      sensor_msgs::msg::Image::SharedPtr msg_m_l =
          cv_bridge::CvImage(header, "mono8", gray_l).toImageMsg();
      cam_l_mono_pub_->publish(*msg_m_l);
    }
  }
};

/**
 * @brief Main entry point for ImageNode.
 *
 * Initializes ROS 2, spins the ImageNode instance, then shuts down.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageNode>());
  rclcpp::shutdown();
  return 0;
}
