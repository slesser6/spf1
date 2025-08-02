#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

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

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(5000), // 5s
                                std::bind(&ImageNode::processFrames, this));

    cap_r_.open(0);
    cap_l_.open(2);

    if (!cap_r_.isOpened() || !cap_l_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video streams");
    }
  }

private:
  void processFrames() {
    cv::Mat frame_r, frame_l;
    cap_r_ >> frame_r;
    cap_l_ >> frame_l;

    if (frame_r.empty() || frame_l.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty frames");
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_right_frame";
    sensor_msgs::msg::Image::SharedPtr msg_r =
        cv_bridge::CvImage(header, "bgr8", frame_r).toImageMsg();
    cam_r_pub_->publish(*msg_r);

    header.frame_id = "camera_left_frame";
    sensor_msgs::msg::Image::SharedPtr msg_l =
        cv_bridge::CvImage(header, "bgr8", frame_l).toImageMsg();
    cam_r_pub_->publish(*msg_l);

    cv::Mat gray_r, gray_l;
    cv::cvtColor(frame_r, gray_r, cv::COLOR_BGR2GRAY);
    header.frame_id = "camera_right_mono_frame";
    sensor_msgs::msg::Image::SharedPtr msg_m_r =
        cv_bridge::CvImage(header, "mono8", gray_r).toImageMsg();
    cam_r_mono_pub_.publish(msg_m_r);

    cv::cvtColor(frame_l, gray_l, cv::COLOR_BGR2GRAY);
    header.frame_id = "camera_left_mono_frame";
    sensor_msgs::msg::Image::SharedPtr msg_m_l =
        cv_bridge::CvImage(header, "mono8", gray_l).toImageMsg();
    cam_l_mono_pub_.publish(msg_m_l);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_r_pub_, cam_l_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_r_mono_pub_,
      cam_l_mono_pub_;
  cv::VideoCapture cap_r_, cap_l_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageNode>());
  rclcpp::shutdown();
  return 0;
}
