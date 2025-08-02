#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class StereoNode : public rclcpp::Node {
public:
  StereoNode()
      : Node("stereo_node"), cam_r_received_(false), cam_l_received_(false) {
    cam_r_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/right/image_raw", 10,
        std::bind(&StereoNode::onCamR, this, std::placeholders::_1));
    cam_l_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/left/image_raw", 10,
        std::bind(&StereoNode::onCamL, this, std::placeholders::_1));

    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/depth", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                     std::bind(&StereoNode::controlLoop, this));
  }

private:
  void onCamR(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!msg->data.empty()) {
      current_r_ = msg;
      cam_r_received_ = true;
    }
  }

  void onCamL(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!msg->data.empty()) {
      current_l_ = msg;
      cam_l_received_ = true;
    }
  }

  void controlLoop() {
    if (!cam_l_received_ || !cam_r_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for inputs...");
      return;
    }

    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
    cv::Mat disparity;

    cv::Mat img_l = cv_bridge::toCvCopy(current_l_, "mono8")->image;
    cv::Mat img_r = cv_bridge::toCvCopy(current_r_, "mono8")->image;
    stereo->compute(img_l, img_r, disparity);

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_depth_frame";
    sensor_msgs::msg::Image::SharedPtr img_msg =
        cv_bridge::CvImage(header, "16SC1", disparity).toImageMsg();
    depth_pub_->publish(*img_msg);

    cam_l_received_ = false;
    cam_r_received_ = false;
  }
  bool cam_r_received_, cam_l_received_;
  sensor_msgs::msg::Image::SharedPtr current_r_, current_l_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_r_sub_,
      cam_l_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoNode>());
  rclcpp::shutdown();
  return 0;
}
