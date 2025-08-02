#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#define WINDOW_SIZE 50

/**
 * @class SunFinderNode
 * @brief Finds the brightest area in an image and uses a depth image to compute
 * 3D pose.
 */
class SunFinderNode : public rclcpp::Node {
public:
  SunFinderNode() : Node("sun_finder_node") {
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/stereo/depth", 10,
        std::bind(&SunFinderNode::depth_callback, this, std::placeholders::_1));

    brightness_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/left/image_mono", 10,
        std::bind(&SunFinderNode::brightness_callback, this,
                  std::placeholders::_1));

    pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/sun_pos", 10);

    // TODO: camera params
    fx_ = 525.0; // Focal length x
    fy_ = 525.0; // Focal length y
    cx_ = 319.5; // Principal point x
    cy_ = 239.5; // Principal point y

    RCLCPP_DEBUG(this->get_logger(), "SunFinder initialized.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr brightness_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  cv::Mat last_depth_;
  cv::Mat last_brightness_;
  std::mutex data_mutex_;

  double fx_, fy_, cx_, cy_;

  /**
   * @brief Callback called when a depth message is received from the stereo
   * vision node.
   *
   * Extracts the frame from the topic and processes it.
   *
   * @param msg Pointer to the received sensor_msgs::msg::Image structure.
   */
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    try {
      last_depth_ = cv_bridge::toCvCopy(msg, "32FC1")->image;
      RCLCPP_DEBUG(this->get_logger(), "Received depth image.");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Depth image conversion error: %s",
                   e.what());
    }
    process();
  }

  /**
   * @brief Callback called when a brightness message is received from the image
   * capture node.
   *
   * Extracts the frame from the topic and processes it.
   *
   * @param msg Pointer to the received sensor_msgs::msg::Image structure.
   */
  void brightness_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    try {
      auto cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      last_brightness_ = cv_ptr->image;
      RCLCPP_DEBUG(this->get_logger(), "Received brightness image.");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Brightness image conversion error: %s",
                   e.what());
    }
    process();
  }

  /**
   * @brief Processes the depth and brightness to find the sunniest area.
   *
   * Locates the brightest point using the brightness image, then uses
   * the depth image to get the 3D pose.
   *
   */
  void process() {
    if (last_depth_.empty() || last_brightness_.empty())
      return;

    RCLCPP_DEBUG(this->get_logger(),
                 "Processing depth and brightness to find sunniest area.");

    cv::Rect sunniest_area = findSunniestArea(last_brightness_);

    cv::Point sun_pos(sunniest_area.x + (WINDOW_SIZE / 2),
                      sunniest_area.y + (WINDOW_SIZE / 2));

    float depth = last_depth_.at<float>(sun_pos);

    if (!std::isfinite(depth) || depth <= 0.0f) {
      RCLCPP_WARN(this->get_logger(), "Invalid depth at bright point (%d, %d)",
                  sun_pos.x, sun_pos.y);
      return;
    }

    float x = (sun_pos.x - cx_) * depth / fx_;
    float y = (sun_pos.y - cy_) * depth / fy_;
    float z = depth;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "sun_pos";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1.0;

    pose_pub_->publish(pose);

    RCLCPP_DEBUG(this->get_logger(),
                 "Published sunniest position at (%.2f, %.2f, %.2f)", x, y, z);

    last_depth_.release();
    last_brightness_.release();
  }

  /**
   * @brief Use a sliding window to find the brightest region
   *
   * Use integral image to find the average brightness of each window
   * and return the max
   *
   * @param image a cv::Mat grayscale image
   * @return cv::Rect the rectangle with the highest average brightness
   *
   */
  cv::Rect findSunniestArea(const cv::Mat &image) {
    CV_Assert(!image.empty());

    // Compute integral image for fast summation
    cv::Mat integralImg;
    cv::integral(image, integralImg, CV_64F);

    double maxSum = -1;
    cv::Point maxLoc;

    for (int y = 0; y <= image.rows - WINDOW_SIZE; ++y) {
      for (int x = 0; x <= image.cols - WINDOW_SIZE; ++x) {
        int x1 = x;
        int y1 = y;
        int x2 = x + WINDOW_SIZE;
        int y2 = y + WINDOW_SIZE;

        // Sum of pixels in the window using integral image
        double sum =
            integralImg.at<double>(y2, x2) - integralImg.at<double>(y1, x2) -
            integralImg.at<double>(y2, x1) + integralImg.at<double>(y1, x1);

        if (sum > maxSum) {
          maxSum = sum;
          maxLoc = cv::Point(x, y);
        }
      }
    }

    return cv::Rect(maxLoc.x, maxLoc.y, WINDOW_SIZE, WINDOW_SIZE);
  }
};

/**
 * @brief Main function for SunFinder node.
 *
 * Initializes ROS 2, creates an SunFinderNode instance, spins it, and then
 * shuts down ROS 2.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit status code.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SunFinderNode>());
  rclcpp::shutdown();
  return 0;
}
