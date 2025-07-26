#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>


class PathNode : public rclcpp::Node
{
public:
    PathNode()
        : Node("path_planner")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathNode::process_frames, this));

        cap1_.open(0);
        cap2_.open(2);

        if (!cap1_.isOpened() || !cap2_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video streams");
        }
    }

private:
    void process_frames()
    {
        cv::Mat frame1, frame2;
        cap1_ >> frame1;
        cap2_ >> frame2;

        if (frame1.empty() || frame2.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frames");
            return;
        }

        // Placeholder-----------------------------------------------------------

        auto path_msg = nav_msgs::msg::Path();

        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = i * 1.0;
            pose.pose.position.y = i * 0.5;
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);

        // ------------------------------------------------------------------------
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap1_, cap2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathNode>());
    rclcpp::shutdown();
    return 0;
}
