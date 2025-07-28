#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

class StereoNode : public rclcpp::Node
{
public:
    StereoNode()
        : Node("stereo_node"),
        cam_l_received_(false), cam_r_received_(false)
    {
        cam_r_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/right/image_raw", 10,
            std::bind(&StereoNode::on_cam_r, this, std::placeholders::_1));
        cam_l_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/left/image_raw", 10,
            std::bind(&StereoNode::on_cam_l, this, std::placeholders::_2));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&StereoNode::control_loop, this));


    }

private:
    void on_cam_r(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!msg->data.data().empty())
        {
            current_r_ = msg
            cam_r_received_ = true;
        }
    }

    void on_cam_l(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!msg->data.data().empty())
        {
            current_l_ = msg
            cam_l_received_ = true;
        }
    }
    
    void control_loop()
    {
        if (!cap_l_received_ || !cap_r_received_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for inputs...");
            return;
        }

        int width_r = current_r_->width;
        int height_r = current_r_->height;
        std::string encoding_r = current_r_->encoding;
        const uint8_t* data_r = current_r_->data.data();

        int width_r = current_l_->width;
        int height_r = current_l_->height;
        std::string encoding_l = current_l_->encoding;
        const uint8_t* data_l = current_l_->data.data();

        cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
        cv::Mat disparity;
        stereo->compute(data_l, data_r, disparity);

        cap_l_received_ = false;
        cap_r_received_ = false;
    }
    bool cam_r_received_, cam_l_received_;
    sensor_msgs::msg::Image::SharedPtr current_r_, current_l_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_r_sub, cam_l_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoNode>());
    rclcpp::shutdown();
    return 0;
}
