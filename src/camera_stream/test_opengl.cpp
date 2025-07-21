#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

int main() {
    cv::namedWindow("OpenGL Window", cv::WINDOW_OPENGL);
    cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC3);
    img.setTo(cv::Scalar(0, 255, 0));  // Green screen
    while (true) {
        cv::imshow("OpenGL Window", img);
        if (cv::waitKey(10) == 27) break;  // ESC to quit
    }
    return 0;
}
