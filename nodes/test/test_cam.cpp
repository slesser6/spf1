#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    cv::VideoCapture cap1("/dev/video0", cv::CAP_V4L2);

    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait a bit

    if (!cap1.isOpened()) {
        std::cerr << "❌ Failed to open camera 0\n";
        return -1;
    }
    std::cout << "Opened camera 0\n";

    // continue with capture...
}
