#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: ./test_gstreamer <video_file>" << std::endl;
        return -1;
    }

    cv::VideoCapture cap(argv[1], cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "❌ Failed to open video with GStreamer." << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        cv::imshow("GStreamer Video", frame);
        if (cv::waitKey(10) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
