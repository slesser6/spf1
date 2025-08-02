#include <opencv2/opencv.hpp>

int main() {
    cv::VideoCapture cap1("/dev/video0");  // /dev/video0
    cv::VideoCapture cap2("/dev/video2");  // /dev/video2

    if (!cap1.isOpened()) {
        printf("❌ Failed to open camera 0\n");
        return -1;
    }
    if (!cap2.isOpened()) {
        printf("❌ Failed to open camera 1\n");
        return -1;
    }

    cv::Mat frame1, frame2;
    while (true) {
        cap1 >> frame1;
        cap2 >> frame2;

        if (frame1.empty() || frame2.empty()) {
            printf("⚠️ One of the frames is empty\n");
            break;
        }

        cv::imshow("Camera 0", frame1);
        cv::imshow("Camera 1", frame2);

        // Exit on Esc key
        if (cv::waitKey(10) == 27)
            break;
    }

    cap1.release();
    cap2.release();
    cv::destroyAllWindows();
    return 0;
}
