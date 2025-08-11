#include <opencv2/opencv.hpp>
#include <iostream>

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 360
#define SKIP_FRAMES 29  // set >0 if you want to skip frames

int numDisparities = 16; // must be divisible by 16
int blockSize = 15;      // must be odd and >=5

void onTrackbar(int, void*) {
    // just here so OpenCV trackbar has a callback
}

int main() {
    cv::VideoCapture cap_r(0, cv::CAP_V4L2);
    cv::VideoCapture cap_l(2, cv::CAP_V4L2);

    cap_r.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap_r.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap_l.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap_l.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    if (!cap_r.isOpened() || !cap_l.isOpened()) {
        std::cerr << "Could not open video streams\n";
        return -1;
    }

    cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
    cv::createTrackbar("NumDisparities", "Disparity", &numDisparities, 256, onTrackbar);
    cv::createTrackbar("BlockSize", "Disparity", &blockSize, 51, onTrackbar);

    int skip_count = 0;

    while (true) {
        if (skip_count < SKIP_FRAMES) {
            cv::Mat tmp;
            cap_r >> tmp;
            cap_l >> tmp;
            skip_count = (skip_count + 1) % (SKIP_FRAMES + 1);
            continue;
        }

        cv::Mat frame_r, frame_l, gray_r, gray_l, disparity;
        cap_r >> frame_r;
        cap_l >> frame_l;

        if (frame_r.empty() || frame_l.empty()) break;

        cv::cvtColor(frame_r, gray_r, cv::COLOR_BGR2GRAY);
        cv::cvtColor(frame_l, gray_l, cv::COLOR_BGR2GRAY);

        // Ensure valid parameters
        int numDisp = ((numDisparities / 16) * 16);
        if (numDisp < 16) numDisp = 16;
        int bSize = blockSize;
        if (bSize % 2 == 0) bSize++;  // must be odd
        if (bSize < 5) bSize = 5;

        cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(numDisp, bSize);
        stereo->compute(gray_l, gray_r, disparity);

        cv::Mat disp8;
        disparity.convertTo(disp8, CV_8U, 255 / (numDisp * 16.));

        cv::imshow("Disparity", disp8);

        char key = (char)cv::waitKey(1);
        if (key == 27) break; // ESC
    }

    return 0;
}
