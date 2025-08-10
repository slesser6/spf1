#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <iostream>

int main() {
    if (!cv::ocl::haveOpenCL()) {
        std::cout << "❌ OpenCL not available." << std::endl;
        return 1;
    }

    std::vector<cv::ocl::PlatformInfo> platforms;
    cv::ocl::getPlatfomsInfo(platforms);
    std::cout << "Found " << platforms.size() << " OpenCL platform(s)" << std::endl;

    for (size_t i = 0; i < platforms.size(); ++i) {
        std::cout << "Platform " << i << ": " << platforms[i].name() << std::endl;
        for (int j = 0; j < platforms[i].deviceNumber(); ++j) {
            cv::ocl::Device dev;
            platforms[i].getDevice(dev, j);
            std::cout << "  Device " << j << ": " << dev.name() << ", Type: " << dev.type() << std::endl;

            // Try setting this device as default
            cv::ocl::setUseOpenCL(true);

            if (cv::ocl::useOpenCL()) {
                std::cout << "✅ Successfully set and using OpenCL device: " << dev.name() << std::endl;
                return 0;
            }
        }
    }

    std::cout << "❌ Could not set any OpenCL device for usage." << std::endl;
    return 1;
}
