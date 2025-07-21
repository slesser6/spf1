#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>

#include <iostream>

int main() {
    if (cv::ocl::haveOpenCL()) {
        std::cout << "✅ OpenCL is available!" << std::endl;
        cv::ocl::Context context;
        if (context.create(cv::ocl::Device::TYPE_GPU)) {
            std::cout << "✅ OpenCL context created successfully!" << std::endl;
            std::cout << "Device: " << context.device(0).name() << std::endl;
        } else {
            std::cout << "❌ Failed to create OpenCL context." << std::endl;
        }
    } else {
        std::cout << "❌ OpenCL not available." << std::endl;
    }
    return 0;
}
