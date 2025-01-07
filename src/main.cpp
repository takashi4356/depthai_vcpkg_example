#include <iostream>

#include "depthai/depthai.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto* output = camRgb->requestOutput({640, 480});
    if(output == nullptr) {
        std::cout << "Error creating output, exiting\n";
        return 1;
    }
    auto outputQueue = output->createOutputQueue();
    pipeline.start();

    while(pipeline.isRunning()) {
        auto imgFrame = outputQueue->get<dai::ImgFrame>();
        auto cvFrame = imgFrame->getCvFrame();
        cv::imshow("rgb", cvFrame);
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            pipeline.stop();
            return 0;
        }
    }
    return 0;
}
