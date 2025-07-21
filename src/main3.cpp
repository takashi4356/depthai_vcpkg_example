#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    // デバイスリストを取得
    auto availableDevices = dai::Device::getAllAvailableDevices();
    // ***************************************************
    // **************** デバイスを選択
    // ***************************************************
    dai::DeviceInfo deviceInfo1 = availableDevices[0];
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(deviceInfo1);

    // ***************************************************
    // *************** パイプラインを構築 ***************
    // ***************************************************
    dai::Pipeline pipeline(device);
    dai::PipelineImpl(pipeline, false);

    // ***************************************************
    // *************** ノードを構築 ***************
    // ***************************************************
    //auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::make_pair(1280, 720), 143);
    //auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::make_pair(1280, 720), 143);
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::make_pair(1280, 720), 143);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::make_pair(1280, 720), 143);
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto manip_l = pipeline.create<dai::node::ImageManip>();
    auto manip_r = pipeline.create<dai::node::ImageManip>();
    auto manip_hanten = pipeline.create<dai::node::ImageManip>();

    // ***************************************************
    // *************** プロパティを設定 ***************
    // ***************************************************

    auto depthPresetMode = dai::node::StereoDepth::PresetMode::FAST_ACCURACY; //(V3)
    stereo->setDefaultProfilePreset(depthPresetMode);                       // HIGH_ACCURACY/HIGH_DENSITY/DEFAULT/FACE/HIGH_DETAIL/ROBOTICS);
    /* V2->V3
    depth->initialConfig.setConfidenceThreshold(c_setConfidenceThreshold); // 視差計算の信頼度しきい値(小さくすると検知が厳しくなる)
    */
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)

    dai::StereoDepthConfig::MedianFilter medianFilter = dai::StereoDepthConfig::MedianFilter::KERNEL_7x7; //(V3)
    stereo->initialConfig->setMedianFilter(medianFilter); //(V3)
    stereo->setLeftRightCheck(false);              // LR方向とRL方向の両方の視差を計算して結合し、それらを結合します。オクルージョン処理を改善するために、無効な視差値を破棄します
    stereo->setExtendedDisparity(false); // フル解像度とダウンスケールされた画像を組み合わせて、視差範囲が0-95から0-190に増加しました。短距離のオブジェクトに適しています。現在、サブピクセルの視差と互換性がありません
    stereo->setSubpixel(true);                    // サブピクセル補間（5小数ビット）で視差を計算します。長距離射撃に適しています。現在、拡張された視差と互換性がありません
    // サブピクセル精度の小数点以下のビット数を設定
    // 例: 3ビットに設定 (0.125ピクセル単位の精度)
    stereo->setSubpixelFractionalBits(3);  // ★ここが設定する場所です
    stereo->initialConfig->setLeftRightCheckThreshold(3000);

    manip_hanten->initialConfig->addFlipVertical();    // 反転 V2->V3
    manip_hanten->initialConfig->addFlipHorizontal();  // V2->V3
    manip_hanten->initialConfig->setOutputSize(
        256, // 幅の計算
        400,                                        // 高さの計算
        dai::ImageManipConfig::ResizeMode::STRETCH                   // アスペクト比を維持せず引き伸ばす
    );                                                               // V3向け修正

    //manip_l->initialConfig->addCrop(dai::Rect(0,0,256,400), true); // V2->V3
    //manip_r->initialConfig->addCrop(dai::Rect(0,0,256,400), true); // V2->V3
    manip_l->initialConfig->setOutputSize(256, 400, dai::ImageManipConfig::ResizeMode::STRETCH); // アスペクト比の固定(V2->V3)
    manip_r->initialConfig->setOutputSize(256, 400, dai::ImageManipConfig::ResizeMode::STRETCH); // アスペクト比の固定(V2->V3)

    // ***************************************************
    // *************** リンクを設定 ***************
    // ***************************************************
    monoLeft->requestOutput({ 256, 400 }, dai::ImgFrame::Type::NV12,
        dai::ImgResizeMode::CROP, 200)->link(manip_l->inputImage); //V3
    monoRight->requestOutput({ 256, 400 }, dai::ImgFrame::Type::NV12,
        dai::ImgResizeMode::CROP, 200)->link(manip_r->inputImage); //V3
    manip_l->out.link(stereo->left);
    manip_r->out.link(stereo->right);

    stereo->disparity.link(manip_hanten->inputImage);

    // ***************************************************
    // *************** プロパティを設定 ***************
    // ***************************************************
    stereo->setRectification(true);
    stereo->setExtendedDisparity(true);
    stereo->setLeftRightCheck(true);

    /**********************************************************************************/
    // キューの設定
    /**********************************************************************************/
    auto syncedLeftQueue = stereo->syncedLeft.createOutputQueue();
    auto syncedRightQueue = stereo->syncedRight.createOutputQueue();
    auto disparityQueue = stereo->disparity.createOutputQueue();

    auto q_f = manip_hanten->out.createOutputQueue(); // NEW (v3)

    double maxDisparity = 1.0;

    /**********************************************************************************/
    // パイプラインの開始
    /**********************************************************************************/
    pipeline.start();


    while (true) {
        auto leftSynced = syncedLeftQueue->get<dai::ImgFrame>();
        auto rightSynced = syncedRightQueue->get<dai::ImgFrame>();
        auto disparity = q_f->get<dai::ImgFrame>();

        cv::imshow("left", leftSynced->getCvFrame());
        cv::imshow("right", rightSynced->getCvFrame());

        cv::Mat npDisparity = disparity->getFrame();

        double minVal, curMax;
        cv::minMaxLoc(npDisparity, &minVal, &curMax);
        maxDisparity = std::max(maxDisparity, curMax);

        // Normalize the disparity image to an 8-bit scale.
        cv::Mat normalized;
        npDisparity.convertTo(normalized, CV_8UC1, 255.0 / maxDisparity);

        cv::Mat colorizedDisparity;
        cv::applyColorMap(normalized, colorizedDisparity, cv::COLORMAP_JET);

        // Set pixels with zero disparity to black.
        colorizedDisparity.setTo(cv::Scalar(0, 0, 0), normalized == 0);

        cv::imshow("disparity", colorizedDisparity);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    pipeline.stop();
    return 0;
}