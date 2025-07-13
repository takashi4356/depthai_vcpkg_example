//g++ -o camera_capture camera_capture.cpp $(pkg-config --cflags --libs opencv4)
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // 📌 カメラ設定（640x240, MJPG, 210 FPS）
    system("v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=240,pixelformat=MJPG");
    system("v4l2-ctl -d /dev/video0 --set-parm=210");

    // 📌 カメラを開く
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    // 📌 設定の適用
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 210);

    cv::Mat frame, left_img, right_img, gray_left, gray_right, disparity;

    // 📌 SGBM の設定
    int numDisparities = 64; // 視差範囲（16の倍数）
    int blockSize = 5; // マッチングブロックサイズ（3より大きいほうが速いことが多い）
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, numDisparities, blockSize);

    // FPS 計測用
    int frame_count = 0;
    double start_time = (double)cv::getTickCount();
    double fps = 0.0;
    double elapsed_ms = 0.0;  // 経過時間（ミリ秒）

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame" << std::endl;
            break;
        }

        // 📌 左右画像を分割（640x240 -> 320x240 ×2）
        left_img = frame(cv::Rect(0, 0, 320, 240));   // 左カメラ
        right_img = frame(cv::Rect(320, 0, 320, 240)); // 右カメラ

        // 📌 SGBM で視差画像を計算
        //sgbm->compute(left_img, right_img, disparity);

        // 📌 FPS 計測（1秒ごとに更新）
        frame_count++;
        double current_time = (double)cv::getTickCount();
        elapsed_ms = (current_time - start_time) * 1000.0 / cv::getTickFrequency(); // ミリ秒単位の経過時間

        if (elapsed_ms >= 1000.0) { // 1秒（1000ms）ごとにFPS更新
            fps = frame_count / (elapsed_ms / 1000.0);
            frame_count = 0;
            start_time = (double)cv::getTickCount();
        }

        // 📌 FPS & ミリ秒単位の計測時間を画面に表示
        std::string fps_text = "FPS: " + std::to_string((int)fps) + "  Time: " + std::to_string((int)elapsed_ms) + " ms";
        cv::putText(left_img, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        // 📌 画像表示
        cv::imshow("Left Camera", left_img);
        cv::imshow("Right Camera", right_img);
        //cv::imshow("Disparity Map", disparity);

        // 'q' で終了
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
