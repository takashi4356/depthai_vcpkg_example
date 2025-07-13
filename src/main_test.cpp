#include <cstdio>
#include "C:\Users\Takashi\OneDrive\depthaiC++\depthai-core-example-main\depthai-core\examples\src\utility.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Closer-in minimum depth, disparity range is doubled (from 95 to 190):
static std::atomic<bool> extended_disparity{false};
// Better accuracy for longer distance, fractional disparity 32-levels:
static std::atomic<bool> subpixel{false};
// Better handling for occlusions:
static std::atomic<bool> lr_check{false};

int main()
{
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    xout->setStreamName("disparity");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth->initialConfig.setConfidenceThreshold(200);
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depth->setLeftRightCheck(lr_check);
    depth->setExtendedDisparity(extended_disparity);
    depth->setSubpixel(subpixel);

    // Linking
    monoLeft->out.link(depth->left);
    monoRight->out.link(depth->right);
    depth->disparity.link(xout->input);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setPreviewSize(640, 400);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

    // Linking
    camRgb->preview.link(xoutRgb->input);
    //IMU initialize
    using namespace std;
    using namespace std::chrono;
    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    auto xlinkOutq = pipeline.create<dai::node::XLinkOut>();
    xlinkOutq->setStreamName("imu");
    // enable ACCELEROMETER_RAW and GYROSCOPE_RAW at 500 hz rate
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 500);
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(10);
    // Link plugins IMU -> XLINK
    imu->out.link(xlinkOutq->input);
    // Pipeline is defined, now we can connect to the device
    //dai::Device d(pipeline);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the disparity frames from the outputs defined above
    auto q = device.getOutputQueue("disparity", 4, false);
    // Output queue will be used to get the rgb frames from the output defined above
    auto qRgb = device.getOutputQueue("rgb", 4, false);

    bool firstTs = false;
    auto imuQueue = device.getOutputQueue("imu", 50, false);
    auto baseTs = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>();
    bool pr_flg = false;
    bool zone_flg = false; //ゾーン検出住みか？
    int64 old_tick=0;
    int frame_cnt=0;
    //輪郭の数
    int roiCnt = 0;

    Point front_r;
    Point front_l;
    Point back_r;
    Point back_l;
    Point base;
    int base_h;//ベースの高さ
    int base_w;//ベースのhaba
    int zone_h;
    int zone_l;
    int pr_time;//
    float radius;
    //録画
    cv::VideoWriter writer1;
    cv::VideoWriter writer2;
    cv::VideoWriter writer3;
    int fourcc1 = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');	// Xvid
    int fourcc2 = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');	// Xvid
    int fourcc3 = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');	// Xvid
    writer1.open("ball_MASK_HSV.avi", fourcc1, 30.0, cv::Size(640, 400));
    writer2.open("ball_MASK_BGR.avi", fourcc2, 30.0, cv::Size(640, 400));
    writer3.open("ball_MASK_BIN.avi", fourcc3, 30.0, cv::Size(640, 400));
    // 動画ファイルを書き出すためのオブジェクトを宣言する

    while (true)
    {
        auto inRgb = qRgb->get<dai::ImgFrame>();
        // Retrieve 'bgr' (opencv format) frame
        //cv::imshow("rgb", inRgb->getCvFrame());

        auto inDepth = q->get<dai::ImgFrame>();
        auto frame = inDepth->getFrame();
        // Normalization for better visualization
        frame.convertTo(frame, CV_8UC1, 255 / depth->getMaxDisparity());

        //cv::imshow("disparity", frame);

        // Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        cv::applyColorMap(frame, frame, cv::COLORMAP_JET);
        //cv::imshow("disparity_color", frame);

        // inRangeを用いてフィルタリング
        Mat mask_image, frame_hsv;
        cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
        Scalar s_min = Scalar(110, 0, 0);     // 近景
        Scalar s_max = Scalar(119, 255, 255); //遠景
        inRange(frame_hsv, s_min, s_max, mask_image);

        //リサイズ
        Mat mask_image_rsize;
        resize(mask_image, mask_image, cv::Size(), 1.2, 1.2);
        //mask_image(mask_image, cv::rect(mask_image.shape[0] - frame.shape[0]));
        mask_image_rsize = cv::Mat(mask_image, cv::Rect((mask_image.cols - frame.cols) / 2, (mask_image.rows - frame.rows) / 2, frame.cols, frame.rows));
        mask_image = mask_image_rsize;

        // マスク画像を表示
        //namedWindow("mask");
        //imshow("mask", mask_image);

        //加速度から傾きを算出
        auto imuData = imuQueue->get<dai::IMUData>();
        auto imuPackets = imuData->packets;
        float tmp_r, tmp_x, tmp_y, tmp_x_old, tmp_y_old; //画像傾き
        tmp_r = 0;
        tmp_x_old = 0;
        tmp_y_old = 0;
        tmp_x = 0;
        tmp_y = 0;
 
        for (auto& imuPacket : imuPackets)
        {
            auto& acceleroValues = imuPacket.acceleroMeter;
            auto& gyroValues = imuPacket.gyroscope;

            auto acceleroTs1 = acceleroValues.timestamp.get();
            auto gyroTs1 = gyroValues.timestamp.get();
            if (!firstTs)
            {
                baseTs = std::min(acceleroTs1, gyroTs1);
                firstTs = true;
            }

            auto acceleroTs = acceleroTs1 - baseTs;
            auto gyroTs = gyroTs1 - baseTs;

            tmp_x = acceleroValues.x * 0.1 + tmp_x_old * 0.9; //ハイパスフィルタ
            tmp_y = acceleroValues.y * 0.1 + tmp_y_old * 0.9;
            tmp_r = atan2(tmp_x, tmp_y) * 180 / 3.14; //画像傾き記憶
            tmp_x_old = tmp_x;
            tmp_y_old = tmp_y;

            //printf("Accelerometer timestamp: %ld ms\n", duration_cast<milliseconds>(acceleroTs).count());
            //printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
            //printf("Gyroscope timestamp: %ld ms\n", duration_cast<milliseconds>(gyroTs).count());
            //printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
        }

        //画像回転
        Mat r_image, r_mask_image; //傾き補正後の画像
        float width = inRgb->getCvFrame().cols;
        float height = inRgb->getCvFrame().rows;

        cv::Point2f center = cv::Point2f((width / 2), (height / 2)); //図形の中心
        double degree = -tmp_r;                                      // 回転角度
        double scale = 1;                                            //大きさの定義

        cv::Mat change = cv::getRotationMatrix2D(center, degree, scale);                                                                 //回転&拡大縮小
        cv::warpAffine(inRgb->getCvFrame(), r_image, change, r_image.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0)); //画像の変換(アフィン変換)
        //imshow("origin img", r_image);
        cv::warpAffine(mask_image, r_mask_image, change, r_mask_image.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0)); //画像の変換(アフィン変換)
        //imshow("mask img", r_mask_image);

        //マスク合成
        Mat mask_image_add;
        r_image.copyTo(mask_image_add, r_mask_image);
        //２値化
        Mat gray_img, gray_img_tmp;                        //グレースケール画像を入れておくためのMat
        Mat bin_img, bin_img2;                             //2値画像を入れておくためのMat
        cvtColor(mask_image_add, gray_img, COLOR_BGR2HSV); //HSVに変換
        int dilation_size = 2;
        //Mat element = getStructuringElement(MORPH_CROSS, //膨張　Eroding and Dilating
        //                                    Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        //                                    Point(dilation_size, dilation_size));
        //dilate(gray_img, gray_img_tmp, element);
        // inRangeを用いてフィルタリング(色相、彩度、明度)
        //imshow("before 2color", gray_img_tmp);
        Scalar s_min_a = Scalar(0, 0, 10);
        Scalar s_max_a = Scalar(180, 40, 255);
        inRange(gray_img, s_min_a, s_max_a, bin_img);
        erode(bin_img,gray_img_tmp, cv::Mat(), cv::Point(-1, -1), 1);//縮小（ノイズ除去）
        dilate(gray_img_tmp, bin_img, cv::Mat(), cv::Point(-1, -1), 1);//膨張
        //threshold(gray_img, bin_img, 80, 255, THRESH_BINARY); //閾値160で2値画像に変換
        imshow("masked 2color", bin_img);

        // 画像 dst を動画ファイルへ書き出す
        
		writer1 << gray_img;
        writer2 << mask_image_add;
        Mat tmp_r_mask_image;
        cvtColor(r_mask_image, tmp_r_mask_image, COLOR_GRAY2BGR);
        writer3 << tmp_r_mask_image;
        

        //ベース検出
        Mat img, tmp, result, base_img;
        // 画像読み込み
        img = bin_img;
        //base_img = imread("base.png", IMREAD_COLOR);
        //輪郭の座標リスト
        std::vector<std::vector<cv::Point>> contours;
        //輪郭取得
        //cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        findContours(img, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
        // 検出された輪郭線を緑で描画
        int i = 0;
        //for (auto contour = contours.begin(); contour != contours.end(); contour++)
        //{
            //    cv::polylines(r_image, *contour, true, cv::Scalar(0, 255, 0), 2);
        //}
        for (auto contour = contours.begin(); contour != contours.end(); contour++)
        {
            std::vector<cv::Point> approx;

            //輪郭を直線近似する
            cv::approxPolyDP(cv::Mat(*contour), approx, 0.02 * cv::arcLength(*contour, true), true);

            // 近似の面積が一定以上なら取得
            double area = cv::contourArea(approx);

            if ((area > 2000) && (area < 4000))
            {
                //printf("area-%f\n", area);
                //青で囲む場合
                if ((approx.size() <= 20) && (approx.size() >= 4) && (isContourConvex(approx)))
                {
                    //cv::polylines(r_image, approx, true, cv::Scalar(255, 0, 0), 2);
                    std::vector<cv::Point> approx2;
                    auto app = approx.begin();
                    auto i = 0;
                    auto i_MAX = 0;
                    //base = approx[0];
                    //back_r = approx[0];
                    //back_l = approx[0];
                    int nw, ne, sw, se;
                    nw = 9999;
                    ne = 9999, sw = 0, se = 0;
                    for (app; app != approx.end(); app++)
                    {
                        if (app[0].y > base.y)
                        {
                            base = app[0];
                            i++;
                        }
                        if (nw > (app[0].x + app[0].y))
                        {
                            nw = (app[0].x + app[0].y);
                            front_l = app[0];
                        }
                        if (se < (app[0].x + app[0].y))
                        {
                            se = (app[0].x + app[0].y);
                            back_r = app[0];
                        }
                        if (ne > (640-app[0].x + app[0].y))
                        {
                            ne = (640-app[0].x + app[0].y);
                            front_r = app[0];
                        }
                        if (sw < (640-app[0].x + app[0].y))
                        {
                            sw = (640-app[0].x + app[0].y);
                            back_l = app[0];
                        }
                        i_MAX++;
                    }
                    if (front_r.y > front_l.y) {
                        base_h = base.y - front_l.y;
                    }
                    else {
                        base_h = base.y - front_r.y;
                    }
                    base_w = back_r.x - back_l.x;
                    zone_h= (base_h / 25) * 270;//ゾーンの高さ
                    zone_l = 70;

                    if ((base_h < 50) && (base_h > 20) && (base_w < 120) && (base_w > 80)) {
                        //printf("basesize(w,h) :%d/%d ", base_w,base_h);
                        zone_flg = true;

                        cv::polylines(r_image, approx, true, cv::Scalar(255, 0, 0), 2);
                        cv::fillPoly(img, approx, cv::Scalar(0, 0, 0),LINE_8,1.2);//ボール判定用にホームベースを塗りつぶす

                        //ストライクゾーン描画
                        //cv::rectangle(r_image, cv::Point(back_l.x, base.y - 250), cv::Point(back_r.x, base.y - 100), cv::Scalar(0, 255, 0), 5);
                        line(r_image, Point(back_l.x, back_l.y - zone_l), Point(back_l.x, base.y - zone_h), Scalar(255, 255, 255), 2, LINE_8);
                        line(r_image, Point(front_l.x, front_l.y - zone_l), Point(front_l.x, base.y - zone_h), Scalar(255, 255, 255), 5, LINE_8);
                        line(r_image, Point(back_r.x, back_r.y - zone_l), Point(back_r.x, base.y - zone_h), Scalar(255, 255, 255), 2, LINE_8);
                        line(r_image, Point(front_r.x, front_r.y - zone_l), Point(front_r.x, base.y - zone_h), Scalar(255, 255, 255), 5, LINE_8);
                        line(r_image, Point(base.x, base.y - zone_l), Point(base.x, base.y - zone_h), Scalar(255, 255, 255), 1, LINE_8);

                        line(r_image, Point(back_l.x, back_l.y - zone_l), Point(base.x, base.y - zone_l), Scalar(255, 255, 255), 5, LINE_8);
                        line(r_image, Point(back_r.x, back_r.y - zone_l), Point(base.x, base.y - zone_l), Scalar(255, 255, 255), 5, LINE_8);
                        line(r_image, Point(back_l.x, back_l.y - zone_l), Point(front_l.x, front_l.y - zone_l), Scalar(255, 255, 255), 5, LINE_8);
                        line(r_image, Point(back_r.x, back_r.y - zone_l), Point(front_r.x, front_r.y - zone_l), Scalar(255, 255, 255), 5, LINE_8);
                        line(r_image, Point(front_l.x, front_l.y - zone_l), Point(front_r.x, front_r.y - zone_l), Scalar(255, 255, 255), 5, LINE_8);

                        line(r_image, Point(back_l.x, base.y - zone_h), Point(base.x, base.y - zone_h), Scalar(255, 255, 255), 2, LINE_8);
                        line(r_image, Point(back_r.x, base.y - zone_h), Point(base.x, base.y - zone_h), Scalar(255, 255, 255), 2, LINE_8);
                        line(r_image, Point(back_l.x, base.y - zone_h), Point(front_l.x, base.y - zone_h), Scalar(255, 255, 255), 2, LINE_8);
                        line(r_image, Point(back_r.x, base.y - zone_h), Point(front_r.x, base.y - zone_h), Scalar(255, 255, 255), 2, LINE_8);
                        line(r_image, Point(front_l.x, base.y - zone_h), Point(front_r.x, base.y - zone_h), Scalar(255, 255, 255), 5, LINE_8);

                        //printf("L front/back,R front/back :%d/%d,%d/%d ", front_l, back_l, front_r, back_r);
                        //printf("\n");
                        break;
                    }
                }
                //輪郭に隣接する矩形の取得
                //cv::Rect brect = cv::boundingRect(cv::Mat(approx).reshape(2));
                //roi[roiCnt] = cv::Mat(img, brect);

                //入力画像に表示する場合
                //cv::drawContours(imgIn, contours, i, CV_RGB(0, 0, 255), 4);

                //表示
                //cv::imshow("label" + std::to_string(roiCnt + 1), roi[roiCnt]);

                roiCnt++;

                //念のため輪郭をカウント
                if (roiCnt == 99)
                {
                    break;
                }
            }
            i++;
        }

        //ボール検出
        findContours(r_mask_image, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
        for (auto contour = contours.begin(); contour != contours.end(); contour++)
        {
            if (!pr_flg && zone_flg) {
                minEnclosingCircle(cv::Mat(*contour), center, radius);
                // 外接円を描画
                if ((radius >= 6) && (radius <= 11)) {

                    if (((back_l.x * 0.9) <= center.x) && ((back_r.x * 1.1) >= center.x) && (((base.y - zone_l) * 1.05) >= center.y) && (((base.y - zone_h) * 0.9) <= center.y)) {
                        imwrite("ball find.jpg", r_mask_image);
                        //if (((back_l.x * 0.9) <= center.x) && ((back_r.x * 1.1) >= center.x) && (((base.y - zone_l) * 1.1) >= center.y) && (((base.y - zone_h) * 0.9) <= center.y) && (bin_img.at<unsigned char>(center.x, center.y) != 0)) {
                        if ((back_l.x <= center.x) && (back_r.x >= center.x) && ((base.y - zone_l) >= center.y) && ((base.y - zone_h) <= center.y)) {
                            // 円の中心を描画します．
                            circle(r_image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
                            // 円を描画します．
                            circle(r_image, center, radius, Scalar(255, 0, 0), 3, 8, 0);
                            //amedWindow("circles", 1);
                            imshow("base & ball", r_image);
                            printf("STRIKE!!! %f\n:", radius);
                            //destroyWindow("base & ball");
                            //printf("---%d\n", clock() / CLOCKS_PER_SEC);
                            pr_time = clock() / CLOCKS_PER_SEC;
                            pr_flg = true;
                        }
                        else {
                            // 円の中心を描画します．
                            circle(r_image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
                            // 円を描画します．
                            circle(r_image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
                            //amedWindow("circles", 1);
                            imshow("base & ball", r_image);
                            printf("Ball--- %f\n:", radius);
                            //destroyWindow("base & ball");
                            //printf("---%d\n", clock() / CLOCKS_PER_SEC);
                            pr_time = clock() / CLOCKS_PER_SEC;
                            pr_flg = true;
                        }
                    }
                }
            }
        }

        imshow("img-rinkaku", r_image);
        //mask_image_add.convertTo(img, -1, 10, 0); //コントラスト　１０倍、明るさ０
        //imshow("img-ballqq", img);


        //printf("gaso:%d\n", r_mask_image.at<unsigned char>(300, 200));
        if (pr_flg) {
            if ((pr_time + 8) < (clock() / CLOCKS_PER_SEC)) {
                destroyWindow("base & ball");
                pr_flg = false;
            }
        }
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            writer1.release();
            writer2.release();
            writer3.release();
            return 0;
        }
        /*
        frame_cnt++;
        printf("frame:%d\n", frame_cnt);
        if ((getTickCount()) > (old_tick + 10000)) {
            printf("frame:%d\n", frame_cnt);
            frame_cnt = 0;
            old_tick = (getTickCount());
        }
        else {
            frame_cnt++;
        }*/
    }
    return 0;
}

Mat img_rotate(Mat img_p, int arg_p)
{
    Mat image_r;
    //int arg = int(get_degree(img_p));
    //高さを定義
    //height = img_p.shape[0];
    //幅を定義
    //width = img_p.shape[1];
    //回転の中心を指定
    //center = (int(width / 2), int(height / 2));
    //getRotationMatrix2D関数を使用 trans = cv2.getRotationMatrix2D(center, arg_p, 1)
    //アフィン変換
    //Mat image_r = cv2.warpAffine(img_p, trans, (width, height));
    return image_r;
}
