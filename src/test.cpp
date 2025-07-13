#include <cstdio>
#include "C:\Users\Takashi\OneDrive\depthaiC++\depthai-core-example-main\depthai-core\examples\src\utility.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

class Base
{
private:
    Mat result_img;
    Mat tmp, bin_img, gray_img_tmp;
    double maxVal, max_match = 0;
    double best_v_filter = 0;
    cv::Point max_pt;
    //輪郭の数
    int roiCnt = 0;
    Point front_r_temp;
    Point front_l_temp;
    Point back_r_temp;
    Point back_l_temp;
    Point base_temp;
    char value_c[256];

public:
    Point front_r;
    Point front_l;
    Point back_r;
    Point back_l;
    Point base;
    int base_h = 0; //ベースの高さ
    int base_w = 0; //ベースの幅
    int zone_h = 0;
    int zone_l = 0;
    bool zone_flg; //ゾーン検出住みか？
    std::vector<cv::Point> approx;
    //輪郭の座標リスト
    std::vector<std::vector<cv::Point>> contours;
    float radius;
    cv::Point2f center = cv::Point2f((640 / 2), (400 / 2)); //図形の中心

    bool Base_calibration(Mat gray_img, Mat base_template_img, double* max_v)
    {
        bool ret = false;
        for (double v_filter = 0; v_filter <= 255; v_filter++)
        {
            //printf("v val %f\n", v_filter);
            Scalar s_min_a = Scalar(0, 0, v_filter);
            Scalar s_max_a = Scalar(255, 100, 255);
            inRange(gray_img, s_min_a, s_max_a, bin_img);
            erode(bin_img, gray_img_tmp, cv::Mat(), cv::Point(-1, -1), 1);  //縮小（ノイズ除去）
            dilate(gray_img_tmp, bin_img, cv::Mat(), cv::Point(-1, -1), 1); //膨張
            //S ベーステンプレートマッチング
            cvtColor(bin_img, tmp, COLOR_GRAY2BGR);
            cv::matchTemplate(tmp, base_template_img, result_img, cv::TM_CCOEFF_NORMED);
            cv::minMaxLoc(result_img, NULL, &maxVal, NULL, &max_pt);
            if (max_match < maxVal)
            {
                max_match = maxVal;
                best_v_filter = v_filter;
                //printf("maxval %f\n", maxVal);
            }
            if (maxVal > 0.7)
            {
                ret = true;
                printf("maxval %f\n", maxVal);
            }
            sprintf(value_c, "value=%f", v_filter); //変数の値も含めた表示したい文字列をchar型変数に格納
            cv::putText(
                bin_img,
                value_c,
                cv::Point(20, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                cv::Scalar(255, 255, 255),
                2);
            imshow("BASE calibation 2", bin_img);
            waitKey(1);
            //E　ベーステンプレートマッチング
        }
        if (ret)
        {
            *max_v = best_v_filter;
        }
        else
        {
            *max_v = 200;
        }
        return ret;
    }

    bool Base_find(Mat bin_img)
    {
        bool ret;
        ret = false;
        //S ベース　輪郭検出
        findContours(bin_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        int i = 0;
        for (auto contour = contours.begin(); contour != contours.end(); contour++)
        {
            //輪郭を直線近似する
            cv::approxPolyDP(cv::Mat(*contour), approx, 0.02 * cv::arcLength(*contour, true), true);
            // 近似の面積が一定以上なら取得
            double area = cv::contourArea(approx); //ベースの面積
            double leng = arcLength(approx, true); //ベースの輪郭長さ
            if ((area > 500) && (area < 2800) && (leng < 280) && (leng > 50))
            {
                //printf("base menseki %f\n", area);
                //printf("base length  %f\n", leng);
                if ((approx.size() <= 15) && (approx.size() >= 3))
                {
                    std::vector<cv::Point> approx2;
                    auto app = approx.begin();
                    auto i = 0;
                    auto i_MAX = 0;
                    int nw, ne, sw, se, tmp_l;
                    int nw_min, ne_min, sw_min, se_min;
                    int r_max, l_max;
                    nw = 0;
                    ne = 0, sw = 0, se = 0, tmp_l = 0;
                    nw_min = 99999, ne_min = 99999, sw_min = 99999, se_min = 99999;
                    r_max = 0;
                    l_max = 9999;
                    base_temp.y = 0;
                    for (app; app != approx.end(); app++)
                    {
                        if (app[0].y > base_temp.y)
                        {
                            base_temp = app[0];
                            i++;
                        }
                        if (r_max < app[0].x)
                        {
                            r_max = app[0].x;
                        }
                        if (l_max > app[0].x)
                        {
                            l_max = app[0].x;
                        }
                        i_MAX++;
                    }
                    app = approx.begin();
                    for (app; app != approx.end(); app++)
                    {
                        if (base_temp.y > app[0].y) //ベース基底以外
                        {
                            tmp_l = ((base_temp.x - app[0].x) * (base_temp.x - app[0].x)) + ((base_temp.y - app[0].y) * (base_temp.y - app[0].y)); //ベース基底からの距離計算
                            if (app[0].x > base_temp.x) //右側か左側か
                            {
                                if (tmp_l > ne)
                                {
                                    front_r_temp = app[0]; //右上
                                    ne = tmp_l;
                                }
                                if (r_max == app[0].x) //右端
                                {
                                    back_r_temp = app[0]; //右下
                                    se_min = tmp_l;
                                }
                            }
                            else
                            {
                                if (tmp_l > nw)
                                {
                                    front_l_temp = app[0]; //左上
                                    nw = tmp_l;
                                }
                                if (l_max == app[0].x) //左端
                                {
                                    back_l_temp = app[0]; //左下
                                    sw_min = tmp_l;
                                }
                            }
                        }
                    }
                    if (front_r.y > front_l.y)
                    {
                        base_h = base_temp.y - front_l_temp.y;
                    }
                    else
                    {
                        base_h = base_temp.y - front_r_temp.y;
                    }
                    zone_h = (((25-((25-float(base_h))*0.3)) / 25) * 260); //ゾーンの高さ
                    zone_l = 70;
                    base_w = (back_r_temp.x - back_l_temp.x);
                    if ((base_w > 70) && (base_h > 15) && (base_w < 120) && (base_h < 70))
                    {
                        base = base_temp;
                        front_l = front_l_temp;
                        front_r = front_r_temp;
                        back_l = back_l_temp;
                        back_r = back_r_temp;
                        zone_flg = true;
                        ret = true;
                        break;
                    }
                    else
                    {
                        //printf("error base size :%d X %d\n", base_w, base_h);
                    }
                }
                roiCnt++;

                //念のため輪郭をカウント
                if (roiCnt == 99)
                {
                    break;
                }
            } else {
            //printf("base menseki %f\n", area);
            //printf("base length  %f\n", leng);
}
            i++;
        }
        //E ベース　輪郭検出
        return ret;
    }

    bool Ball_find(Mat img, bool* strike)
    {
        bool ret;
        ret = false;
        *strike = false;
        //ボール検出
        Point min_point, max_point;
        std::vector<cv::Point> approx;
        std::vector<std::vector<cv::Point>> contours;
        findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        int i = 0;
        int ball_x_min = 9999, ball_x_max = 0, ball_y_min = 9999, ball_y_max = 0;
        for (auto contour = contours.begin(); contour != contours.end(); contour++)
        {
            //輪郭を直線近似する
            cv::approxPolyDP(cv::Mat(*contour), approx, 0.01 * cv::arcLength(*contour, true), true);
            // 近似の面積が一定以上なら取得
            double area = cv::contourArea(approx); //ボールの面積
            double leng = arcLength(approx, true); //ボールの輪郭長さ
            if ((area < 350) && (area > 100))
            {
                //printf("area:%f,leng:%f\n", area, leng);
                //if ((approx.size() <= 5) && (approx.size() >= 4) && (isContourConvex(approx))) {
                //円周と面積の比率から円かどうかを判断
                if (((area * 1.5) > (3 * ((leng / 6) * (leng / 6)))) && ((area * 0.7) < (3 * ((leng / 6) * (leng / 6)))))
                {
                    //printf("area:%f,leng:%f\n", area, leng);
                    auto app = approx.begin();
                    for (app; app != approx.end(); app++)
                    {
                        if (app[0].x < ball_x_min)
                            ball_x_min = app[0].x;
                        if (app[0].x > ball_x_max)
                            ball_x_max = app[0].x;
                        if (app[0].y < ball_y_min)
                            ball_y_min = app[0].y;
                        if (app[0].y > ball_y_max)
                            ball_y_max = app[0].y;
                    }
                    radius = (ball_x_max - ball_x_min) / 2;
                    float radius_h= (ball_y_max - ball_y_min) / 2;
                    if ((radius < radius_h*3.0) && (radius >radius_h*0.3)) {
                        center = Point2f(ball_x_min + radius, ball_y_min + radius_h);
                        //center.x = 320 + ((center.x - 320) * (10 / radius)); //ボールサイズに応じた位置補正
                        //center.y = 240 + ((center.y - 240) * (10 / radius)); //ボールサイズに応じた位置補正
                        if (((back_l.x -(base_w * 0.5)) <= (center.x + radius)) && ((back_r.x +((base_w * 0.5))) >= (center.x - radius)) && (((base.y - zone_l) * 1.05) >= center.y) && (((base.y - zone_h) * 0.9) <= center.y))
                        //if (true)
                                //ゾーン近辺か？
                        {
                            ret = true;
                            printf("size:%f,%f\n", area);
                            printf("radius:%f,%f\n", radius, radius_h);
                            //STRIKE or BALL?
                            if ((back_l.x <= (center.x+radius)) && (back_r.x >= (center.x - radius)) && ((base.y - zone_l) >= center.y) && ((base.y - zone_h) <= center.y))
                            {
                                *strike = true;
                            }
                            break;
                        }
                        else
                        {
                            //printf("zone err\n");
                            radius = 0;
                        }
                    }
                    else {
                        //printf("keijyou err\n");
                    }
                }
                else {
                    //printf("ensyuu err\n");
                }
                //}
            }
            else {
                //printf("size err\n");
            }
        }
        return ret;
    }
};

int main()
{
    double n = 0;
    bool pr_flg = false;
    int64 old_tick = 0;
    int frame_cnt = 0;
    //輪郭の数
    int roiCnt = 0;

    int pr_time;      //
    double scale = 1; //大きさの定義

    // 画像を格納するオブジェクトを宣言する
    cv::Mat base_img, base_template_img, base_template_bin_img;

    // 画像ファイルから画像データを読み込む
    base_img = cv::imread("base_snap.png");
    base_template_img = cv::imread("base6.jpg");
    cv::threshold(base_template_img, base_template_bin_img, 80, 255, THRESH_BINARY); //閾値160で2値画像に変換

    // 動画ファイルを取り込むためのオブジェクトを宣言する
    cv::VideoCapture cap, cap2,cap3;
    // 動画ファイルを開く
    //cap.open("ball_MASK_BIN-CHUWI-Hi10X-6.avi");
    cap.open("ball_MASK_BIN17.avi");
    //ball_MASK_BGR - CHUWI - Hi10X - 6
    // 画像を格納するオブジェクトを宣言する
    cv::Mat frame;
    //cap2.open("ball_MASK_HSV-CHUWI-Hi10X-6.avi");
    cap2.open("ball_MASK_HSV17.avi");
    // 画像を格納するオブジェクトを宣言する
    cv::Mat frame2;
    //cap3.open("ball_MASK_BGR-CHUWI-Hi10X-6.avi");
    cap3.open("ball_MASK_BGR17.avi");
    // 画像を格納するオブジェクトを宣言する
    cv::Mat frame3;

    //cv::imshow("disparity", frame);

    //マスク合成
    resize(base_img, base_img, cv::Size(640, 400), 1, 1);
    Mat mask_image_add, org_img;
    base_img.copyTo(mask_image_add, base_img);
    base_img.copyTo(org_img); //オリジナルの保存
    //２値化
    Mat gray_img, gray_img_tmp; //グレースケール画像を入れておくためのMat
    Mat bin_img, bin_img2;      //2値画像を入れておくためのMat

    cvtColor(mask_image_add, gray_img, COLOR_BGR2HSV); //HSVに変換
    int dilation_size = 2;
    //Mat element = getStructuringElement(MORPH_CROSS, //膨張　Eroding and Dilating
    //                                    Size(2 * dilation_size + 1, 2 * dilation_size + 1),
    //                                    Point(dilation_size, dilation_size));
    //dilate(gray_img, gray_img_tmp, element);
    // inRangeを用いてフィルタリング(色相、彩度、明度)
    //imshow("before 2color", gray_img_tmp);

    //S キャリブレーション
    double best_v_filter;
    Base basez;
    //basez.Base_calibration(gray_img, base_template_img, &best_v_filter);
    best_v_filter = 200;

    printf("best val %lf\n", best_v_filter);
    //E キャリブレーション

    while (true)
    {
        cv::Mat frame2;
        cap2 >> frame2;
        cap3 >> frame3;

        if (frame2.empty() == true)
        {
            // 画像が読み込めなかったとき、無限ループを抜ける
            break;
        }

        org_img.copyTo(base_img); //画像クリア
        frame3.copyTo(base_img);//ホームベース判定に静止画を判定するときはこの行をコメントアウト

        Scalar s_min_a = Scalar(0, 0, best_v_filter);
        Scalar s_max_a = Scalar(255, 100, 255);
        inRange(frame2, s_min_a, s_max_a, bin_img);
        erode(bin_img, gray_img_tmp, cv::Mat(), cv::Point(-1, -1), 2);  //縮小（ノイズ除去）
        dilate(gray_img_tmp, bin_img, cv::Mat(), cv::Point(-1, -1), 2); //膨張
        //threshold(gray_img, bin_img, 80, 255, THRESH_BINARY); //閾値160で2値画像に変換

        if (basez.Base_find(bin_img)) { //ベース検出

            cv::polylines(base_img, basez.approx, true, cv::Scalar(255, 255, 0), 2);
            //cv::fillPoly(img, approx, cv::Scalar(0, 0, 0), LINE_8, 1.2);//ボール判定用にホームベースを塗りつぶす

            //ストライクゾーン描画
            //cv::rectangle(base_img, cv::Point(back_l.x, base.y - 250), cv::Point(back_r.x, base.y - 100), cv::Scalar(0, 255, 0), 5);
            line(base_img, Point(basez.back_l.x, basez.back_l.y - basez.zone_l), Point(basez.back_l.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 1, LINE_8);
            line(base_img, Point(basez.front_l.x, basez.front_l.y - basez.zone_l), Point(basez.front_l.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 2, LINE_8);
            line(base_img, Point(basez.back_r.x, basez.back_r.y - basez.zone_l), Point(basez.back_r.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 1, LINE_8);
            line(base_img, Point(basez.front_r.x, basez.front_r.y - basez.zone_l), Point(basez.front_r.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 2, LINE_8);
            line(base_img, Point(basez.base.x, basez.base.y - basez.zone_l), Point(basez.base.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 1, LINE_8);

            line(base_img, Point(basez.back_l.x, basez.back_l.y - basez.zone_l), Point(basez.base.x, basez.base.y - basez.zone_l), Scalar(255, 255, 0), 2, LINE_8);
            line(base_img, Point(basez.back_r.x, basez.back_r.y - basez.zone_l), Point(basez.base.x, basez.base.y - basez.zone_l), Scalar(255, 255, 0), 2, LINE_8);
            line(base_img, Point(basez.back_l.x, basez.back_l.y - basez.zone_l), Point(basez.front_l.x, basez.front_l.y - basez.zone_l), Scalar(255, 255, 0), 2, LINE_8);
            line(base_img, Point(basez.back_r.x, basez.back_r.y - basez.zone_l), Point(basez.front_r.x, basez.front_r.y - basez.zone_l), Scalar(255, 255, 0), 2, LINE_8);
            line(base_img, Point(basez.front_l.x, basez.front_l.y - basez.zone_l), Point(basez.front_r.x, basez.front_r.y - basez.zone_l), Scalar(255, 255, 0), 2, LINE_8);

            line(base_img, Point(basez.back_l.x, basez.base.y - basez.zone_h), Point(basez.base.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 1, LINE_8);
            line(base_img, Point(basez.back_r.x, basez.base.y - basez.zone_h), Point(basez.base.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 1, LINE_8);
            line(base_img, Point(basez.back_l.x, basez.base.y - basez.zone_h), Point(basez.front_l.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 1, LINE_8);
            line(base_img, Point(basez.back_r.x, basez.base.y - basez.zone_h), Point(basez.front_r.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 1, LINE_8);
            line(base_img, Point(basez.front_l.x, basez.base.y - basez.zone_h), Point(basez.front_r.x, basez.base.y - basez.zone_h), Scalar(255, 255, 0), 2, LINE_8);
        }
        imshow("zone2", base_img);
        imshow("zone3", frame3);
        // 画像を格納するオブジェクトを宣言する
        cv::Mat frame;
        cap >> frame;

        if (frame.empty() == true)
        {
            // 画像が読み込めなかったとき、無限ループを抜ける
            break;
        }

        //ベース検出
        Mat img, tmp, result, r_image;
        // 画像読み込み
        // グレースケール画像に変換
        cv::Mat grayImage;
        cv::cvtColor(frame, grayImage, COLOR_BGR2GRAY);
        threshold(grayImage, img, 160, 255, THRESH_BINARY); //閾値160で2値画像に変換


        //ボール検出
        r_image = frame;
        bool strike;
        if (basez.Ball_find(img, &strike) && !pr_flg)
        {
            if (strike)
            {
                // 円を描画します．
                circle(base_img, basez.center, basez.radius, Scalar(255, 255, 0), 3, 8, 0);
                printf("STRIKE!!! :%f\n", basez.radius);
            }
            else {
                // 円を描画します．
                circle(base_img, basez.center, basez.radius, Scalar(0, 0, 255), 3, 8, 0);
                printf("BAll :%f\n", basez.radius);

            }
            //amedWindow("circles", 1);
            imshow("base & ball", r_image);
            imshow("base & ball2", base_img);

            pr_time = clock() / CLOCKS_PER_SEC;
            pr_flg = true;
        }
        if (pr_flg)
        {
            if ((pr_time + 3) < (clock() / CLOCKS_PER_SEC))
            {
                destroyWindow("base & ball");
                destroyWindow("base & ball2");
                pr_flg = false;
            }
        }
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            return 0;
        }
        n=cap.get(cv::CAP_PROP_POS_FRAMES);
        if (key == 's' || key == 'S')
        {
            cap.set(cv::CAP_PROP_POS_FRAMES, n+30);	// n+30に設定    }
            cap2.set(cv::CAP_PROP_POS_FRAMES, n+30);	// n+30に設定    }
            cap3.set(cv::CAP_PROP_POS_FRAMES, n+30);	// n+30に設定    }
            printf("frame no:%f\n", n);
        }
        if (key == 'r' || key == 'R')
        {
            cap.set(cv::CAP_PROP_POS_FRAMES, n-30);	// n+30に設定    }
            cap2.set(cv::CAP_PROP_POS_FRAMES, n-30);	// n+30に設定    }
            cap3.set(cv::CAP_PROP_POS_FRAMES, n-30);	// n+30に設定    }
            printf("frame no:%f\n", n);
        }
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
