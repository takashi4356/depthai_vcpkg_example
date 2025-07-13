class Base
{
#include "strike.hpp"
#include <cstdio>
    // #include "C:\Users\Takashi\OneDrive\depthaiC++\depthai-core-example-main\depthai-core\examples\src\utility.hpp"
#include "depthai/depthai.hpp"
#include "plog/Initializers/RollingFileInitializer.h"
#include <Eigen/Dense>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Log.h>
#include <thread>
#include <time.h>

    using json = nlohmann::json;

  private:
    struct Camera
    {
        Eigen::Vector3f position; // カメラの位置
        float pitch;              // ピッチ角（X軸回転、ラジアン単位）
        float roll;               // ロール角（Z軸回転、ラジアン単位）
        float yaw;                // ヨー角（Y軸回転、ラジアン単位）
        float vertical_fov;       // 垂直視野角（度数）
        float horizontal_fov;     // 水平視野角（度数）
        float near;               // 近接平面
        float far;                // 遠方平面
    };

    Mat tmp, bin_img, gray_img_tmp, gray_img_tmp2, gray_img_tmp3;
    double maxVal, max_match = 0;
    float best_v_filter = 0;
    cv::Point max_pt;
    // 輪郭の数
    int roiCnt = 0;
    Point front_r_temp;
    Point front_l_temp;
    Point back_r_temp;
    Point back_l_temp;
    Point base_temp; // ベース基底座標算出ワーク
    char value_c[256];
    Point2f center_temp; // = cv::Point2f((640 / 2), (400 / 2)); //図形の中心
    Point2f center_temp_cm;
    Mat clip;
    double def_v_filter = 140; // デフォルト値
    int old_base_dis = 0;
    float c_enkeido;
    float enkeido;
    float ball_r_min;
    float ball_r_max;
    float ball_size;         // ボールサイズチェック用
    int ball_size_b;         // ボールサイズチェック用
    int max_ball_area = 880; // ball area 400 ball area:102.500000,leng:39.556349
    int mini_ball_area = 25; // ball area 60
    int c_max_ball_area;
    int c_mini_ball_area;

    float knee_dep_cm, knee_cm_y_temp, knee_cm_x_temp;
    float shoulder_dep_cm, shoulder_cm_y_temp, shoulder_cm_x_temp;
    float wrist_dep_cm, wrist_cm_y_temp, wrist_cm_x_temp;
    float hip_dep_cm, hip_cm_y_temp, hip_cm_x_temp;
    nlohmann::json conf_json;
    int rect_x_hi_640; // 画面端計算用
    int init_i_old = 180;

    //! ボールノイズ記録
    struct ball_noise
    {
        int x;
        int y;
        int depth;
        int cnt;
        int cnt_nodep;
        int camera_n;
        uint64_t time;
    };
    ball_noise ballnoise[c_ball_noise_i_max];
    int c_rez_x_r;
    int c_rez_y_r;

    Mat nw_tmp, ne_tmp, sw_tmp, se_tmp, base_tmp; //ベース角テンプレート
    Mat mat_nw_temp, mat_ne_temp, mat_sw_temp, mat_se_temp, mat_base_temp;
    int c_base_depth_bias = 0;
    Mat matBallTemplate2;
    Mat matBallTemplate3;
    Mat matBallTemplate4;
    Mat matBallTemplate5;
    Mat matBallTemplate6;
    Mat matBallTemplate7;
    Mat matBallTemplate8;
    Mat matBallTemplate9;
    Mat matBallTemplate10;
    Mat matBallTemplate11;
    std::vector<cv::Mat> ball10Templates;
    // テンプレート画像の読み込み
    Mat templateB; //ホームベース
    Mat templateB_base = Mat::zeros(80, 100, CV_8UC1);
    Point bestMatchOld = Point(999, 999);
    uint64_t bestMatchTime = 0;
    float enkeido_old = 0; // 円形度の前回値

  public:
    int dep_l_en;  // 遠方フィルタ
    int dep_l_kin; // 近距離フィルタ
    int dep_base_en = c_dep_base_en;
    int dep_base_kin = c_dep_base_kin;
    Point baseCoordinateFront_r;
    Point baseCoordinateFront_l;
    Point baseCoordinateBack_r;
    Point baseCoordinateBack_l;
    Point baseCoordinateBase; // ベース基底座標
    Point base_center;
    int base_h = 0;             // ベースの高さ
    int base_w, base_w_old = 0; // ベースの幅

    int zone_high_front = 0;  // ゾーン表示座標(y) 上部 フロント
    int zone_high_middle = 0; // ゾーン表示座標(y) 上部 ミドル
    int zone_high_back = 0;   // ゾーン表示座標(y) 上部 バック
    int zone_low_front = 0;   // ゾーン表示座標(y) 下部 フロント
    int zone_low_middle = 0;  // ゾーン表示座標(y) 下部 ミドル
    int zone_low_back = 0;    // ゾーン表示座標(y) 下部 バック

    int zone_left_front_x = 0;  // ゾーン表示座標(x) 左 フロント
    int zone_right_front_x = 0; // ゾーン表示座標(x) 右 フロント
    int zone_left_back_x = 0;   // ゾーン表示座標(x) 左 バック
    int zone_right_back_x = 0;  // ゾーン表示座標(x) 右 バック
    int zone_base_x = 0;        // ゾーン表示座標(x) 中央

    Point2i zone_right_front_hi;
    Point2i zone_right_front_low;
    Point2i zone_right_front_gnd;
    Point2i zone_right_middle_hi;
    Point2i zone_right_middle_low;
    Point2i zone_right_middle_gnd;
    Point2i zone_left_front_hi;
    Point2i zone_left_front_low;
    Point2i zone_left_front_gnd;
    Point2i zone_left_middle_hi;
    Point2i zone_left_middle_low;
    Point2i zone_left_middle_gnd;
    Point2i zone_base_hi;
    Point2i zone_base_low;
    Point2i zone_base_gnd;

    bool zone_flg;        // ゾーン検出住みか？
    Point zone_nw;        // ゾーンの左上
    Point zone_ne;        // ゾーンの右上
    Point zone_sw;        // ゾーンの左下
    Point zone_se;        // ゾーンの右下
    Point2f jyuushin;     // base jyuushin
    Point2f jyuushin_tmp; // base jyuushin
    int base_katamuki_hosei;
    bool base_find_f = false;
    uint64_t base_find_time = 0;
    float area; // ベースの面積
    float leng; // ベースの輪郭長さ
    int c_saido;
    int knee; // 膝たかさ
    int knee_detection;
    bool right_batter; // 打者左右
    float dot_hi_err;
    int base_detection_maker = 0;
    int ball_i;                          // ボール距離
    float straightLineDistanceToBase_cm; // ベースまでの直線距離
    int straightLineDistanceToBase_i;    // ベースまでの直線距離(i)
    float horizontalDistanceToBase_cm;   // ベースまでの水平距離
    int horizontalDistanceToBase_i;      //ベースまでの水平距離(i)
    float c_horizontalDistanceToBase_cm; // ベースま水平距離(固定)
    float zone_low_cm;                   // ゾーン下部の高さ（ゾーン中心からの距離(cm)）
    float zone_hi_cm;                    // ゾーン上部の高さ（ゾーン中心からの距離(cm)）
    float zone_r_cm;                     // ゾーン右端（ゾーン中心からの距離(cm)）
    float zone_l_cm;                     // ゾーン左端（ゾーン中心からの距離(cm)）
    float zone_c_dis_cm;                 // ゾーン中心の高さ(地面からゾーン中心までの距離(cm))
    float zone_c_dis_w_cm;               // ゾーン中心の中心ずれ(画面中心からゾーン中心までの距離(cm))
    float hfov;
    float hfov_rgb;
    float vfov_rgb;
    float vfov_mono;
    int C_monorgb_hi_x, c_bias_x, c_bias_x_ork_d, c_bias_x_ork_d_lite, C_monorgb_hi_y, c_bias_y, c_bias_y_ork_d, c_bias_y_ork_d_lite;
    float ball_dep_bias;
    float ball_high_bias;
    float area_ball; // ボールの面積
    float radius_temp;
    float radius_h;
    bool isSecondCamera;       //セカンドカメラかどうか
    bool is720p;               //720pかどうか
    Point2f center_temp_old;   // ボールのXY座標（前回）
    Point2f center_temp_old_f; // ボールのXY座標（前回）
    Point2f center_temp_old_s; // ボールのXY座標（前回）
    int ball_cours_cnt = 0;

    float fov;
    float fov_2;
    bool no_base_mode = false; //ベース検知なし
    bool ground_mode;          //グラウンド設置モード

    // バッター座標
    int batter_chest;  // 胸の高さ（足からの相対距離）
    int batter_knees;  // 膝の高さ（足からの相対距離）
    int batter_top;    // 座標
    int batter_bottom; // 座標
    int batter_right;  // X座標（右端）
    int batter_left;   // X座標（左端）
    int batter_hight;  // 打者身長(1-5)
    float c_zone_hi_per;
    float c_zone_low_per;
    bool bat_flg = false; // 打者検知済か

    std::vector<cv::Point> approx;
    std::vector<cv::Point> base_app;
    // 輪郭の座標リスト
    std::vector<std::vector<cv::Point>> contours;
    float radius;
    Point2i center; // = cv::Point2f((640 / 2), (400 / 2)); //ボールの中心
    Point2f hosei_center;
    Point2f hoseimae_center;
    float ballCenter_cm_x;               // 補正算出したボールの位置（ｃｍ）
    float ballCenter_cm_y;               // 補正算出したボールの位置（ｃｍ）
    float center_cm_x_temp;              // 補正算出したボールの位置（ｃｍ）
    float center_cm_y_temp;              // 補正算出したボールの位置（ｃｍ）
    float ball_dep_cm;                   // ボールまでの水平距離（ｃｍ）
    float straightLineDistanceToBall_cm; // ボールまでの直線距離
    int straightLineDistanceToBall_i;    // ボールまでの直線距離(i)
    float base_roll = 0;                 // ベースの傾き(X-Y平面)
    float base_yaw = 0;                  // ベースの傾き(X-Z平面)
    float base_pitch = 0;                // ベースの傾き(Y-Z平面)
    //! 画面中心の距離
    int center_dep;
    float c_camera_pitch = 0;     // カメラの下向き角度
    float camera_yaw_org = 0;     // カメラの傾き角度
    float camera_yaw_org_old = 0; // カメラの傾き角度
    float c_camera_high;
    float camera_high = c_camera_high;
    float camera_high_old = c_camera_high;
    cv::Point bestMatch;
    bool test_measurement_mode;
    int x_bias;          // 深度画像のオフセット
    int y_bias;          // 深度画像のオフセット
    int camera_center_x; // カメラ中心点
    int camera_center_y; // カメラ中心点
    float camera_pitch_org;
    float camera_pitch_keisuu;
    float camera_high_keisu;
    Camera camera;
    float base_h_shift;
    float c_zone_low_cm;
    float c_zone_hi_cm;
    float c_camera_high_no_base;
    float c_base_kyori_cm_no_base;
    int noize_dot;
    bool ball_noise_sw = false; //
    bool last_strike = false;
    // ベース検知　基準座標
    float dx = 0.015;   //0.045
    float dy = 0.20;    //0.20
    float dx_c = 0.007; //0.01
    float dy_c = 0.030; //0.055
    float dy_b = 0.040; //0.045
    Point2f target_nw = Point2f((0.5 - dx) * 640, (0.5 + dy) * 400);
    Point2f target_ne = Point2f((0.5 + dx) * 640, (0.5 + dy) * 400);
    Point2f target_sw = Point2f((0.5 - dx - dx_c) * 640, (0.5 + dy + dy_c) * 400);
    Point2f target_se = Point2f((0.5 + dx + dx_c) * 640, (0.5 + dy + dy_c) * 400);
    Point2f target_bs = Point2f(0.5 * 640, (0.5 + dy + dy_c + dy_b) * 400);
    bool camera_p_home_dis;
    bool imu_use;
    Dimensions ball_position;          // ボール位置（ベース相対）
    Dimensions ball_position_old;      // ボール位置（ベース相対）
    Dimensions ball_position_temp;     // ボール位置（判定途中）
    Dimensions ball_position_temp_fix; // ボール位置（一時判定後）
    Dimensions ball_position_old_1, ball_position_old_2;
    int camera_high_key_bias = 0;
    int base_kyori_cm_key_bias = 0;
    //速度向上むけ事前計算
    struct pre_calc
    {
        float cos_x_r;
        float cos_y_r;
        float sin_x_r;
        float sin_y_r;
        float tan_x_r;
        float tan_y_r;
        int side_filter;
    };
    pre_calc precalc[640];

    Base() // コンストラクタ
    {
        knee = 0;
        batter_top = 0;
        batter_bottom = 0;
        batter_chest = 0;
        batter_knees = 0;
        batter_hight = 1;
        batter_top = 0;
        batter_bottom = 0;
        batter_right = 0;
        batter_left = 0;
        zone_flg = false;
        dot_hi_err = c_dot_hi_err;
        rect_x_hi_640 = c_rect_x_hi * 640;

        //ベーステンプレート
        templateB = imread("baseTemplate.jpg", IMREAD_GRAYSCALE); //120×90(baseは256X180程度)
        //ベース角テンプレート画像の読み込み
        mat_nw_temp = cv::imread("nw32.png", cv::IMREAD_GRAYSCALE);
        threshold(mat_nw_temp, nw_tmp, 228, 255, cv::THRESH_BINARY);
        mat_ne_temp = cv::imread("ne32.png", cv::IMREAD_GRAYSCALE);
        threshold(mat_ne_temp, ne_tmp, 228, 255, cv::THRESH_BINARY);
        mat_sw_temp = cv::imread("sw32.png", cv::IMREAD_GRAYSCALE);
        threshold(mat_sw_temp, sw_tmp, 228, 255, cv::THRESH_BINARY);
        mat_se_temp = cv::imread("se32.png", cv::IMREAD_GRAYSCALE);
        threshold(mat_se_temp, se_tmp, 228, 255, cv::THRESH_BINARY);
        mat_base_temp = cv::imread("base32.png", cv::IMREAD_GRAYSCALE);
        threshold(mat_base_temp, base_tmp, 228, 255, cv::THRESH_BINARY);

        //ボールテンプレート画像の読み込み
        matBallTemplate2 = cv::imread("ballTemplate2.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate3 = cv::imread("ballTemplate3.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate4 = cv::imread("ballTemplate4.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate5 = cv::imread("ballTemplate5.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate6 = cv::imread("ballTemplate6.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate7 = cv::imread("ballTemplate7.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate8 = cv::imread("ballTemplate8.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate9 = cv::imread("ballTemplate9.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate10 = cv::imread("ballTemplate10.jpg", cv::IMREAD_GRAYSCALE);
        matBallTemplate11 = cv::imread("ballTemplate11.jpg", cv::IMREAD_GRAYSCALE);
        ball10Templates = {matBallTemplate2, matBallTemplate3, matBallTemplate4, matBallTemplate5, matBallTemplate6, matBallTemplate7, matBallTemplate8, matBallTemplate9, matBallTemplate10, matBallTemplate11}; // 10個のテンプレート画像

        std::ifstream conf_f("strike_config.json");
        conf_json = json::parse(conf_f);
        base_detection_maker = conf_json["base_detection_maker"];
        C_monorgb_hi_x = conf_json["C_monorgb_hi_x"];
        c_bias_x_ork_d = conf_json["c_bias_x_oak-d"];
        c_bias_x_ork_d_lite = conf_json["c_bias_x_oak-d_lite"];
        C_monorgb_hi_y = conf_json["C_monorgb_hi_y"];
        c_bias_y_ork_d = conf_json["c_bias_y_oak-d"];
        c_bias_y_ork_d_lite = conf_json["c_bias_y_oak-d_lite"];
        camera_pitch_keisuu = conf_json["c_camera_pitch_keisuu"];
        c_camera_high = conf_json["camera_high"];
        camera_high_keisu = conf_json["c_camera_high_keisu"];
        c_camera_pitch = conf_json["camera_pitch"];
        base_h_shift = conf_json["c_base_h_shift"];
        float hfov_bias = conf_json["hfov_bias"];
        float vfov_bias = conf_json["vfov_bias"];
        c_saido = conf_json["c_saido"];
        noize_dot = conf_json["noize_dot"];
        ball_dep_bias = conf_json["ball_dep_bias"];
        ball_high_bias = conf_json["ball_high_bias"];
        // fov = conf_json["fov"];
        base_katamuki_hosei = conf_json["base_katamuki_hosei"];
        c_enkeido = conf_json["c_enkeido"];
        ball_r_min = conf_json["ball_r_min"];
        ball_r_max = conf_json["ball_r_max"];
        ball_size = conf_json["ball_size"];
        ball_size_b = conf_json["ball_size_b"];
        c_max_ball_area = conf_json["max_ball_area"];   // ball area 400 ball area:102.500000,leng:39.556349
        c_mini_ball_area = conf_json["mini_ball_area"]; // ball area 60
        c_zone_hi_per = conf_json["c_zone_hi_per"];
        c_zone_low_per = conf_json["c_zone_low_per"];
        c_zone_low_cm = (float)c_zone_low_per * 1.85;
        c_zone_hi_cm = (float)c_zone_hi_per * 1.85;
        c_rez_x_r = conf_json["c_rez_x_r"];
        c_rez_y_r = conf_json["c_rez_y_r"];
        c_horizontalDistanceToBase_cm = conf_json["horizontalDistanceToBase_cm"];
        camera_p_home_dis = conf_json["camera_p_home_dis"]; // ホーム周辺距離からカメラ角度を求めるか
        no_base_mode = conf_json["no_base_mode"];
        c_camera_high_no_base = conf_json["camera_high_no_base"];
        c_base_kyori_cm_no_base = conf_json["base_kyori_cm_no_base"];
        c_base_depth_bias = conf_json["base_depth_bias"];

        camera.position = Eigen::Vector3f(0.0f, c_camera_high_no_base, 0.0f); // カメラ位置を165cmに設定
        camera.pitch = 0.0f * M_PI / 180.0f;
        camera.roll = 0.0f * M_PI / 180.0f;
        camera.yaw = 0.0f * M_PI / 180.0f;
        camera.vertical_fov = 75.0f;   // 垂直視野角(main側から再設定)
        camera.horizontal_fov = 49.0f; // 水平視野角(main側から再設定)
        camera.near = 100.0f;          // 近接平面を10cmに設定
        camera.far = 4000.0f;          // 遠方平面を1000cmに設定
    }

    //プレ計算
    void precalc_func()
    {
        for (int i = 0; i < 640; i++)
        {
            precalc[i].cos_x_r = abs(cos((float)(i - camera_center_x) * (rdn(hfov) / 640)));
            precalc[i].cos_y_r = abs(cos((float)(i - camera_center_y) * (rdn(vfov_mono) / 400)));
            precalc[i].sin_x_r = abs(sin((float)(i - camera_center_x) * (rdn(hfov) / 640)));
            precalc[i].sin_y_r = abs(sin((float)(i - camera_center_y) * (rdn(vfov_mono) / 400)));
            precalc[i].tan_x_r = abs(tan((float)(i - camera_center_x) * (rdn(hfov) / 640)));
            precalc[i].tan_y_r = abs(tan((float)(i - camera_center_y) * (rdn(vfov_mono) / 400)));
        }
        for (int i = 0; i < 400; i++)
        {
            float test = 0.01 * (((camera_center_x - c_rect_x) - i) * ((camera_center_x - c_rect_x) - i));
            precalc[i].side_filter = c_dis_dep_i(test);
            /*if (i%10 == 0){
            printf(" %d test %.6f\n",i,test);
            printf(" i:%d xx:%d\n", i,precalc[i].side_filter);
            }*/
        }
    }

    // カメラの回転行列を計算（ラジアン単位で入力）
    Eigen::Matrix3f calculateRotationMatrix(float pitch, float roll, float yaw)
    {
        pitch = (1.505f - pitch);
        roll += 3.14f;
        Eigen::Matrix3f rotX;
        rotX << 1, 0, 0,
            0, cos(pitch), -sin(pitch),
            0, sin(pitch), cos(pitch);

        Eigen::Matrix3f rotY;
        rotY << cos(yaw), 0, sin(yaw),
            0, 1, 0,
            -sin(yaw), 0, cos(yaw);

        Eigen::Matrix3f rotZ;
        rotZ << cos(roll), -sin(roll), 0,
            sin(roll), cos(roll), 0,
            0, 0, 1;

        return rotY * rotX * rotZ;
    }

    // ワールド座標をスクリーン座標に変換する関数
    Point2f worldToScreen(Dimensions world_point)
    {
        //二つのカメラのピッチの違いで表示がブレるのを避けるためcamera_pitch_orgを利用
        Eigen::Matrix3f rotation_matrix = calculateRotationMatrix(camera_pitch_org, camera.roll, camera_yaw_org);
        int screen_width = 640;
        int screen_height = 400;

        // ビュー行列を計算
        Eigen::Matrix4f view_matrix = Eigen::Matrix4f::Identity();
        view_matrix.block<3, 3>(0, 0) = rotation_matrix;
        view_matrix.block<3, 1>(0, 3) = -rotation_matrix * camera.position;

        // 射影行列を計算
        float vertical_fov_rad = camera.vertical_fov * M_PI / 180.0f;
        float horizontal_fov_rad = camera.horizontal_fov * M_PI / 180.0f;
        float tan_half_vertical_fov = tan(vertical_fov_rad / 2.0f);
        float tan_half_horizontal_fov = tan(horizontal_fov_rad / 2.0f);

        Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Zero();
        projection_matrix(0, 0) = 1.0f / tan_half_horizontal_fov;
        projection_matrix(1, 1) = 1.0f / tan_half_vertical_fov;
        projection_matrix(2, 2) = -(camera.far + camera.near) / (camera.far - camera.near);
        projection_matrix(2, 3) = -(2.0f * camera.far * camera.near) / (camera.far - camera.near);
        projection_matrix(3, 2) = -1.0f;

        // ワールド座標をカメラ空間へ変換
        Eigen::Vector4f world_point_homogeneous(-world_point.w, world_point.h, world_point.d, 1.0f);
        Eigen::Vector4f camera_space_point = view_matrix * world_point_homogeneous;
        Eigen::Vector4f ndc_point = projection_matrix * camera_space_point;

        // NDCをスクリーン座標に変換
        if (ndc_point.w() != 0)
        {
            ndc_point /= ndc_point.w();
        }

        Point2f screen_point;
        screen_point.x = 640 - (ndc_point.x() * 0.5f + 0.5f) * screen_width;
        screen_point.y = (1.0f - (ndc_point.y() * 0.5f + 0.5f)) * screen_height;

        return screen_point;
    }

    // スクリーン座標をワールド座標に変換する関数
    Dimensions screenToWorld(Point2f screen_point, float depth)
    {
        int screen_width = 640;
        int screen_height = 400;
        float vertical_fov_rad = camera.vertical_fov * M_PI / 180.0f;
        float horizontal_fov_rad = camera.horizontal_fov * M_PI / 180.0f;
        float tan_half_vertical_fov = tan(vertical_fov_rad / 2.0f);
        float tan_half_horizontal_fov = tan(horizontal_fov_rad / 2.0f);

        // NDC座標に変換
        float ndc_x = (screen_point.x / screen_width) * 2.0f - 1.0f;
        float ndc_y = 1.0f - (screen_point.y / screen_height) * 2.0f;

        // カメラ空間での位置を計算
        Eigen::Vector4f camera_space_point;
        camera_space_point.x() = ndc_x * -depth * tan_half_horizontal_fov;
        camera_space_point.y() = ndc_y * -depth * tan_half_vertical_fov;
        camera_space_point.z() = depth;
        camera_space_point.w() = 1.0f;

        Eigen::Matrix3f rotation_matrix = calculateRotationMatrix(camera.pitch, -camera.roll, camera.yaw); //ロールは判定させる？
        Eigen::Matrix4f view_matrix = Eigen::Matrix4f::Identity();
        view_matrix.block<3, 3>(0, 0) = rotation_matrix;
        view_matrix.block<3, 1>(0, 3) = -rotation_matrix * Eigen::Vector3f(0.0f, c_camera_high_no_base, 0.0f);
        Eigen::Matrix4f inv_view_matrix = view_matrix.inverse();

        // カメラ空間からワールド空間に変換
        Eigen::Vector4f world_point_homogeneous = inv_view_matrix * camera_space_point;
        Eigen::Vector3f world_point(world_point_homogeneous.x(), world_point_homogeneous.y(), world_point_homogeneous.z());

        Dimensions dims;
        dims.w = world_point.x();
        dims.h = world_point.y();
        dims.d = world_point.z();
        return dims;
    }

    void base_target_add_set()
    {
        // ベース検知　基準座標
        float dx = 0.015;   //0.045
        float dy = 0.20;    //0.20
        float dx_c = 0.007; //0.01
        float dy_c = 0.030; //0.055
        float dy_b = 0.040; //0.045
        target_nw = rgb_mono_xy_change(camera_center_x + (-dx) * 640, camera_center_y + (dy)*400);
        target_ne = rgb_mono_xy_change(camera_center_x + (dx)*640, camera_center_y + (dy)*400);
        target_sw = rgb_mono_xy_change(camera_center_x + (-dx - dx_c) * 640, camera_center_y + (dy + dy_c) * 400);
        target_se = rgb_mono_xy_change(camera_center_x + (dx + dx_c) * 640, camera_center_y + (dy + dy_c) * 400);
        target_bs = rgb_mono_xy_change(camera_center_x, camera_center_y + (dy + dy_c + dy_b) * 400);
        // rgb_mono_xy_change()
    }
    /**
    * @brief ボールノイズテーブルを初期化する関数
    *
    * @details この関数は、ボールノイズテーブルを初期化します。
    * ボールノイズテーブルは、画像処理中に発生するノイズを格納するために使用されます。
    * 各ノイズは、x座標、y座標、深度を格納する構造体として定義されています。
    * 初期化後、テーブル内のすべてのノイズのx座標とy座標は0に設定され、深度は0に設定されます。
    *
    * @return なし
    */
    void ball_noise_clear()
    {
        for (int i_ballnoise = 0; i_ballnoise <= c_ball_noise_i_max; i_ballnoise++)
        {
            auto &noise = ballnoise[i_ballnoise];
            noise.x = 0;
            noise.y = 0;
            noise.depth = 0;
            noise.cnt_nodep = 0;
            noise.cnt = 0;
            noise.camera_n = 0;
            noise.time = 0;
        }
        LOG_DEBUG.printf("ボールノイズテーブルクリア");
    }

    // 外部関数: ボールノイズチェック
    bool checkBallNoise(const cv::Point &center_temp, int depth, int *noise_table_no, int camera_n)
    {
        if (test_measurement_mode)
            return (false);
        bool ballnoisecheck = false;     // ノイズが近くにあるかどうか
        bool ballnoisecheck_rec = false; // ノイズが近くにあるかどうか
        int x_yure = 3;                  // x座標の許容誤差
        int y_yure = 3;                  // y座標の許容誤差
        int kyori_yure = 1;              // depthの許容誤差
        bool isMatch = false;
        const int out_time = 5;

        for (int i_ballnoise = 0; i_ballnoise <= c_ball_noise_i_max; i_ballnoise++)
        {
            isMatch = false;
            auto &noise = ballnoise[i_ballnoise];
            if (noise.time == 0)
                continue;
            if ((noise.time + out_time) < get_sec())
            {
                noise.x = 0;
                noise.y = 0;
                noise.depth = 0;
                noise.cnt_nodep = 0;
                noise.cnt = 0;
                noise.camera_n = 0;
                LOG_DEBUG.printf("noiseテーブルクリア(noise#:%d(SEC:%d/%d))", i_ballnoise, noise.time, get_sec());
                noise.time = 0;
            }

            bool depthCheck = depth != 0 && noise.depth != 0                                                                   //
                              && isWithinRange(depth, noise.depth - kyori_yure, noise.depth + kyori_yure)                      // 深度が一致しているか
                              && isWithinRange(center_temp.x, noise.x - x_yure, noise.x + x_yure)                              //
                              && isWithinRange(center_temp.y, noise.y - y_yure, noise.y + y_yure);                             //座標一致
            bool isCoordinateCheck_ok = (depth != 0 && noise.depth == 0)                                                       //
                                        && isWithinRange(center_temp.x, noise.x - x_yure, noise.x + x_yure)                    //
                                        && isWithinRange(center_temp.y, noise.y - y_yure, noise.y + y_yure);                   //座標一致
            bool isCoordinateCheck_ok_nodep = (depth == 0)                                                                     //
                                              && isWithinRange(center_temp.x, noise.x - (x_yure + 1), noise.x + (x_yure + 1))  //
                                              && isWithinRange(center_temp.y, noise.y - (y_yure + 1), noise.y + (y_yure + 1)); //座標一致（nodep）

            int a = noise.cnt;
            int b = noise.cnt_nodep;

            // 深度 && 場所が一致 or 深度が未設定
            if (depthCheck)
            {
                if (noise.cnt < 20)
                    noise.cnt = noise.cnt + 1;
                isMatch = true;
            }
            else
            {
                if (isCoordinateCheck_ok)
                {
                    noise.cnt = noise.cnt + 1;
                    noise.depth = depth; // 深度更新
                    isMatch = true;
                    LOG_DEBUG.printf(" noiseテーブル深度更新(x:%d,y:%d,dep(%d),camera:#%d),noise#:%d,cnt:%d,nodep:%d", center_temp.x, center_temp.y, depth, camera_n, i_ballnoise, noise.cnt, noise.cnt_nodep);
                }
            }

            //場所のみ一致
            if (isCoordinateCheck_ok_nodep //
                //       && noise.camera_n == camera_n //
            )
            {
                if (noise.cnt_nodep < 20)
                    noise.cnt_nodep = noise.cnt_nodep + 1;
                isMatch = true;
            }

            if (isMatch)
            {
                ballnoisecheck_rec = true;                                                        // 新規に記録しない
                noise.time = get_sec();                                                           //ノイズ検知時間の更新
                LOG_VERBOSE_IF((a != noise.cnt || b != noise.cnt_nodep) && !test_measurement_mode //
                               && a < 10 && b < 10                                                //
                               )                                                                  //
                    .printf(" noise_c_カウントアップ(x:%d,y:%d,noise_tbl_dep:%d,dep:%d),camera:#%d),noise#:%d,cnt:%d->%d,cnt_nodep:%d->%d", center_temp.x, center_temp.y, noise.depth, depth, camera_n, i_ballnoise, a, noise.cnt, b, noise.cnt_nodep);
                //画面中心部はノイズチェックしきい値を緩和する
                if (                                                                                                                         //
                    ((!isWithinRange(center_temp.x, 320 - c_no_noize_area, 320 + c_no_noize_area) && !isWithinRange(center_temp.y, 50, 380)) //
                     && ((noise.cnt > 1) || (noise.cnt_nodep > 1)))                                                                          //
                    || ((noise.cnt > 5) || (noise.cnt_nodep > 4))                                                                            //
                )
                {
                    ballnoisecheck = true; // ノイズエラー
                    LOG_DEBUG_IF(noise.cnt_nodep < 10 && !test_measurement_mode).printf(" noise_c_エラー(x:%d,y:%d,dep(%d),camera:#%d),noise#:%d,cnt:%d,nodep:%d", center_temp.x, center_temp.y, noise.depth, camera_n, i_ballnoise, noise.cnt, noise.cnt_nodep);
                    *noise_table_no = i_ballnoise;
                    break;
                }
                else
                {
                    ballnoisecheck = false;
                }
            }
        }

        if (!ballnoisecheck_rec)
        {
            int oldest_index = 0;
            uint64_t oldest_time = UINT64_MAX;
            for (int i_ballnoise = 0; i_ballnoise <= c_ball_noise_i_max; i_ballnoise++)
            {
                auto &noise = ballnoise[i_ballnoise];

                // 空いているテーブルを探す
                if (noise.time == 0)
                {
                    noise = {center_temp.x, center_temp.y, depth, 0, 1, camera_n, get_sec()}; // 新しい位置を記録
                    *noise_table_no = i_ballnoise;
                    LOG_DEBUG.printf(" noiseテーブル登録(x:%d,y:%d,dep(%d),camera:#%d),noise#:%d,cnt:%d,nodep:%d", center_temp.x, center_temp.y, depth, camera_n, i_ballnoise, noise.cnt, noise.cnt_nodep);
                    return false;
                }

                // 最も古いレコードを探す
                if (noise.time < oldest_time && noise.time != 0) //初期状態は対象にしない
                {
                    oldest_time = noise.time;
                    oldest_index = i_ballnoise;
                }
            }

            // 最も古いレコードに上書き
            auto &oldest_noise = ballnoise[oldest_index];
            oldest_noise = {center_temp.x, center_temp.y, depth, 0, 1, camera_n, get_sec()}; // 新しい位置を記録
            LOG_DEBUG.printf(" noiseテーブル上書き NEW(x:%d,y:%d,dep(%d),camera:#%d),noise#:%d,cnt:%d,nodep:%d,time:%d", center_temp.x, center_temp.y, depth, camera_n, oldest_index, oldest_noise.cnt, oldest_noise.cnt_nodep, oldest_noise.time);
            *noise_table_no = oldest_index;
            return false;
        }

        return ballnoisecheck;
    }

    /**
    * @brief 度数からラジアンに変換する関数
    *
    * @param deg  変換する度数
    *
    * @return ラジアンに変換された値
    *
    * @details この関数は、度数をラジアンに変換します。
    * ラジアンは、円の周長を360度で表した単位です。
    * 変換には、度数を3.14で割り、その結果を180で乗算します。
    */
    float
    rdn(float deg)
    {
        return (deg * 3.14 / 180);
    }

    /**
    * @brief ラジアンから度数に変換する関数
    *
    * @param rad  変換するラジアン
    *
    * @return 度数に変換された値
    *
    * @details この関数は、ラジアンを度数に変換します。
    * ラジアンは、円の周長を360度で表した単位です。
    * 変換には、ラジアンを180で乗算し、その結果を3.14で割ります。
    */
    float ardn(float rad)
    {
        return (rad * 180.0 / 3.14);
    }

    //モノ画像とカラー画像の座標、縮尺変換
    /**
     * Converts the given (x, y) coordinates from the RGB image to the corresponding coordinates in the mono image.
     *
     * @param x The x-coordinate in the RGB image.
     * @param y The y-coordinate in the RGB image.
     * @return The converted (x, y) coordinates in the mono image.
     */
    Point2i rgb_mono_xy_change(int x, int y)
    {
        int x_r = (float)x / 640 * (float)C_monorgb_hi_x + c_bias_x;
        int y_r = (float)y / 400 * (float)C_monorgb_hi_y + c_bias_y;
        return (Point2i(x_r, y_r));
        //return (Point2i(x,y));
    }
    Point2i rgb_mono_xy_change(Point2f ball_point_2f)
    {
        return (rgb_mono_xy_change((int)ball_point_2f.x, (int)ball_point_2f.y));
    }
    Point2i rgb_mono_xy_change(Point2i ball_point_2i)
    {
        return (rgb_mono_xy_change(ball_point_2i.x, ball_point_2i.y));
    }

    /**
    * @brief 線分(p1,p2)と直線(p3,p4)の交点を求める
    *
    * @param p1  線分の1点
    * @param p2  線分の2点
    * @param p3  直線の1点
    * @param p4  直線の2点
    *
    * @return 交点の座標を表すPoint構造体
    *
    * @details この関数は、2次元空間での線分と直線の交点を求める。
    * 平行な場合は、無限遠点が交点として返される。
    */
    Point crossPoint(
        const Point &p1,
        const Point &p2,
        const Point &p3,
        const Point &p4)
    {
        Point ap1; // 交点

        double dev = (p2.y - p1.y) * (p4.x - p3.x) - (p2.x - p1.x) * (p4.y - p3.y);
        if (!dev)
        {
            ap1.x = INFINITY;
            ap1.y = INFINITY;
            return ap1;
        }

        double d1, d2;

        d1 = (p3.y * p4.x - p3.x * p4.y);
        d2 = (p1.y * p2.x - p1.x * p2.y);

        ap1.x = d1 * (p2.x - p1.x) - d2 * (p4.x - p3.x);
        ap1.x /= dev;
        ap1.y = d1 * (p2.y - p1.y) - d2 * (p4.y - p3.y);
        ap1.y /= dev;

        return ap1;
    }
    // 座標を表す構造体
    struct Point_angle
    {
        double x;
        double y;
    };

    /**
    * @brief ベクトルを表す構造体
    *
    * @details この構造体は、2次元空間でのベクトルを表します。
    * 2次元空間でのベクトルは、x軸とy軸に沿った2つの成分で表されます。
    * この構造体には、x成分とy成分を表す2つのdouble型のメンバ変数が含まれています。
    */
    struct Vector_angle
    {
        double x; ///< x成分
        double y; ///< y成分
    };
    /**
    * @brief ベクトルのなす角を計算する関数。
    *
    * @param p1  点1の座標。
    * @param p2  点2の座標。
    * @param p3  点3の座標。
    *
    * @return 点1と点2を結ぶベクトルと点2と点3を結ぶベクトルとのなす角の大きさをラジアンで返す。
    *
    * @details この関数は、点1と点2を結ぶベクトルと点2と点3を結ぶベクトルとのなす角の大きさをラジアンで計算する。
    * 点1と点2を結ぶベクトルと点2と点3を結ぶベクトルとのなす角は、点1から点2へのベクトルと点1から点3へのベクトルとのなす角である。
    * ラジアンで表現され、0から360の範囲に収まる。
    */
    double calculateAngle(Point_angle p1, Point_angle p2, Point_angle p3)
    {
        double angleInRadians = 0;
        double atan_angle_v1 = 0;
        double atan_angle_v3 = 0;

        // ベクトルを計算
        float bunshi, bunbo;
        bunshi = p2.y - p1.y;
        bunbo = p2.x - p1.x;
        bunbo = (bunbo == 0) ? 1 : bunbo;
        atan_angle_v1 = ardn(atan(bunshi / bunbo)); // ラディアン→度数
        atan_angle_v1 = ((bunbo < 0) && (bunshi >= 0)) ? (180 + atan_angle_v1) : atan_angle_v1;
        atan_angle_v1 = ((bunbo < 0) && (bunshi < 0)) ? (-180 + atan_angle_v1) : atan_angle_v1;

        bunshi = p2.y - p3.y;
        bunbo = p2.x - p3.x;
        bunbo = (bunbo == 0) ? 1 : bunbo;
        atan_angle_v3 = ardn(atan(bunshi / bunbo));
        atan_angle_v3 = ((bunbo < 0) && (bunshi >= 0)) ? (180 + atan_angle_v3) : atan_angle_v3;
        atan_angle_v3 = ((bunbo < 0) && (bunshi < 0)) ? (-180 + atan_angle_v3) : atan_angle_v3;

        // アークコサインを用いて角度を計算（ラジアン）
        angleInRadians = abs(atan_angle_v1 - atan_angle_v3);
        angleInRadians = (angleInRadians > 180) ? (360 - angleInRadians) : angleInRadians;
        return angleInRadians;
    }
    /**
    * @brief テンプレートマッチングにより、画像からテンプレートを探し、その位置を特定する関数。
    *
    * @param img  テンプレートの探索対象となる画像。8ビットまたは32ビットの浮動小数点型。
    * @param tmp  探索されるテンプレート。探索対象となる画像以下のサイズで、同じデータ型でなければならない。
    * @param val  テンプレートマッチングの結果のスコア。
    *
    * @return  テンプレートが見つかった位置を表すPoint構造体。
    *
    * @details この関数は、テンプレートマッチングアルゴリズムを用いて、与えられた画像からテンプレートを探し、その位置を特定します。
    * スコアは、テンプレートが見つかった位置の類似度を表します。
    * スコアが大きいほど、テンプレートが見つかった位置がより類似していることを意味します。
    * スコアが0の場合、テンプレートは見つかりませんでした。
    */
    Point tmpmat(Mat *img, Mat *tmp, double *val)
    {
        Mat result;
        // テンプレートと，それに重なった画像領域とを比較
        cv::matchTemplate(
            *img,                // テンプレートの探索対象となる画像．8ビットまたは32ビットの浮動小数点型．
            *tmp,                // 探索されるテンプレート．探索対象となる画像以下のサイズで，同じデータ型でなければならない
            result,              // 比較結果のマップ
            cv::TM_CCOEFF_NORMED // 比較手法
            // cv::TM_SQDIFF // 比較手法(速い)
        );

        // double val;
        cv::Point pt;

        // 配列全体あるいは部分配列に対する，大域的最小値
        cv::minMaxLoc(result, val, nullptr, &pt, nullptr);
        // cv::minMaxLoc(result, nullptr, val,nullptr,&pt);
        *val = abs(*val);
        if ((pt.x == 0) || (pt.y == 0))
            *val = 0;

        return (pt);
    }

    /**
    * 画像からベースを特定色で2値化する関数。
    *
    * @param in_BGR_img       入力画像（BGRカラースペース）
    * @param out_img          出力画像（2値化画像）
    * @param best_v_filter    しきい値（HSVのV値）
    *
    * @return void
    */
    void base_Binarization(Mat *in_BGR_img, Mat *out_img, int best_v_filter)
    {
        std::vector<cv::Mat> planes;
        Mat gray_img, gray_img_tmp2;
        // カラーチャンネルに分離
        vector<Mat> channels;
        split(*in_BGR_img, channels);

        // 各チャンネルに対してヒストグラム平坦化
        for (int i = 0; i < 3; i++)
        {
            equalizeHist(channels[i], channels[i]);
        }

        // 平坦化後のチャンネルを結合
        Mat dst;
        merge(channels, dst);
        // ２値化
        cvtColor(dst, gray_img, COLOR_BGR2HSV); // HSVに変換

        switch (base_detection_maker)
        {
        case 0:
        {
            // 彩度判定
            // Channel毎に分解
            cv::split(gray_img, planes);
            // bitwise_not(planes[1], gray_img_tmp2);
            //  addWeighted(planes[2], 1.0, gray_img_tmp2, 0.0, 0, gray_img_tmp3);

            //  ガウシアンぼかしを適用して平滑化
            cv::GaussianBlur(planes[2], gray_img_tmp2, cv::Size(1, 1), 0);
            // 二値化
            threshold(gray_img_tmp2, *out_img, best_v_filter, 255, cv::THRESH_BINARY);
        }
        break;

        case 1:
        {
            // 特定色判定
            // 抽出する色の範囲を設定（例: 緑色）
            /*  赤色のHue	0～60 300～360[度]  0～30 150～179
                緑色のHue	60～189[度]         30～90
                青色のHue	180～300[度]        90～150
                */
            int h_value;
            h_value = best_v_filter + 20;
            if (h_value >= 255)
                h_value -= 255;
            // best_v_filter = 60;
            // h_value = 90;
            cv::Scalar lowerBound(best_v_filter - 10, 80, 10); // 下限値 (H, S, V)
            cv::Scalar upperBound(h_value, 255, 255);          // 上限値 (H, S, V)
            // 指定した色範囲内のピクセルを抽出
            cv::inRange(gray_img, lowerBound, upperBound, *out_img);
            //
        }
        break;
        case 2:
        {
            // 特定色判定
            // 抽出する色の範囲を設定（例: 緑色）
            /*  赤色のHue	0～60 300～360[度]  0～30 150～179
                緑色のHue	60～189[度]         30～90
                青色のHue	180～300[度]        90～150
                */
            int h_value2;
            h_value2 = best_v_filter + 15;
            if (h_value2 >= 255)
                h_value2 -= 255;
            // best_v_filter = 60;
            // h_value = 90;
            cv::Scalar lowerBound2(best_v_filter - 15, 60, 30); // 下限値 (H, S, V)
            cv::Scalar upperBound2(h_value2, 255, 255);         // 上限値 (H, S, V)
            // 指定した色範囲内のピクセルを抽出
            cv::inRange(gray_img, lowerBound2, upperBound2, gray_img_tmp2);
            bitwise_not(gray_img_tmp2, *out_img);
            //
        }
        break;
        /*
このコードは、与えられた画像を処理して、5つの領域に分割し、それぞれの領域に対して適切なしきい値を求めています。具体的には、与えられた画像をグレースケールに変換し、5つの領域に分割します。そして、各領域に対してしきい値を変化させながら二値化を行い、最適なしきい値で画像を二値化します。
このコードの中で使われている関数や変数について説明します。
Mat: OpenCVライブラリで提供される行列データ型です。画像データを格納するために使用されます。
cv::split(): 与えられた画像をチャンネルごとに分割します。このコードでは、グレースケール画像をチャンネルごとに分割しています。
cv::GaussianBlur(): 与えられた画像にガウシアンフィルタを適用します。このコードでは、特定の条件下でチャンネル2に対してガウシアンフィルタを適用しています。
bitwise_not(): 与えられた画像の各ピクセルのビットを反転させます。このコードでは、チャンネル0の画像を反転させています。
addWeighted(): 2つの画像を重み付けして合成します。このコードでは、チャンネル2の画像と反転させたチャンネル0の画像を合成しています。
cv::Rect(): 与えられた画像の領域を矩形で指定します。このコードでは、グレースケール画像を5つの領域に分割するために使用されています。
threshold(): 与えられた画像を二値化します。このコードでは、各領域に対してしきい値を変化させながら二値化を行っています。
countNonZero(): 与えられた画像内の非ゼロのピクセル数を数えます。このコードでは、特定の領域内の非ゼロのピクセル数を数えています。
paste(): 与えられた画像を別の画像の指定された位置に貼り付けます。このコードでは、二値化された画像を5つの領域に貼り付けています。
このコードは、与えられた画像を5つの領域に分割し、それぞれの領域に対して適切なしきい値を求めて二値化する処理を行っています。*/
        case 3:
        {
            Mat nw_tmp_img, ne_tmp_img, sw_tmp_img, se_tmp_img, b_tmp_img, tmp_img;
            int gray_img_hx = (float)gray_img.cols / 2;
            int gray_img_hy = (float)gray_img.rows / 3;
            int gray_img_b = (float)gray_img.rows / 5;
            // printf("xy %d,%d", gray_img_hx, gray_img_hy);
            cv::split(gray_img, planes);
            if (base_detection_maker == 3)
                cv::GaussianBlur(planes[2], gray_img, cv::Size(1, 1), 0);
            if (base_detection_maker == 4)
            {
                bitwise_not(planes[0], gray_img_tmp2);
                addWeighted(planes[2], 0.0, gray_img_tmp2, 1.0, 0, gray_img);
                // cv::GaussianBlur(gray_img_tmp3, gray_img, cv::Size(1, 1), 0);
            }
            // 画面分割　ベースの５つの角判定のため、５つのエリアに分けて適正なしきい値を求める
            // cv::Rect roi(x, y, width, height);
            Mat nw_gray = gray_img(Rect(0, 0, gray_img_hx, gray_img_hy));
            Mat ne_gray = gray_img(Rect(gray_img_hx, 0, gray_img_hx, gray_img_hy));
            Mat sw_gray = gray_img(Rect(0, gray_img_hy, gray_img_hx, gray_img_hy));
            Mat se_gray = gray_img(Rect(gray_img_hx, gray_img_hy, gray_img_hx, gray_img_hy));
            Mat b_gray = gray_img(Rect(0, gray_img.rows - gray_img_b, gray_img.cols, gray_img_b));
            for (int i = 0; i < 256; i++)
            {
                threshold(nw_gray, nw_tmp_img, 255 - i, 255, cv::THRESH_BINARY);
                int dot_suu = countNonZero(nw_tmp_img(Rect((nw_tmp_img.cols - 5), (nw_tmp_img.rows - 5), 5, 5)));
                if (dot_suu > 24)
                {
                    break;
                }
            }
            for (int i = 0; i < 256; i++)
            {
                threshold(ne_gray, ne_tmp_img, 255 - i, 255, cv::THRESH_BINARY);
                int dot_suu = countNonZero(ne_tmp_img(Rect(5, (ne_tmp_img.rows - 5), 5, 5)));
                if (dot_suu > 24)
                    break;
            }
            for (int i = 0; i < 256; i++)
            {
                threshold(sw_gray, sw_tmp_img, 255 - i, 255, cv::THRESH_BINARY);
                int dot_suu = countNonZero(sw_tmp_img(Rect((sw_tmp_img.cols - 10), (sw_tmp_img.rows - 10), 5, 5)));
                if (dot_suu > 24)
                    break;
            }
            for (int i = 0; i < 256; i++)
            {
                threshold(se_gray, se_tmp_img, 255 - i, 255, cv::THRESH_BINARY);
                int dot_suu = countNonZero(se_tmp_img(Rect(10, (se_tmp_img.rows - 10), 5, 5)));
                if (dot_suu > 24)
                    break;
            }
            for (int i = 0; i < 256; i++)
            {
                threshold(b_gray, b_tmp_img, 255 - i, 255, cv::THRESH_BINARY);
                int dot_suu = countNonZero(b_tmp_img(Rect((b_tmp_img.cols / 2 - 2), 0, 5, 5)));
                if (dot_suu > 24)
                    break;
            }
            // 二値化
            threshold(gray_img, *out_img, best_v_filter, 255, cv::THRESH_BINARY);
            paste(*out_img, nw_tmp_img, 0, 0);
            paste(*out_img, ne_tmp_img, gray_img_hx, 0);
            paste(*out_img, sw_tmp_img, 0, gray_img_hy);
            paste(*out_img, se_tmp_img, gray_img_hx, gray_img_hy);
            paste(*out_img, b_tmp_img, 0, b_tmp_img.rows - gray_img_b);
        }
        break;
        case 4:
        {
            // 彩度判定
            // Channel毎に分解
            cv::split(gray_img, planes);
            bitwise_not(planes[1], gray_img_tmp2);
            //  addWeighted(planes[2], 1.0, gray_img_tmp2, 0.0, 0, gray_img_tmp3);

            //  ガウシアンぼかしを適用して平滑化
            // cv::GaussianBlur(planes[2], gray_img_tmp2, cv::Size(1, 1), 0);
            // 二値化
            threshold(gray_img_tmp2, *out_img, init_i_old, 255, cv::THRESH_BINARY);
        }
        break;
            /**/

        default:
            break;
        }

        // best_v_filter = (best_v_filter%2 == 1)? best_v_filter:best_v_filter+1;
        // adaptiveThreshold(gray_img_tmp3, bin_img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, best_v_filter, 5);

        // Cannyエッジ検出
        // cv::Mat edges;
        // cv::Canny(bin_img, bin_img, 50, 150);
    }

    /** 距離変換
     * @param change distance(→cm)
     * @param int dep_i
     * @return
     * @sa
     */
    float c_dis_cm(int dep_i)
    {
        if (dep_i != 0)
        {
            return (float)(fov / dep_i);
        }
        else
        {
            return 0;
        }
    }

    /** 距離変換(画角付き)
     * @param change distance(→cm)
     * @param int dep_i
     * @return
     * @sa
     */
    float c_dis_cm(int dep_i, int iy)
    {
        if (dep_i != 0)
        {
            return ((float)(fov / dep_i) / (precalc[iy].cos_y_r));
        }
        else
        {
            return 0;
        }
    }

    /** 距離変換
     * @param change distance(→dep)
     * @param int cm
     * @return
     * @sa
     */
    int c_dis_dep_i(float cm)
    {
        if (cm != 0)
        {
            // 距離から深度iへの変換は400pを標準とする
            return ((int)((float)fov / cm));
        }
        else
        {
            return 255;
        }
    }

    /**
 * @brief 画像から計算されたボールの高さからy軸の角度を求める関数
 *
 * @param h 画像上のボールの垂直位置
 *
 * @return y軸の角度
 *
 * @details この関数は、与えられた画像上のボールの垂直位置から、y軸の角度を計算します。
 * 計算には、カメラのピッチとカメラの視野が使用されます。
 * 計算されたy軸の角度は、関数の戻り値として返されます。
 * ただし、hが0の場合、0を返します。
 */
    float y_r_rgb(int h)
    {
        return (rdn((float)(h - camera_center_y) * (vfov_rgb / 400)));
    }
    float y_r_mono(int h) // 画面下方向がプラス、上方向がマイナス
    {
        return (rdn((float)(h - camera_center_y) * (vfov_mono / 400)));
    }

    // ベース中心を基点としたボールの位置を計算
    Dimensions calculateBallPositionByBase(float length, Point2f ball)
    {
        Dimensions dims;
        Point point_tmp;

        ball = zahyou_kaiten_xy(ball, jyuushin, -base_roll); //ベース傾き角度に基づき、回転
        Point(ball.x, length) = zahyou_kaiten_xy(Point(ball.x, length), Point(0, length), -base_yaw);

        float x_r = (ball.x - jyuushin.x) * (rdn(hfov) / 640);
        float y_r = (ball.y - jyuushin.y) * (rdn(vfov_mono) / 400);
        float camera_ball_x_r = (ball.x - camera_center_x) * (rdn(hfov) / 640);
        float camera_ball_y_r = (ball.y - camera_center_y) * (rdn(vfov_mono) / 400);
        float y_l = abs(length * cos(y_r) * cos(base_pitch + y_r));
        // length * cos(y_r) * cos(camera.pitch - y_r)

        //ベース傾き角度に基づき、回転
        // (参考) camera_high = (c_camera_high == 0) ? length * cos(y_r) * cos(camera.pitch - y_r) : c_camera_high;                                    // カメラの高さを計算
        // dims.h = camera_high - cos(base_pitch + y_r) * length * cos(camera_ball_y_r) * cos(camera_ball_x_r); // 高さとして設定
        // dims.h = camera_high - length * cos(camera.pitch - camera_ball_y_r); // 高さとして設定
        dims.w = tan(x_r) * length / cos(abs(y_r)) + zone_c_dis_w_cm;                           //(y_l * cos(y_r)); // x座標を横位置として設定
        dims.h = camera_high - length / cos(abs(y_r)) * cos(base_pitch - y_r) + ball_high_bias; // 高さとして設定
        dims.d = sin(abs(base_pitch - y_r)) * length / cos(abs(y_r));                           // / cos(camera_ball_y_r); // ベース中心からボールまでの水平距離を計算
        // printf(" l:%.1f,%.1f,%.1f\n", length, base_pitch, y_r);

        /*
        point_tmp = zahyou_kaiten_xy(Point(dims.w,dims.d),Point(0,dims.d),-base_yaw); //ベース向き角度に基づき、回転
        dims.w=point_tmp.x; //横位置として設定
        dims.d=point_tmp.y; //ボールまでの水平距離として設定
        point_tmp = zahyou_kaiten_xy(Point(dims.w,dims.h),Point(0,0),base_roll); //ベース傾き角度に基づき、回転
        dims.w=point_tmp.x; //横位置として設定
        dims.h=point_tmp.y; //高さとして設定
        */

        return dims;
    }

    Point2i worldToScreen()
    {
        /*
        LOG_DEBUG.printf("test_ball position w,h,d %.0f,%.0f,%.0f",ball_position.w,ball_position.h,ball_position.d);
        LOG_DEBUG.printf("test_ball position x,y %d,%d",worldToScreen(ball_position).x,worldToScreen(ball_position).y);
        LOG_DEBUG.printf("test_ball position x,y %.0f,%.0f", center_temp_cm.x,center_temp_cm.y);
        LOG_DEBUG.printf("test_ball position w,h,d %.0f,%.0f,%.0f",ballCenter_cm_x,ballCenter_cm_y,ball_dep_cm);
        */
        return (worldToScreen(ball_position));
    }
    Point2i worldToScreen(float w, float h, float d)
    {
        return (worldToScreen(Dimensions(w, h, d)));
    }
    /*このコードは、画像からボールの高さと水平距離を計算する関数calc_h_d_ballを含んでいます。関数の引数として、カメラからボールまでの直線距離、ボールまたはベースのx座標とy座標、ベースの中心からボールまでの水平距離、地面からボールまでの高さ、カメラの中心からボールまでの水平距離、キャリブレーションの計算フラグがあります。
この関数では、与えられたパラメータを使用して、ボールの位置を計算します。具体的には、与えられたx座標とy座標からカメラ中心との角度を計算し、それを使用してボールの高さと水平距離を求めます。さらに、カメラの高さやベースまでの直線距離が与えられている場合は、それらの値を使用して計算を行います。
関数内部では、三角関数や平方根などの数学的な計算が行われています。また、計算結果や中間結果をログに出力する部分もあります。
この関数は、カメラの視野角やキャリブレーションの設定に基づいて、ボールの位置を正確に計算するためのものです。関数の使い方や引数の意味については、関数のコメントに詳細が記載されています。
    // 画像から、ボールの高さ、水平距離を算出(Calculate the height and horizontal distance of the ball from the image)
    // (int 距離、int ベース中心からの水平距離、int 画像上の高さ,水平距離、高さ)  カメラ視野の７３度,５８度(横、縦)をラジアン変換（×π/180))
    //
    /**
    * 画像からボールの高さと水平距離を計算します。
    *
    * @param length           カメラからボールまでの直線距離。
    * @param x                ボール/ベースのx座標。
    * @param y                ボール/ベースのy座標。
    * @param h_distance       ベースの中心からボールまでの水平距離。
    * @param high             地面からボールまでの高さ。
    * @param width            カメラの中心からボールまでの水平距離。
    * @param cal              キャリブレーションの計算かどうかを示すフラグ。
    *
    * @return void
    */
    void calc_h_d_ball(float length, float x, float y, float *h_distance, float *high, float *width, bool cal)
    {
        float x_r; // x軸の角度
        float y_r; // y軸の角度
        float x_l;
        float y_l;
        float xy_r;
        float xy_l;
        float world_x;
        float world_y;
        float temp_x;
        float temp_y;
        // float base_pitch;
        *high = (*high < 0) ? 0 : *high;
        x_r = 0;
        y_r = 0;
        x_r = (x - camera_center_x) * (rdn(hfov) / 640);
        y_r = (y - camera_center_y) * (rdn(vfov_mono) / 400);

        y_l = abs(length / cos(y_r));
        // y_l=abs(length * cos(y_r));
        // y_l=length;
        // base_pitch = acos((float)base_h / base_w);
        /*
        base_pitch = rdn(90) - base_pitch - y_r;

        camera_high = (c_camera_high == 0) ? length * sin(rdn(90) - base_pitch) : c_camera_high;
        base_pitch = (c_camera_high == 0) ? acos(camera_high / length) : base_pitch;
        base_pitch = (c_horizontalDistanceToBase_cm == 0) ? base_pitch : (atan(c_horizontalDistanceToBase_cm / camera_high));
        */

        if (cal)
        {
            // カメラピッチ角度計算
            // input:
            // length           直線距離
            // straightLineDistanceToBase_cm      ベースまでの直線距離
            // horizontalDistanceToBase_cm    ベースまでの水平距離
            // base_pitch       ベースピッチ

            // c_horizontalDistanceToBase_cm  ベースまでの水平距離(固定)
            // c_camera_high    カメラ高さ(固定)
            // c_camera_pitch   カメラピッチ(固定)

            // ＜すべて自動算出＞
            // in:      ベースまでの直線距離、ベースピッチ（←撮影環境に左右されるので信頼が低い）、カメラ中心とベース中心の角度(y_r)
            // out:     カメラ高さ、カメラピッチ、ベースまでの水平距離
            // order:   カメラ高さ = sin(ベースピッチ) * ベースまでの直線距離
            //          カメラピッチ = ベースピッチ + カメラ中心とベース中心の角度(y_r)
            //          ベースまでの水平距離 = sqrl(ベースまでの直線距離^2 - カメラ高さ^2)

            // ＜カメラ高さあり＞
            // in:      カメラ高さ(固定値)、ベースまでの直線距離、カメラ中心とベース中心の角度(y_r)
            // out:     カメラピッチ、ベースまでの水平距離
            // order:   ベースピッチ = acos(カメラ高さ/ベースまでの直線距離)
            //          カメラピッチ = ベースピッチ + カメラ中心とベース中心の角度(y_r)
            //          ベースまでの水平距離 = sqrl(ベースまでの直線距離^2 - カメラ高さ^2)

            // ＜カメラ高さあり/ベースまでの直線距離あり＞
            // in:      カメラ高さ(固定値)、ベースまでの水平距離(固定値)、カメラ中心とベース中心の角度(y_r)
            // out:     カメラピッチ、ベースまでの水平距離
            // order:   ベースピッチ = acos(カメラ高さ/ベースまでの直線距離)
            //          カメラピッチ = ベースピッチ + カメラ中心とベース中心の角度(y_r)
            //          ベースまでの直線距離 = sqrl(ベースまでの水平距離^2 + カメラ高さ^2)

            if (camera_p_home_dis || imu_use)
                base_pitch = rdn(90) - camera.pitch;   //2点距離からピッチ測定する場合（それ以外はコメントアウト）
            base_pitch = (rdn(90) - base_pitch - y_r); // ホームベースのピッチ角を計算
            // base_pitch = (rdn(90) - base_pitch - 0); // ホームベースのピッチ角を計算
            // camera_high = (c_camera_high == 0) ? length * sin(rdn(90) - base_pitch) : c_camera_high;                             // カメラの高さを計算
            if (!no_base_mode)
                camera_high = (c_camera_high == 0) ? length * cos(camera.pitch - y_r) : c_camera_high;                                                                                                              // カメラの高さを計算
            straightLineDistanceToBase_cm = (c_horizontalDistanceToBase_cm == 0) ? straightLineDistanceToBase_cm : sqrt(c_horizontalDistanceToBase_cm * c_horizontalDistanceToBase_cm + camera_high * camera_high); // ホームベースまでの直線距離を計算
            straightLineDistanceToBase_i = c_dis_dep_i(straightLineDistanceToBase_cm);

            // base_pitch = (c_camera_high == 0) ? acos(camera_high / length) : base_pitch; // コメントアウトされたピッチ角の計算
            // base_pitch = (c_horizontalDistanceToBase_cm == 0) ? base_pitch : (atan(c_horizontalDistanceToBase_cm / camera_high)); // コメントアウトされたピッチ角の計算
            // camera.pitch = (c_camera_pitch == 0) ? base_pitch + y_r : rdn(c_camera_pitch); // コメントアウトされたカメラピッチ角の計算
            // ↓2点距離からピッチ測定する場合はコメントアウト
            if ((!camera_p_home_dis && !imu_use) || ground_mode)
                camera.pitch = (c_camera_pitch == 0) ? rdn(90) - (base_pitch + 0) : rdn(c_camera_pitch);                                                                                                          // カメラピッチ角を計算
            horizontalDistanceToBase_cm = (c_horizontalDistanceToBase_cm == 0) ? sqrt(straightLineDistanceToBase_cm * straightLineDistanceToBase_cm - camera_high * camera_high) : c_horizontalDistanceToBase_cm; // 基本の距離を計算
            horizontalDistanceToBase_i = c_dis_dep_i(horizontalDistanceToBase_cm);
            LOG_VERBOSE.printf("horizontalDistanceToBase_cm:%.0f", horizontalDistanceToBase_cm);

            if ((c_camera_high != 0) && (c_horizontalDistanceToBase_cm != 0)) //カメラ高さ、ベースまでの水平距離が事前に与えられる場合
            {
                if (!imu_use)
                    camera.pitch = atan(c_horizontalDistanceToBase_cm / camera_high) + y_r; // カメラのピッチ角を再計算
                base_pitch = camera.pitch - y_r;                                            // 基本のピッチ角を再計算
            }

            // no_base_mode
            if (no_base_mode)
            {
                // camera_high = c_camera_high_no_base + camera_high_key_bias;
                horizontalDistanceToBase_cm = c_base_kyori_cm_no_base + base_kyori_cm_key_bias;
                LOG_VERBOSE.printf("horizontalDistanceToBase_cm:%.0f", horizontalDistanceToBase_cm);
                horizontalDistanceToBase_i = c_dis_dep_i(horizontalDistanceToBase_cm);
                straightLineDistanceToBase_cm = sqrt(horizontalDistanceToBase_cm * horizontalDistanceToBase_cm + camera_high * camera_high);
                straightLineDistanceToBase_i = c_dis_dep_i(straightLineDistanceToBase_cm);
                base_pitch = atan(horizontalDistanceToBase_cm / camera_high);
            }

            LOG_VERBOSE.printf("camera_pitch:%.1f,y_r:%.2f,y_l:%.0f", ardn(camera.pitch), ardn(y_r), y_l);
            LOG_VERBOSE.printf("base_length:%.1f,base_pitch:%.1f", length, ardn(base_pitch));
            LOG_VERBOSE.printf("camera_high:%.1f,straightLineDistanceToBase_cm:%.0f,horizontalDistanceToBase_cm:%.0f", camera_high, straightLineDistanceToBase_cm, horizontalDistanceToBase_cm);
        }

        world_x = sin(camera.pitch - y_r) * y_l;
        /*
        if (!cal)
        {
        printf(" aa:%0.f,%.0f\n",length,world_x);
        printf(" bb:%0.f\n",ardn(camera.pitch - y_r));
        }
        */
        world_y = camera_high - cos(camera.pitch - y_r) * y_l;
        *h_distance = world_x;
        *width = tan(x_r) * (length * cos(y_r));
        // *width = tan(x_r) * length;
        // if (!cal)         printf(" bb:%0.f\n", ardn(y_r));
        *high = world_y;
        *h_distance = ((c_horizontalDistanceToBase_cm != 0) && cal) ? c_horizontalDistanceToBase_cm : *h_distance;

        if (cal)
        {
            if (no_base_mode)
                *width = 0;

            // ログに距離情報を出力
            LOG_VERBOSE.printf("*h_distance:%.0f", *h_distance);

            // ベースまでの距離を設定（c_base_kyori_cmが0でない場合はそれを使用）
            horizontalDistanceToBase_cm = (c_horizontalDistanceToBase_cm == 0) ? horizontalDistanceToBase_cm : c_horizontalDistanceToBase_cm;
            LOG_VERBOSE.printf("horizontalDistanceToBase_cm:%.0f", horizontalDistanceToBase_cm);

            // ゾーン中心の高さと中心ずれの設定
            zone_c_dis_cm = *high;    // 地面からゾーン中心までの距離(cm)
            zone_c_dis_w_cm = *width; // 画面中心からゾーン中心までの距離(cm)

            // ベースの左右端からボール半径を考慮したゾーンの設定
            zone_r_cm = zone_c_dis_w_cm + c_zone_r_cm; // 右側ゾーン
            zone_l_cm = zone_c_dis_w_cm + c_zone_l_cm; // 左側ゾーン

            // ゾーンの距離とバイアスの計算
            float zone_kyori = horizontalDistanceToBase_cm * 1.00; // / sin(camera.pitch); // ベースの距離からのゾーン距離
            float zone_kyori_bias = 21.6;                          // * cos(1.57 - camera.pitch); // ピッチ角によるバイアス

            zone_right_front_hi = worldToScreen(zone_kyori_bias - 1.6, zone_hi_cm, zone_kyori + zone_kyori_bias);
            zone_right_front_low = worldToScreen(zone_kyori_bias - 1.6, zone_low_cm, zone_kyori + zone_kyori_bias);
            zone_right_middle_hi = worldToScreen(zone_kyori_bias - 1.6, zone_hi_cm, zone_kyori);
            zone_right_middle_low = worldToScreen(zone_kyori_bias - 1.6, zone_low_cm, zone_kyori);
            zone_left_front_hi = worldToScreen(-zone_kyori_bias + 1.6, zone_hi_cm, zone_kyori + zone_kyori_bias);
            zone_left_front_low = worldToScreen(-zone_kyori_bias + 1.6, zone_low_cm, zone_kyori + zone_kyori_bias);
            zone_left_middle_hi = worldToScreen(-zone_kyori_bias + 1.6, zone_hi_cm, zone_kyori);
            zone_left_middle_low = worldToScreen(-zone_kyori_bias + 1.6, zone_low_cm, zone_kyori);
            zone_base_hi = worldToScreen(0, zone_hi_cm, zone_kyori - zone_kyori_bias);
            zone_base_low = worldToScreen(0, zone_low_cm, zone_kyori - zone_kyori_bias);
            if (no_base_mode)
            {
                zone_right_front_gnd = worldToScreen(zone_kyori_bias, 0, zone_kyori + zone_kyori_bias);
                zone_right_middle_gnd = worldToScreen(zone_kyori_bias, 0, zone_kyori);
                zone_left_front_gnd = worldToScreen(-zone_kyori_bias, 0, zone_kyori + zone_kyori_bias);
                zone_left_middle_gnd = worldToScreen(-zone_kyori_bias, 0, zone_kyori);
                zone_base_gnd = worldToScreen(0, 0, zone_kyori - zone_kyori_bias);
            }
        }

        // LOG_DEBUG_IF(!cal && !test_measurement_mode).printf(" x_l,y_l %.1f,%.1f", x_l, y_l);
        // test_measurement_mode = false;cal=false;
        LOG_VERBOSE_IF(!cal && !test_measurement_mode).printf(" length:%.1f,c_pitch:%.1f", length, camera.pitch * 180 / 3.14);
        LOG_VERBOSE_IF(!cal && !test_measurement_mode).printf(" l:%d,x_r:%.1f,y_r:%.1f", (int)length, x_r * 180 / 3.14, y_r * 180 / 3.14);
        LOG_VERBOSE_IF(!cal && !test_measurement_mode).printf(" x:%d,y:%d", (int)x, (int)y);
        LOG_VERBOSE_IF(!cal && !test_measurement_mode).printf(" *h_distance:%.1f,*width:%.1f,*high:%.1f(camera_hi:%.0f)", *h_distance, *width, *high, camera_high);
    }

    /**
    * @brief  ボール深度判定
    *
    * @param check_zahyou 調べる座標
    * @param mask_image 深度マスク画像へのポインタ
    *
    * @return 調べた座標の深度値
    *
    * @details 与えられた座標の周囲の深度値を調べ、平均を計算して深度値を返します。
    * 調べる座標の周囲は3x3の領域で、深度値が基準深度の±50cmの範囲内のものだけを考慮します。
    * 調べる座標は640x400換算？です
    */
    float depth_measure(Point2i check_zahyou, Mat *mask_image, bool filter)
    {
        int dep_i;
        float dep_i_heikin = 0;
        int dep_cnt = 0;
        int dep_cnt_base = 0;
        uchar dep_tmp;

        dep_cnt_base = 0;
        dep_cnt = 0;
        dep_i_heikin = 0;
        int old_i = 0;

        // 調べる座標の周囲の3x3の領域でループ
        for (int iy = clamp(check_zahyou.y - 9, 0, mask_image->rows); iy <= clamp(check_zahyou.y + 9, 0, mask_image->rows); iy++)
        {
            for (int ix = clamp(check_zahyou.x - 9, 0, mask_image->cols); ix <= clamp(check_zahyou.x + 9, 0, mask_image->cols); ix++)
            {
                // 深度マスク画像から深度値を取得
                dep_tmp = mask_image->data[iy * mask_image->step + ix * mask_image->dims];

                if (filter)
                {
                    // 取得した深度値が基準深度の±50cmの範囲内なら、dep_i_heikinに加算
                    if (isWithinRange(c_dis_cm(dep_tmp), c_dis_cm(dep_base_kin) - 100, c_dis_cm(dep_base_en) + 100))
                    {
                        if (dep_tmp != 0)
                        {
                            dep_i_heikin += static_cast<int>(dep_tmp);
                            old_i = dep_tmp;
                            dep_cnt++;
                        }
                    }
                }
                else
                {
                    // フィルターなしの場合、すべての深度値を加算
                    if (dep_tmp != 0)
                    {
                        dep_i_heikin += static_cast<int>(dep_tmp);
                        old_i = dep_tmp;
                        dep_cnt++;
                    }
                }
                dep_cnt_base++;
            }
        }

        // dep_i = old_i;
        // 取得した深度値の平均を計算し、dep_iに代入
        if (dep_cnt > 0)
        {
            dep_i = (int)(dep_i_heikin / dep_cnt);
        }
        else
        {
            dep_i = 0;
        } //

        // 調べた座標の深度値を返す
        return (c_dis_cm(dep_i));
    }
    /**フィルターなし */
    float depth_measure(Point2i check_zahyou, Mat *mask_image)
    {
        return (depth_measure(check_zahyou, mask_image, true));
    }

    /**
    * @brief 脚関節の傾きをX-Y面とX-Z面で補正する関数。
    *
    * @param knee_center_temp 脚関節の元の座標。
    * @param knee_i 脚関節の深度値。
    * @param ret_i 脚関節の補正された深度値を格納するためのポインタ。
    *
    * @return 脚関節のX-Y面で補正された座標。
    */
    Point2i katamuki_hosei(Point2i knee_center_temp, int knee_i, int *ret_i)
    {
        Point2i knee_center_temp_cm;
        // X-Y平面の傾き補正
        // printf ("b:%.1f ",knee_center_temp_cm.x);
        knee_center_temp_cm.x = knee_center_temp.x * cos(base_roll) - knee_center_temp.y * sin(base_roll) + (base_temp.x - base_temp.x * cos(base_roll) + base_temp.y * sin(base_roll));
        knee_center_temp_cm.y = knee_center_temp.x * sin(base_roll) + knee_center_temp.y * cos(base_roll) + (base_temp.y - base_temp.x * sin(base_roll) - base_temp.y * cos(base_roll));
        // printf ("a:%.1f\n",knee_center_temp_cm.x);

        // X-Z平面の傾き補正
        // printf ("b:%.1f ",knee_center_temp_cm.x);
        knee_center_temp_cm.x = knee_center_temp_cm.x * cos(base_yaw) - c_dis_cm(knee_i) * sin(base_yaw) + (base_temp.x - base_temp.x * cos(base_yaw) + straightLineDistanceToBase_cm * sin(base_yaw));
        *ret_i = c_dis_dep_i(knee_center_temp_cm.x * sin(base_yaw) + c_dis_cm(knee_i) * cos(base_yaw) + (straightLineDistanceToBase_cm - base_temp.x * sin(base_yaw) - straightLineDistanceToBase_cm * cos(base_yaw)));
        // printf ("a:%.1f\n",knee_center_temp_cm.x);

        return (knee_center_temp_cm);
    }

    // ベース形状チェック
    bool base_check(Point front_l_temp, Point front_r_temp, Point back_l_temp, Point back_r_temp, Point base_temp)
    {
        bool ret = false;
        // ベースの高さを検出
        base_h = abs((base_temp.y - ((front_l_temp.y + front_r_temp.y) / 2)) / cos(base_roll));
        base_w = abs((back_r_temp.x - back_l_temp.x) / cos(base_roll));
        if (!(((base_w < c_base_w_max) && base_h < (cos(base_pitch - rdn(10)) * base_w * 1.3)) && (base_h > (cos(camera.pitch - rdn(10)) * base_w * 0.7))))
        //if (!((base_w < c_base_w_max) && (base_w > c_base_w_min) && (base_h < (cos(camera.pitch - rdn(10)) * base_w * 1.2)) && (base_h > (cos(camera.pitch - rdn(10)) * base_w * 0.8))))
        //if (!((base_w < c_base_w_max) && (base_w > c_base_w_min) && (base_h < c_base_h_max) && (base_h > c_base_h_min)))
        {
            LOG_DEBUG_IF(base_w != base_w_old).printf(" base size error(w,h) %d,%d", base_w, base_h);
            LOG_DEBUG_IF(base_w != base_w_old).printf("   cos(camera.pitch),base_pitch %.1f,%.1f", cos(camera.pitch - rdn(10)) * base_w, ardn(camera.pitch));
            base_w_old = base_w;
            return (false);
        }

        // ベースの形状判定
        // 角度を計算
        double front_l_angle = abs(calculateAngle({front_r_temp.x, front_r_temp.y}, {front_l_temp.x, front_l_temp.y}, {back_l_temp.x, back_l_temp.y}));
        double front_r_angle = abs(calculateAngle({front_l_temp.x, front_l_temp.y}, {front_r_temp.x, front_r_temp.y}, {back_r_temp.x, back_r_temp.y}));
        double base_angle = abs(calculateAngle({back_l_temp.x, back_l_temp.y}, {base_temp.x, base_temp.y}, {back_r_temp.x, back_r_temp.y}));
        int angle_mutch_cnt = 0;
        auto check_angle = [&](double base_roll, double min, double max, const char *label, auto... coords) {
            if (base_roll > min && base_roll < max)
            {
                angle_mutch_cnt++;
                return true;
            }
            else
            {
                LOG_VERBOSE.printf("%s:%.2f", label, base_roll);
                // (LOG_VERBOSE.printf(coords), ...);
                return false;
            }
        };

        bool front_l_angle_f = check_angle(front_l_angle, 70, 150, "angle_l", "front_l_temp.x:%d,%d", front_l_temp.x, front_l_temp.y, "front_r_temp.x:%d,%d", front_r_temp.x, front_r_temp.y, "back_l_temp.x:%d,%d", back_l_temp.x, back_l_temp.y);
        bool front_r_angle_f = check_angle(front_r_angle, 70, 150, "angle_r", "front_l_temp.x:%d,%d", front_l_temp.x, front_l_temp.y, "front_r_temp.x:%d,%d", front_r_temp.x, front_r_temp.y, "back_r_temp.x:%d,%d", back_r_temp.x, back_r_temp.y);
        bool base_angle_f = check_angle(base_angle, 70, 160, "base", "back_l_temp.x:%d,%d", back_l_temp.x, back_l_temp.y, "back_r_temp.x:%d,%d", back_r_temp.x, back_r_temp.y, "base_temp.x:%d,%d", base_temp.x, base_temp.y);

        ret = (angle_mutch_cnt >= 3);
        // ret=true;
        if (!ret)
        {
            LOG_VERBOSE.printf(" base error w,h:%d,%d", base_w, base_h);
        }
        return (ret);
    }
    /** *
    @brief 指定された中心点を基準に、与えられた点を指定された角度だけ回転させる関数 * *
    @param x 回転させる点のx座標 *
    @param y 回転させる点のy座標 *
    @param c_x 回転の中心点のx座標 *
    @param c_y 回転の中心点のy座標 *
    @param ang 回転角度（ラジアン） *
    @param r_x 回転後の点のx座標を格納するポインタ *
    @param r_y 回転後の点のy座標を格納するポインタ * *
    @details * この関数は、指定された点 (x, y) を中心点 (c_x, c_y) を基準にして、指定された角度 ang だけ回転させます。 * 回転後の座標は、ポインタ r_x および r_y に格納されます。 * *
    使用例: *
    @code * int x = 10, y = 20, c_x = 5, c_y = 5; * double ang = 3.14 / 4; // 45度 * int r_x, r_y; * calc_rotation(x, y, c_x, c_y, ang, &r_x, &r_y); *
    @endcode */
    void calc_rotation(float x, float y, float c_x, float c_y, double ang, float *r_x, float *r_y)
    {
        *r_x = (x - c_x) * cos(ang) - (y - c_y) * sin(ang) + c_x;
        *r_y = (x - c_x) * sin(ang) + (y - c_y) * cos(ang) + c_y;
    }
    Point2f calc_rotation(float x, float y, float c_x, float c_y, double ang)
    {
        return (Point((x - c_x) * cos(ang) - (y - c_y) * sin(ang) + c_x, (x - c_x) * sin(ang) + (y - c_y) * cos(ang) + c_y));
    }

    bool Base_calibration2(Mat *bgr_img, int *max_v) // 仕掛かり ベース判定でキャリブレーションしようと思う
    {
        if (no_base_mode)
        {
            LOG_VERBOSE.printf("no_base_mode(no calibration)");
            return (true);
        }
        if ((base_detection_maker == 3) || (base_detection_maker == 4))
            return (true);
        // if (base_detection_maker == 1)
        //      return (true);
        Mat clip_img;
        best_v_filter = *max_v;
        int v_filter_h, v_filter_l;
        int area_tmp;
        v_filter_h = 0, v_filter_l = 256;
        bool ret = false;
        int v_step = 3;
        std::vector<cv::Mat> planes;
        for (int v_filter = 255; v_filter > 0; v_filter = v_filter - 2)
        {
            LOG_DEBUG.printf("v val %d", v_filter);

            base_Binarization(bgr_img, &bin_img, v_filter);

            bool base_find_f_tmp;
            switch (base_detection_maker)
            {
            case 0:
                base_find_f_tmp = Base_find(&bin_img, *bgr_img, true);
                break;

            case 1:
                base_find_f_tmp = Base_find_maker(&bin_img, true);
                break;

            case 2:
                base_find_f_tmp = Base_find(&bin_img, *bgr_img, true);
                break;
            case 3:
            case 4:
                // base_find_f_tmp = Base_find(&bin_img, *bgr_img, true);
                base_find_f_tmp = true;
                v_filter = 140;
                break;
            default:
                break;
            }

            if (base_find_f_tmp)
            {
                if (v_filter_h < v_filter)
                {
                    v_filter_h = v_filter;
                }
                v_filter_l = v_filter;
                ret = true;
                jyuushin_tmp = jyuushin;
                area_tmp = area;
                v_step = 1;
                // v_filter = v_filter - 2;
                LOG_VERBOSE.printf("Find Base  L:%d,H:%d", (int)v_filter_l, v_filter_h);
                break;
            }
            LOG_VERBOSE.printf("value=%d/%d", (int)best_v_filter, (int)v_filter); // 変数の値も含めた表示したい文字列をchar型変数に格納
        }

        v_step = 10;
        for (int v_filter_r = 0; v_filter_r < v_filter_h; v_filter_r = v_filter_r + v_step)
        {
            LOG_VERBOSE.printf("v val %d", v_filter_r);
            base_Binarization(bgr_img, &bin_img, v_filter_r);

            bool base_find_f_tmp;
            switch (base_detection_maker)
            {
            case 0:
                base_find_f_tmp = Base_find(&bin_img, *bgr_img, true);
                break;
            case 1:
                base_find_f_tmp = Base_find_maker(&bin_img, true);
                break;
            case 2:
                base_find_f_tmp = Base_find(&bin_img, *bgr_img, true);
                break;
            case 3:
            case 4:
                // base_find_f_tmp = Base_find(&bin_img, *bgr_img, true);
                base_find_f_tmp = true;
                v_filter_r = 100;
                break;
            default:
                break;
            }

            //全く違う位置ならエラーとする
            if ((jyuushin_tmp.x + 5 < jyuushin.x) || (jyuushin_tmp.x - 5 > jyuushin.x) || (jyuushin_tmp.y + 5 < jyuushin.y) || (jyuushin_tmp.y - 5 > jyuushin.y) || (area_tmp + 100 < area) || (area_tmp - 100 > area))
            {
                base_find_f_tmp = false;
            }
            if (base_find_f_tmp)
            {
                v_filter_l = v_filter_r;
                v_step = 1;
                LOG_VERBOSE.printf("Find Base  L:%d,H:%d", (int)v_filter_l, v_filter_h);
                break;
            }
            else
            {
                v_filter_l = v_filter_h;
            }
        }

        // best_v_filter = v_filter_h - 3;
        best_v_filter = clamp(v_filter_l + (int)((float)(v_filter_h - v_filter_l) * 0.55), 1, 254);
        if (ret)
        {
            *max_v = (int)best_v_filter;
            def_v_filter = *max_v;
        }
        else
        {
            *max_v = def_v_filter;
        }
        LOG_VERBOSE.printf(" L:%d,H:%d", (int)v_filter_l, v_filter_h);
        LOG_VERBOSE_IF(ret).printf("v_filter value=%d", (int)*max_v);
        return ret;
    }

    /** *
    @brief ベースを検出する関数 * *
    @param bin_img バイナリ画像 *
    @param bgr_img カラー画像 *
    @param cal キャリブレーションフラグ *
    @return bool ベースが検出されたかどうか
    */
    bool Base_find(Mat *bin_img, Mat bgr_img, bool cal) // cal kyaribure-syonn ka douka
    {
        bool ret;
        ret = false;
        base_find_f = false;
        Mat clip_img;

        if (!no_base_mode)
        {
            if ((base_detection_maker != 3) && (base_detection_maker != 4))
            // if (true)
            {
                // S ベース 輪郭検出
                findContours(*bin_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                int i = 0;
                std::vector<cv::Point>::iterator app;
                int i_MAX = 0;
                int nw, ne, sw, se, tmp_back_l, tmp_back_r, tmp_front_l, tmp_front_r;
                int nw_min, ne_min, sw_min, se_min;
                int r_max, l_max;
                roiCnt = 0;
                for (auto contour = contours.begin(); contour != contours.end(); contour++)
                {
                    // 輪郭を直線近似する
                    cv::approxPolyDP(cv::Mat(*contour), approx, 0.01 * cv::arcLength(*contour, true), true);
                    // 近似の面積が一定以上なら取得
                    area = cv::contourArea(approx); // ベースの面積
                    leng = arcLength(approx, true); // ベースの輪郭長さ
                    LOG_VERBOSE_IF(area > c_base_area_max).printf("BASE size (area:%d,leng:%d)", (int)area, (int)leng);
                    if ((area > c_base_area_min) && (area < c_base_area_max)  //
                        && (leng < c_base_len_max) && (leng > c_base_len_min) //
                    )
                    // best ha 2500 220 kurai
                    {
                        if ((approx.size() <= 150) && (approx.size() >= 5) && (moments(approx).m00 != 0))
                        {
                            std::vector<cv::Point> approx2;
                            app = approx.begin();
                            i = 0;
                            i_MAX = 0;
                            nw = 0;
                            ne = 0, sw = 0, se = 0, tmp_back_l = 0, tmp_back_r = 0, tmp_front_l = 0, tmp_front_r = 0;
                            nw_min = 99999, ne_min = 99999, sw_min = 99999, se_min = 99999;
                            r_max = 0;
                            l_max = 9999;
                            base_temp.y = 0;
                            for (app; app != approx.end(); app++)
                            {
                                app[0].x = app[0].x + c_rect_x;      //バイアス座標を加えて、640X400に変換
                                app[0].y = app[0].y + c_rect_zone_y; //バイアス座標を加えて、640X400に変換
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

                                nw = app[0].x + app[0].y * 1.3;
                                se = (640 - app[0].x) + ((float)(400 - app[0].y) / 3);
                                ne = (640 - app[0].x) + app[0].y * 1.3;
                                sw = app[0].x + ((float)(400 - app[0].y) / 3);
                                if (nw_min > nw)
                                {
                                    nw_min = nw;
                                    front_l_temp = app[0];
                                }
                                if (se_min > se)
                                {
                                    se_min = se;
                                    back_r_temp = app[0];
                                }
                                if (ne_min > ne)
                                {
                                    ne_min = ne;
                                    front_r_temp = app[0];
                                }
                                if (sw_min > sw)
                                {
                                    sw_min = sw;
                                    back_l_temp = app[0];
                                }
                            }
                            base_temp.y *= 1.005;
                            base_temp.x *= 1.000;
                            // base_check
                            ret = base_check(front_l_temp, front_r_temp, back_l_temp, back_r_temp, base_temp);

                            // 検出したbaseが真ん中でなければEXIT
                            if ((base_temp.x < (320 - 50)) || (base_temp.x > (320 + 50)))
                            {
                                LOG_VERBOSE.printf("base center error!!!:%d,%d", base_temp.x, base_temp.y);
                                ret = false;
                                // break;
                            }
                            if (front_l_temp.x < 200 || front_r_temp.x > 640 - 200)
                            {
                                LOG_VERBOSE.printf("front_l_temp.x, front_r_temp.x error!!!:%d,%d", front_l_temp.x, front_r_temp.x);
                                ret = false;
                                // break;
                            }

                            if (ret)
                                break;
                        }
                        roiCnt++;

                        // 念のため輪郭をカウント
                        if (roiCnt == 10)
                        {
                            break;
                        }
                    }
                    i++;
                }
            }
            if ((base_detection_maker == 3) || (base_detection_maker == 4)) // 3は明度判定＋画面分割　４は＋彩度判定＋画面分割　＋角のパターンマッチング
            {
                int plane_i;
                if (base_detection_maker == 3)
                {
                    plane_i = 2;
                }
                else
                {
                    plane_i = 1;
                }

                Mat tmp_img, tmp_img_s_w, tmp_img_s_e, tmp_img_s_l, gray_img_s_w, gray_img_s_e, gray_img_s_l;
                bool nw_f, ne_f, sw_f, se_f, b_f;
                nw_f = false, ne_f = false, sw_f = false, se_f = false, b_f = false;
                // カラーチャンネルに分離
                std::vector<cv::Mat> planes;
                cv::split(bgr_img, planes);

                // 各チャンネルに対してヒストグラム平坦化
                for (int i = 0; i < 3; i++)
                {
                    equalizeHist(planes[i], planes[i]);
                }

                /* 平坦化後のチャンネルを結合
            Mat dst;
            merge(planes, dst);
            // ２値化
            cvtColor(dst, gray_img, COLOR_BGR2HSV); // HSVに変換
            */

                //bitwise_not(planes[0], gray_img_tmp2);
                //addWeighted(planes[2], 0.5, gray_img_tmp2, 0.5, 0, gray_img_tmp3);
                //cv::GaussianBlur(gray_img_tmp3, bgr_img, cv::Size(1, 1), 0);
                //cv::GaussianBlur(planes[2], bgr_img, cv::Size(1, 1), 0);
                // printf("--%d,%d",bgr_img.cols,bgr_img.rows);
                //320 * 160
                int hx_bias_w = (int)(bgr_img.cols / 4);
                int hx_bias_e = (int)(bgr_img.cols / 2);
                int hy_bias = 10;   //(int)(bgr_img.rows / 2);
                int hy_bias_c = 30; //(int)(bgr_img.rows / 2); //ベース
                //画面分割
                planes[plane_i](Rect(hx_bias_w, hy_bias, hx_bias_w, bgr_img.rows - hy_bias - 30)).copyTo(gray_img_s_w); //後ろの processImage呼び出し時のbiasを合わせること
                planes[plane_i](Rect(hx_bias_e, hy_bias, hx_bias_w, bgr_img.rows - hy_bias - 30)).copyTo(gray_img_s_e);
                planes[plane_i](Rect(hx_bias_w, hy_bias_c, hx_bias_e, bgr_img.rows - hy_bias_c)).copyTo(gray_img_s_l);

                // printf("x y %d,%d\n", bgr_img.cols, bgr_img.rows);
                // printf("x y %d,%d\n", gray_img_s.cols, gray_img_s.rows);
                double nw_val, ne_val, sw_val, se_val, b_val;
                Point base_eg;
                double nw_max, ne_max, sw_max, se_max, b_max;
                nw_max = ne_max = sw_max = se_max = b_max = 999999999;
                double shiki_val = 0.55; //テンプレートマッチングの閾値
                double bi = 0.0001;      //ピークを超えたかの閾値
                int tmp_c = nw_tmp.cols / 2;

                // 画像を反転させる
                UMat destination;
                bitwise_not(gray_img_s_w, destination);
                destination.copyTo(gray_img_s_w);
                bitwise_not(gray_img_s_e, destination);
                destination.copyTo(gray_img_s_e);
                bitwise_not(gray_img_s_l, destination);
                destination.copyTo(gray_img_s_l);

                float shikii = 30; //ステップを小さくする閾値
                int step = 5, init_i = 165, i_max = 255;
#define arc_i 0.96 //
                if (base_detection_maker == 3)
                {
                    init_i = (init_i_old > 5) ? init_i_old - 10 : 150;
                    i_max = 235;
                }
                if (base_detection_maker == 4)
                {
                    init_i = (init_i_old > 5) ? init_i_old - 10 : 150;
                    i_max = 235;
                }

                int max_val2 = 0;
                for (int i = init_i; i < i_max; i += step)
                {
                    // printf("xxx:%d", i);
                    threshold(gray_img_s_w, tmp_img_s_w, 255 - i, 255, cv::THRESH_BINARY);
                    threshold(gray_img_s_e, tmp_img_s_e, 255 - i, 255, cv::THRESH_BINARY);
                    threshold(gray_img_s_l, tmp_img_s_l, 255 - i, 255, cv::THRESH_BINARY);

                    /*
                引数:

                tmp_img: 処理対象の画像。
                tmp: テンプレート画像。
                val: 距離値。
                max_val: 最大距離値。
                flag: フラグ。
                temp: 一時的な座標。
                bias_x, bias_y: バイアス値。
                target_p: 目標座標。
                処理内容:

                flagがfalseの場合のみ処理を行う。
                tmpmat関数を使ってテンプレートマッチングを行い、val2を取得。
                val2が0.7以上の場合、距離valを計算。
                max_val + biがvalより小さい場合、flagをtrueに設定。
                それ以外の場合、max_valがvalより大きい場合、max_valを更新し、tempの座標を更新。
                */
                    auto processImage = [&](cv::Mat &tmp_img, cv::Mat &tmp, double &val, double &max_val, bool &flag, cv::Point &temp, int bias_x, int bias_y, Point target_p, int n) {
                        if (!flag)
                        {
                            double val2;
                            base_eg = tmpmat(&tmp_img, &tmp, &val2);
                            //LOG_DEBUG.printf(" val2 = %.2f",val2);
                            if (val2 > shiki_val)
                            {
                                temp.x = base_eg.x + tmp_c + bias_x + c_rect_x;
                                temp.y = base_eg.y + tmp_c + bias_y + c_rect_zone_y;
                                // val = (int)distance(temp, target_p);
                                val = sqrt(pow(temp.x - target_p.x, 2) + 0.2 * pow(temp.y - target_p.y, 2));
                                // LOG_VERBOSE_IF(val == 0).printf(" distance = zero %d,%d,,,%d,%d",temp.x,temp.y,target_p.x,target_p.y);
                                if ((int)max_val < (int)val) //ピークを超えたか
                                {
                                    // if (val < 15)
                                    flag = true;
                                    /*
                                printf("%d:", n);
                                printf(" i:%d,val:%d,val2:%.2f\n", i, (int)val, (int)val2);
                                printf(" x:%d,y:%d\n\n", temp.x, temp.y);
                                */
                                    // LOG_DEBUG.printf("peek %d i:%d,val:%d,flag:%d", n, i, (int)val, flag);
                                }
                                else
                                {
                                    max_val = val;
                                    // LOG_DEBUG.printf("base position %d i:%d,val:%d,flag:%d",n,i,(int)val,flag);
                                }
                                if (max_val2 > val2)
                                {
                                    flag = true;
                                }
                                else
                                {
                                    max_val2 = val2;
                                }
                            }
                            else
                            {
                                if (max_val != 999999999)
                                    flag = true;
                            }
                        }
                        // LOG_VERBOSE.printf(" flag:%d,%d,%d",flag,i,(int)val);
                    };

                    //各方向（北西、北東、南西、南東、基準）の画像に対してprocessImageを適用。
                    processImage(tmp_img_s_w, nw_tmp, nw_val, nw_max, nw_f, front_l_temp, hx_bias_w, hy_bias, target_nw, 1);
                    processImage(tmp_img_s_e, ne_tmp, ne_val, ne_max, ne_f, front_r_temp, hx_bias_e, hy_bias, target_ne, 2);
                    processImage(tmp_img_s_w, sw_tmp, sw_val, sw_max, sw_f, back_l_temp, hx_bias_w, hy_bias, target_sw, 3);
                    processImage(tmp_img_s_e, se_tmp, se_val, se_max, se_f, back_r_temp, hx_bias_e, hy_bias, target_se, 4);
                    processImage(tmp_img_s_l, base_tmp, b_val, b_max, b_f, base_temp, hx_bias_w, hy_bias_c, target_bs, 5);

                    best_v_filter = i;
                    if (nw_f && ne_f && sw_f && se_f && b_f)
                    {
                        // printf("--------------------\n");
                        break;
                    }

                    /*すべてのフラグがtrueの場合、ループを終了。
                いずれかの値が閾値shikii以上の場合、stepを1に設定し、init_i_oldを更新。*/
                    if ((nw_val <= shikii) || (ne_val <= shikii) || (sw_val <= shikii) || (se_val <= shikii) || (b_val <= shikii))
                    {
                        init_i_old = i;
                        step = 3;
                    }
                }

                // base_check
                ret = base_check(front_l_temp, front_r_temp, back_l_temp, back_r_temp, base_temp);
                /* カラー画像をシュリンクして、モノ画像と位置合わせ
            paste(bgr_img, nw_tmp_img, 0, 0);
            paste(bgr_img, ne_tmp_img, gray_img_hx, 0);
            paste(bgr_img, sw_tmp_img, 0, gray_img_hy);
            paste(bgr_img, se_tmp_img, gray_img_hx, gray_img_hy);
            paste(bgr_img, b_tmp_img, 0, gray_img_hy + sw_tmp_img.rows);
            paste(bgr_img, nw_tmp_img, 0, 0);
            paste(bgr_img, ne_tmp_img, gray_img_hx, 0);
            paste(bgr_img, sw_tmp_img, 0, gray_img_hy);
            paste(bgr_img, se_tmp_img, gray_img_hx, gray_img_hy);
            paste(bgr_img, b_tmp_img, 0, gray_img_hy + sw_tmp_img.rows);
            */
            }

            // ベース検知後のゾーン設定

            // ゾーン関連変数セット
            base_app = approx;
            baseCoordinateBase = base_temp; // ベース基底座標セット
            baseCoordinateFront_l = front_l_temp;
            baseCoordinateFront_r = front_r_temp;
            baseCoordinateBack_l = back_l_temp;
            baseCoordinateBack_r = back_r_temp;
            // Point temp_point = Point((baseCoordinateFront_l.x + baseCoordinateFront_r.x) / 2,(baseCoordinateFront_l.y + baseCoordinateFront_r.y) / 2);
            //  ↓外角に少し場所をシフト
            Point temp_point = Point((baseCoordinateFront_l.x + baseCoordinateFront_r.x) / 2 + (int)((float)(baseCoordinateBack_r.x - baseCoordinateFront_r.x) * base_h_shift) - (int)((float)(baseCoordinateFront_l.x - baseCoordinateBack_l.x) * base_h_shift), (baseCoordinateFront_l.y + baseCoordinateFront_r.y) / 2);
            jyuushin = crossPoint(baseCoordinateBack_l, baseCoordinateBack_r, Point(baseCoordinateBase.x + (int)((float)(baseCoordinateBack_r.x - baseCoordinateFront_r.x) * base_h_shift) - (int)((float)(baseCoordinateFront_l.x - baseCoordinateBack_l.x) * base_h_shift), baseCoordinateBase.y), temp_point);
            jyuushin.x = clamp(jyuushin.x, 320 - 50, 320 + 50);
            jyuushin.y = clamp(jyuushin.y, 200, 360);
        }
        else
        {
            //ベースなし処理
            ret = true;
            jyuushin = worldToScreen(0, 0, horizontalDistanceToBase_cm);
            baseCoordinateBase = jyuushin;
            base_target_add_set();

            // 検索エリアを指定 bin_imgは192-,220-の画像
            cv::Rect searchArea(0, 0, bin_img->cols, bin_img->rows); // x, y, width, height
            // マッチング結果を格納するベクタ
            std::vector<cv::Point> results;
            // 最良マッチ位置の検出
            cv::Mat bestMatchRegion, bestXorResult;
            // templateB_base = Mat::zeros(80, 100, CV_8UC1);
            // pasteCenterAligned(templateB_base, templateB, camera.roll);
            bestMatch = findBestMatch(*bin_img, templateB, searchArea, bestMatchRegion, bestXorResult);

            if (bestMatch.x != -1 && bestMatch.y != -1)
            {
                bestMatch = Point(bestMatch.x + c_rect_x, bestMatch.y + c_rect_zone_y);
                bestMatchTime = get_minutes();
            }
            else
            {
                if (bestMatchOld != Point(999, 999) && (bestMatchTime + 3) > get_minutes())
                {
                    // bestMatch = bestMatchOld;
                }
                else
                {
                    // ベース未検知の場合
                    // デフォルトベース距離設定
                    camera.yaw = 0;
                    horizontalDistanceToBase_cm = c_base_kyori_cm_no_base;
                    LOG_VERBOSE.printf("horizontalDistanceToBase_cm:%.0f", horizontalDistanceToBase_cm);
                    goto skipYaw;
                }
            }
            // camera.roll = 0;//↓下の座標変換を正確にするため（すでにrollが設定されていると意図通り変換できないため）
            // Point2i temp_point = zahyou_kaiten_xy(worldToScreen(0, 0, c_base_kyori_cm_no_base), Point(320, 200), camera.roll); //デフォルト距離の設定
            Point2i temp_point = worldToScreen(0, 0, c_base_kyori_cm_no_base), Point(320, 200); //デフォルト距離の設定
            //カメラヨーの補正
            camera_yaw_org = camera_yaw_org_old * 0.8 + 0.2 * clamp(rdn((float)(bestMatch.x - temp_point.x) * (hfov_rgb / 640)), -rdn(1.0f), rdn(1.0f));
            // camera_yaw_org = 0;
            camera_yaw_org_old = camera_yaw_org;
            // camera.yaw = camera_yaw_org; // clamp(rdn((float)(bestMatch.x - temp_point.x) * (hfov_rgb / 640)), -rdn(10.0f), rdn(10.0f));
            LOG_DEBUG_IF(bestMatch != bestMatchOld).printf("最良マッチ位置: (%d,%d) yaw:%.2f", bestMatch.x, bestMatch.y, ardn(camera_yaw_org));

            //カメラ距離の補正
            horizontalDistanceToBase_cm = (c_base_kyori_cm_no_base / sin(camera.pitch)) * sin(camera.pitch - rdn((float)(bestMatch.y - temp_point.y) * (vfov_rgb / C_monorgb_hi_y)));
            LOG_DEBUG_IF(bestMatch != bestMatchOld).printf("補正後カメラ距離: %.2f", horizontalDistanceToBase_cm);
            bestMatchOld = bestMatch;
        }
skipYaw:

        if (ret)
        {
            base_find_f = true;
            LOG_DEBUG_IF(!zone_flg).printf(" base hit w,h:%d,%d", base_w, base_h);
            LOG_DEBUG_IF(!zone_flg).printf("      BASE size (area:%.0f,leng:%.0f)", area, leng);
            LOG_DEBUG_IF(!zone_flg).printf("      base cente(x,y):%d,%d", base_temp.x, base_temp.y);

            //
        }
        // バッターの高さに基づくゾーンの低めと高めの設定
        zone_low_cm = c_zone_low_cm - (3 * batter_hight) - 3.6; // 低め https://okwave.jp/qa/q3922955.html
        zone_hi_cm = c_zone_hi_cm - (7 * batter_hight) + 3.6;   // 高め

        return ret;
    }

    /*このC++関数Base_find_makerは、画像処理のコンテキストで、特定のマーカーを検出し、それらの位置を基にして物体の基底座標を計算するために使用されます。関数は、二値画像を引数として受け取り、マーカーが見つかったかどうかを示すブール値を返します。また、オプションでキャリブレーションモードを指定することができます。
    関数内で、まずマーカーの座標とベースの四隅の基準座標を格納するための配列が定義されます。その後、二値画像から輪郭を検出し、それらの輪郭を直線近似してマーカーの候補を見つけ出します。このプロセスでは、面積と輪郭の長さを基にして、マーカーとして適切なものをフィルタリングします。
    見つかったマーカーの中から、ベースの四隅に最も近いマーカーを特定し、それらの座標をbase_near配列に格納します。このステップは、物体の向きや位置を特定するのに重要です。
    最終的に、これらのマーカーを使用して、物体の基底座標を計算し、物体が画像の中心に位置しているかどうかを検証します。物体が中心から大きくずれている場合、関数はfalseを返して処理を終了します。そうでない場合、物体の高さを検出し、さらに処理を続けます。
    この関数は、画像内の特定の物体を検出し、その位置と向きを特定するためのものです。画像処理、特にコンピュータビジョンやロボティクスの分野で役立つ機能です。コードはOpenCVライブラリを使用しており、画像処理のための強力なツールセットを提供します。*/
    bool Base_find_maker(Mat *bin_img, bool cal) // cal kyaribure-syonn ka douka
    {
        //! マーカー座標
        Point maker_ad[256]; // マーカー座標
        Point base_edge[5];  // ベース四隅の基準座標
        base_edge[0] = target_nw;
        base_edge[1] = target_ne;
        base_edge[2] = target_sw;
        base_edge[3] = target_se;
        base_edge[4] = target_bs;
        Point base_near[5]; // ベース四隅に一番近いマーカーの座標
        float nw_x, nw_y, ne_x, ne_y, sw_x, sw_y, se_x, se_y;

        bool ret, r_tmp;
        ret = false;
        r_tmp = false;
        base_find_f = false;
        Mat clip_img;

        // 周辺ノイズチェック
        //Rect roi_u(0, 0, bin_img->cols, 10);
        //Rect roi_l(0, bin_img->rows - 10, bin_img->cols, 10);

        // S ベース 輪郭検出
        findContours(*bin_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        int i = 0;
        std::vector<cv::Point>::iterator app;
        int i_MAX = 0;
        float nw, ne, sw, se, base_len, tmp_back_l, tmp_back_r, tmp_front_l, tmp_front_r;
        float nw_min, ne_min, sw_min, se_min;
        int r_max, l_max;
        roiCnt = 0;
        i = 0;
        i_MAX = 0;
        nw = 0, ne = 0, sw = 0, se = 0, tmp_back_l = 0, tmp_back_r = 0, tmp_front_l = 0, tmp_front_r = 0;
        nw_min = INT_MAX, ne_min = INT_MAX, sw_min = INT_MAX, se_min = INT_MAX;
        int base_min = INT_MAX;
        r_max = 0;
        l_max = INT_MAX;
        base_temp.y = 0;
        int loop_c = 0;
        for (auto contour = contours.begin(); contour != contours.end(); contour++)
        {
            // 輪郭を直線近似する
            cv::approxPolyDP(cv::Mat(*contour), approx, 0.001 * cv::arcLength(*contour, true), true);
            // 近似の面積が一定以上なら取得
            area = cv::contourArea(approx); // マーカーの面積
            leng = arcLength(approx, true); // マーカーの輪郭長さ
            Point2f center_of_gravity = Point2f(moments(approx).m10 / moments(approx).m00 + c_rect_x, moments(approx).m01 / moments(approx).m00 + c_rect_zone_y);
            LOG_VERBOSE_IF((area >= 6) && (area < 150)).printf("BASE_maker size (area:%d,leng:%d,x*%.0f,y:%.0f)", (int)area, (int)leng, center_of_gravity.x, center_of_gravity.y);
            if ((area >= 8) && (area < 70) && (leng < 100) && (leng > 15) //
            )
            {
                if (moments(approx).m00 != 0)
                {
                    std::vector<cv::Point> approx2;
                    app = approx.begin();
                    // 重心計算
                    Point2f center_of_gravity = Point2f(moments(approx).m10 / moments(approx).m00 + c_rect_x, moments(approx).m01 / moments(approx).m00 + c_rect_zone_y);

                    maker_ad[loop_c] = (Point)center_of_gravity;
                    loop_c++;
                }
            }
        }

        for (int i = 0; i < loop_c; i++)
        {
            int len, len_min;
            len_min = INT_MAX;
            // ベース四隅に一番近いマーカーを見つける
            for (int l = 0; l < 5; l++)
            {
                len = (int)distance(maker_ad[i], base_edge[l]);
                if (len_min > len)
                {
                    len_min = len;
                    base_near[l] = maker_ad[i];
                }
            }
        }
        // ベース四隅に一番近かったマーカーの座標をセット
        front_l_temp = base_near[0];
        front_r_temp = base_near[1];
        back_l_temp = base_near[2];
        back_r_temp = base_near[3];
        base_temp = base_near[4];

        LOG_VERBOSE.printf(" ma_y:%d,ma_y:%d,ma_y:%d,ma_y:%d", int((0.5 + dy) * 400), int((0.5 + dy) * 400), int((0.5 + dy + 0.12) * 400), int((0.5 + dy + 0.12) * 400));
        LOG_VERBOSE.printf(" nw_x:%.0f,ne_x:%.0f,sw_x:%.0f,se_x:%.0f", nw_x, ne_x, sw_x, se_x);
        LOG_VERBOSE.printf(" nw_y:%.0f,ne_y:%.0f,sw_y:%.0f,se_y:%.0f", nw_y, ne_y, sw_y, se_y);
        LOG_VERBOSE.printf(" nw:%.0f,ne:%.0f,sw:%.0f,se:%.0f,base:%.0f", nw, ne, sw, se, base_len);
        if (base_temp == Point(0, 0))
            return (false);
        LOG_VERBOSE.printf("base front_l_temp:%d,%d", front_l_temp.x, front_l_temp.y);
        LOG_VERBOSE.printf("base front_r_temp:%d,%d", front_r_temp.x, front_r_temp.y);
        LOG_VERBOSE.printf("base back_l_temp:%d,%d", back_l_temp.x, back_l_temp.y);
        LOG_VERBOSE.printf("base back_r_temp:%d,%d", back_r_temp.x, back_r_temp.y);
        LOG_VERBOSE.printf("base base_temp:%d,%d", base_temp.x, base_temp.y);
        // 検出したbaseが真ん中でなければEXIT
        if ((base_temp.x < (320 - 50)) || (base_temp.x > (320 + 50)))
        {
            LOG_VERBOSE.printf("base center error!!!:%d,%d", base_temp.x, base_temp.y);
        }

        // ベースの高さを検出//base_check
        ret = base_check(front_l_temp, front_r_temp, back_l_temp, back_r_temp, base_temp);
        roiCnt++;

        if (ret)
        {
            // ベース検知後のゾーン設定

            // ゾーン関連変数セット
            base_app = approx;
            baseCoordinateBase = base_temp; // ベース基底座標セット
            baseCoordinateFront_l = front_l_temp;
            baseCoordinateFront_r = front_r_temp;
            baseCoordinateBack_l = back_l_temp;
            baseCoordinateBack_r = back_r_temp;
            // Point temp_point = Point((baseCoordinateFront_l.x + baseCoordinateFront_r.x) / 2,(baseCoordinateFront_l.y + baseCoordinateFront_r.y) / 2);
            //  ↓外角に少し場所をシフト
            Point temp_point = Point((baseCoordinateFront_l.x + baseCoordinateFront_r.x) / 2 + (int)((float)(baseCoordinateBack_r.x - baseCoordinateFront_r.x) * base_h_shift) - (int)((float)(baseCoordinateFront_l.x - baseCoordinateBack_l.x) * base_h_shift), (baseCoordinateFront_l.y + baseCoordinateFront_r.y) / 2);
            jyuushin = crossPoint(baseCoordinateBack_l, baseCoordinateBack_r, Point(baseCoordinateBase.x + (int)((float)(baseCoordinateBack_r.x - baseCoordinateFront_r.x) * base_h_shift) - (int)((float)(baseCoordinateFront_l.x - baseCoordinateBack_l.x) * base_h_shift), baseCoordinateBase.y), temp_point);

            base_find_f = true;
            LOG_DEBUG_IF(!zone_flg).printf(" base hit w,h:%d,%d", base_w, base_h);
            LOG_DEBUG_IF(!zone_flg).printf("      BASE size (area:%.0f,leng:%.0f)", area, leng);
            LOG_DEBUG_IF(!zone_flg).printf("      base cente(x,y):%d,%d", base_temp.x, base_temp.y);

            //
        }

        return ret;
    }

    //このコードは、C++で書かれたボール検出の関数です。この関数は、与えられた画像からボールを検出し、ボールの位置、距離、サイズなどの情報を返します。
    //関数の最初の行では、検出されたボールの状態を表す変数 ret を初期化しています。ret の値は、ボールが検出されなかった場合は ball_none（0）、ゾーンの近くにある場合は base_shortly（1）、ゾーンから外れている場合は base_far_away（2）となります。
    //次に、いくつかの変数を初期化しています。strike はストライク判定の結果を格納するブール変数で、mask_image は距離情報画像を表す Mat オブジェクトです。
    //その後、与えられた画像からボールを検出するために、輪郭検出を行います。findContours 関数を使用して、与えられた画像からボールの輪郭を見つけます。輪郭は contours_ball というベクトルに格納されます。
    //次に、各輪郭に対して以下の処理を行います。まず、ボールの面積と輪郭の長さを計算します。次に、ボールの面積が一定の範囲内にあるかどうかをチェックします。さらに、ボールの形状が円形であるかどうかを判断するために、円形度を計算します。円形度が一定の閾値を超える場合、ボールとして認識します。
    //ボールとして認識された場合、さまざまなチェックが行われます。例えば、ボールが画面の端にある場合や、ボールのサイズが範囲外の場合、ボールの位置がベースの位置に対して正しいかどうかなどをチェックします。これらのチェックに合格したボールは、ゾーンの近くにあるかどうかを判断し、ストライクかボールかを判定します。
    //最後に、検出されたボールの情報をログに出力し、結果を返します。
    //この関数は、ボールの検出と位置判定を行うための重要な処理を含んでいます。ボール検出アルゴリズムやパラメータについては、このコードの他の部分で定義されている可能性があります。
    /**
    * @brief 画像内でボールを見つける関数。
    *
    * この関数は、OpenCVの輪郭検出およびフィルタリング技術を使用して、画像内のボールを見つけます。
    * また、距離計算、誤差検出、および基準位置に基づく補正も行います。
    *
    * @param img ボールを見つける入力画像へのポインタ。
    * @param strike ストライクが検出されたかどうかを示すブール変数へのポインタ。
    * @param mask_image 距離計算に使用するマスク画像へのポインタ。
    * @param i_ball 計算されたボールの距離を格納する整数変数へのポインタ。
    * @param ball_i_old 以前に計算されたボールの距離。
    *
    * 下記がボールの位置出力
    * ballCenter_cm_x(ボール横位置（画面中央）)
    * ballCenter_cm_y(ボール高さ位置（地表から）)
    * @return ボール検出の結果を表す整数。
    *
    * @note この関数は、さまざまなパラメータと計算にグローバル変数を使用します。
    */
    int findBall(Mat *img, bool *strike, Mat *mask_image, Mat *mask_image_batter, int ball_detection_phase, int dep_l_en_i, int dep_l_kin_i, int *i_ball) // 讀懃ｴ｢逕ｻ蜒上√せ繝医Λ繧､繧ｯ蛻､螳夂ｵ先棡・亥・蜉幢ｼ峨∬ｷ晞屬諠・ｱ逕ｻ蜒・
    {
        if (isSecondCamera)
            center_temp_old = center_temp_old_s;
        else
            center_temp_old = center_temp_old_f;
        float kahen_enkeido = c_enkeido;
        float i_ball_cm_temp; //直線距離
        // const int roi_x_bias = img->cols / 5;
        const int roi_x_bias = 0;
        int bias_x = 35;
        const int bias_y = 50;
        int roi_x1 = 0;
        int roi_y1 = 0;
        int roi_x2 = 256;
        int roi_y2 = 400;
        // ball_detection_phase = 99;
        switch (ball_detection_phase)
        {
        case far_away:
            mini_ball_area = c_mini_ball_area;
            max_ball_area = 50; // c_mini_ball_area * 3;
            kahen_enkeido = clamp(c_enkeido - 0.4f, 0.45f, c_enkeido);
            roi_x1 = c_no_noize_area; //192 + 60 = 252,640 -192 - 60 = 388
            roi_y1 = 50;
            roi_x2 = img->cols - roi_x1 * 2; // img->cols-roi_x1*2; //
            roi_y2 = img->rows - roi_y1 - 150;
            break;
        case beyond_the_base:
            mini_ball_area = 20; //c_mini_ball_area;
            max_ball_area = 150;
            kahen_enkeido = c_enkeido - 0.1;
            if (center_temp_old.x != 0)
            {                                                                                      //
                roi_x1 = (int)clamp(center_temp_old.x - c_rect_x - bias_x, 0, img->cols - bias_x); //
                roi_y1 = (int)clamp(center_temp_old.y - c_rect_y - 20, 50, img->rows - bias_y);
                roi_x2 = (int)clamp(bias_x * 2, 0, img->cols - roi_x1); //
                roi_y2 = (int)clamp(bias_y + 20, 0, img->rows - roi_y1);
            }
            else
            {
                roi_x1 = c_no_noize_area; //192 + 60 = 252,640 -192 - 60 = 388
                roi_y1 = 50;
                roi_x2 = img->cols - roi_x1 * 2; // img->cols-roi_x1*2; //
                roi_y2 = img->rows - roi_y1 - 150;
            }
            //
            if (ball_cours_cnt < 3)
            {
                roi_x1 = 0; //
                roi_y1 = 50;
                roi_x2 = img->cols - roi_x1 * 2; // img->cols-roi_x1*2; //
                roi_y2 = img->rows - roi_y1 - 150;
            }
            //
            break;
        case on_the_base:
            mini_ball_area = 25; //c_mini_ball_area;
            max_ball_area = c_max_ball_area;
            kahen_enkeido = c_enkeido - 0.3;
#if (false)             //
            roi_x1 = 0; //
            roi_y1 = 100;
            roi_x2 = img->cols - roi_x1 * 2; // img->cols-roi_x1*2; //
            roi_y2 = img->rows - roi_y1 - 10;
#endif                                                                                         //
#if (true)                                                                                     //
            roi_x1 = (int)clamp(center_temp_old.x - c_rect_x - bias_x, 0, img->cols - bias_x); //
            roi_y1 = (int)clamp(center_temp_old.y - c_rect_y - 20, 70, img->rows - bias_y);
            roi_x2 = (int)clamp(bias_x * 2, 0, img->cols - roi_x1); //
            roi_y2 = (int)clamp(bias_y + 20, 0, img->rows - roi_y1);
#endif //                                                                                      \
    // LOG_DEBUG.printf(" aaa:%d,%d,%d,%d",old_ball_center.x,old_ball_center.y,bias_x,bias_y); \
    // LOG_DEBUG.printf(" bbb:%d,%d,%d,%d",roi_x1 + c_rect_x,roi_y1 + c_rect_y,roi_x2,roi_y2); \
    // LOG_DEBUG.printf(" ccc:%d,%d,%d,%d",roi_x1, roi_y1,roi_x2,roi_y2);                      \
    // LOG_DEBUG.printf(" ddd:%d,%d,%d,%d",img->cols,img->rows,c_rect_x,c_rect_y);
            break;
        case passing_the_base:
            bias_x += 15;
            mini_ball_area = 40; // c_mini_ball_area;
            max_ball_area = c_max_ball_area;
            kahen_enkeido = c_enkeido - 0.4;
#if (false)              //
            roi_x1 = 10; //
            roi_y1 = 130;
            roi_x2 = img->cols - roi_x1 * 2; // img->cols-roi_x1*2; //
            roi_y2 = img->rows - roi_y1 - 10;
#endif                                                                                         //
#if (true)                                                                                     //
            roi_x1 = (int)clamp(center_temp_old.x - c_rect_x - bias_x, 0, img->cols - bias_x); //
            roi_y1 = (int)clamp(center_temp_old.y - c_rect_y - 20, 100, img->rows - bias_y);
            roi_x2 = (int)clamp(bias_x * 2, 0, img->cols - roi_x1); //
            roi_y2 = (int)clamp(bias_y + 20, 0, img->rows - roi_y1);
#endif //
            break;
        default:
            mini_ball_area = c_mini_ball_area;
            max_ball_area = c_max_ball_area;
            kahen_enkeido = c_enkeido;
            roi_x1 = 0; //
            roi_y1 = 0;
            roi_x2 = img->cols - roi_x1 * 2; // img->cols-roi_x1*2; //
            roi_y2 = img->rows;
            break;
        }
        // 低円形度が連続しないように一旦戻す
        if (enkeido_old < c_enkeido)
            kahen_enkeido = clamp(kahen_enkeido + 0.1, kahen_enkeido, c_enkeido);

        Rect roi(roi_x1, roi_y1, roi_x2, roi_y2);
        Rect roi_batter(0, 0, mask_image_batter->cols, 300); //バッターのROI
        // Mat img2 = Mat(*img, roi);
        int ret;
        ret = ball_none; // 0:未検出,１：ゾーン近辺、2：ゾーン外れ
        *strike = false;
        int zone_kyori_old = 9999; // ゾーン中心からの距離
        // uint32_t zone_kyori = 0;          // ゾーン中心からの距離
        int zone_kyori_x = 0;
        int zone_kyori_y = 0;
        bool contin;
        contin = false;
        Mat clip_img;
        int last_dep_i = 0;
        float dep_i_heikin;
        int kenchi_cnt = 0; // 同じ画像で複数回検知回数
        ball_err_cd ball_err;
        // ボール検出
        Point min_point, max_point;
        std::vector<cv::Point> approx;
        std::vector<cv::Point> app;
        std::vector<std::vector<cv::Point>> pre_contours_ball, pre_contours_batter;
        std::vector<std::vector<cv::Point>> contours_ball, contours_batter;
        // findContours(Mat(*img, roi), contours_ball, RETR_EXTERNAL, CHAIN_APPROX_NONE); //こちらのほうが速いようだ
        findContours(Mat(*img, roi), pre_contours_ball, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        findContours(Mat(*mask_image_batter, roi_batter), pre_contours_batter, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //打者の輪郭を取得

        // ボールの輪郭をフィルタリング
        for (auto &contour : pre_contours_ball)
        {
            if (isWithinRange(contourArea(contour), 5, 500)) // 最小面積しきい値) //
            {
                // printf("contourArea:%.0f", contourArea(contour));
                contours_ball.push_back(contour);
            }
        }
        for (auto &contour : contours_ball)
        {
            for (auto &point : contour)
            {
                point.x += roi_x1; //roi.x;
                point.y += roi_y1; //roi.y;
            }
        }

        // バッターの輪郭を取得
        if (isSecondCamera)
        {
            contours_batter.clear();
            int i = 0;
            for (auto &contour : pre_contours_batter)
            {
                if (isWithinRange(contourArea(contour), 300.0f, 1000000.0f)) // 最小面積しきい値) //
                {
                    contours_batter.push_back(contour);
                    i++;
                    LOG_DEBUG_IF(!test_measurement_mode).printf(" %d:batter areasize:%.0f,x:%d,y:%d", i, contourArea(contour), contour[0].x + c_rect_x, contour[0].y + c_rect_y);
                }
            } //
        }

        Point2f ball_x_min, ball_x_max, ball_y_min, ball_y_max;
        ball_x_min.x = 9999, ball_x_max.x = 0, ball_y_min.y = 9999, ball_y_max.y = 0;
        center.x = 0;
        center.y = 0;
        radius = 0;
        float leng_base; // ボールの輪郭長さ
        // printf(" contours_size:%d", contours_ball.size());
        int kenchi_ball_suu = 150;                                           // ボールの検知数
        LOG_DEBUG_IF(kenchi_ball_suu <= contours_ball.size()).printf("ball_count over flow!!! :%d < %d", kenchi_ball_suu, contours_ball.size());
        LOG_DEBUG_IF(contours_ball.size() == 0 && ball_detection_phase != far_away).printf("ball_count Zero !!! :%d", contours_ball.size());
        int contours_size = clamp(contours_ball.size(), 0, kenchi_ball_suu); //最大でも１０個を検出
        Point2i err_ball;
        for (int i = 0; i < contours_size; i++)
        {
            area_ball = contourArea(contours_ball[i]); // ボールの面積
            // 面積が一定以上か
            if ( //
                (!isWithinRange(area_ball, c_mini_ball_area, c_max_ball_area) //
                    // && isSecondCamera //
                    ) //
                )
            {
                LOG_DEBUG.printf(" area size hazure %d,%d",contours_ball[i][0].x+(c_rect_x + roi_x_bias),contours_ball[i][0].y+c_rect_y);
                continue;
                }
            contin = false; //2025 01 16 bug fix
            ball_x_min.x = 639, ball_x_max.x = 0, ball_y_min.y = 399, ball_y_max.y = 0;
            leng = arcLength(contours_ball[i], true); // ボールの輪郭長さ
            center_temp.x = 0;
            center_temp.y = 0;
            center_temp_cm.x = 0;
            center_temp_cm.y = 0;
            radius_temp = 0;
            int noise_table_no = 0; //ヒットしたノイズテーブル番号

            app = contours_ball[i];
            // ボールの座標（上下左右の端）を取得
            for (int i_app = 0; i_app < app.size(); i_app++)
            {
                app[i_app].x += (c_rect_x + roi_x_bias); // 座標補正(cols:256->640)
                app[i_app].y += c_rect_y;
                if (app[i_app].x < ball_x_min.x)
                    ball_x_min = app[i_app];
                if (app[i_app].x > ball_x_max.x)
                    ball_x_max = app[i_app];
                if (app[i_app].y < ball_y_min.y)
                    ball_y_min = app[i_app];
                if (app[i_app].y > ball_y_max.y)
                    ball_y_max = app[i_app];
            }
            radius_temp = ((ball_x_max.x - ball_x_min.x) / 2); // * (float)C_monorgb_hi_x;
            radius_h = ((ball_y_max.y - ball_y_min.y) / 2);    // * (float)C_monorgb_hi_y;
            // center_temp:座標補正済 ボール座標
            center_temp.x = (ball_x_max.x + ball_x_min.x) / 2;
            center_temp.y = (ball_y_max.y + ball_y_min.y) / 2;
            // 打者の輪郭ないであればスキップ
            if (contours_batter.size() > 0)
            {
                for (int j = 0; j < contours_batter.size(); j++)
                {
                    // LOG_DEBUG_IF(!test_measurement_mode).printf(" %d:batter areasize:%.0f,x:%d,y:%d", j, contourArea(contours_batter[j]), contours_batter[j][0].x + c_rect_x, contours_batter[j][0].y + c_rect_y);
                    if (pointPolygonTest(contours_batter[j], center_temp, false) >= 0)
                    {
                        ball_err = batter_pos_err;
                        contin = true;
                        break;
                    }
                }
                if (contin)
                {
                    LOG_DEBUG_IF(!test_measurement_mode).printf(" %d:batter err :%d,%d", i, contours_ball[i][0].x + c_rect_x, contours_ball[i][0].y);
                    continue;
                }
            }

            // 円形度から円かどうかを判断
            enkeido = 12.5 * (area_ball / (leng * leng));
            // LOG_DEBUG_IF(test_measurement_mode).printf(" test %d:円形の図形を検出: 中心 (%.0f,%.0f, leng:%.1f 半径 (%.1f,%.1f),area:%.1f,enkeido:%.2f,camera:#%d", i, center_temp.x, center_temp.y, leng, radius_temp, radius_h, area_ball, enkeido, int(isSecondCamera + 1));
            // LOG_DEBUG_IF(test_measurement_mode).printf(" test mini_ball_area:%d,max_ball_area:%d", mini_ball_area, max_ball_area);
            // LOG_DEBUG_IF(test_measurement_mode).printf(" test kahen_enkeido:%.2f", kahen_enkeido);

            noise_table_no = 0; //ヒットしたノイズテーブル番号
                                // if (!isWithinRange(center_temp.x, 320 - c_no_noize_area, 320 + c_no_noize_area))
                                // {
            // 近似した位置のボールか
            bool ballOldAddMutch = isWithinRange(center_temp.x, center_temp_old.x - 12, center_temp_old.x + 12)     //
                                   && isWithinRange(center_temp.y, center_temp_old.y - 16, center_temp_old.y + 45); //
            if (checkBallNoise(center_temp, 0, &noise_table_no, int(isSecondCamera + 1)))
            {
                if (!test_measurement_mode && !ballOldAddMutch)
                {
                    ball_err = noise_check_err;
                    contin = true;
                    continue;
                }
            }
            //}
            // 面積が一定以上か
            if (ballOldAddMutch)
                mini_ball_area = 8; // c_mini_ball_area;
            if (isWithinRange(area_ball, mini_ball_area, max_ball_area) && (leng != 0))
            {
                kenchi_cnt = i + 1;
                LOG_DEBUG_IF(!test_measurement_mode && contours_size > kenchi_ball_suu).printf(" %d:kenchi-cnt ::%d/%d ,x:%.0f,y%.0f(areaSize,enkeidoは通過)", i, kenchi_cnt, contours_size, center_temp.x, center_temp.y);

                // 画面端はスキップ
#if (false)
                if (!isWithinRange(contours_ball[i][0].x, c_rect_x / 10, 640 - (c_rect_x * 2) - c_rect_x / 10)) // || (contours_ball[i][0].x) > (Mat(*img, roi).cols - 5))
                {
                    ball_err = gamen_hashi_err;
                    err_ball = contours_ball[i][0];
                    contin = true;
                    LOG_DEBUG_IF(!test_measurement_mode).printf(" %d:hashi err :%d,%d", i, err_ball.x + c_rect_x, err_ball.y);
                    continue;
                }
#endif

                // enkeido = (radius_temp < radius_h) ? (radius_temp / radius_h) : (radius_h / radius_temp);
                LOG_DEBUG_IF(ball_detection_phase != far_away && ball_detection_phase != invalid).printf(" %d:enkeido:%.2f,kahen_enkeido:%.2f,(%.0f,%.0f)", i, enkeido, kahen_enkeido, center_temp.x, center_temp.y);
                LOG_DEBUG_IF(ball_detection_phase != far_away && ball_detection_phase != invalid).printf("");
                if (enkeido > kahen_enkeido //
                    || ballOldAddMutch      //
                )
                {
                    // LOG_DEBUG_IF(!test_measurement_mode).printf("\n");
                    // LOG_DEBUG_IF(!test_measurement_mode).printf("findBall start camera:#%d", int(isSecondCamera + 1));
                    LOG_DEBUG_IF(!test_measurement_mode).printf(" %d:円形の図形を検出: 中心 (%.0f,%.0f, leng:%.1f 半径 (%.1f,%.1f),area:%.1f,enkeido:%.2f,camera:#%d", i, center_temp.x, center_temp.y, leng, radius_temp, radius_h, area_ball, enkeido, int(isSecondCamera + 1));
                    LOG_DEBUG_IF(!test_measurement_mode && ballOldAddMutch).printf("  %d:ballOldAddMutch", i);
                    // LOG_DEBUG_IF(!test_measurement_mode && ball_detection_phase != far_away).printf(" %d:radius:%.1f,radius_h:%.1f", i, radius_temp, radius_h);
                    if ((radius_temp != 0) && (radius_h != 0))
                    {
                        // 極端に細長くないか
                        if (                                                                                       //
                            !isWithinRange((leng / radius_temp), 2, 25) && isWithinRange((leng / radius_h), 2, 25) //
                        )
                        {
                            ball_err = radius_hi_err;
                            contin = true;
                            goto contin0;
                        }
                    }
                    else
                    {
                        ball_err = radius_hi_err;
                        contin = true;
                        goto contin0;
                    }
                    if (!(isWithinRange(radius_temp, ball_r_min, ball_r_max) && isWithinRange(radius_h, ball_r_min, ball_r_max)))
                    {
                        ball_err = radius_size_err;
                        contin = true;
                        goto contin0;
                    }

contin0:
                    // サイズチェック
                    if (contin && !ballOldAddMutch)
                    {
                        contin = false;
                        switch (ball_err)
                        {
                        case gamen_hashi_err:
                            LOG_VERBOSE_IF(!test_measurement_mode).printf(" %d:hashi err :%d,%d", i, err_ball.x, err_ball.y);
                            break;
                        case radius_hi_err:
                            LOG_VERBOSE.printf(" %d:radius_hi error radius_hi_x:%.1f,radius_hi_y:%.1f", i, leng / radius_temp, leng / radius_h);
                            break;
                        case radius_size_err:
                            LOG_VERBOSE.printf(" %d:radius_size error:%.1f,%.1f", i, radius_temp, radius_h);
                            break;
                        case batter_pos_err:
                            LOG_VERBOSE.printf(" %d:batter_pos error:%.0f,%.0f", i, center_temp.x, center_temp.y);
                            break;

                        default:
                            break;
                        }

                        // LOG_DEBUG_IF(ball_detection_phase != far_away).printf("  %d:SKIP!!!(x:%.0f,y:%.0f,err:%d)", i, center_temp.x, center_temp.y, ball_err);
                        LOG_DEBUG.printf("   %d:SKIP!!!(x:%.0f,y:%.0f,err:%d)", i, center_temp.x, center_temp.y, ball_err);
                        continue;
                    }

                    // 処理領域を設定
                    int check_x = clamp((int)(center_temp.x - c_rect_x), 3, mask_image->cols - 3); // 補正前座標に戻す
                    int check_y = (int)(center_temp.y - c_rect_y);
                    Rect roi_noise((int)clamp((int)(check_x - radius_temp * 6), 0, (int)(mask_image->rows - c_rect_x * 2 - radius_temp * 6)), (int)clamp((int)(check_y - radius_h * 6), 0, (int)(c_rect_y_h - c_rect_y * 2 - radius_h * 4)), (int)(radius_temp * 4), (int)(radius_h * 4));
                    // 領域を切り取りコピー
                    try
                    {
                        clip_img = Mat(*img, roi_noise);
                    }
                    catch (const std::exception &e)
                    {
                    }
                    // 一定比率以上ならノイズの一つと判断
                    int count_dot_ball = countNonZero(clip_img);
                    if (count_dot_ball > (area_ball * c_dot_hi_err))
                    {
                        ball_err = ball_dot_hi_err;
                        LOG_DEBUG.printf("  %d:dot_hi error:%.1f,%.1f", i, (float)count_dot_ball, area_ball);
                        contin = false;
                        continue;
                    }

                    // ボール距離判定
                    dep_i_heikin = 0;
                    int dep_cnt = 0;
                    uchar dep_tmp;
                    int dep_cnt_base = 0;
                    float basic_ball_area_tmp;
                    dep_i_heikin = 0;
                    dep_cnt = 0;
                    dep_cnt_base = 0;
                    int max_dep_tmp = 0;   // 最大値を保持する変数を初期化
                    int min_dep_tmp = 999; // 最小値を保持する変数を初期化

                    for (int iy = clamp(check_y - radius_h, 0, mask_image->rows - radius_h); iy <= clamp(check_y + radius_h, 0, mask_image->rows); iy++)
                    {
                        ushort *src = mask_image->ptr<ushort>(iy);
                        for (int ix = clamp(check_x - radius_temp, 0, mask_image->cols - radius_temp); ix <= clamp(check_x + radius_temp, 0, mask_image->cols); ix++)
                        {
                            int dep_tmp = (int)src[ix];
                            if (isWithinRange(dep_tmp, 30, 250))
                            {
                                // 最大値を更新
                                if (dep_tmp > max_dep_tmp)
                                {
                                    max_dep_tmp = dep_tmp;
                                }
                                // 最小値を更新
                                if (dep_tmp < min_dep_tmp)
                                {
                                    min_dep_tmp = dep_tmp;
                                }

                                dep_i_heikin += dep_tmp;
                                dep_cnt++;
                            }
                            dep_cnt_base++;
                        }
                    }
                    //printf(" dep:%d\n",dep_cnt);
                    /*このコードのセクションは、ボールの位置データに関するノイズチェックを行うプロセスを実装しています。
                        まず、`ballnoisecheck`というブール変数を`false`に初期化して、同じ場所や距離で既に検知されたボールがあるかどうかをチェックするフラグとして使用します。
                        ループを使用して、過去に検知されたボールの位置（`ballnoise`配列に格納されている）と現在のボールの位置（`center_temp`変数に格納されている）を比較します。
                        この比較では、`x`座標、`y`座標、そして`depth`（深さ）を考慮して、一定の許容範囲（`x_yure`、`y_yure`、`kyori_yure`で定義）内にあるかどうかを確認します。
                        もし現在のボールの位置が過去に検知された位置の近くにあれば、`ballnoisecheck`を`true`に設定し、ループから抜け出します。
                        その後、`ballnoisecheck`が`true`であり、かつ`test_measurement_mode`が`false`の場合、ノイズとして判断し、エラー処理を行います。
                        具体的には、`ball_err`にエラーコードを設定し、処理を続行しないように`contin`を`true`に設定します。
                        一方、`ballnoisecheck`が`false`の場合、つまり新しい位置が過去のデータとは異なる場合は、`ballnoise`配列に新しい位置情報を追加します。
                        `ball_noise_i`は現在のインデックスを示し、このインデックスが最大値`ball_noise_i_max`に達した場合は、インデックスを0にリセットして上書きを開始します。
                        これにより、`ballnoise`配列は循環バッファのように機能し、最新の位置データを保持しながら古いデータを上書きしていきます。
                        このプロセス全体は、ボールの位置データの信頼性を確保し、ノイズによる誤検知を最小限に抑えるために重要です。*/
                    // ボールノイズチェック

                    if (center_temp.y == 0)
                        contin = true;
                    int leng_tmp;
                    if (dep_cnt > 2)
                    {
                        i_ball_cm_temp = c_dis_cm((uint8_t)((dep_i_heikin - max_dep_tmp - min_dep_tmp) / (dep_cnt - 2)));

                        // 傾き補正
                        center_temp_cm = center_temp;
                        hoseimae_center = center_temp;

                        // ボールのワールド座標を算出
                        if (no_base_mode)
                        {
                            ball_position_temp = screenToWorld(center_temp_cm, (i_ball_cm_temp + ball_dep_bias)); /// / precalc[(int)center_temp_cm.y].cos_y_r);
                            ball_position_temp.h += ball_high_bias;
                        }
                        else
                        {
                            ball_position_temp = calculateBallPositionByBase(i_ball_cm_temp + ball_dep_bias, center_temp_cm);
                        }
                        //
                        /* ball_position_temp = screenToWorld(Point(320,200),340);
                        LOG_DEBUG.printf(" %d:ball距離:%.1f", i, i_ball_cm_temp);
                        LOG_DEBUG.printf("  %d:w:%.1f,h:%.1f,d:%.1f",i ,ball_position.w,ball_position.h,ball_position.d);
                        LOG_DEBUG.printf("  %d:x:%.0f,y:%.0f", i ,worldToScreen(ball_position).x,worldToScreen(ball_position).y);
                        */
                        last_dep_i = c_dis_dep_i(ball_position_temp.d);
                        ballCenter_cm_x = ball_position_temp.w; // +zone_c_dis_w_cm;
                        ballCenter_cm_y = ball_position_temp.h; // + ball_high_bias;
                        ball_dep_cm = ball_position_temp.d;

                        // ノイズチェック（深度つき）
                        noise_table_no = 0; //ヒットしたノイズテーブル番号
                        if (!isWithinRange(ballCenter_cm_x, -25, 25))
                        {
                            if (last_dep_i != 0 && checkBallNoise(center_temp, last_dep_i, &noise_table_no, int(isSecondCamera + 1)) //
                                && !ballOldAddMutch                                                                                  //
                                && !isWithinRange(ballCenter_cm_x, -25, 25))
                            {
                                ball_err = noise_check_err;
                                contin = true;
                                continue;
                            }
                        }

                        //ゾーン中心に近いボールを優先
                        if (true) //(test_measurement_mode)
                        {
                            ball_position_old.w = 0;
                            ball_position_old.h = 60;
                            ball_position_old.d = 220;
                        }
                        float w_tmp = abs(ballCenter_cm_x - ball_position_old.w);
                        float h_tmp = abs(ballCenter_cm_y - ball_position_old.h);
                        float d_tmp = (!(ball_detection_phase != far_away && ball_detection_phase != invalid) || test_measurement_mode) ? abs(ball_dep_cm - ball_position_old.d) : 0; //テストモード以外は距離を考慮に入れない
                        leng_tmp = abs(w_tmp) + (0.6 * abs(h_tmp)) + (0.3 * abs(d_tmp));                                                                                              //横位置を優先
                        LOG_VERBOSE.printf("  %2d:new  leng_tmp        :%d,    w:%.1f,    h:%.1f,    d:%.1f", i, leng_tmp, ballCenter_cm_x, ballCenter_cm_y, ball_dep_cm);
                        LOG_VERBOSE.printf("  %2d:old  zone_kyori_old  :%d old_w:%.1f,old_h:%.1f,old_d:%.1f", i, zone_kyori_old, ball_position_old.w, ball_position_old.h, ball_position_old.d);
                        // LOG_DEBUG_IF(zone_kyori_old < 99999).printf("  %d:zone_kyori_old:%.3f  leng_tmp:%.3f,old_w:%.1f,old_h:%.1f,old_d:%.1f", i, zone_kyori_old, leng_tmp,ball_position_old.w,ball_position_old.h,ball_position_old.d);

                        // ゾーン真ん中に近い方を採用(横と奥行き)
                        if (zone_kyori_old < leng_tmp) // && zone_kyori_old != 0)
                        {
                            ball_err = kyori_err;
                            contin = true;
                            goto contin1;
                        }
                        //高さがマイナス
                        if (!isWithinRange(ballCenter_cm_y, 5, 200))
                        {
                            ball_err = takasa_err;
                            contin = true;
                            goto contin1;
                        }
                        // ベース外れ度合い
                        if (                                                                                                                            //
                            ball_detection_phase == far_away && !isWithinRange(ballCenter_cm_x, -60, 60)                                                //
                            || (ball_detection_phase == beyond_the_base && (!isWithinRange(ballCenter_cm_x, -55, 55)))                                  // && !isWithinRange(w_tmp, -8, 8)))  //
                            || (ball_detection_phase == on_the_base && (!isWithinRange(ballCenter_cm_x, -55, 55) && !isWithinRange(w_tmp, -15, 15)))    //
                            || (ball_detection_phase == passing_the_base && (!isWithinRange(ballCenter_cm_x, -55, 55) && !isWithinRange(w_tmp, -5, 5))) //
                        )
                        {
                            LOG_DEBUG_IF(true).printf(" %d:ベース幅距離エラー w:%.1f,h:%.1f", i, ballCenter_cm_x, ballCenter_cm_y);
                            ball_err = base_haba_err;
                            contin = true;
                            goto contin1;
                        }
                        //

                        // 距離比でのボール大きさチェック
                        if ((c_mini_ball_area + 10) > ball_size)
                        {
                            basic_ball_area_tmp = ball_size / ((ball_dep_cm + ball_size_b) * (ball_dep_cm + ball_size_b));
                            if (!isWithinRange(area_ball, basic_ball_area_tmp * 0.7, basic_ball_area_tmp * 1.3))
                            {
                                ball_err = ball_size_err;
                                contin = true;
                                goto contin1;
                            }
                        }
                        //
                        // 画面端チェック 画面端は誤検知の可能性高いので排除
                        if ((center_temp.x < (rect_x_hi_640 + 10)) || (center_temp.x > (640 - rect_x_hi_640 - 10)))
                        {
                            ball_err = gamen_hashi_err;
                            contin = true;
                            goto contin1;
                        }
                    }
                    else
                    {
                        ball_err = dep_cnt_err;
                        contin = true;
                    }

contin1:
                    if (contin)
                    {
                        contin = false;
                        switch (ball_err)
                        {
                        case kyori_err:
                            // 距離エラーの場合のログ出力
                            LOG_DEBUG_IF(!test_measurement_mode).printf(" %d:ball kyori_err", i);
                            break;
                        case ball_size_err:
                            // ボールサイズエラーの場合のログ出力
                            LOG_DEBUG_IF(!test_measurement_mode).printf("  %d:ボールサイズエラー %.0fcm,area:%.0f,chk_tmp:%.0f", i, c_dis_cm(last_dep_i), area_ball, basic_ball_area_tmp);
                            break;
                        case gamen_hashi_err:
                            // 画面端エラーの場合のログ出力 ball_detection_phase != far_away
                            LOG_DEBUG_IF(!test_measurement_mode).printf("  %d:画面端エラー(x,dep(cm)) %.1f,%.1f", i, center_temp.x, ball_dep_cm);
                            break;
                        case noise_check_err:
                            // ノイズチェックエラーの場合のログ出力
                            LOG_DEBUG_IF(!test_measurement_mode).printf("  %d:noise_c_エラー(x:%.0f,y:%.0f,dep:%d,noise#:%d)", i, center_temp.x, center_temp.y, last_dep_i, noise_table_no);
                            break;
                        case takasa_err:
                            // 高さエラーの場合のログ出力
                            LOG_DEBUG_IF(!test_measurement_mode).printf("  %d:ball高さマイナスエラー(xy,dep(cm)) %.0f,%.0f,%d,%.1f", i, center_temp.x, center_temp.y, last_dep_i, ball_dep_cm);
                            break;
                        case dep_cnt_err:
                            // 距離計測エラーの場合のログ出力
                            LOG_DEBUG_IF(!test_measurement_mode).printf("  %d:距離計測エラー(xy,dep(cm)) %.0f,%.0f,%d,%.1f", i, center_temp.x, center_temp.y, last_dep_i, ball_dep_cm);
                            break;

                        default:
                            break;
                        }
                        // スキップする際のログ出力
                        LOG_DEBUG_IF(!test_measurement_mode).printf("   %d:SKIP!!!(x:%.0f,y:%.0f,err:%d)", i, center_temp.x, center_temp.y, ball_err);
                        continue;
                    }
                    else
                    {
                        int hanteihaba_i = 0;
                        hosei_center = center_temp;
                        center = center_temp;

                        // ゾーン周辺か？あるオブジェクト（おそらくボール）が特定のゾーンの周辺にあるかどうかを判断し、その条件に基づいて特定のアクションを実行するロジックを含んでいます。
                        // if文は、オブジェクトが指定されたゾーンの周辺にあるかどうかを判断するための条件を評価します。この条件は、オブジェクトの中心位置（center_cm_x）がゾーンの右端（zone_r_cm）から20cm以内にあり、かつ左端（zone_l_cm）から20cm以内にあること、そしてオブジェクトの深さ（*i_ball）が特定の範囲（dep_l_enからdep_l_kin）内にあることをチェックします。
                        bool isInZoneNear = (                                            //
                            (                                                            //
                                (isWithinRange(ball_position_temp.d, 0, 280)             //
                                 && isWithinRange(ball_position_temp.w, -55.4, 55.4))    //
                                || (isWithinRange(ball_position_temp.d, 280, 999)        //
                                    && isWithinRange(ball_position_temp.w, -58.4, 58.4)) //
                                )                                                        //
                            && isWithinRange(ball_position_temp.h, 5, zone_hi_cm + 45)   //
                        );
                        // isOutsideBatter変数は、オブジェクトがバッターの領域の外側にあるかどうかを判断します。バッターの領域は、左端（batter_left）、右端（batter_right）、上端（batter_top）、下端（batter_bottom）によって定義され、一定の半径（radius_temp）が考慮されます。また、バッターの領域の上1/3の範囲が特に考慮されています。この条件が偽であれば、オブジェクトはバッターの領域の外側にあると判断されます。
                        bool isOutsideBatter = (                                                                            //
                            (bat_flg)                                                                                       // && (ball_position_temp.d < straightLineDistanceToBase_cm + 100))  //
                            && ((right_batter && (ball_position_temp.w < -39.0))                                            //
                                || (!right_batter && (ball_position_temp.w > 39.0)))                                        //
                            && isWithinRange(ball_position_temp.d, c_dis_cm(dep_base_kin) - 20, c_dis_cm(dep_base_en) + 60) //
                        );
                        if (
                            isInZoneNear && !isOutsideBatter
                            // ((zone_r_cm + 30) >= ballCenter_cm_x) && ((zone_l_cm - 30) <= ballCenter_cm_x) //
                            // && (*i_ball >= dep_l_en) && (*i_ball <= dep_l_kin)                     //
                        )
                        {
                            // 条件がすべて真である場合、オブジェクトはゾーンの周辺にあると判断され、以下のアクションが実行されます：
                            // retにbase_far_awayを代入し、これはおそらくオブジェクトがゾーンから遠い、または特定の状態にあることを示します。
                            // radiusにradius_tempを代入し、これはオブジェクトの半径または関連する何らかの測定値を設定します。
                            // last_dep_iにオブジェクトの深さ（*i_ball）を代入し、これは最後に検出されたオブジェクトの深さを記録します。
                            // last_ball_xとlast_ball_yにオブジェクトの中心位置（center.x、center.y）を整数型にキャストして代入し、これは最後に検出されたオブジェクトの位置を記録します。
                            ret = base_far_away;
                            radius = radius_temp;
                            zone_kyori_old = leng_tmp;
                            //zone_kyori_old = (leng_tmp == 0)? 99999:leng_tmp;

                            // ゾーン近辺か？
                            // あるオブジェクトが特定の「ゾーン」内に位置しているかどうか、そしてそのオブジェクトがバッターの領域の外側にあるかどうかを判断するためのものです。
                            // isInZone変数は、オブジェクトの中心（center_cm_x、center_cm_y）が指定されたゾーン内にあるかどうかを判断するために使用されます。このゾーンは、右端（zone_r_cm）、左端（zone_l_cm）、下端（zone_low_cm）、上端（zone_hi_cm）によって定義され、各辺からのマージン（18.4cmまたは45cm）を考慮しています。この条件が真であれば、オブジェクトはゾーン内にあると判断されます。
                            bool isInZone = ( //
                                // isWithinRange(ball_position_temp.w, zone_l_cm - 38.4, zone_r_cm + 38.4) //
                                (                                                            //
                                    (isWithinRange(ball_position_temp.d, 0, 280)             //
                                     && isWithinRange(ball_position_temp.w, -45.4, 45.4))    //
                                    || (isWithinRange(ball_position_temp.d, 280, 350)        //
                                        && isWithinRange(ball_position_temp.w, -48.4, 48.4)) //
                                    || (isWithinRange(ball_position_temp.d, 350, 500)        //
                                        && isWithinRange(ball_position_temp.w, -38.4, 38.4)) //
                                    )                                                        //
                                && isWithinRange(ball_position_temp.h, 5, zone_hi_cm + 45)   //
                            );
                            if (isInZone && !isOutsideBatter)
                            {
                                ret = base_shortly;
                                // STRIKE or BALL?
                                // ストライク判定
                                // このコードは、あるオブジェクト（おそらくボール）が特定のゾーン内にあるかどうかを判断し、それに基づいてストライクかボールかを判定するロジックを含んでいます。このプロセスは、野球やソフトボールのピッチングマシン、または自動的にストライクゾーンを判定するシステムで使用される可能性があります。
                                // まず、w_biasという変数が定義され、初期値として0が設定されています。この変数は、後にストライクゾーンの調整に使用される「補正値」を表します。次に、dep_dst2_cm変数に、c_dis_cm関数を使用してボールの深さ（距離）を計算し、代入します。
                                float w_bias = 0; // 5角柱の手前の範を補
                                float dep_dst2_cm = c_dis_cm(last_dep_i);
                                // horizontalDistanceToBase_cm （基準点の深さ）とdep_dst2_cm（ボールの深さ）を比較し、基準点の方が深い場合、w_biasに補正値を設定します。この補正値は、基準点とボールの深さの差に基づいており、0から21.6の範囲に制限されます。これにより、ストライクゾーンの手前の範囲を調整することができます。
                                if (horizontalDistanceToBase_cm > dep_dst2_cm)
                                {
                                    w_bias = clamp((horizontalDistanceToBase_cm - dep_dst2_cm), 0, 21.6); // 5角柱の手前の範を補
                                }
                                // 複数の条件を使用して、ボールがストライクゾーン内にあるかどうかを判断します。これらの条件は、ボールの中心位置（center_cm_x、center_cm_y）が、ゾーンの右端、左端、下端、上端からの補正値を考慮した範囲内にあるかどうかをチェックします。
                                if (((zone_r_cm - w_bias) >= ballCenter_cm_x)    //
                                    && ((zone_l_cm + w_bias) <= ballCenter_cm_x) //
                                    && ((zone_low_cm - 3.6) <= ballCenter_cm_y)  //
                                    && ((zone_hi_cm + 3.6) >= ballCenter_cm_y)   //
                                )
                                {
                                    // 深さの条件（dep_base_kin、dep_base_en）をチェックし、これらも満たされる場合は、*strikeをtrueに設定して、ストライクであると判定します。そうでない場合は、デバッグログに「ZONE kyori OUT」と出力します。
                                    if (isWithinRange(ball_position.d, c_dis_cm(dep_base_kin), c_dis_cm(dep_base_en)))
                                    {
                                        *strike = true;
                                        LOG_DEBUG_IF(!test_measurement_mode).printf(" STRIKE!! %d:ZONE kyori IN:%.0fcm", last_dep_i, c_dis_cm(last_dep_i));
                                    }
                                    else
                                    {
                                        LOG_DEBUG_IF(!test_measurement_mode).printf("  STRIKEコース　%d:ZONE kyori OUT:%.0fcm", last_dep_i, c_dis_cm(last_dep_i));
                                    }
                                    // LOG_DEBUG_IF(!test_measurement_mode).printf(" %d:ボール水平距離 %.0f,kin:%.0f,en:%.0f", i, ball_position_temp.d, c_dis_cm(dep_base_kin), c_dis_cm(dep_base_en));
                                }
                            }
                            else
                            {
                                LOG_DEBUG_IF(!test_measurement_mode && !isInZone).printf("  %d:not isInZone w:%.1f,h:%.1f,d:%.1f", i, ball_position_temp.w, ball_position_temp.h, ball_position_temp.d);
                            }
                        }
                        else //ball none
                        {
                            LOG_DEBUG_IF(!test_measurement_mode).printf("  %d:ZONE hazure w:%.1f(cm),h:%.1f(cm),%d(cm)", i, ballCenter_cm_x, ballCenter_cm_y, c_dis_cm(last_dep_i));
                            LOG_DEBUG_IF(!test_measurement_mode && isOutsideBatter).printf("  %d:isOutsideBatter Error! %.1f,%.1f,%.1f", i, ball_position_temp.w, ball_position_temp.h, ball_position_temp.d);
                        }
                        ball_position_temp_fix = ball_position_temp;
                    }
                }
                else
                {
                    if (isWithinRange(contours_ball[i][0].x + c_rect_x, 150, 450))
                        LOG_DEBUG_IF((isWithinRange(area_ball, c_mini_ball_area, c_max_ball_area) && enkeido >= 0.2 && ((ball_detection_phase != far_away && ball_detection_phase != invalid))) && !test_measurement_mode).printf("  %d:enkeido:%.2f enkeidoErr(x:%.0f,y:%.0f) area:%.1f", i, enkeido, center_temp.x, center_temp.y, area_ball);
                }
            }
            else
            {
                if (isWithinRange(contours_ball[i][0].x + c_rect_x, 150, 450))
                    LOG_DEBUG_IF(isWithinRange(area_ball, 10, c_max_ball_area) && (ball_detection_phase != far_away && ball_detection_phase != invalid) && !test_measurement_mode).printf("  %d:areaSize err:%.1f(%d,%d)", i, area_ball, contours_ball[i][0].x + c_rect_x, contours_ball[i][0].y);
                // area_ball = 0;
            }
        }
        if (ret != ball_none)
        {
            LOG_VERBOSE_IF(!test_measurement_mode).printf("  ボール画面位置(x,y):(%d,%d)", (int)center_temp.x, (int)center_temp.y);
            LOG_VERBOSE_IF(!test_measurement_mode).printf("  ボールまでの水平距離:%.0f dep:%d,dep(cm):%.1fcm", ball_position_temp.d, last_dep_i, ball_dep_cm);
            LOG_VERBOSE_IF(!test_measurement_mode).printf("  ボール大きさ ball area:%d,leng:%d,r_w:%.1f,r_h:%.1f", (int)area_ball, (int)leng, radius_temp, radius_h);
            LOG_VERBOSE_IF(!test_measurement_mode).printf("  ボール円形度(enkeido):%.2f", enkeido);
            LOG_VERBOSE_IF(!test_measurement_mode).printf("  ボール横位置(width(cm),(ベース相対):%.1fcm,(%.1fcm)", ballCenter_cm_x, ballCenter_cm_x - zone_c_dis_w_cm);
            LOG_VERBOSE_IF(!test_measurement_mode).printf("  ボール高さ(high(cm)):%.1fcm", ballCenter_cm_y);
            LOG_DEBUG_IF(!test_measurement_mode).printf("*** find ball ***");
            // break;
            straightLineDistanceToBall_cm = i_ball_cm_temp + ball_dep_bias; // ボールまでの距離を保持
            ball_position = ball_position_temp_fix;
            ball_position_old = ball_position;
            last_strike = *strike;
            *i_ball = last_dep_i;
        }
        else
        {
            last_strike = false;
            *i_ball = 0;
            hoseimae_center = Point(0, 0);
        }

        if (ball_detection_phase == invalid)
            ret = ball_none;
        return ret;
    }

    /**
* @brief 画像内の膝の位置を見つける関数。
*
* この関数は深度測定を利用し、2次元空間で膝の位置を計算します。
* 必要に応じて基本的な体の姿勢に対する補正も行います。
* This function calculates the position and depth of the knee, shoulder, hip, and wrist.
* It uses the positions of these body parts on a mask image to derive further information such as height classification.
* @param knee_x 画像内の膝のx座標。
* @param knee_y 画像内の膝のy座標。
* @param shoulder_x 画像内の肩のx座標。
* @param shoulder_y 画像内の肩のy座標。
* @param hip_x 画像内の腰のx座標。
* @param hip_y 画像内の腰のy座標。
* @param mask_image 深度測定に使用されるマスク画像。
*
* @return 補正された2次元空間の膝の位置を表すPointオブジェクト。
*/
    Point2i
    knee_find(int knee_x, int knee_y, int shoulder_x, int shoulder_y, int wrist_x, int wrist_y, int hip_x, int hip_y, Mat *mask_image) // bin_img:2値化画像
    {
        // このコードのセクションは、特定の身体部位（膝、肩、腰、手首）の位置を計算し、それらの深さを測定するためのものです。まず、膝のx座標またはy座標が0であるかどうかをチェックします。これは、膝の位置が検出されていないか、無効であることを示しています。その場合、関数は原点を表すPoint(0, 0)を返します。
        if ((knee_x == 0) || (knee_y == 0))
            return (Point(0, 0));

        // 膝の位置が有効である場合、膝、肩、腰、手首の各部位の位置を計算します。これらの位置は、各部位のx座標とy座標から、ある基準点（c_rect_x、c_rect_y）を減算することによって求められます。この基準点は、おそらく画像内の特定の領域を示しており、各部位の相対的な位置を計算するために使用されます。
        Point2i check_zahyou = {knee_x - c_rect_x, knee_y - c_rect_y};
        Point2i check_zahyou_shoulder = {shoulder_x - c_rect_x, shoulder_y - c_rect_y};
        Point2i check_zahyou_hip = {hip_x - c_rect_x, hip_y - c_rect_y};
        Point2i check_zahyou_wrist = {wrist_x - c_rect_x, wrist_y - c_rect_y};

        // 次に、depth_measure関数を使用して、計算された各部位の位置に基づいて深さを測定します。この関数は、部位の位置（Point2iオブジェクト）とマスク画像を引数として受け取り、その部位の深さを示す整数値を返します。マスク画像は、深さ測定に使用される画像データを含んでいる可能性があります。
        float knee_cm = depth_measure(check_zahyou, mask_image);
        float shoulder_cm = depth_measure(check_zahyou_shoulder, mask_image);
        float wrist_cm = depth_measure(check_zahyou_wrist, mask_image);
        float hip_cm = depth_measure(check_zahyou_hip, mask_image);
        shoulder_cm = 230;

        // 最後に、デバッグログに膝のx座標とy座標、および膝と肩の深さの測定値を出力します。
        PLOG_DEBUG.printf(" knee x,y:%.0f,%.0f", (float)check_zahyou.x, (float)check_zahyou.y);
        PLOG_DEBUG.printf(" knee_cm:%.1f,shoulder_cm:%.1f", knee_cm, shoulder_cm);

        //このコードは、特定の条件下で身体の各部位（膝、肩、手首、腰）の位置と距離を計算し、それらの情報をログに記録する処理を行っています。まず、shoulder_cmが0でないかどうかをチェックし、0でない場合（つまり、肩の位置が検出された場合）に処理を続けます。
        if (shoulder_cm != 0)
        {
            // 各部位の位置（check_zahyou、check_zahyou_shoulder、check_zahyou_hip）は、Point2iオブジェクトを使用して、特定の座標（c_rect_x、c_rect_y）を加算することで更新されます。これにより、各部位の現在の位置が更新されます。
            check_zahyou += Point2i(c_rect_x, c_rect_y);
            check_zahyou_shoulder += Point2i(c_rect_x, c_rect_y);
            check_zahyou_hip += Point2i(c_rect_x, c_rect_y);

            Dimensions dim_knee, dim_shoulder, dim_wrist, dim_hip;
            if (no_base_mode)
            {
                dim_knee = screenToWorld(check_zahyou, knee_cm);
                dim_shoulder = screenToWorld(check_zahyou_shoulder, shoulder_cm);
                dim_wrist = screenToWorld(check_zahyou_wrist, wrist_cm);
                dim_hip = screenToWorld(check_zahyou_hip, hip_cm);
                knee_cm_y_temp = dim_knee.h;
                knee_dep_cm = dim_knee.d;
                knee_cm_x_temp = dim_knee.w;
                shoulder_cm_x_temp = dim_shoulder.w;
                shoulder_cm_y_temp = dim_shoulder.h;
                shoulder_dep_cm = dim_shoulder.d;
                wrist_cm_x_temp = dim_wrist.w;
                wrist_cm_y_temp = dim_wrist.h;
                wrist_dep_cm = dim_wrist.d;
                hip_cm_x_temp = dim_hip.w;
                hip_cm_y_temp = dim_hip.h;
                hip_dep_cm = dim_knee.d;
            }
            else
            {
                dim_knee = calculateBallPositionByBase(knee_cm, check_zahyou);
                dim_shoulder = calculateBallPositionByBase(shoulder_cm, check_zahyou_shoulder);
                dim_wrist = calculateBallPositionByBase(wrist_cm, check_zahyou_wrist);
                dim_hip = calculateBallPositionByBase(hip_cm, check_zahyou_hip);
                knee_cm_y_temp = dim_knee.h;
                knee_dep_cm = dim_knee.d;
                knee_cm_x_temp = dim_knee.w;
                shoulder_cm_x_temp = dim_shoulder.w;
                shoulder_cm_y_temp = dim_shoulder.h;
                shoulder_dep_cm = dim_shoulder.d;
                wrist_cm_x_temp = dim_wrist.w;
                wrist_cm_y_temp = dim_wrist.h;
                wrist_dep_cm = dim_wrist.d;
                hip_cm_x_temp = dim_hip.w;
                hip_cm_y_temp = dim_hip.h;
                hip_dep_cm = dim_knee.d;
            }

            PLOG_DEBUG.printf("knee:%dcm dep:%.1fcm", (int)knee_cm_y_temp, knee_cm);
            PLOG_DEBUG.printf("soulder:%dcm dep:%.1fcm", (int)shoulder_cm_y_temp, shoulder_cm);
            PLOG_DEBUG.printf("wrist:%dcm dep:%.1fcm", (int)wrist_cm_y_temp, wrist_cm);
            PLOG_DEBUG.printf("hip:%dcm dep:%.1fcm", (int)hip_cm_y_temp, hip_cm);
        }
        else
        {
            // shoulder_cmが0の場合、つまり肩の位置が検出されなかった場合は、knee_cm_y_tempを0に設定し、check_zahyouを原点にリセットします。これは、肩の位置が検出されなかった場合のデフォルトの処理を表しています。
            knee_cm_y_temp = 0;
            check_zahyou = Point(0, 0);
        }

        // このコードのセクションは、膝の高さが検知された場合に特定の処理を行うためのものです。まず、shoulder_cm_y_temp変数の値をチェックして、それが0でない場合（つまり、何らかの高さが検知された場合）、その値を使用して処理を進めます。この値は、恐らく肩の高さをセンチメートル単位で表しており、これを基にバッターの身長を推定しています。
        LOG_DEBUG_IF(shoulder_cm_y_temp != 0).printf("shoulder_cm_y_temp:%.0f", shoulder_cm_y_temp);
        if (shoulder_cm_y_temp != 0)
        {
            //身長の推定は、shoulder_cm_y_tempに1.32を乗じることで行われます。この計算により得られた値（batter_hight_cm）をもとに、バッターの身長範囲を分類し、それぞれの範囲に応じてbatter_hight変数に異なる値を割り当てます。この値は、後にゾーンの高さを調整する際に使用されます。
            float batter_hight_cm = shoulder_cm_y_temp * 1.32;
            // 肩の高さに基づいてバッターの高さを調整
            // 身長範囲は、175cm以上、165cmから175cm未満、155cmから165cm未満、145cmから155cm未満、135cmから145cm未満、125cmから135cm未満、そして125cm未満の7つに分けられています。それぞれの範囲に対して、batter_hightに0から6までの値が割り当てられ、ログにその身長範囲が出力されます。
            if (batter_hight_cm >= 175)
            {
                batter_hight = 0;
                LOG_DEBUG.printf("打者身長　１７５~ｃｍ");
            }
            else if (isWithinRange(batter_hight_cm, 165, 175))
            {
                batter_hight = 1;
                LOG_DEBUG.printf("打者身長　１６５~１７５ｃｍ");
            }
            else if (isWithinRange(batter_hight_cm, 155, 165))
            {
                batter_hight = 2;
                LOG_DEBUG.printf("打者身長　１５５~１６５ｃｍ");
            }
            else if (isWithinRange(batter_hight_cm, 145, 155))
            {
                batter_hight = 3;
                LOG_DEBUG.printf("打者身長　１４５~１５５ｃｍ");
            }
            else if (isWithinRange(batter_hight_cm, 135, 145))
            {
                batter_hight = 4;
                LOG_DEBUG.printf("打者身長　１３５~１４５ｃｍ");
            }
            else if (isWithinRange(batter_hight_cm, 125, 135))
            {
                batter_hight = 5;
                LOG_DEBUG.printf("打者身長　１２５~１３５ｃｍ");
            }
            else
            {
                batter_hight = 6;
                LOG_DEBUG.printf("打者身長　~１２５ｃｍ");
            }
        }
        else
        {
            // shoulder_cm_y_tempが0の場合、つまり肩の高さが検知されなかった場合は、batter_hightにデフォルト値として1が設定されます。
            batter_hight = 1; // デフォルトの高さを設定
        }
        shoulder_cm_y_temp = 0;

        return check_zahyou;
    }

    /** 
     * @brief 前回検知したボールの位置を更新する関数。
     *
     */
    bool updateOldballPosition(int ball_detection_phase)
    {
        LOG_DEBUG.printf("ボールの位置を更新します。古い位置: (%.0f, %.0f)", center_temp_old.x, center_temp_old.y);
        center_temp_old = center;
        enkeido_old = enkeido;
        LOG_DEBUG.printf("ボールの位置を更新しました。新しい位置: (%.0f, %.0f)", center_temp_old.x, center_temp_old.y);
        if (isSecondCamera)
        {
            center_temp_old_s = center_temp_old;
            LOG_DEBUG.printf(" center_temp_old_f (%.0f, %.0f)", center_temp_old_f.x, center_temp_old_f.y);
        }
        else
        {
            center_temp_old_f = center_temp_old;
            LOG_DEBUG.printf(" center_temp_old_s (%.0f, %.0f)", center_temp_old_s.x, center_temp_old_s.y);
        }
        LOG_DEBUG.printf("\n");
        return true;
    }
};

/**
 * @brief 最小二乗法を用いたロバスト推定による2次式の係数計算
 *
*/
struct QuadraticCoefficients
{
    float a; // 二次の係数
    float b; // 一次の係数
    float c; // 定数項
};

bool isWithinTolerance(float residual, float threshold)
{
    return std::abs(residual) < threshold;
}

#include <algorithm>
#include <cmath>
#include <vector>

// 中央値を計算する関数
float median(std::vector<float> vec)
{
    size_t n = vec.size() / 2;
    std::nth_element(vec.begin(), vec.begin() + n, vec.end());
    return vec[n];
}

// 中央値絶対偏差（MAD）を計算する関数
float mad(const std::vector<float> &vec, float med)
{
    std::vector<float> abs_dev(vec.size());
    for (size_t i = 0; i < vec.size(); ++i)
    {
        abs_dev[i] = std::abs(vec[i] - med);
    }
    return median(abs_dev);
}

// 二次関数へのフィッティング
QuadraticCoefficients fitConstrainedQuadratic(const std::vector<float> &x, const std::vector<float> &y, float p0_min, float p0_max, float p1_min, float p1_max, bool weightB = false)
{
    const size_t n = x.size();
    if (n != y.size() || n < 3)
    {
        // エラー処理: データ点が不十分または x と y のサイズが一致しない
        return {0, 0, 0};
    }

    auto computeError = [&](float a, float b, float c) {
        float error = 0;
        for (size_t i = 0; i < n; ++i)
        {
            float diff = y[i] - (a * x[i] * x[i] + b * x[i] + c);
            error += diff * diff;
        }
        if (weightB)
        {
            // bが0に近いほど誤差を小さくする
            error *= (1 + std::abs(b));
        }
        return error;
    };

    float best_a = 0, best_b = 0, best_c = 0;
    float min_error = std::numeric_limits<float>::max();

    // グリッドサーチによる最適化
    const int grid_size = 100;
    for (int i = 0; i <= grid_size; ++i)
    {
        float a = p0_min + (p0_max - p0_min) * i / grid_size;
        for (int j = 0; j <= grid_size; ++j)
        {
            float b = p1_min + (p1_max - p1_min) * j / grid_size;

            // c の最適値を解析的に計算
            float sum_y = 0, sum_x = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0, sum_xy = 0, sum_x2y = 0;
            for (size_t k = 0; k < n; ++k)
            {
                float xi = x[k], yi = y[k];
                sum_y += yi;
                sum_x += xi;
                sum_x2 += xi * xi;
                sum_x3 += xi * xi * xi;
                sum_x4 += xi * xi * xi * xi;
                sum_xy += xi * yi;
                sum_x2y += xi * xi * yi;
            }
            float c = (sum_y - a * sum_x2 - b * sum_x) / n;

            float error = computeError(a, b, c);
            if (error < min_error)
            {
                min_error = error;
                best_a = a;
                best_b = b;
                best_c = c;
            }
        }
    }

    return {best_a, best_b, best_c};
}

/*
＜関数の概要＞
robustQuadraticRegression 関数は、ノイズのロバスト推定を用いて二次回帰を行います。この関数の目的と機能は以下の通りです：
・依存変数（dep）に基づいて x と y 座標に二次曲線をフィッティングし、ノイズをフィルタリングします。
・曲線フィッティングに制約を用いて外れ値を検出し、それに基づいて外れ値を除去します。
・外れ値を除去した後のデータでもう一度フィッティングを行います。
＜主な構成要素＞
関数の引数:
　ball_dep_cm, ball_x_cm, ball_y_cm: 依存変数とそれに対応する x および y 座標を示す配列です。
　loop_dst: 配列内のポイント数。
　coeffX, coeffY: x と y の二次係数を格納するオブジェクトへの参照です。
　per_hazure: 処理後に外れ率を格納するためのポインタ。
事前定義された制約:
　x および y 座標の二次係数に対する制約として _min と _max が設定されています。
データの初期化:
　入力配列を std::vector にコピーし、操作しやすくしています。
閾値の計算:
　平均残差を使用して一時的な外れ値検出用の閾値を計算します。
二次回帰とノイズフィルタリング:

　x 軸の二次回帰:
　x データに二次関数をフィッティングし、残差を計算します。
　定義された閾値を超えるポイントを除去し、フィルタリングし、ロバスト（IN）または外れ（ハズレ）としてログを取ります。
　y 軸の二次回帰:
　x と同様のプロセスを y データに対して行います。
再フィッティング:

　外れ値を除去した後のフィルタリング済みデータに対して、再度二次回帰を行います。
外れ率の計算:

　per_hazure は、外れ率（全データポイントに対する外れ値の割合）として計算されます。
全体のまとめ
　この関数は、ノイズのあるデータに対してロバストな二次回帰を行い、
　フィッティング中に係数に制約を掛け、残差を計算して外れ値を検出、削除します。
　フィルタリング後のクリーンなデータに対して再度フィッティングを行い、精度を向上させます。
補足
　LOG_DEBUG は仮定ですが、デバッグのために随所で使用され、操作の状態や分類されたデータポイントを出力します。
　fitConstrainedQuadratic と isWithinTolerance は、フィッティングと残差の許容範囲チェックを行うヘルパー関数であると仮定しています。
　c_robustnes_cm のような定数はコードの他の部分で定義されていると思われ、閾値チェックの際の許容誤差を表しています。
*/

// ロバスト推定によるノイズ削除と回帰
void robustQuadraticRegression(
    const float ball_dep_cm[],
    const float ball_x_cm[],
    const float ball_y_cm[],
    int loop_dst,
    QuadraticCoefficients &coeffX,
    QuadraticCoefficients &coeffY,
    float *per_hazure) //ハズレ率
{
    // x軸とy軸の二次係数に対する制約
    const float x_p0_max = 0.0007;
    const float x_p0_min = -0.0007;
    const float x_p1_max = 0.25;
    const float x_p1_min = -0.25;
    const float x_p2_max = 60;
    const float x_p2_min = -60;

    const float y_p0_max = 0.000;
    const float y_p0_min = -0.0035;
    const float y_p1_max = 2.5;
    const float y_p1_min = -0.15;
    const float y_p2_max = 150;
    const float y_p2_min = -150;

    // 入力データをコピー
    std::vector<float> dep(ball_dep_cm, ball_dep_cm + loop_dst);
    std::vector<float> x(ball_x_cm, ball_x_cm + loop_dst);
    std::vector<float> y(ball_y_cm, ball_y_cm + loop_dst);

    float threshold = c_robustnes_cm; // 外れ値検出の許容誤差

    // x軸の二次曲線回帰（bが0に近いほど高い重みを与える）
    coeffX = fitConstrainedQuadratic(dep, x, x_p0_min, x_p0_max, x_p1_min, x_p1_max, true);
    LOG_DEBUG.printf("ロバスト係数 X a:%.6f,b:%.3f,c:%.0f", coeffX.a, coeffX.b, coeffX.c);

    int in_x = 0;
    int out_x = 0;
    int in_y = 0;
    int out_y = 0;
    // 残差を計算しノイズを除去
    std::vector<float> xFiltered, depXFiltered;
    float total_residual = 0;
    int cnt = 0;

    // 中央値を求める
    std::vector<float> residuals_X;
    residuals_X.reserve(loop_dst);
    for (int i = 0; i < loop_dst; ++i)
    {
        float residual = x[i] - (coeffX.a * dep[i] * dep[i] + coeffX.b * dep[i] + coeffX.c);
        residuals_X.push_back(residual);
    }

    /* 中央値を計算して閾値を設定
    if (!residuals_X.empty())
    {
        std::sort(residuals_X.begin(), residuals_X.end());
        size_t size = residuals_X.size();
        if (size % 2 == 0)
        {
            threshold = (residuals_X[size / 2 - 1] + residuals_X[size / 2]) / 2.0f;
        }
        else
        {
            threshold = residuals_X[size / 2];
        }
    }
    else
    {
        threshold = 0.0f; // または適切なデフォルト値
    }
    threshold = clamp(threshold, -c_robustnes_cm, c_robustnes_cm);
    */
    threshold = 0.0f;
    LOG_DEBUG.printf(" w threshold %.1f", threshold);

    // 残差を計算し、ノイズを除去
    for (int i = 0; i < loop_dst; ++i)
    {
        float residual = x[i] - (coeffX.a * dep[i] * dep[i] + coeffX.b * dep[i] + coeffX.c);
        if (isWithinTolerance(residual - threshold, c_robustnes_cm))
        {
            depXFiltered.push_back(dep[i]);
            xFiltered.push_back(x[i]);
            in_x++;
            LOG_DEBUG.printf("w ロバストIN,dep:%.1f,w:%.1f,res:%.1f", dep[i], x[i], residual);
        }
        else
        {
            LOG_DEBUG.printf("w ロバストハズレ,dep:%.1f,w:%.1f,res:%.1f", dep[i], x[i], residual);
            out_x++;
        }
    }

    // y軸の二次曲線回帰
    coeffY = fitConstrainedQuadratic(dep, y, y_p0_min, y_p0_max, y_p1_min, y_p1_max, false);
    LOG_DEBUG.printf("ロバスト係数 Y a:%.6f,b:%.3f,c:%.0f", coeffY.a, coeffY.b, coeffY.c);

    // 残差を計算しノイズを除去
    std::vector<float> yFiltered, depYFiltered;
    total_residual = 0;
    cnt = 0;

    // 中央値を求める
    std::vector<float> residuals_Y;
    residuals_Y.reserve(loop_dst);
    for (int i = 0; i < loop_dst; ++i)
    {
        float residual = y[i] - (coeffY.a * dep[i] * dep[i] + coeffY.b * dep[i] + coeffY.c);
        residuals_Y.push_back(residual);
    }

    // 中央値を計算して閾値を設定
    if (!residuals_Y.empty())
    {
        std::sort(residuals_Y.begin(), residuals_Y.end());
        size_t size = residuals_Y.size();
        if (size % 2 == 0)
        {
            threshold = (residuals_Y[size / 2 - 1] + residuals_Y[size / 2]) / 2.0f;
        }
        else
        {
            threshold = residuals_Y[size / 2];
        }
    }
    else
    {
        threshold = 0.0f; // または適切なデフォルト値
    }
    threshold = clamp(threshold, -c_robustnes_cm - 2, c_robustnes_cm + 2);
    LOG_DEBUG.printf(" h threshold %.1f", threshold);

    // 残差を計算し、ノイズを除去
    for (int i = 0; i < loop_dst; ++i)
    {
        float residual = y[i] - (coeffY.a * dep[i] * dep[i] + coeffY.b * dep[i] + coeffY.c);
        if (isWithinTolerance(abs(residual - threshold), c_robustnes_cm + 2))
        {
            depYFiltered.push_back(dep[i]);
            yFiltered.push_back(y[i]);
            in_y++;
            LOG_DEBUG.printf("h ロバストIN,dep:%.1f,h:%.1f,res:%.1f", dep[i], y[i], residual);
        }
        else
        {
            LOG_DEBUG.printf("h ロバストハズレ,dep:%.1f,h:%.1f,res:%.1f", dep[i], y[i], residual);
            out_y++;
        }
    }

    // 最終的にノイズ除去後のデータで再回帰
    if (in_x > 0 && in_y > 0)
    {
        coeffX = fitConstrainedQuadratic(depXFiltered, xFiltered, x_p0_min, x_p0_max, x_p1_min, x_p1_max, false);
        coeffY = fitConstrainedQuadratic(depYFiltered, yFiltered, y_p0_min, y_p0_max, y_p1_min, y_p1_max, false);
    }

    // 外れ率を計算
    *per_hazure = (float)(out_x + out_y) / (in_x + in_y + out_x + out_y);
    if (in_x <= 2 || in_y <= 2)
        *per_hazure = 1;
}
