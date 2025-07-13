#include <stdint.h>
#define c_fps_rgb 15
#define c_fps_mono 200
#define c_imu_fps 30
#define c_imu_wait 0.600   // s
#define c_automode_wait 15 // s
#define c_exp_time_mono 500
#define c_sens_iso_mono 100
#define c_rez_x 800
#define c_rez_y 480
#define c_mono_rez_x 640
#define c_mono_rez_y 400
#define raspi true // true or false
#define raspi5 true
#define double_camera true
#define c_imu true  //imu use
#define detNN false // HUMAN検知有無//#define c_zone_low_cm 50      // zone_low_cm = (50 - (3 * batter_height)); // 低め https://okwave.jp/qa/q3922955.html
// #define c_zone_hi_cm 110      // zone_hi_cm = (100 - (7 * batter_height)); // 高め

#define c_dot_hi_err 3.7

#define c_dep_l_en 7 * 8      // 遠方フィルタ 300cm 高さによって減算しているので注意
#define c_dep_l_kin 25 * 8    // 近距離フィルタ
#define c_dep_base_en 9 * 8   // base kyori keisannjyouha 13 16
#define c_dep_base_kin 16 * 8 //
#define c_dep_cm_en 500
#define c_Judgment_wait 150 // ns
// BASE size (area:5411,leng:300)
// 2023-08-18 06:58:25.139 DEBUG [11891] [Base::Base_find@582]  base_w,h:114,67
// 2023-08-18 07:17:26.196 DEBUG [14542] [Base::Base_find@490] BASE size (area:4287,leng:271)
// 2023-08-18 07:17:26.196 DEBUG [14542] [Base::Base_find@582]  base_w,h:105,58
#define c_base_area_max 4000
#define c_base_area_min 400
#define c_base_len_max 350
#define c_base_len_min 80
#define c_base_w_max 120
#define c_base_w_min 60
#define c_base_h_max 90
#define c_base_h_min 20
//ボール検知インターバル ms(これを超えるとボールクリア)
#define c_ball_interval 80
#define c_ball_timeout 350
//! base nankobun hanareteruka
#define c_rect_x_hi 0.3f // 0.3f,0.9f -> wh:256,360 xy:192-448,0-360
#define c_rect_x_hi_bias_720p 0.09f
#define c_rect_y_lo 1.0f
#define c_rect_x (int)(640 * c_rect_x_hi) // 640*0.3
#define c_rect_x_bias_720p (int)(640 * c_rect_x_hi_bias_720p)
#define c_rect_y 0
#define c_rect_y_h 400 * c_rect_y_lo // 400*0.9
#define c_rect_zone_y 220
#define c_no_noize_area 40 //ボールの初期検知箇所の制約も兼ねている
#define plog_level plog::debug
/*enum Severity
    {
        none = 0,
        fatal = 1,
        error = 2,
        warning = 3,
        info = 4,
        debug = 5,
        verbose = 6
    };*/
#define rokuga_sw 0 // 0:off 1:on
//! monoカメラとRGBカメラとの位置補正
/*
                                    center_temp.x = (center_temp.x - 320) * 1.2 + 320(c_bias_x);
                                    center_temp.y = (center_temp.y - 200) * 1.2 + 220(c_bias_y);
*/

#define c_hfov 72.90 // ork-d lite 73(640x480)  ork-d 77(1280x800)
#define c_vfov_rgb 55.00
#define c_vfov_mono 49.00
//#define c_dfov 81               // ork-d lite 86(640x480)  ork-d 82(1280x800)
//#define c_hfov_rgb 69           // ork-d lite 69  ork-d 69
//#define c_vfov_rgb 55           // ork-d lite 54  ork-d 55
#define c_focal_length 640 * 0.5 / tan(c_hfov * 0.5 * 3.14 / 180)      // https://docs.luxonis.com/projects/api/en/latest/samples/calibration/calibration_reader/#camera-intrinsics
#define c_fov focal_length * 7.5 * 8                                   // https://docs.luxonis.com/projects/api/en/latest/tutorials/configuring-stereo-depth/#how-baseline-distance-and-focal-length-affect-depth
#define c_focal_length_720 1280 * 0.5 / tan(c_hfov * 0.5 * 3.14 / 180) // https://docs.luxonis.com/projects/api/en/latest/samples/calibration/calibration_reader/#camera-intrinsics

/*
-----ORK-D-----
Color camera
IMX378 (PY011 AF)
DFOV / HFOV / VFOV
81° / 69° / 55°

Stereo pair
OV9282 (PY010 FF)
DFOV / HFOV / VFOV
81° / 72° / 49°
-----------
-----ORK-D Lite-----
Color camera
IMX214 (PY047 AF, PY062 FF)
DFOV / HFOV / VFOV
81° / 69° / 54°

Stereo pair
OV7251 (PY013)
DFOV / HFOV / VFOV
86° / 73° / 58°
-----------
*/
// #define c_speed_lim_min 15 // 検知スピードリミッター（低速） cm 100fpsで5cm程度
#define c_zone_r_cm 21.6 + 3.6
#define c_zone_l_cm -21.6 - 3.6
#define c_robustnes_cm 3.8 // ロバスト推定 外れ値
// TOP irasuto
//  TOP フロント両辺：321*237-463*380 ベースの底頂点：393*380
// #define c_base_illust_haba=463-321;

//                        int x = 400 - (home_center - ball[top_i].x) * (142 / (float)(home_haba));
//                        int y = (206*3.15)+380 - ((float)(441.25 * 7.5) / top_i * 3.15);

/*
    enum Severity
    {
        none = 0,
        fatal = 1,
        error = 2,
        warning = 3,
        info = 4,
        debug = 5,
        verbose = 6
    };
*/
typedef enum
{
    ball_none,
    base_shortly,
    base_far_away,
    shape_none
} Ball_f;
// enum値を文字列に変換する関数
const char *BallResultToString(Ball_f result)
{
    switch (result)
    {
    case ball_none:
        return "未検出";
    case base_shortly:
        return "ゾーン近辺";
    case base_far_away:
        return "ゾーン外れ";
    default:
        return "不明";
    }
}
// 10 330cm
// 11 300cm
// 12 275cm
// 13 254cm base_front
// 14 236cm
// 15 220cm
// 16 206cm base_back
// 17 194cm
// 18 183cm
// 19 174cm
// 20 165cm

//! 録画対象
typedef enum
{
    rec_BGR,
    rec_bin,
    rec_depth,
};

//! ボール検知距離
typedef enum
{
    far_away,
    beyond_the_base,
    on_the_base,
    passing_the_base,
    invalid,
};

//! ボール検知エラーコード
typedef enum
{
    gamen_hashi_err,
    radius_hi_err,
    radius_size_err,
    ball_dot_hi_err,
    kyori_err,
    ball_size_err,
    noise_check_err,
    batter_pos_err,
    takasa_err,
    dep_cnt_err,
    base_haba_err
} ball_err_cd;
// enum値を文字列に変換する関数
const char *BallErrResultToString(ball_err_cd result)
{
    switch (result)
    {
    case gamen_hashi_err:
        return "画面端エラー";
    case radius_hi_err:
        return "半径縦横比エラー";
    case radius_size_err:
        return "半径サイズエラー";
    case ball_dot_hi_err:
        return "周辺ノイズエラー";
    case kyori_err:
        return "他に近い円あり";
    case ball_size_err:
        return "ボールサイズエラー";
    case noise_check_err:
        return "ノイズチェックエラー";
    case batter_pos_err:
        return "バッター位置エラー";
    case takasa_err:
        return "高さハズレ";
    case dep_cnt_err:
        return "距離計測エラー";
    case base_haba_err:
        return "ベース幅エラー";
    default:
        return "不明";
    }
}
//!//投球されたボールのコース（0:ど真ん中、1:高め,2:真ん中低め,3:左右真ん中以下低め）
typedef enum
{
    c_strike_center,
    c_strike_hi,
    c_strike_lo,
    c_strike_nice,
    c_ball_hi,
    c_ball_lo,
    c_ball_right,
    c_ball_left
};

// ボール位置　構造体の定義
struct Dimensions
{
    float w; //横位置
    float h; //高さ
    float d; //水平距離
    // デフォルトコンストラクタを追加
    Dimensions()
      : w(0)
      , h(0)
      , d(0)
    {}
    // コンストラクタを追加
    Dimensions(float width, float height, float depth)
      : w(width)
      , h(height)
      , d(depth)
    {}
};
/**
 * @brief 3D 空間におけるボールの軌跡を記録する構造体
 */
struct ball_cours
{
    int cnt;         //!< ボールの検出回数
    int dep;         //!< 画像から計算されたボールの深度
    float dep_cm;    //!< 画像から計算されたボールの深度 (cm)
    int x;           //!< 画像上のボールのX座標
    int y;           //!< 画像上のボールのY座標
    int z;           //!< 画像上のボールのZ座標 (深度)
    int hosei_x;     //!< ゾーン傾き補正後のX座標
    int hosei_y;     //!< ゾーン傾き補正後のY座標
    float w_cm;      //!< ベース中央からの距離 (cm)
    float h_cm;      //!< 実空間上の高さ (cm)
    int radius;      //!< ボールの半径
    bool strike;     //!< ボールがストライクゾーン内にあるかどうか
    bool ball_hi;    //!< ボールが高い位置にあるかどうか
    bool ball_lo;    //!< ボールが低い位置にあるかどうか
    bool ball_right; //!< ボールが右側にあるかどうか
    bool ball_left;  //!< ボールが左側にあるかどうか
    uint64_t time;   //!< ボールの検出時刻 (マイクロ秒)
    int camera_no;   //!<フレームを撮影したカメラ番号
    bool filter;     //!< 撮影時間以上フィルタ
};

#define c_ball_noise_i_max 50

//サイドビュー画像アドレス
#define c_base_center_x 496
#define c_base_center_y 632
#define c_base_haba_tate_u 579 // y
#define c_base_haba_tate_d 688
#define c_base_haba_yoko_l 386 // x
#define c_base_haba_yoko_r 588
#define c_zone_takasa_l 489
#define c_zone_takasa_h 236
#define c_gnd_zero 633
//! 投手ビュー画像アドレス　
//インロー座標
#define c_in_lo_x 154  // 255
#define c_in_lo_y 224  // 262
#define c_out_hi_x 85  // 186
#define c_out_hi_y 138 // 176
#define c_center_x 120
#define c_center_y 181
#define c_grnd_y 287

/**
 * @brief 値を範囲内に収める。
 * @param int 入力値
 * @param int 最小値
 * @param int 最大値
 * @return 範囲内におさめられた数値
 * @sa
 */
template<typename T, typename U, typename V>
T clamp(T num, U v0, V v1)
{
    // v0 と v1 を num の型にキャストして比較
    T minVal = static_cast<T>(std::min(static_cast<T>(v0), static_cast<T>(v1)));
    T maxVal = static_cast<T>(std::max(static_cast<T>(v0), static_cast<T>(v1)));
    return std::max(minVal, std::min(num, maxVal));
}
/**
 * @brief システム時刻取得(s)
 * @param
 */
uint64_t get_sec()
{
    extern uint64_t tv_sec_init;
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return ((uint64_t)(ts.tv_sec) - tv_sec_init);
}

uint64_t get_m_sec()
{
    extern uint64_t tv_m_sec_init;
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return ((uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000 - tv_m_sec_init);
}

uint64_t get_minutes()
{
    // 現在の時刻を取得
    auto now = std::chrono::system_clock::now();

    // エポック（1970年1月1日）からの経過時間をミリ秒で取得
    auto duration = now.time_since_epoch();

    // ミリ秒を分に変換
    auto minutes = std::chrono::duration_cast<std::chrono::minutes>(duration);

    // uint64_tにキャストして返す
    return static_cast<uint64_t>(minutes.count());
}

/**
 * @brief 画像の保存
 * @param filename 画像ファイル名
 * @param img 画像データへのポインタ
 * @return なし
 * @note 画像をファイルに保存するために使用する。
 * 画像データをファイルに書き出し、その後、必要な処理を実行します。
 */
void matfilesave(char *filename, Mat *img)
{
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);

    auto now_msec = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_msec.time_since_epoch());
    int milliseconds = ms.count() % 1000;

    char buff[128] = "";
    sprintf(buff, "_%d_%02d_%02d_%02d%02d%02d_%02d.jpg", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, pnow->tm_sec, milliseconds);
    string file2 = buff;
    string file = filename + file2;
    imwrite(file, *img);
    LOG_NONE.printf("save image file: %s", file.c_str());
}

/**
 * @brief 画像を指定された座標およびサイズで別の画像に貼り付ける関数
 * @param dst ソース画像が貼り付けられる対象の画像
 * @param src 対象の画像に貼り付けられるソース画像
 * @param x ソース画像が対象の画像に貼り付けられる左上隅のX座標
 * @param y ソース画像が対象の画像に貼り付けられる左上隅のY座標
 * @param width 対象の画像に貼り付けられるソース画像の幅
 * @param height 対象の画像に貼り付けられるソース画像の高さ
 * @return void
 * @sa paste(cv::Mat dst, cv::Mat src, int x, int y)
 */
void paste(cv::Mat dst, cv::Mat src, int x, int y, int width, int height)
{
    cv::Mat resized_img;
    cv::resize(src, resized_img, cv::Size(width, height));

    if (x >= dst.cols || y >= dst.rows)
        return;
    int w = (x >= 0) ? std::min(dst.cols - x, resized_img.cols) : std::min(std::max(resized_img.cols + x, 0), dst.cols);
    int h = (y >= 0) ? std::min(dst.rows - y, resized_img.rows) : std::min(std::max(resized_img.rows + y, 0), dst.rows);
    int u = (x >= 0) ? 0 : std::min(-x, resized_img.cols - 1);
    int v = (y >= 0) ? 0 : std::min(-y, resized_img.rows - 1);
    int px = std::max(x, 0);
    int py = std::max(y, 0);

    cv::Mat roi_dst = dst(cv::Rect(px, py, w, h));
    cv::Mat roi_resized = resized_img(cv::Rect(u, v, w, h));
    roi_resized.copyTo(roi_dst);
}

void paste2(Mat dst, Mat src, int x, int y, int width, int height)
{
    // 埋め込みたいサイズに縮小（リサイズ）
    cv::Size newSize(width, height); // 新しいサイズ（幅100、高さ100にリサイズ）
    cv::resize(src, src, newSize);

    // 貼り付ける位置を指定（例: x=50, y=50の位置に貼り付ける）
    int x_offset = x;
    int y_offset = y;

    // ROI（Region of Interest）を設定
    cv::Mat roi = dst(cv::Rect(x_offset, y_offset, src.cols, src.rows));

    // 埋め込み先にリサイズされた画像をコピー
    src.copyTo(roi);
}

/** 画像の中に別の画像をインポートする関数（サイズ指定を省略したバージョン）
 * @param Mat 対象画像
 * @param Mat インポート画像
 * @param int x
 * @param int y
 * @return
 * @sa
 */
void paste(cv::Mat dst, cv::Mat src, int x, int y)
{
    paste(dst, src, x, y, src.cols, src.rows);
}

// 画素値が０の場合は上書きしない(モノクロ専用)
void paste3(cv::Mat dst, cv::Mat src, int x, int y, int width, int height)
{
    cv::Mat resized_img;
    cv::resize(src, resized_img, cv::Size(width, height));

    if (x >= dst.cols || y >= dst.rows)
        return;
    int w = (x >= 0) ? std::min(dst.cols - x, resized_img.cols) : std::min(std::max(resized_img.cols + x, 0), dst.cols);
    int h = (y >= 0) ? std::min(dst.rows - y, resized_img.rows) : std::min(std::max(resized_img.rows + y, 0), dst.rows);
    int u = (x >= 0) ? 0 : std::min(-x, resized_img.cols - 1);
    int v = (y >= 0) ? 0 : std::min(-y, resized_img.rows - 1);
    int px = std::max(x, 0);
    int py = std::max(y, 0);

    cv::Mat roi_dst = dst(cv::Rect(px, py, w, h));
    cv::Mat roi_resized = resized_img(cv::Rect(u, v, w, h));

    // 画素ごとに処理
    for (int i = 0; i < roi_resized.rows; ++i)
    {
        for (int j = 0; j < roi_resized.cols; ++j)
        {
            uchar pixel = roi_resized.at<uchar>(i, j);
            // 画素値が0でない場合のみ上書き
            if (pixel != 0)
            {
                roi_dst.at<uchar>(i, j) = pixel;
            }
        }
    }
}

void paste3(cv::Mat dst, cv::Mat src, int x, int y)
{
    paste3(dst, src, x, y, src.cols, src.rows);
}

/**
 * @brief 回転させた画像を、中心位置を揃えて別の画像に貼り付ける関数（リサイズなし）
 * @param dst 貼り付け先の画像
 * @param src 貼り付け元の画像（そのままのサイズで使用）
 * @param angle_deg 時計回りの回転角度（度）
 */
void pasteCenterAligned(cv::Mat dst, cv::Mat src, double angle_deg)
{
    // 回転中心
    cv::Point2f center(src.cols / 2.0f, src.rows / 2.0f);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle_deg, 1.0);

    // 回転後のサイズを計算
    cv::Rect2f bbox = cv::RotatedRect(center, src.size(), angle_deg).boundingRect2f();

    // 平行移動補正
    rot.at<double>(0, 2) += bbox.width / 2.0 - center.x;
    rot.at<double>(1, 2) += bbox.height / 2.0 - center.y;

    // 回転画像生成
    cv::Mat rotated_img;
    cv::warpAffine(src, rotated_img, rot, bbox.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

    // 中心を合わせるための貼り付け先の左上座標
    int x = (dst.cols - rotated_img.cols) / 2;
    int y = (dst.rows - rotated_img.rows) / 2;

    // 範囲チェックと切り出し
    int w = std::min(rotated_img.cols, dst.cols - std::max(0, x));
    int h = std::min(rotated_img.rows, dst.rows - std::max(0, y));
    if (w <= 0 || h <= 0)
        return;

    int px = std::max(x, 0);
    int py = std::max(y, 0);
    int u = (x < 0) ? -x : 0;
    int v = (y < 0) ? -y : 0;

    cv::Mat roi_dst = dst(cv::Rect(px, py, w, h));
    cv::Mat roi_src = rotated_img(cv::Rect(u, v, w, h));
    roi_src.copyTo(roi_dst);
}

/** 透過画像オーバーライド
 * @brief
 * @param Mat 対象画像
 * @param Mat PNG画像
 * @param int x座標
 * @param int y座標
 * @return TRUE
 * @sa
 * @details
 */
void putTranspPng(cv::Mat &_frame, cv::Mat &_png_o, int _x, int _y, int width, int height)
{
    cv::Mat _png;
    cv::resize(_png_o, _png, cv::Size(width, height));

    if (((_x + _png.cols) > _frame.cols) || _x < 0)
    {
        PLOG_ERROR.printf(" base_cols %d", (_x + _frame.cols));
        PLOG_ERROR.printf(" cols error %d", (_x + _png.cols));
        return;
    }
    if (((_y + _png.rows) > _frame.rows) || _y < 0)
    {
        PLOG_ERROR.printf(" base_rows %d", (_y + _frame.rows));
        PLOG_ERROR.printf(" rows error %d", (_y + _png.rows));
        return;
    }
    vector<cv::Mat> layers;
    cv::split(_png, layers);
    // 下記の処理は_pngがRGBAの4チャンネルMatである必要がある
    if (4 != layers.size())
    {
        cerr << "putTranspPng() invalid input" << endl;
        PLOG_ERROR.printf("dims:%d", layers.size());
    }
    // 貼り付ける画像（3チャンネル）
    cv::Mat rgb;
    cv::merge(layers.data(), 3, rgb);
    // copyToに使うmask（1チャンネル）
    cv::Mat mask = layers[3];
    // copyToに使うroi
    cv::Mat roi = _frame(cv::Rect(_x, _y, rgb.cols, rgb.rows));
    rgb.copyTo(roi, mask);
}
void putTranspPng(cv::Mat &_frame, cv::Mat &_png_o, int _x, int _y)
{
    putTranspPng(_frame, _png_o, _x, _y, _png_o.cols, _png_o.rows);
}

// 二点間の距離を計算する関数
float distance(const Point p1, const Point p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// 点が線分の周辺にあるかどうかをチェックする関数
/**
 * 点が与えられた線分に近いかどうかを判定します。
 * 
 * @param point 点の座標
 * @param lineStart 線分の始点の座標
 * @param lineEnd 線分の終点の座標
 * @param tolerance 許容範囲内の距離
 * @return 点が線分に近い場合はtrue、そうでない場合はfalse
 */
bool isPointNearLine(const cv::Point &point, const cv::Point &lineStart, const cv::Point &lineEnd, double tolerance)
{
    // 線分のベクトル
    double lineVecX = lineEnd.x - lineStart.x;
    double lineVecY = lineEnd.y - lineStart.y;

    // 点から線分の始点へのベクトル
    double pointVecX = point.x - lineStart.x;
    double pointVecY = point.y - lineStart.y;

    // 線分の長さの二乗
    double lineLenSq = lineVecX * lineVecX + lineVecY * lineVecY;

    // 点と線分の始点の内積
    double dotProduct = pointVecX * lineVecX + pointVecY * lineVecY;

    // 線分上の最近接点のパラメータtを計算
    double t = dotProduct / lineLenSq;

    // tの範囲を0から1に制限
    t = std::max(0.0, std::min(1.0, t));

    // 線分上の最近接点の座標
    double nearestX = lineStart.x + t * lineVecX;
    double nearestY = lineStart.y + t * lineVecY;

    // 最近接点と点の距離を計算
    double distX = point.x - nearestX;
    double distY = point.y - nearestY;
    double distance = std::sqrt(distX * distX + distY * distY);

    // 距離が許容範囲内かどうかをチェック
    return distance <= tolerance;
}

/**
 * @brief 3D 座標変換
 * @param xy 変換前の点
 * @param center 変換の中心点
 * @param r_add_x 変換後の点のx座標のポインタ
 * @param r_add_y 変換後の点のy座標のポインタ
 * @param angle 回転角度
 * @return なし
 * @note 点を回転させるために使用する。
 * 変換後の点のx座標とy座標を計算し、それらをポインタで渡します。
 */
void zahyou_kaiten(Point xy, Point center, int *r_add_x, int *r_add_y, double angle)
{
    *r_add_x = (xy.x * cos(-angle)) - (xy.y * sin(-angle)) + (center.x - center.x * cos(-angle) + center.y * sin(-angle));
    *r_add_y = (xy.x * sin(-angle)) + (xy.y * cos(-angle)) + (center.y - center.x * sin(-angle) - center.y * cos(-angle));
}
Point2f zahyou_kaiten_xy(Point xy, Point center, double angle)
{
    float r_add_x = (xy.x * cos(-angle)) - (xy.y * sin(-angle)) + (center.x - center.x * cos(-angle) + center.y * sin(-angle));
    float r_add_y = (xy.x * sin(-angle)) + (xy.y * cos(-angle)) + (center.y - center.x * sin(-angle) - center.y * cos(-angle));
    return (Point2f(r_add_x, r_add_y));
}
/**********************************************************************************/

/* 使用例
#include <cmath>
#include <opencv2/opencv.hpp>
int main() {
    cv::Point point(10, 10);
    cv::Point lineStart(0, 0);
    cv::Point lineEnd(20, 20);
    double tolerance = 5.0;

    if (isPointNearLine(point, lineStart, lineEnd, tolerance)) {
        std::cout << "Point is near the line." << std::endl;
    } else {
        std::cout << "Point is not near the line." << std::endl;
    }

    return 0;
}
*/

/*
=============

FOV measurement from calib (e.g. after undistortion):
CameraBoardSocket.CAM_C
Horizontal FOV: 70.2683493486316
Vertical FOV: 55.64816052222646
Diagonal FOV: 82.67159261902367

FOV measurement with optimal camera matrix and alpha=1 (e.g. full sensor FOV, without undistortion):
CameraBoardSocket.CAM_C
Horizontal FOV: 71.19014472202618
Vertical FOV: 56.45822834853326
Diagonal FOV: 83.18069076253148

=============
{
   "c_rez_x_r": 800,
   "c_rez_y_r": 480,
   "c_hfov_400_keisuu": 0.93,
   "c_vfov_400_keisuu": 0.93,
   "//c_pitch_keisuu":"高さバイアス",
   "c_pitch_keisuu": 0.985,
   "c_camera_high_keisu":1.00,
   "c_base_h_shift": 0.5,
   "//zone_low_cm = ": "(49 - (3 * batter_height)); // 低め https://okwave.jp/qa/q3922955.html",
   "c_zone_low_cm": 49,
   "c_zone_hi_cm": 110,
   "c_kekka_print_time": 4
}
*/

// 2つのポイントを結ぶ直線の角度を計算する関数
double calculateAngle(const Point &p1, const Point &p2)
{
    double deltaY = p2.y - p1.y;
    double deltaX = p2.x - p1.x;
    double angleInRadians = atan2(deltaY, deltaX);
    return angleInRadians;
}

// テンプレート関数で値が [minValue, maxValue] の範囲にあるかチェック
template<typename T1, typename T2, typename T3>
bool isWithinRange(T1 value, T2 minValue, T3 maxValue)
{
    using CommonType = typename std::common_type<T1, T2, T3>::type;
    return static_cast<CommonType>(value) >= static_cast<CommonType>(minValue) &&
           static_cast<CommonType>(value) <= static_cast<CommonType>(maxValue);
}

// 画像中の点の集合が直線上に並んでいるかをRANSACを用いて判定する関数
bool isLinearArrangementWithNoise(const Mat &img, int threshold = 100, int inlierThreshold = 10, double ransacProb = 0.59, int ransacIters = 100)
{
    try
    {
        // 変更なし: 空画像チェック
        if (img.empty())
        {
            LOG_DEBUG.printf("Error: 画像が空です。");
            return false;
        }

        // 変更なし: グレースケール変換
        Mat grayImg;
        if (img.channels() == 3)
        {
            cvtColor(img, grayImg, COLOR_BGR2GRAY);
        }
        else if (img.channels() == 4)
        {
            cvtColor(img, grayImg, COLOR_BGRA2GRAY);
        }
        else
        {
            grayImg = img.clone();
        }

        // 変更なし: 二値化
        Mat imgBin;
        cv::threshold(grayImg, imgBin, threshold, 255, THRESH_BINARY);

        // 変更なし: 輪郭抽出
        vector<vector<Point>> contours;
        findContours(imgBin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 変更なし: 重心計算
        vector<Point2f> centroids;
        for (const auto &contour : contours)
        {
            Moments M = moments(contour);
            if (M.m00 != 0)
            {
                float cX = static_cast<float>(M.m10 / M.m00);
                float cY = static_cast<float>(M.m01 / M.m00);
                centroids.push_back(Point2f(cX, cY));
            }
        }

        // 変更なし: 重心数チェック
        if (centroids.size() < 4)
        {
            cerr << "Error: 重心が4つ未満です。" << endl;
            return false;
        }

        // 変更なし: RANSAC変数初期化
        vector<int> inliers;
        Vec4f bestLine;

        // 変更なし: RANSACループ
        for (int i = 0; i < ransacIters; ++i)
        {
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> distrib(0, centroids.size() - 1);
            int idx1 = distrib(gen);
            int idx2;
            do
            {
                idx2 = distrib(gen);
            } while (idx2 == idx1);

            Point2f p1 = centroids[idx1];
            Point2f p2 = centroids[idx2];

            float vx = p2.x - p1.x;
            float vy = p2.y - p1.y;
            float norm = sqrt(vx * vx + vy * vy);
            vx /= norm;
            vy /= norm;

            vector<int> currentInliers;
            for (int j = 0; j < centroids.size(); ++j)
            {
                float dist = abs(vy * (centroids[j].x - p1.x) - vx * (centroids[j].y - p1.y));
                if (dist < inlierThreshold)
                {
                    currentInliers.push_back(j);
                }
            }

            if (currentInliers.size() > inliers.size())
            {
                inliers = currentInliers;
                vector<Point2f> inlierPoints;
                for (int idx : inliers)
                    inlierPoints.push_back(centroids[idx]);
                Vec4f currentLine;
                fitLine(inlierPoints, currentLine, DIST_L2, 0, 0.01, 0.01);
                bestLine = currentLine;
            }
        }

        // ▼▼▼ 変更部分: 角度チェック追加 ▼▼▼
        const float vx = bestLine[0];
        const float vy = bestLine[1];
        const float angle = std::acos(std::abs(vy)) * 180.0f / CV_PI;

        LOG_DEBUG.printf("検出角度: %.2f度", angle);
        return (inliers.size() >= 4) && (angle <= 40.0f);
        // ▲▲▲ 変更部分ここまで ▲▲▲
    }
    catch (const exception &e)
    {
        LOG_DEBUG.printf("Error: 予期せぬエラーが発生しました: %d", e.what());
        return false;
    }
}

/*パラメータ調整ガイド
VERTICAL_THRESHOLD：縦方向判定の厳密さ
小さくする→より厳密な縦方向を要求
大きくする→斜めの線形も許容
inlierThreshold：縦線上にあると見なす許容誤差
↑変更時はminEndpointDistも変更すること
返り値の3.0f：縦方向の最小長さ係数（例：inlierThreshold=5 → 最低15pxの長さが必要）
*/
bool isLinearArrangementWithNoise_xy(std::vector<ball_cours> &ball, int inlierThreshold = 3.0, double ransacProb = 0.59, int maxRansacIters = 100)
{
    std::vector<cv::Point2f> points;
    std::vector<int> originalIndices;

    // depが有効な点だけを抽出
    for (size_t i = 0; i < ball.size(); ++i)
    {
        if (isWithinRange(ball[i].dep, 1, 200))
        {
            points.emplace_back(ball[i].x, ball[i].y);
            originalIndices.push_back(i);
        }
    }

    if (points.size() < 2)
        return false;
    if (points.size() == 2)
        return true;

    std::vector<int> bestInliers;
    cv::Vec4f bestLine;
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < maxRansacIters; ++i)
    {
        std::uniform_int_distribution<> distrib(0, points.size() - 1);
        int idx1 = distrib(gen);
        int idx2;
        do
        {
            idx2 = distrib(gen);
        } while (idx2 == idx1);

        cv::Point2f p1 = points[idx1];
        cv::Point2f p2 = points[idx2];

        float vx = p2.x - p1.x;
        float vy = p2.y - p1.y;
        float norm = std::sqrt(vx * vx + vy * vy);
        if (norm == 0)
            continue;
        vx /= norm;
        vy /= norm;

        std::vector<int> currentInliers;
        for (int j = 0; j < points.size(); ++j)
        {
            float dist = std::abs(vy * (points[j].x - p1.x) - vx * (points[j].y - p1.y));
            if (dist < inlierThreshold)
            {
                currentInliers.push_back(j);
            }
        }

        if (currentInliers.size() > bestInliers.size())
        {
            std::vector<cv::Point2f> inlierPoints;
            for (int idx : currentInliers)
            {
                inlierPoints.push_back(points[idx]);
            }

            cv::Vec4f currentLine;
            cv::fitLine(inlierPoints, currentLine, cv::DIST_L2, 0, 0.01, 0.01);
            bestInliers = currentInliers;
            bestLine = currentLine;
        }
    }

    if (bestInliers.size() <= 4)
    {
        LOG_DEBUG.printf("inliers too few: %d", bestInliers.size());
        return false;
    }

    std::vector<cv::Point2f> inlierPoints;
    for (int idx : bestInliers)
    {
        inlierPoints.push_back(points[idx]);
    }

    // 平均残差による直線性チェック
    const float vx = bestLine[0], vy = bestLine[1], x0 = bestLine[2], y0 = bestLine[3];
    float totalResidual = 0.0f;
    for (auto &pt : inlierPoints)
    {
        float dist = std::abs(vy * (pt.x - x0) - vx * (pt.y - y0));
        totalResidual += dist;
    }
    float avgResidual = totalResidual / inlierPoints.size();
    if (avgResidual > inlierThreshold)
    {
        LOG_DEBUG.printf("avgResidual too high: %.2f > %d", avgResidual, inlierThreshold);
        return false;
    }

    // 端点間距離チェック
    float minProj = FLT_MAX, maxProj = -FLT_MAX;
    cv::Point2f minPt, maxPt;
    for (const auto &pt : inlierPoints)
    {
        const float proj = (pt.x - x0) * vx + (pt.y - y0) * vy;
        if (proj < minProj)
        {
            minProj = proj;
            minPt = pt;
        }
        if (proj > maxProj)
        {
            maxProj = proj;
            maxPt = pt;
        }
    }
    const float endpointDistance = cv::norm(maxPt - minPt);
    const float minEndpointDist = 30.0f; // 15.0f * inlierThreshold;
    LOG_DEBUG.printf("inliers: %d, avgResidual: %.2f, endpointDistance: %.2f, minEndpointDist: %.2f", (int)bestInliers.size(), avgResidual, endpointDistance, minEndpointDist);

    // アウトライアは dep = 0 に
    std::set<int> inlierSet(bestInliers.begin(), bestInliers.end());
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (inlierSet.find(i) == inlierSet.end())
        {
            int originalIdx = originalIndices[i];
            ball[originalIdx].dep = 0;
        }
    }

    return endpointDistance >= minEndpointDist;
}

bool filterLinearPoints3D(
    std::vector<ball_cours> &ball,
    float inlierThreshold = 15.0f, // 直線からの最大距離
    float minLineLength = 30.0f,  // 最小直線長
    double ransacProb = 0.59,
    int maxRansacIters = 100)
{
    std::vector<cv::Point3f> points;
    std::vector<int> originalIndices;

    // depが有効な点のみ抽出
    for (size_t i = 0; i < ball.size(); ++i)
    {
        if (isWithinRange(ball[i].dep, 1, 200))
        {
            points.emplace_back(ball[i].x, ball[i].y, ball[i].dep_cm); // 3D点
            originalIndices.push_back(i);
        }
    }

    if (points.size() < 2)
        return false;

    std::vector<int> bestInliers;
    cv::Vec6f bestLine; // 拡張：vx, vy, vz, x0, y0, z0
    std::mt19937 gen(std::random_device{}());

    for (int i = 0; i < maxRansacIters; ++i)
    {
        std::uniform_int_distribution<> distrib(0, points.size() - 1);
        int idx1 = distrib(gen), idx2;
        do
        {
            idx2 = distrib(gen);
        } while (idx2 == idx1);

        cv::Point3f p1 = points[idx1], p2 = points[idx2];

        // ★ 追加: 直線の起点 (p1) の Z座標が300以上である線のみを考慮
        if (p1.z < 300) { // Z座標が300未満であればスキップ
            continue;
        }

        cv::Point3f dir = p2 - p1;
        float norm = std::sqrt(dir.dot(dir));
        if (norm == 0)
            continue;
        dir *= 1.0f / norm;

        // ★ 追加: dz がマイナスである線のみを考慮
        if (dir.z >= 0) { // Z成分が0以上であればスキップ
            continue;
        }

        std::vector<int> inliers;
        for (int j = 0; j < points.size(); ++j)
        {
            cv::Point3f vec = points[j] - p1;
            cv::Point3f proj = dir * vec.dot(dir);
            float dist = cv::norm(vec - proj);
            if (dist < inlierThreshold)
                inliers.push_back(j);
            // else
            //    LOG_DEBUG.printf(" outlinear:%f %d, %d, %d", dist, originalIndices[j], ball[originalIndices[j]].x, ball[originalIndices[j]].y);
        }

        if (inliers.size() > bestInliers.size())
        {
            // 平均方向と基準点を保存
            bestLine = {dir.x, dir.y, dir.z, p1.x, p1.y, p1.z};
            bestInliers = inliers;
        }
    }

    LOG_DEBUG.printf("bestInliers: %d, maxRansacIters: %d, bestLine(平均方向と基準点): dx%.2f, dy%.2f, dz%.2f, x%.2f, y%.2f, z%.2f", (int)bestInliers.size(), maxRansacIters, bestLine[0], bestLine[1], bestLine[2], bestLine[3], bestLine[4], bestLine[5]);
    if (bestInliers.size() <= 4)
        return false;

    // 残差チェック
    float vx = bestLine[0], vy = bestLine[1], vz = bestLine[2];
    float x0 = bestLine[3], y0 = bestLine[4], z0 = bestLine[5];
    float totalResidual = 0.0f;
    for (int idx : bestInliers)
    {
        const auto &pt = points[idx];
        cv::Point3f vec(pt.x - x0, pt.y - y0, pt.z - z0);
        cv::Point3f proj = cv::Point3f(vx, vy, vz) * vec.dot(cv::Point3f(vx, vy, vz));
        float dist = cv::norm(vec - proj);
        totalResidual += dist;
    }
    float avgResidual = totalResidual / bestInliers.size();
    if (avgResidual > inlierThreshold)
        return false;

    // 直線の長さチェック（投影距離）
    float minProj = FLT_MAX, maxProj = -FLT_MAX;
    cv::Point3f minPt, maxPt;
    for (int idx : bestInliers)
    {
        const auto &pt = points[idx];
        float proj = (pt.x - x0) * vx + (pt.y - y0) * vy + (pt.z - z0) * vz;
        if (proj < minProj)
        {
            minProj = proj;
            minPt = pt;
        }
        if (proj > maxProj)
        {
            maxProj = proj;
            maxPt = pt;
        }
    }
    float lineLength = cv::norm(maxPt - minPt);

    // 外れ値 dep = 0 に
    std::set<int> inlierSet(bestInliers.begin(), bestInliers.end());
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (inlierSet.find(i) == inlierSet.end())
        {
            int originalIdx = originalIndices[i];
            // ball[originalIdx].dep = 0;
            LOG_DEBUG.printf("outlier: %d, (x:%d, y:%d, d:%.1f)", originalIdx, ball[originalIdx].x, ball[originalIdx].y, ball[originalIdx].dep_cm);
        }
    }
    LOG_DEBUG.printf("inliers: %d, avgResidual: %.2f, lineLength: %.2f, minLineLength: %.2f", (int)bestInliers.size(), avgResidual, lineLength, minLineLength);

    return lineLength >= minLineLength;
}

// 画像Aと図形画像BのXORを取って、ゼロピクセルの割合が最大となる位置を検出
cv::Point findBestMatch(const cv::Mat &imageA, const cv::Mat &templateB, const cv::Rect &searchArea, cv::Mat &bestMatchRegion, cv::Mat &bestXorResult)
{
    if (imageA.empty() || templateB.empty())
    {
        throw std::invalid_argument("Input images are empty.");
    }

    if (searchArea.x < 0 || searchArea.y < 0 ||
        searchArea.x + searchArea.width > imageA.cols ||
        searchArea.y + searchArea.height > imageA.rows)
    {
        throw std::out_of_range("Search area is out of image bounds.");
    }

    if (templateB.rows == 0 || templateB.cols == 0)
    {
        throw std::invalid_argument("Template image is empty.");
    }

    double maxMedianRatio = 0.0;
    int bestMatchX = -1;
    int bestMatchY = -1;

    int subWidth = templateB.cols / 3;
    int subHeight = templateB.rows / 3;

    // 左上から右下へのスキャン
    for (int y = searchArea.y; y <= searchArea.y + searchArea.height - templateB.rows; y++)
    {
        for (int x = searchArea.x; x <= searchArea.x + searchArea.width - templateB.cols; x++)
        {
            // テンプレートが収まる範囲内か確認
            if (x >= searchArea.x && x <= searchArea.x + searchArea.width - templateB.cols &&
                y >= searchArea.y && y <= searchArea.y + searchArea.height - templateB.rows)
            {
                cv::Mat region = imageA(cv::Rect(x, y, templateB.cols, templateB.rows));
                std::vector<double> subRegionRatios;

                // 9分割してマッチング度合いを計算
                for (int sy = 0; sy < 3; sy++)
                {
                    for (int sx = 0; sx < 3; sx++)
                    {
                        cv::Rect subRect(sx * subWidth, sy * subHeight, subWidth, subHeight);
                        cv::Mat subRegion = region(subRect);
                        cv::Mat subTemplate = templateB(subRect);
                        cv::Mat subXorResult;
                        cv::bitwise_xor(subRegion, subTemplate, subXorResult);

                        cv::Mat subMask = subXorResult == 0;
                        int subZeroCount = cv::countNonZero(subMask);
                        double subZeroRatio = static_cast<double>(subZeroCount) / (subWidth * subHeight);
                        subRegionRatios.push_back(subZeroRatio);
                    }
                }

                // 中央値を計算
                std::nth_element(subRegionRatios.begin(), subRegionRatios.begin() + subRegionRatios.size() / 2, subRegionRatios.end());
                double medianRatio = subRegionRatios[subRegionRatios.size() / 2];

                // 最良のマッチングを更新
                if (medianRatio > maxMedianRatio)
                {
                    maxMedianRatio = medianRatio;
                    bestMatchX = x;
                    bestMatchY = y;
                    bestMatchRegion = region.clone();
                    cv::bitwise_xor(region, templateB, bestXorResult);

                    // 十分なマッチングが見つかったら早期終了
                    if (medianRatio > 0.9)
                    {
                        return cv::Point(bestMatchX + templateB.cols / 2, bestMatchY + templateB.rows / 2);
                    }
                }
            }
        }
    }

    // マッチングがしきい値以下の場合は無効な結果を返す
    if (maxMedianRatio < 0.5)
    {
        return cv::Point(-1, -1);
    }

    return cv::Point(bestMatchX + templateB.cols / 2, bestMatchY + templateB.rows / 2);
}

void displayMatchResult(const cv::Mat &bestMatchRegion, const cv::Mat &bestXorResult)
{
    cv::imshow("Best Match Region", bestMatchRegion);
    cv::imshow("Best XOR Result", bestXorResult);
    cv::waitKey(1);
}
//

/* 画像Aと図形画像BのXORを取って、ゼロピクセルの割合が最大となる位置を検出
cv::Point findBestMatch(const cv::Mat& imageA, const cv::Mat& templateB, const cv::Rect& searchArea) {
    if (imageA.empty() || templateB.empty()) {
        throw std::invalid_argument("Input images are empty.");
    }

    if (searchArea.x < 0 || searchArea.y < 0 || 
        searchArea.x + searchArea.width > imageA.cols || 
        searchArea.y + searchArea.height > imageA.rows) {
        throw std::out_of_range("Search area is out of image bounds.");
    }

    if (templateB.rows == 0 || templateB.cols == 0) {
        throw std::invalid_argument("Template image is empty.");
    }

    int bestMatchX = searchArea.x;
    int bestMatchY = searchArea.y;
    double maxZeroRatio = 0.0;

    // printf("searchArea:%d,%d,%d,%d\n", searchArea.x, searchArea.y, searchArea.width, searchArea.height);

    for (int y = searchArea.y; y <= searchArea.y + searchArea.height - templateB.rows; y++) {
        for (int x = searchArea.x; x <= searchArea.x + searchArea.width - templateB.cols; x++) {
            cv::Mat region = imageA(cv::Rect(x, y, templateB.cols, templateB.rows));
            cv::Mat xorResult;
            cv::bitwise_xor(region, templateB, xorResult);

            // ピクセルカウントを効率的に行う
            cv::Mat mask = xorResult == 0;
            int zeroCount = cv::countNonZero(mask);

            double zeroRatio = static_cast<double>(zeroCount) / (templateB.rows * templateB.cols);
            //if (zeroRatio > 0.4)
            //printf("x:%d,y:%d,ratio:%.2f\n", x, y, zeroRatio);

            if (zeroRatio > maxZeroRatio) {
                maxZeroRatio = zeroRatio;
                bestMatchX = x;
                bestMatchY = y;
            }
        }
    }

    LOG_DEBUG.printf("x:%d,y:%d,ratio:%.2f", bestMatchX,bestMatchY,maxZeroRatio);

    if (maxZeroRatio < 0.5) {
        return cv::Point(-1, -1);
    }

    return cv::Point(bestMatchX + templateB.rows/2, bestMatchY + templateB.cols/2);
}
*/
float adjust_ratio(float base, float ratio)
{
    // 1. 通常の割合で乗算
    float raw = base * ratio;

    // 2. 16の倍数に丸め（ここでは切り上げ）
    int rounded = static_cast<int>((raw + 15) / 16) * 16;

    // 3. 割合Xを再計算（16の倍数になるように調整されたX）
    float adjusted_ratio = static_cast<float>(rounded) / base;

    return adjusted_ratio;
}

// 型を文字列として取得する関数
/**
 * @brief OpenCVのcv::Mat型のtype情報（int値）から、人間が読みやすい型名文字列（例: "8UC3"）を生成する。
 * @param type OpenCVのcv::Mat型のtype（CV_8UC3などのint値）
 * @return 画像型を表す文字列（例: "8UC1", "8UC3", "32FC1" など）
 * @details
 *   typeは画素の深度（8U, 16S, 32Fなど）とチャンネル数（C1, C3など）の情報を持つ。
 *   この関数はデバッグやログ出力時に画像型を分かりやすく表示したいときに便利。
 */std::string type2str(int type)
{
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C" + std::to_string(chans);
    return r;
}
