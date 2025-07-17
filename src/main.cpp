/**
 * @file ストライク判定メインプロセス
 * @brief ファイルの説明
 * @author 吉田 孝
 * @date 作成日
 *
 * @details 詳細説明
 * @note 補足説明とかメモ
 * <今後の改良>
 * 検知したいボールの距離によって、ボール検知の閾値を変動させる
 *
 * ボール検知において端っこは除外する
 * モノクロ画像からベース判定させる
 * 2024/06/22
 *  -地面排除でボール検知できない問題修正
 *  -カメラ傾き計算バグの修正
 *  -打者検知の大幅改善
 *  -本番ボール検知中はFPSを表示しない
 *  -録画修正
 *  -ベース距離計測場所修正
 *  -スナップショット取得修正
 *  -python変数値取得（仕掛）
 * 2024/06/25
 *  -python変数値取得
 *  -サイドビューのゾーン大きさ調整
 *  -彩度調整範囲を限定（未確認）
 * 2024/07/09
 *  -マウス処理改善
 *  -ゾーン表示線の太さを変更
 *  -opencvのログレベルを設定
 *  -加速度測定の無駄排除
 *  -base_detection_makerが3,4ではキャリブレーションでbasefind実行しない
 *  -solve_pnpの実施頻度を減らす
 *  -実計測でのボールストライク判定よりも軌道計算での判定を優先
 *  -ベース高さのしきい値をカメラピッチに合わせる
 *  -ボールノイズチェックを厳しく（ボール検知レートアップ）
 * 2024/07/10
 *  -マルチスレッド終了待ちしないようにした
 *  -ベース角ターゲットを小さくする（ベース角バグ修正）
 * 2024/07/13
 *  -ベース判定バグ修正
 *  -マルチスレッド起動を関数に入れ込み
 * 2024/07/14
 *  -RGB画像取得時のロック
 *  -ベース距離計測をモード０に固定
 *  -ベース検索頻度
 *  -ベース大きさしきい値緩和
 *  -ベース検知で異常終了する問題の対応
 *  -ベース検知範囲を絞って高速化
 * 2024/07/15
 *  -rgbカメラとモノクロカメラの画角を使い分け（高さ・左右からカラー画像に合わせてXY座標を求める部分を修正）
 *  -ベース検知改良（キャリブレーションでベース位置やサイズが大きくずれたらエラー）
 * 2024/07/17
 *  -ゾーン表示で基底をちゃんと回転させるようにした
 *  -打者検知中はキャリブレーションしない
 *  -カメラXZ面の傾きについてPNPで算出できない場合（思案中）
 *  -ゾーン左右表示バグ修正（過去にcalc_h_d_ball_Rev_xを作成したときにバグらせた）"ゾーンの左右の表示座標を計算（見かけ大きさに関係）"
 *  -calc_h_d_ball_Revのh_distanceが間違ってたので修正(ゾーン表示の上下高さに関係)
 * 2024/07/18
 *  -ゾーン表示高さをベース中心基底に変更(clac_h_d_ball_Revの修正)
 *  -両サイドの遠方検知しないように修正(rndフィルター部分）
 * 2024/07/21
 *  -ゾーン表示横位置バグ修正
 *  -rndフィルターバグ修正
 *  -ボール位置算出関数を分離
 * 2024/07/24
 *  -ゾーンイラスト左右打者表示修正
 *  -liteで動作しなかった問題の対応（raspi5->c_imu）liteはコンフィグ変えて再コンパイル
 * 2024/07/25
 *  -カメラ種別によるIMU利用自動切り替え
 *  -mono画像取得スレッドを分離
 *  -zone_il_print関数で左右打者表示わけ
 * 2024/08/12
 *  -ノーベースモードカメラ中心基準の追加
 *  -ボール位置、座標変換関数の追加
 *  -カラー画像取得時のエラー対応
 * 2024/08/20
 *  - ゾーン中心の座標を取得する（輪郭は検知しない）→no base mode
 *  - ゾーン表示を立体的に
 *  - グラウンドモード追加（カメラ近すぎて無理）
 *  - セカンドカメラピッチの補正パラメータ
 *  - RNDフィルターの高速化
 *  - RNDフィルターの距離表示（補正用）
 *  - 画面座標＜ー＞ワールド座標変換
 *  - (calculateBallPositionByBase,calculateBallPositionByCenter,calculateVisionPositionByCenter)
 * 距離はy軸もｘ軸も角度によって変わらないことがわかったのでその反映
 * 2024/08/24
 *  -異常終了対応（ｑや打者検知のroi対応）
 *  -カメラ距離、高さ調整（no_base_mode）
 * 2025/02/24
 *  -ノイズ処理の改善
 *  -ボール検知半径を厳しく
 *  -ボールのゾーン近辺での検知を厳しく（ボール検知ステータス更新（２－＞３）を厳密に）codeBallPosition_base_shortly）
 *  -3204行に注意
 * 2025/02/26
 * -ボール検知画像記録
 * -ロバスト推定の見直し
 * -ノイズテーブル数見直し
 * -判定待ちインターバル
 * -円形度しきい値変更
 * 2025/03/02
 * -線形判定処理の追加
 * 2025/03/06
 * -ベーステンプレートマッチングとカメラヨーの補正
 * -ボール履歴の線形判定を、画像からXY座標に変更
 * -４１８３行のカメラピッチの更新フィルタをなくしている
 * -４５４８行以下、打者姿勢判定を厳しく
 * -findBestMatch関数（XORテンプレートマッチング）の追加
 * -baseTemplate.jpgの追加
 *
 * 2025/03/19
 * <main>
 * ball_clearの待ち時間バグ修正
 * ball_detection_t_oldの初期化をball_clearに入れる
 * MedianFilter::KERNEL_3x3
 * 初回検知時より遠くはフィルタする
 * ts_saのチェックを外す
 * ボールの残像をCIRCLEに
 * 線形判定を残像から座標に
 * 残像のsave関係
 * ボール外れ距離の表示
 * <base>
 * ベース位置調整
 * ボール検知範囲や大きさの調整
 * ノイズ検知と円形度判定の順序入れ替え
 * ロバスト少し修正
 *2025/3/23
 * <main>
 * 前回ボールより近づいているかの判定追加
 * ゾーン近辺通過の判定緩和
 * 打者検知のバグ修正
 * <base>
 * ロバスト推定を第二パラメータがゼロに近づくことを優先するように修正
 * <autost>
 * ピン番号変更　５－＞１６
 * 2025/03/29
 * 近くなるほど円形度のしきい値を緩和
 * ボール残像保存のバグを修正
 * 円形度：低が連続しないように
 *2025/03/30
 * AUTOでも打者検知
 * ノイズテーブル処理
 *2025/04/01
 * get_secなどを起動時からの相対時間に
 * ノイズ処理大幅修正
 * ボール大きさ判定前にノイズ処理実施
 * ball_detection_phase:1->2の距離を手前に修正
 * 判定中もバッターを検知するようにする
 *2025/04/08
 * ボールチェックで前回近時位置なら円形度やサイズ条件を緩和する
 * ボール進行チェック辞める
 * ノイズ処理修正
 *2025/04/09
 * 720p対応（manipでアスペクト比を固定するのがポイント）
 * noise深度チェック修正
 * 2025/04/19　野外テスト実施
 * 連続ボール検知なしはリセット
 * ノイズ深度更新のFIX
 * ボール最大検知数を１５０から１０に
 * ワンバウンドチェック強化
 * モノ画面バッファ、TRUE
 *2025/04/22
 * ボール進行チェックを復活
 * ボールクリアタイムの短縮修正
 * RGB画像取得時のロック（モノリードエラー対応）
 * 直線有無判定を３次元に変更
 *2025/04/24
 * ノイズチェック緩和範囲　初期ボール検知範囲の表示
 * ボールの進行にしたがって、深度画像の距離範囲を狭める
 * ボール進行判定においてファストカメラ、セカンドカメラの分離
 * 連続ボール検知エラー関連の修正
 * マックスボールエリアを拡大
 * RGBリード、モノリードの排他ロック処理
 *2025/05/03
 * 連続エラーを厳しく
 * 打者輪郭をフィルタリング
 * ジャッジ開始待ち時間延長
 *2025/05/06
 * ノイズカウント厳しく
 * 関節距離測定を平均に
 * 関節距離を暫定的に固定(230cm)　（ball.cppの３０４２行）
 * ボール検知幅をワイドに（ball.cppの２７８９行から、２８７６行）
 * 各スレッドのウェイト時間を全体的に短く
 * GPIO関係大幅見直し
 * oprecvマルチスレッド設定（２０７９行目）
 * ボール初回検知時にLEDを点灯
 * 打者検知関係を大幅に見直し
 * main.cppでのストライクゾーン幅見直し1.5cmのマージン（５４９０行から）
 *2025/05/10
 * ハング対応
 * 連続エラーのバグ修正
 *2025/05/15
 * 距離計測に関するバグ（考え違い）を修正
 * 距離の少数は５ビットなので３２で割る
 *2025/05/25
 * 複数カメラのスレッドを分離
 * ベース調整を待つように修正
 * 
 *<今後改善>
 * 変数名　水平距離/直線距離＋CM/i　RGB/mono+full/shlink
 */

/**********************************************************************************/
#include <cstdio>
// #include "C:\Users\Takashi\OneDrive\depthaiC++\depthai-core-example-main\depthai-core\examples\src\utility.hpp"
#include "depthai/depthai.hpp"
#include "plog/Initializers/RollingFileInitializer.h"
#include <algorithm> // std::sort
#include <arpa/inet.h>
#include <condition_variable>
#include <iostream>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Log.h>
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <utility> // C++11から

using namespace std;
using namespace cv;
// Include nlohmann json
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
uint64_t tv_sec_init = 0;
uint64_t tv_m_sec_init = 0;
#include "strike.hpp"
#if (raspi5 == true)
    #include <gpiod.h>
    #include <pigpiod_if2.h>

#else
    #include "wiringPi.h"
#endif
#include "Python.h"
#include <Eigen/Dense>
#include <cmath>
#include <inttypes.h> // inttypes.hが必要
#include <iostream>
#include <numpy/arrayobject.h>
#include <omp.h>

/**********************************************************************************/

/**********************************************************************************/
struct gpiod_chip *chip;
struct gpiod_line *sw_5_AutoJudgh, *sw_6_JudghTrigger, *sw_26_LED_sw, *sw_16_KneeDetection, *sw_20_MOTORSW;

//! 距離画像スレッド動作
bool isActJudgment_main = true;        // 判定スレッド動作中フラグ
bool end_flg_Judgment_main_get = true; // 判定スレッド終了フラグ
bool end_flg_rgb_read = true;          // RGB画像取得スレッド終了フラグ
bool end_flg_mono_read = true;         // モノクロ画像取得スレッド終了フラグ
bool mono_read_lock_f = false;         // モノクロ画像取得ロック（ファーストカメラ）
bool mono_read_lock_s = false;         // モノクロ画像取得ロック（セカンドカメラ）
bool rgb_read_lock = false;            // RGB画像取得ロック
bool end_flg_img_key = true;           // 画像取得スレッド終了フラグ
bool rnd_filter_syori_flg = false;     // RNDフィルター処理中フラグ
int base_katamuki_hosei;               // ベース傾き補正値

#if (double_camera == true)
// 判定メイン関数（ダブルカメラ用）
int Judgment_main(dai::Device &device, dai::Device &device_B);
#else
// 判定メイン関数（シングルカメラ用）
int Judgment_main(dai::Device &device);
#endif

#if (double_camera == true)
// モノクロ画像取得スレッド（ファースト・セカンドカメラ）
void mono_read_f(dai::Device &device);
void mono_read_s(dai::Device &device);
// タイムスタンプ（ファースト・セカンドカメラ）
std::chrono::time_point<std::chrono::steady_clock> ts_f;
std::chrono::time_point<std::chrono::steady_clock> ts_s;
#else
// モノクロ画像取得スレッド（シングルカメラ）
void mono_read_f(dai::Device &device);
// タイムスタンプ（シングルカメラ）
std::chrono::time_point<std::chrono::steady_clock> ts_f;
#endif

// カラー画像取得スレッド
void img_key(dai::Device &device);

// --- カメラ設定・状態管理 ---
bool autoex_mono = false;                // モノクロ自動露出フラグ
bool autoex_rgb = true;                  // RGB自動露出フラグ
bool cond_flg = true;                    // 画像取得条件フラグ
bool rgb_frame_taken = false;            // RGB画像取得済みフラグ
bool rgb_frame_taken_img_key = false;    // カラー画像取得済み（img_key用）
bool mono_frame_taken_img_key_f = false; // モノクロ画像取得済み（ファーストカメラ、img_key用）
bool mono_frame_taken_img_key_s = false; // モノクロ画像取得済み（セカンドカメラ、img_key用）
bool camera_p_home_dis;                  // カメラ原点基準フラグ
float camera_pitch_keisuu;               // カメラピッチ補正係数
float c_zone_hi_per;                     // ゾーン上端比率
float c_zone_low_per;                    // ゾーン下端比率
float c_zone_low_cm;                     // ゾーン下端高さ(cm)
float c_zone_hi_cm;                      // ゾーン上端高さ(cm)
int camera_sel = 0;                      // 有効カメラ（0：ダブル、1：ファースト、2：セカンド）
int strike_course = 0;                   // 投球コース（0:ど真ん中、1:高め,2:真ん中低め,3:左右真ん中以下低め）

// Inludes common necessary includes for development using depthai library

#include "base.cpp"
//! Closer-in minimum depth, disparity range is doubled (from 95 to 190):
//! フル解像度とダウンスケールされた画像を組み合わせて、視差範囲が0-95から0-190に増加しました。短距離のオブジェクトに適しています。現在、サブピクセルの視差と互換性がありません
static std::atomic<bool> extended_disparity{false};
//! Better accuracy for longer distance, fractional disparity 32-levels:
//! サブピクセル補間（5小数ビット）で視差を計算します。長距離射撃に適しています。現在、拡張された視差と互換性がありません
static std::atomic<bool> subpixel{true}; //変更時はMat frame_mono_f_tmp(400, 256, CV_8UC1, cv::Scalar(0));を変更する必要あり
//! Better handling for occlusions:
//! LR方向とRL方向の両方の視差を計算して結合し、それらを結合します。オクルージョン処理を改善するために、無効な視差値を破棄します
static std::atomic<bool> lr_check{false};

// DEPTHAI ノード 型宣言
//! DEPTHAI NODE 左モノカメラ
std::shared_ptr<dai::node::MonoCamera> monoLeft;
//! DEPTHAI NODE 右モノカメラ
std::shared_ptr<dai::node::MonoCamera> monoRight;
//! DEPTHAI NODE 深度
std::shared_ptr<dai::node::StereoDepth> depth;
/* V2->V3
//! DEPTHAI NODE xout(深度出力)
//std::shared_ptr<dai::node::XLinkOut> xout;
//! DEPTHAI NODE xout_mono(モノ出力)
// std::shared_ptr<dai::node::XLinkOut> xout_mono;
//! DEPTHAI NODE xout(信頼値)
// std::shared_ptr<dai::node::XLinkOut> xoutconfidenceMap;
//! DEPTHAI NODE xout( )
// std::shared_ptr<dai::node::XLinkOut> xoutrectifiedLeft;
//! DEPTHAI NODE xout(カラー出力)
std::shared_ptr<dai::node::XLinkOut> xoutRgb;
//! DEPTHAI NODE xlinkin(monoカメラ設定)
std::shared_ptr<dai::node::XLinkIn> controlIn_mono;
std::shared_ptr<dai::node::XLinkIn> controlIn_Rgb;
std::shared_ptr<dai::node::XLinkIn> controlIn_script;
std::shared_ptr<dai::node::XLinkIn> configIn;
*/
//! DEPTHAI NODE カラーカメラ
std::shared_ptr<dai::node::ColorCamera> camRgb;
//! DEPTHAI NODE manip
std::shared_ptr<dai::node::ImageManip> manip_l, manip_r;
//! DEPTHAI NODE script
std::shared_ptr<dai::node::Script> script;

// Output queue will be used to get the disparity frames from the outputs defined above
// (V2) std::shared_ptr<dai::DataOutputQueue> qDet, qconfidenceMap, qrectifiedLeft;
std::shared_ptr<dai::ImgDetections> inDet;
dai::CameraControl ctrl_Rgb;
// (V2) std::shared_ptr<dai::DataInputQueue> controlQueue_Rgb, controlQueue_script;
// (V2) std::shared_ptr<dai::DataOutputQueue> imuQueue;
std::shared_ptr<dai::InputQueue> controlQueue_Rgb, controlQueue_script; //V3
// (V2) std::shared_ptr<dai::DataOutputQueue> imuQueue; //V3
// DEPTHAI NODE IMU
std::shared_ptr<dai::node::IMU> imu;// V3
// IMU 出力キュー
std::shared_ptr<dai::MessageQueue> imuQueue;//V3

// std::shared_ptr<dai::IMUData> imuData;
// std::shared_ptr<dai::DataOutputQueue> imuQueue;

//! DEPTHAI NODE 左モノカメラ
std::shared_ptr<dai::node::MonoCamera> monoLeft_B;
//! DEPTHAI NODE 右モノカメラ
std::shared_ptr<dai::node::MonoCamera> monoRight_B;
//! DEPTHAI NODE 深度
std::shared_ptr<dai::node::StereoDepth> depth_B;
/* V2->V3
//! DEPTHAI NODE xout(深度出力)
std::shared_ptr<dai::node::XLinkOut> xout_B;
//! DEPTHAI NODE xout_mono(モノ出力)
// std::shared_ptr<dai::node::XLinkOut> xout_mono;
//! DEPTHAI NODE xout(信頼値)
// std::shared_ptr<dai::node::XLinkOut> xoutconfidenceMap;
//! DEPTHAI NODE xout( )
// std::shared_ptr<dai::node::XLinkOut> xoutrectifiedLeft;
//! DEPTHAI NODE xout(カラー出力)
std::shared_ptr<dai::node::XLinkOut> xoutRgb_B;
//! DEPTHAI NODE xlinkin(monoカメラ設定)
std::shared_ptr<dai::node::XLinkIn> controlIn_mono_B;
std::shared_ptr<dai::node::XLinkIn> controlIn_Rgb_B;
std::shared_ptr<dai::node::XLinkIn> controlIn_script_B;
std::shared_ptr<dai::node::XLinkIn> configIn_B;
*/
//! DEPTHAI NODE カラーカメラ
std::shared_ptr<dai::node::ColorCamera> camRgb_B;
//! DEPTHAI NODE manip
std::shared_ptr<dai::node::ImageManip> manip_l_B, manip_r_B;
std::shared_ptr<dai::node::ImageManip> manip_hanten;   //縮尺上下左右反転専用
std::shared_ptr<dai::node::ImageManip> manip_hanten_B; //上下左右反転専用
//! DEPTHAI NODE script
std::shared_ptr<dai::node::Script> script_B;
// (V2) std::shared_ptr<dai::DataInputQueue> controlQueue_mono;
// (V2) std::shared_ptr<dai::DataInputQueue> controlQueue_mono_B;
std::shared_ptr<dai::InputQueue> controlQueue_mono_left,controlQueue_mono_right; //(V3)
std::shared_ptr<dai::InputQueue> controlQueue_mono_B_left, controlQueue_mono_B_right; //(V3)
dai::CameraControl ctrl_mono;
dai::CameraControl ctrl_mono_B;

// Output queue will be used to get the disparity frames from the outputs defined above
// (V2) std::shared_ptr<dai::DataOutputQueue> qDet_B, qconfidenceMap_B, qrectifiedLeft_B;
dai::CameraControl ctrl_Rgb_B;
// (V2) std::shared_ptr<dai::DataInputQueue> controlQueue_Rgb_B, controlQueue_script_B;
std::shared_ptr<dai::InputQueue> controlQueue_Rgb_B, controlQueue_script_B; //(V3)

std::chrono::steady_clock::time_point baseTs;

//! ベース画像を録画用にカラー化するためのテンポラリー変数
Mat mask_image_add;           // ベース距離判定のための２値変換のためのテンポラリー
Mat tmp_binary_mask_distance; // ボール判定２値画像動画をファイル書き出しするためのテンポラリー
Mat bin_img;                  // 2値画像を入れておくためのMat
Mat binary_mask_distance;     // 距離フィルター後の２値画像、ボール検出に利用
Mat r_image_tmp, r_image_tmp_over;
Mat r_image_SARA; // Scaling and Aspect Ratio Adjustment 画面表示のメインとなる。カラー画像を縮小し貼り付け;
Mat mask_image;
Mat frame_mono_org;   // ２つのカメラから採用する深度画像を選択済みの画像
Mat frame_mono_org_f; // ２つのカメラから採用する深度画像を選択済みの画像
Mat frame_mono_org_s; // ２つのカメラから採用する深度画像を選択済みの画像
// 幅400、高さ256、8ビットのグレースケール画像
cv::Mat frame_mono_f_tmp(400, 256, CV_16UC1, cv::Scalar(0));                                          // 初期値は黒（0）
cv::Mat frame_mono_s_tmp(400, 256, CV_16UC1, cv::Scalar(0));                                          // 初期値は黒（0）
Mat frame_mono, frame_mono_tmp, frame_mono_batter;                                                    // 深度画像
Mat frame_mono_l;                                                                                     // 左モノクロカメラ画像（未使用または予備）
Mat clip_img, clip_img_o, clip_img_o2;                                                                // 画像の切り出し・一時保存用
Mat tmp_frame_depth, tmp_frame2_depth;                                                                // 深度画像の一時保存用
Mat ball_rireki = Mat::zeros(400, 640, CV_8UC1);                                                      // ボール履歴画像（残像・軌跡描画用、初期化は黒画像）
/* Mat frame_confidenceMap; */                                                                        // 信頼度マップ画像（コメントアウト中）
Mat frame_rectifiedLeft;                                                                              // 補正済み左画像（未使用または予備）
/* Mat ball_snap; */                                                                                  // ボールスナップショット画像（コメントアウト中）
Mat resize_frame, tmp_frame, tmp_frame2, r_image_org, r_image_org_shrink;                             // 各種画像のリサイズ・一時保存用
std::shared_ptr<dai::ImgFrame> inRgb, inDepth_f, inDepth_s, inMono, inconfidenceMap, inrectifiedLeft; // DepthAI画像フレーム（RGB, 深度, モノクロ, 信頼度, 補正済み左）
bool JudghTrigger_sw = false;                                                                         // 判定トリガースイッチ状態（ON: true, OFF: false）
bool AutoJudgh_sw = false;                                                                            // 自動判定モードスイッチ状態
bool AutoJudgh_sw_keyboad = false;                                                                    // キーボードによる自動判定モード状態
bool Judgment_start = false;                                                                          // 判定開始指示フラグ（TRUE: 判定モードへ移行, 判定開始でFALSEに）
bool syuuryousyori = false;                                                                           // 終了処理開始指示フラグ（TRUE: 各種フラグ初期化）
bool bat_flg = false;                                                                                 // 打者検知済みフラグ（TRUE: 打者検知済み）
bool right_bat_f = false;                                                                             // 右打者検知フラグ
bool left_bat_f = false;                                                                              // 左打者検知フラグ
bool bat_pose_flg = false;                                                                            // 打撃ポーズ検知フラグ（TRUE: 打撃ポーズ中）
int bat_pose_flg_cnt = 0;                                                                             // 打撃ポーズ検知カウンタ

int c_rez_x_r;
int c_rez_y_r;

static constexpr int EXP_STEP = 100; // us
static constexpr int ISO_STEP = 100;
int exp_time_mono = c_exp_time_mono;
int exp_time_mono_B = c_exp_time_mono;
int exp_time_Rgb = 500;
int exp_min = 1;
int exp_max = 33000;
int sens_iso_mono = c_sens_iso_mono;
int sens_iso_mono_B = c_sens_iso_mono;
int sens_iso_Rgb = 500;
int sens_min = 100;
int sens_max = 1600;
// # For OAK-D @ 400P mono cameras and disparity of eg. 50 pixels
//  depth = 441.25 * 7.5 / 50 = 66.19 # cm
int dep_l_en;     // = c_dep_l_en;   // (i)遠方フィルタ 300cm
int dep_l_kin;    // = c_dep_l_kin; // (i)近距離フィルタ
int dep_base_en;  // = c_dep_base_en;
int dep_base_kin; // = c_dep_base_kin;
int dep_l_en_bias;
int dep_l_kin_bias;
int dep_threshold; //ボールを検知する距離しきい値（可変）
int gpio;

int focus_mono = 130;
int focus_Rgb = 130;
int DEP_STEP = 1;
char value_c[255];
enum
{
    button_long,
    button_mini,
    button_f
};

// RNDフィルターのしきい値
float c_rnd_filter;
// RNDフィルター有効フラグ
bool rnd_filter_f = true;
// RNDフィルター用ピッチ補正係数
float rnd_pitch_keisuu;
// カメラの水平誤差検出フラグ
bool isHorizontalError = false;
// モノクロ・RGBカメラの画角やバイアス関連
int C_monorgb_hi_x;
int c_bias_x, c_bias_x_ork_d, c_bias_x_ork_d_lite;
int C_monorgb_hi_y;
int c_bias_y, c_bias_y_ork_d, c_bias_y_ork_d_lite;
// OAK-D Lite/Monoカメラの垂直画角
float vfov_oak_d_lite_mono, vfov_mono_org;
float vfov_oak_d_mono;
// カメラバイアス（オリジナル値）
int c_bias_x_org, c_bias_y_org;
float fov_org;
// タイムスタンプ（ファースト/セカンドカメラ用）
std::chrono::time_point<std::chrono::steady_clock> ts_org_f;
std::chrono::time_point<std::chrono::steady_clock> ts_org_s;
// タイムスタンプ（ミリ秒）
uint64_t initialTs_ms_f;
uint64_t initialTs_ms_s;
// カメラ中心座標（各種カメラ用）
int camera_center_x_org, camera_center_y_org;
int camera_center_x_oak_d_lite;
int camera_center_y_oak_d_lite;
int camera_center_x_oak_d;
int camera_center_y_oak_d;
// カメラの画角（OAK-D, Lite）
float fov_ork_d, fov_ork_d_lite;
float fov;
float fov_2;
// ベース距離バイアス（各種モード用）
int base_depth_bias = 0;
int base_depth_bias_s = 0;
float base_depth_bias_hi = 0;
float base_depth_bias_hi_s = 0;
int base_depth_bias_720p = 0;
int base_depth_bias_s_720p = 0;
float base_depth_bias_hi_720p = 0;
float base_depth_bias_hi_s_720p = 0;
// セカンドカメラの補正値
int second_camera_length_bias;
int c_second_camera_length_bias;
int second_camera_high_bias;
float c_second_camera_high_bias;
int second_camera_high_bias_org;
int second_camera_x_bias, second_camera_y_bias;
int second_camera_length_bias_org;
float second_camera_pitch_bias, first_camera_pitch_bias, camera_pitch_org;
float second_camera_yaw_bias, first_camera_yaw_bias;
// セカンドカメラのシャッターフラグ
bool second_camera_sht;
// セカンドカメラのシャッター状態（直前値）
bool second_camera_sht_old = 999;
// モノクロ画像取得状態（ファースト/セカンド）
bool first_mono_act = false;
bool second_mono_act = false;
// セカンドカメラ有無
bool second_camera; // セカンドカメラ有無
// セカンドカメラYバイアス（オリジナル値）
int second_camera_y_bias_org;
// キー入力値
int key, key_old;
// 判定処理用フラグ
bool firstTs = false;
bool shutdown_f = false;
bool pr_flg = false;            // 判定結果を８秒間表示するフラグ
bool Start_of_judgment = false; // 判定結果を動画オーバライド＆一時表示フラグ
bool zone_flg = false;          // ゾーン検出済みか？
// ベースまでの直線距離（cm, int）
float straightLineDistanceToBase_cm; // ベースまでの直線距離
int straightLineDistanceToBase_i;    // ベースまでの直線距離(i)
// ベースまでの水平距離（cm, int）
float horizontalDistanceToBase_cm; // ベースまでの水平距離
int horizontalDistanceToBase_i;    // ベースまでの水平距離(i)
// ベース距離の前回値
int base_dep_i_old;
// ボール検知状態
bool ball_detection_f = false;
// ボール検知時刻（フレーム/直前）
uint64_t ball_detection_t_frame = 0;
uint64_t ball_detection_t_old = UINT64_MAX - 1000; // ボールを検知した時間（ms）ボール配列クリア用
// フレームカウント
int frame_cnt = 0;
// ボール検知時の各種タイムスタンプ
uint64_t first_time = 0;
uint64_t second_time = 0;
uint64_t trd_time = 0;
// 深度値
int s_dep = 256;
float first_dep_cm = 0;
float second_dep_cm = 0;
float trd_dep_cm = 0;
// 判定終了時刻
uint64_t e_time = 0;
uint64_t bat_f_time = UINT64_MAX;
// 球速
float snap_speed = 0;
// モノクロ画像取得エラーチェックフラグ
bool mono_read_check_f = true; // mono_read_errorのチェック是非
bool mono_read_check_s = true; // mono_read_errorのチェック是非

// --- 判定・カウント関連 ---
int e_dep = 256;
int base_detection_maker;
// 輪郭の数
int roiCnt = 0;
// ボール・ストライク・アウトカウント
int ball_no = 1;                // 投球数
int ball_cnt = 0;               // ボールカウント
int strike_cnt = 0;             // ストライクカウント
int out_cnt = 0;                // アウトカウント
int ball_detection_cnt = 3;     // ボール検知カウンタ
int ball_detection_cnt_old = 0; // ボール検知カウンタ
int ball_find_cnt;              // ボール検知数（エラー検知も含む）
bool strike_last = false;       // 前回ストライクだったか
bool strike;
// ボール検知エラー連続カウント
int ball_err_renzoku_cnt = 0;
// ボール進行カウント
int cntShinkou = 0;
// 膝高さ補正係数
float knee_keisuu;
// ボール中心座標（直前値）
Point2i center_last;
Dimensions center_last_3d;
int x_last = 0, y_last = 0;
// ボール描画色
Scalar bcolor, bcolor_shadow;
// FPSカウント
int fps_cnt = 0, fps_cnt_fix = 0;
// 各種ウィンドウ表示フラグ
bool wnd_dis_f_base = false;          // ウィンドウ表示フラグ
bool wnd_dis_f_rectifiedLeft = false; // ウィンドウ表示フラグ
bool wnd_dis_f_mask = false;          // ウィンドウ表示フラグ
bool wnd_dis_f_nama_depth = false;    // ウィンドウ表示フラグ

// フォーカス・露出関連
bool focus_comp = false;  // フォーカス合わせ済か？
bool monocam_cng = false; // 露出変更ありなし
// 打者の関節座標（各関節の画像上のx, y座標を格納）
// 例：knee_x, knee_y は膝のx座標・y座標
int knee_x, knee_y;         // 膝の座標
int shoulder_x, shoulder_y; // 肩の座標
int hip_x, hip_y;           // 腰の座標
int wrist_x, wrist_y;       // 手首の座標
int elbow_x, elbow_y;       // 肘の座標

// OpenCVのPoint2i型で各関節の座標をまとめて管理
Point2i knee_point;     // 膝の座標（x, y）
Point2i shoulder_point; // 肩の座標（x, y）
Point2i hip_point;      // 腰の座標（x, y）
Point2i wrist_point;    // 手首の座標（x, y）
Point2i elbow_point;    // 肘の座標（x, y）

// ボール移動距離しきい値（cm）
int c_speed_lim_min; // 前フレームからのボール移動距離のしきい値（cm）。この値以下の場合はボール検知カウントしない

// タイムカウント
uint64_t cnt_t = 0;
// ベース水平距離（前回値）
float old_straightLinelDistanceToBase_cm = 200; // ベース水平距離（OLD)
float c_horizontalDistanceToBase_cm;            // ベースまでの水平距離(固定)

// 判定・プリント関連
uint64_t pr_time;                          // プリント時間
uint64_t Judgment_machi_time = UINT64_MAX; // 判定待ち時間
bool calibrate = false;                    // キャリブレーション済か
int ball_detection_phase = invalid;        // ボール検知フェイズ
int x_bias_d;                              // ボール探索範囲
int y_bias_d;                              // ボール探索範囲
float ball_first_len_cm;                   // ボールを最初に検知したときの距離
float ball_d_1_old, ball_d_2_old;
// 投球検知判定のしきい値（cm）
int tsuuka_filter; // 最初にと最後にボール検知した距離の差がこの値以下であれば投球としない
// ベース判定クラス
Base basez;
// ゾーンイラスト画像
Mat zone_il, zone_il_r, zone_il_l, zone_il_top, zone_il_top_over;
// 2値化フィルタ値
int best_v_filter = 201;
// 判定結果表示時間
int c_kekka_print_time;
// 録画フラグ
bool rokuga_flg = false;  // カラー画像と２値画像の録画ON/OFF
bool rokuga_flg2 = false; // DEPTH画像の録画ON/OFF
//! ボール検知状態:true
bool Judgment_mode = false;
// ボール記録待ち時間
uint64_t ball_kiroku_machi = 0;
// スナップショット機能のための変数
bool snap_shot = false;       // スナップショットが要求されたかどうかを示すフラグ
bool snap_shot_sumi = false;  // スナップショットが撮られたかどうかを示すフラグ
bool snap_shot_start = false; // スナップショットを開始する必要があるかどうかを示すフラグ

// ベース検知なし・地面設置モード
bool no_base_mode = false; // ベース検知なし
bool ground_mode;          // 地面設置モード
// ボール初回検知距離・時刻
float ball_i_fast = 0;   // 最初のボール距離
uint64_t ts_msec_fast;   // 最初のボール時間
float ball_i_fast_2 = 0; // 最初のボール距離(セカンドカメラ)
uint64_t ts_msec_fast_2; // 最初のボール時間(セカンドカメラ)
// ボール検知距離（直前値）
float last_dep_cm_old = c_dep_cm_en;
float last_dep_cm_old_f = c_dep_cm_en;
float last_dep_cm_old_s = c_dep_cm_en;
float last_ball_h_old_f = 999;
float last_ball_h_old_s = 999;
int snapshot_cnt;
// カラー画像の保存設定（処理）
bool snap_shot_onoff = true; // カラー画像の保存が有効かどうかを示すフラグ
// 開始待ちカウント
uint8_t Judgment_mode_kaishimachi_cnt = 0;
//! WAV 再生
bool play_sound_f = false;
cv::VideoWriter writer2;                                   // カラー画像
cv::VideoWriter writer3;                                   // 距離フィルター後の２値画像 Mat binary_mask_distance, binary_mask_distance_in
cv::VideoWriter writer5;                                   // 深度画像
int fourcc1 = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // Xvid
int fourcc2 = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // Xvid
int fourcc3 = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // Xvid
int fourcc4 = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // Xvid
int fourcc5 = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // Xvid
int fourcc6 = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // Xvid
bool bin_img_f = false, r_image_f = false, frame_mono_f = false, binary_mask_distance_f = false, r_image_tmp_f = false;
bool test_measurement_mode = false, confidenceMap_f = false, rectifiedLeft_f = false;
char *parm;
bool v_rokuga_sw = rokuga_sw;
bool imu_use; //ork-dの可読度センサーを利用してカメラ角度を採取するか
//! ゾーン近くで検知されたか
bool ball_zone_s = false;
mutex mtx;
// "ステート"変数＋ミューテックスmtx＋条件変数cv
int state;                          // 注: 変数型やその個数は目的による
bool RGB_image_acquisition = false; // 深度取得ステータス
std::condition_variable cond;
// dai::CameraControl ctrl_script;
// モノクロカメラ(ファーストカメラ)の解像度を720pにする場合はtrue、400pの場合はfalse
bool p720p_on = false;
// モノクロカメラ(セカンドカメラ)の解像度を720pにする場合はtrue、400pの場合はfalse
bool p720p_2_on = false;
nlohmann::json conf_json;

// グローバルなKalmanフィルタ
KalmanFilter kf(9, 3, 0); // 状態ベクトル9次元、観測ベクトル3次元
bool isFirstObservation = true;
double prevTime = 0;

//FPS監視（ゾーンカラー）
Scalar shiro = Scalar(200, 200, 200);

/**********************************************************************************/

/**********************************************************************************/
std::vector<dai::ImgDetection> detections;
static const std::vector<std::string> labelMap = {
    "person",
    "bicycle",
    "car",
    "motorbike",
    "aeroplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "sofa",
    "pottedplant",
    "bed",
    "diningtable",
    "toilet",
    "tvmonitor",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush"};

// Add bounding boxes and text to the frame and show it to the user
auto displayFrame = [](std::string name, cv::Mat *frame, std::vector<dai::ImgDetection> &detections) {
    auto color = cv::Scalar(0, 128, 255);
    // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    for (auto &detection : detections)
    {
        int x1 = detection.xmin * 416;
        int y1 = detection.ymin * 416;
        int x2 = detection.xmax * 416;
        int y2 = detection.ymax * 416;

        int labelIndex = detection.label;
        std::string labelStr = to_string(labelIndex);
        if (labelIndex < labelMap.size())
        {
            labelStr = labelMap[labelIndex];
        }
        basez.batter_top = 0;
        basez.batter_bottom = 0;
        basez.batter_right = 0;
        basez.batter_left = 0;
        if (labelIndex == 0)
        {
            cv::putText(*frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 1, 255);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;

            cv::putText(*frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 1, 255);
            cv::rectangle(*frame, cv::Rect(cv::Point(x1 - 208 + 320, y1 - 208 + 200), cv::Point(x2 - 208 + 320, y2 - 208 + 200)), color, 2);

            if ((x2 < basez.zone_nw.x) || (x1 > basez.zone_ne.x))
            {
                basez.batter_top = y1 - 208 + 200;
                basez.batter_bottom = y2 - 208 + 200;
                basez.batter_right = x1 - 208 + 320;
                basez.batter_left = x2 - 208 + 320;
            }
        }
        break;
    }
};
/**********************************************************************************/

/**
 * @brief LEDの点滅制御
 * @details スイッチ26のGPIOを制御し、LEDを点滅させる
 * @param ms 点灯時間（ms）
 * @param ms2 消灯時間（ms）
 * @param time 点滅回数
 * @return なし
 * @sa led_sig()
 */
void led_sig(int ms, int ms2, int time)
{
    LOG_DEBUG.printf("LED bring # %dms,time:%d", ms, time);
    for (int i = 0; i < time; i++)
    {
#if (raspi5 == true)
        // GPIOの値を1に設定する
        gpiod_line_set_value(sw_26_LED_sw, 1);
        usleep(ms * 1000);
        gpiod_line_set_value(sw_26_LED_sw, 0);
        usleep(ms2 * 1000);
#endif
    }
}
void led(int ms, int ms2, int time)
{
    thread th_led(led_sig, ms, ms2, time);
    th_led.detach();
}
void led(int ms)
{
    led(ms, 0, 1);
}

/**
 * @brief MOTOR ON
 * @details スイッチ20のGPIOを制御し、MOTORをONする
 * @param ms ON時間（ms）
 * @return なし
 * @sa motor_on()
 */
void motor_on(int ms)
{
#if (raspi5 == true)
    LOG_DEBUG.printf("MOTOR ON");
    // GPIOの値を1に設定する
    gpiod_line_set_value(sw_20_MOTORSW, 1);
    usleep(ms * 1000);
    gpiod_line_set_value(sw_20_MOTORSW, 0);
#endif
}
void motor(int ms)
{
    thread th_motor(motor_on, ms);
    th_motor.detach();
}

/**********************************************************************************/

/**
 * @brief サイドからのボールコース表示座標（横幅）計算
 * @param dep_cm ボールの深さ（cm）
 * @param w_cm ボールの横幅（cm）
 * @return 画像上の横幅の座標
 * @note 画像上の横幅の座標を計算する。
 * 画像の横幅は1900px、横幅は487pxとする。
 * 横幅の計算式は、ボールの深さから、
 * 400cmとボールの横幅の和で引いたものを、
 * 400cmとボールの横幅の和で割り、1900pxにスケーリングし、487pxを足したものとする。
 */
#define c_camera_distance 250 // サイドカメラからベースまでの距離cm
#define magnification 1200    //bairitsu
float side_view_add_y(float h_cm, float w_cm)
{
    h_cm = (h_cm < 0) ? 0 : h_cm;
    // return ((120 - h_cm) / (250 + w_cm) * 1200 + 57); // 250離れた場所から、距離：400　ｘ：2000、ｙ：33
    // return ((120 - h_cm) / (250 + w_cm) * 1000) + 120; // 距離：400　ｘ：2000、ｙ：33
    /*
    int a = 3000;                              //倍率 (ゾーンの大きさに影響)
    float x = 250;                             //カメラ距離
    float b = 220;                             //カメラ高さ
    float y = (b - h_cm) / (x + w_cm) * a + 0; // 距離：400　ｘ：2000、ｙ：33
    int bi;
    int loc = 75; //カメラピッチ
    bi = 634 - basez.calc_rotation(x, b / x * a, 0, 0, basez.rdn(loc)).y;
    return (basez.calc_rotation(x, y, 0, 0, basez.rdn(loc)).y + bi);
    */
    return (h_cm / (c_camera_distance - w_cm) * magnification); // 250離れた場所から、距離：400　ｘ：2000、ｙ：33
}
float side_view_add_x(float dep_cm, float w_cm)
{
    return ((dep_cm - horizontalDistanceToBase_cm) / (c_camera_distance - w_cm) * magnification + c_base_center_x);
}
float side_view_add_x_zone(float dep_cm, float w_cm)
{
    return ((dep_cm) / (c_camera_distance - w_cm) * magnification + c_base_center_x);
}
/**********************************************************************************/

ball_cours ball[256]; // ボールコース記録
cv::Mat ball_cours_xyz = cv::Mat::zeros(256, 1, CV_32FC3);
//! ボールコースカウント
int ball_cours_cnt = 0;
/**********************************************************************************/

uint64_t ts_msec, ts_msec_old, ts_msec_1_old, ts_msec_2_old; //距離画像フレーム取得時間
/**********************************************************************************/

/**********************************************************************************/
int mouse_event;          // マウスイベント種別
int mouse_x, mouse_x_old; //
int mouse_y, mouse_y_old; //
/**
 * @brief マウスイベント
 * @param event マウスイベントの種類
 * @param x マウスカーソルのx座標
 * @param y マウスカーソルのy座標
 * @param flags マウスイベントのフラグ
 * @param param 任意のパラメータ (オプション)
 * @return なし
 * @sa
 * @note この関数は、マウスイベントが発生したときに呼び出されます。
 * イベントの種類、カーソルの位置、フラグ、オプションのパラメータを処理します。
 */
uint64_t click_time = 0;
void on_mouse(int event, int x, int y, int flags, void *param = NULL)
{
    mouse_event = event;
    mouse_x = x * (float)c_rez_x / c_rez_x_r;
    mouse_y = y * (float)c_rez_y / c_rez_y_r;
}
/**********************************************************************************/

// カルマンフィルタ初期化関数
void initializeKalmanFilter()
{
    //カルマンフィルタ start
    // 状態遷移行列 (9x9)
    kf.transitionMatrix = Mat::eye(9, 9, CV_32F);

    // 観測行列 (3x9)
    kf.measurementMatrix = Mat::zeros(3, 9, CV_32F);
    kf.measurementMatrix.at<float>(0, 0) = 1.0; // w に対応
    kf.measurementMatrix.at<float>(1, 1) = 1.0; // h に対応
    kf.measurementMatrix.at<float>(2, 2) = 1.0; // d に対応

    // 状態ノイズ共分散行列 (9x9)
    setIdentity(kf.processNoiseCov, Scalar::all(1e-2));

    // 観測ノイズ共分散行列 (3x3)
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));

    // 誤差共分散行列 (9x9)
    setIdentity(kf.errorCovPost, Scalar::all(1));
    isFirstObservation = true;
    //カルマンフィルタ end
}

/**
 * @brief ボールコースの初期化
 * @details ballコースを記録したBall配列の中身をクリアする
 * @param
 * @return
 * @sa
 */
bool ball_clear_sleep = false; // トリガーモードの場合はボールクリアを停止するため
void ball_clear(int st_d)
{
    if (!ball_clear_sleep)
    {
        for (int loopcnt = st_d; loopcnt < 256; loopcnt++)
        {
            ball[loopcnt].cnt = 999;
            ball[loopcnt].dep = 999;
            ball[loopcnt].dep_cm = 0;
            ball[loopcnt].x = 0;
            ball[loopcnt].y = 0;
            ball[loopcnt].z = 0;
            ball[loopcnt].hosei_x = 0;
            ball[loopcnt].hosei_y = 0;
            ball[loopcnt].w_cm = 0;
            ball[loopcnt].h_cm = 0;
            ball[loopcnt].radius = 0;
            ball[loopcnt].strike = false;
            ball[loopcnt].ball_hi = true;
            ball[loopcnt].ball_lo = true;
            ball[loopcnt].ball_right = true;
            ball[loopcnt].ball_left = true;
            ball[loopcnt].time = 0;
            ball_cours_xyz.at<float>(loopcnt, 0) = 0;
            ball_cours_xyz.at<float>(loopcnt, 1) = 0;
            ball_cours_xyz.at<float>(loopcnt, 2) = 0;
            ball[loopcnt].camera_no = 0;
        }
        LOG_DEBUG.printf("clear ball record");
        if (st_d == 0)
        {
            ball_cours_cnt = 0;
            basez.ball_cours_cnt = 0;
            ball_detection_phase = (Judgment_mode) ? far_away : invalid;
            ball_detection_f = false;
            ball_detection_cnt = 0;
            ball_detection_cnt_old = 0;
            ball_find_cnt = 0;
            ball_first_len_cm = 0;
            basez.ball_position_old.w = 0;
            basez.ball_position_old.h = 100;
            basez.ball_position_old.d = 420;
            basez.x_bias = c_rect_x;
            basez.y_bias = c_rect_y;
            x_bias_d = 640 - c_rect_x * 2;
            y_bias_d = 400 - c_rect_y;
            ball_i_fast = 0;          //最初のボール距離の記憶
            ts_msec_fast = ts_msec;   //最初のボールの時間記憶
            ball_i_fast_2 = 0;        //最初のボール距離の記憶(セカンドカメラ)
            ts_msec_fast_2 = ts_msec; //最初のボールの時間記憶(セカンドカメラ)
            ts_msec_1_old = 0;
            ts_msec_2_old = 0;
            // basez.ball_noise_clear();
            initializeKalmanFilter();
            cntShinkou = 0;
            dep_threshold = dep_base_en;
            ball_detection_t_old = UINT64_MAX - 1000;
            first_time = UINT64_MAX - 1000;
            last_dep_cm_old = c_dep_cm_en;   //前回ボール検出時の距離
            last_dep_cm_old_f = c_dep_cm_en; //前回ボール検出時の距離
            last_dep_cm_old_s = c_dep_cm_en; //前回ボール検出時の距離
            last_ball_h_old_f = 999;         //前回ボール検出時の高さ
            last_ball_h_old_s = 999;         //前回ボール検出時の高さ
        }
        ball_rireki = Mat::zeros(400, 640, CV_8UC1);
        basez.center_temp_old = Point(0, 0);
        basez.center_temp_old_f = Point(0, 0);
        basez.center_temp_old_s = Point(0, 0);
        ball_err_renzoku_cnt = 0;
    }
}
void ball_clear()
{
    ball_clear(0);
}

/**********************************************************************************/

// Point2i の座標を指定した範囲内に制限する関数
Point2i clampPoint(const Point2i &point, int minX, int maxX, int minY, int maxY)
{
    // x 座標と y 座標をそれぞれ指定した範囲に収める
    int clampedX = std::max(minX, std::min(point.x, maxX));
    int clampedY = std::max(minY, std::min(point.y, maxY));
    return Point2i(clampedX, clampedY);
}
Point2i clampPoint(const Point2i &point)
{
    return clampPoint(point, 0, 639, 0, 399);
}

/**
 * @brief ホームベース/ストライクゾーンの描画
 * @details ゾーンデータはbasezに記録した座標
 * @param Mat *描画対象画像
 * @return
 * @sa
 */
void zone_print(Mat *r_image_SARA)
{
    if (!basez.base_find_f)
        return;
    Point hosei_back_l, hosei_back_r, hosei_front_l, hosei_front_r, hosei_back_l_h, hosei_back_r_h, hosei_front_l_h, hosei_front_r_h, hosei_base_h, hosei_base_l;

    /*
    zahyou_kaiten(basez.zone_left_middle_low, basez.baseCoordinateBase, &hosei_back_l.x, &hosei_back_l.y, basez.base_roll);
    zahyou_kaiten(basez.zone_right_middle_low, basez.baseCoordinateBase, &hosei_back_r.x, &hosei_back_r.y, basez.base_roll);
    zahyou_kaiten(basez.zone_left_front_low, basez.baseCoordinateBase, &hosei_front_l.x, &hosei_front_l.y, basez.base_roll);
    zahyou_kaiten(basez.zone_right_front_low, basez.baseCoordinateBase, &hosei_front_r.x, &hosei_front_r.y, basez.base_roll);
    zahyou_kaiten(basez.zone_base_low, basez.baseCoordinateBase, &hosei_base_l.x, &hosei_base_l.y, basez.base_roll);

    zahyou_kaiten(basez.zone_left_middle_hi, basez.baseCoordinateBase, &hosei_back_l_h.x, &hosei_back_l_h.y, basez.base_roll);
    zahyou_kaiten(basez.zone_right_middle_hi, basez.baseCoordinateBase, &hosei_back_r_h.x, &hosei_back_r_h.y, basez.base_roll);
    zahyou_kaiten(basez.zone_left_front_hi, basez.baseCoordinateBase, &hosei_front_l_h.x, &hosei_front_l_h.y, basez.base_roll);
    zahyou_kaiten(basez.zone_right_front_hi, basez.baseCoordinateBase, &hosei_front_r_h.x, &hosei_front_r_h.y, basez.base_roll);
    zahyou_kaiten(basez.zone_base_hi, basez.baseCoordinateBase, &hosei_base_h.x, &hosei_base_h.y, basez.base_roll);
*/

    // ストライクゾーン描画
    hosei_back_l.x = clamp(basez.zone_left_middle_low.x, 0, 639);
    hosei_back_l.y = clamp(basez.zone_left_middle_low.y, 0, 399);
    hosei_front_l.x = clamp(basez.zone_left_front_low.x, 0, 639);
    hosei_front_l.y = clamp(basez.zone_left_front_low.y, 0, 399);
    hosei_back_r.x = clamp(basez.zone_right_middle_low.x, 0, 639);
    hosei_back_r.y = clamp(basez.zone_right_middle_low.y, 0, 399);
    hosei_front_r.x = clamp(basez.zone_right_front_low.x, 0, 639);
    hosei_front_r.y = clamp(basez.zone_right_front_low.y, 0, 399);
    hosei_back_l_h.x = clamp(basez.zone_left_middle_hi.x, 0, 639);
    hosei_back_l_h.y = clamp(basez.zone_left_middle_hi.y, 0, 399);
    hosei_front_l_h.x = clamp(basez.zone_left_front_hi.x, 0, 639);
    hosei_front_l_h.y = clamp(basez.zone_left_front_hi.y, 0, 399);
    hosei_back_r_h.x = clamp(basez.zone_right_middle_hi.x, 0, 639);
    hosei_back_r_h.y = clamp(basez.zone_right_middle_hi.y, 0, 399);
    hosei_front_r_h.x = clamp(basez.zone_right_front_hi.x, 0, 639);
    hosei_front_r_h.y = clamp(basez.zone_right_front_hi.y, 0, 399);
    hosei_base_h.x = clamp(basez.zone_base_hi.x, 0, 639);
    hosei_base_h.y = clamp(basez.zone_base_hi.y, 0, 399);
    hosei_base_l.x = clamp(basez.zone_base_low.x, 0, 639);
    hosei_base_l.y = clamp(basez.zone_base_low.y, 0, 399);

    line(*r_image_SARA, hosei_back_l, hosei_back_l_h, shiro, 2, LINE_AA);   // 内角手前縦線
    line(*r_image_SARA, hosei_front_l, hosei_front_l_h, shiro, 2, LINE_AA); // 内角投手側縦線
    line(*r_image_SARA, hosei_back_r, hosei_back_r_h, shiro, 2, LINE_AA);   // 外角手前縦線
    line(*r_image_SARA, hosei_front_r, hosei_front_r_h, shiro, 2, LINE_AA); // 外角投手側縦線

    line(*r_image_SARA, hosei_back_l, hosei_base_l, shiro, 2, LINE_AA);   //
    line(*r_image_SARA, hosei_back_r, hosei_base_l, shiro, 2, LINE_AA);   //
    line(*r_image_SARA, hosei_back_l_h, hosei_base_h, shiro, 2, LINE_AA); //
    line(*r_image_SARA, hosei_back_r_h, hosei_base_h, shiro, 2, LINE_AA); //

    line(*r_image_SARA, hosei_front_l, hosei_front_r, shiro, 2, LINE_AA);     // 投手側横線
    line(*r_image_SARA, hosei_front_l_h, hosei_front_r_h, shiro, 2, LINE_AA); //  投手側横線
    line(*r_image_SARA, hosei_base_h, hosei_base_l, shiro, 2, LINE_AA);       // 中心縦線
    // line(*r_image_SARA, hosei_back_l, hosei_back_r, shiro, 1, LINE_AA);       // 手前側横線
    // line(*r_image_SARA, hosei_back_l_h, hosei_back_r_h, shiro, 1, LINE_AA);   //  手前側横線

    // ゾーン底辺
    line(*r_image_SARA, hosei_back_l, hosei_front_l, shiro, 2, LINE_AA);
    line(*r_image_SARA, hosei_back_r, hosei_front_r, shiro, 2, LINE_AA);
    line(*r_image_SARA, hosei_back_l_h, hosei_front_l_h, shiro, 2, LINE_AA);
    line(*r_image_SARA, hosei_back_r_h, hosei_front_r_h, shiro, 2, LINE_AA);

    line(*r_image_SARA, Point((hosei_back_l.x + hosei_front_l.x) / 2, (hosei_back_l.y + hosei_front_l.y) / 2), Point((hosei_back_l_h.x + hosei_front_l_h.x) / 2, (hosei_back_l_h.y + hosei_front_l_h.y) / 2), Scalar(0, 155, 205), 3, LINE_AA);
    line(*r_image_SARA, Point((hosei_back_r.x + hosei_front_r.x) / 2, (hosei_back_r.y + hosei_front_r.y) / 2), Point((hosei_back_r_h.x + hosei_front_r_h.x) / 2, (hosei_back_r_h.y + hosei_front_r_h.y) / 2), Scalar(0, 155, 205), 3, LINE_AA);
    line(*r_image_SARA, Point((hosei_back_l.x + hosei_front_l.x) / 2, (hosei_back_l.y + hosei_front_l.y) / 2), Point((hosei_back_r.x + hosei_front_r.x) / 2, (hosei_back_r.y + hosei_front_r.y) / 2), Scalar(0, 155, 205), 3, LINE_AA);
    line(*r_image_SARA, Point((hosei_back_r_h.x + hosei_front_r_h.x) / 2, (hosei_back_r_h.y + hosei_front_r_h.y) / 2), Point((hosei_back_l_h.x + hosei_front_l_h.x) / 2, (hosei_back_l_h.y + hosei_front_l_h.y) / 2), Scalar(0, 155, 205), 3, LINE_AA);

    if (no_base_mode)
    {
        //ベース（地面）
        if (!Judgment_mode)
        {
            Scalar gr = Scalar(0, 155, 0);
            line(*r_image_SARA, clampPoint(basez.zone_left_front_gnd), clampPoint(basez.zone_right_front_gnd), gr, 2, LINE_AA);
            line(*r_image_SARA, clampPoint(basez.zone_left_front_gnd), clampPoint(basez.zone_left_middle_gnd), gr, 2, LINE_AA);
            line(*r_image_SARA, clampPoint(basez.zone_right_front_gnd), clampPoint(basez.zone_right_middle_gnd), gr, 2, LINE_AA);
            line(*r_image_SARA, clampPoint(basez.zone_base_gnd), clampPoint(basez.zone_left_middle_gnd), gr, 2, LINE_AA);
            line(*r_image_SARA, clampPoint(basez.zone_base_gnd), clampPoint(basez.zone_right_middle_gnd), gr, 2, LINE_AA);
        }
        /* 五角形の頂点座標
        std::vector<cv::Point> points;
        points.push_back(clampPoint(basez.zone_left_front_gnd));
        points.push_back(clampPoint(basez.zone_left_middle_gnd));
        points.push_back(clampPoint(basez.zone_base_gnd));
        points.push_back(clampPoint(basez.zone_right_middle_gnd));
        points.push_back(clampPoint(basez.zone_right_front_gnd));
        // 頂点を多次元配列に変換
        std::vector<std::vector<cv::Point>> pts = {points};
        // 五角形を塗りつぶして描画
        if (Judgment_mode)
        {
            cv::fillPoly(*r_image_SARA, pts, cv::Scalar(255, 255, 255));
        }
        */
        //ベース目印
        if (!Judgment_mode)
        {
            line(*r_image_SARA, Point(320 - c_no_noize_area, 250), Point(320 - c_no_noize_area, 350), Scalar(205, 155, 0), 1, LINE_AA);
            line(*r_image_SARA, Point(320 + c_no_noize_area, 250), Point(320 + c_no_noize_area, 350), Scalar(205, 155, 0), 1, LINE_AA);
        }
    }
}
/**********************************************************************************/

/**********************************************************************************/

// 球筋データをCSVに書き出す関数
void writeBallDataToCSV(
    ball_cours ball[],                                   // ボールデータ配列
    int dep_l_kin,                                       // データ開始インデックス
    int dep_l_en,                                        // データ終了インデックス
    int raw_flg,                                         //0:raw,1:ロバスト後
    const std::string &log_dir = "/home/pi/kyouyuu/LOG/" // 書き出し先ディレクトリ
)
{
    // 現在時刻を取得してファイル名を生成
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    char buff[128] = "";
    std::string file_name;
    int bis;
    sprintf(buff, "_%d_%02d_%02d_%02d%02d%02d_%02d.csv", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
    if (raw_flg == 0)
    {
        file_name = log_dir + "ball(raw)" + buff;
        bis = 1;
    }
    else
    {
        file_name = log_dir + "ball" + buff;
        bis = 2;
    }

    // ファイルを開く
    std::ofstream file(file_name);
    if (!file.is_open())
    {
        std::cerr << "ファイルのオープンに失敗しました: " << file_name << std::endl;
        return;
    }

    // CSVヘッダの書き出し
    file << "x,y,z,radius,camera_no" << std::endl;

    // ボールデータを書き出し
    for (int top_i = dep_l_kin + 1; top_i >= dep_l_en; top_i -= bis)
    {
        if (ball[top_i].dep_cm != 0)
        { // インデックス範囲チェック
            file << std::fixed << std::setprecision(2)
                 << ball[top_i].w_cm << ","
                 << ball[top_i].dep_cm << ","
                 << ball[top_i].h_cm << ","
                 << ball[top_i].radius << ","
                 << ball[top_i].camera_no << std::endl;
        }
    }

    // ファイルを閉じる
    file.close();
    std::cout << "ファイルに書き出しました: " << file_name << std::endl;
}

/** GUI オブジェクト
 * @brief x
 * @brief y
 */
struct button
{
    int x;
    int y;
    int x_size;
    int y_size;
    bool click;
    int button;
};
#define button_num 10;
button button[10] = {
    10,
    10,
    130,
    30,
    false,
    button_long,
    10,
    50,
    130,
    30,
    false,
    button_long,
    10,
    90,
    130,
    30,
    true,
    button_long,
    10,
    130,
    130,
    30,
    false,
    button_long,
    10,
    170,
    130,
    30,
    false,
    button_long,
    440,
    150,
    60,
    30,
    false,
    button_mini,
    600,
    15,
    30,
    30,
    false,
    button_f,
    600,
    65,
    30,
    30,
    false,
    button_f,
    600,
    115,
    30,
    30,
    false,
    button_f,
    540,
    150,
    60,
    30,
    false,
    button_mini,
};
std::vector<std::string> button_str = {"180cm-", "170cm-", "160cm-", "150cm-", "140cm-", "CLR", "", "", "", "RST"};
/**********************************************************************************/

//! 3d zone イラスト座標
struct zone_3d_il
{
    int dep_i;
    Point2f nw;
    Point2f ne;
    int y_size;
};
/**********************************************************************************/

//! zone_il イラスト座標
struct st_zone_il
{
    int num;
    Point2i ball_position;
    bool strike;
};

st_zone_il ball_il_position[20];

/**
 * @brief スリープを実施する
 * @param sleeptime スリープする時間 (単位: 秒)
 * @return なし
 * @note 現在の時間から指定された時間だけスリープする。
 * スリープ中は、CPUの処理が一切行われない。
 */
void sleep_sec(int sleeptime)
{
    int i = 0;
    while (i < sleeptime)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 1秒スリープ
        i++;
    }
}
void sleep_msec(int sleeptime)
{
    int i = 0;
    while (i < sleeptime)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1m秒スリープ
        i++;
    }
}
/**********************************************************************************/

/**
 * @brief WAV再生
 * @param なし
 * @return なし
 * @note ストライクかボールの音を再生するために使用する。
 * ストライクかボールの判定が行われた後、再生する音を決定し、
 * 対応するコマンドを実行して音を再生する。
 */
void play_sound_func()
{
    if (play_sound_f)
    {
        if (strike_last)
        {
#if (raspi == false) || (raspi == FALSE)
            system("python strike.py");
#else
            switch (strike_course)
            {
            case c_strike_center: // center
                system("aplay strike.wav");
                break;
            case c_strike_hi: // hi
                system("aplay strike_s1.wav");
                break;
            case c_strike_lo: //center lo
                system("aplay strike_s2.wav");
                break;
            case c_strike_nice: // out/in middle/lo
                system("aplay strike_l1.wav");
                break;

            default:
                system("aplay strike.wav");
                break;
            }
#endif
        }
        else
        {
#if (raspi == false) || (raspi == FALSE)
            system("python ball.py");
#else
            switch (ball_cnt) // ボールの音声準備ができればstrike_courseに変える
            {
            case 0:
                system("aplay ball.wav");
                break;
            case 1:
                system("aplay ball_s1.wav");
                break;
            case 2:
                system("aplay ball_s2.wav");
                break;
            case 3:
                system("aplay ball_s3.wav");
                break;
            case 4:
                system("aplay ball_l1.wav");
                break;

            default:
                break;
            }
#endif
        }
        play_sound_f = false;
    }
    return;
}
void play_sound()
{
    thread play_s(play_sound_func);
    play_s.detach();
}

void play_sound_func_2(int n)
{
    //外部pythonへの音声シグナル送信の初期定義
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(12345);
    inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        LOG_DEBUG.printf("Connection Failed");
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 200msスリープ

    String message = "play1.wav";
    if (n == 0)
    {
        send(sock, message.c_str(), message.length(), 0);
    }
    else

    // while(true)
    {
        if (play_sound_f)
        {
            if (strike_last)
            {
                switch (strike_course)
                {
                case c_strike_center: // center
                    message = "strike.wav";
                    break;
                case c_strike_hi: // hi
                    message = "strike_s1.wav";
                    break;
                case c_strike_lo: //center lo
                    message = "strike_s2.wav";
                    break;
                case c_strike_nice: // out/in middle/lo
                    message = "strike_l1.wav";
                    break;
                default:
                    message = "strike.wav";
                    break;
                }
            }
            else
            {
                switch (ball_cnt) // ボールの音声準備ができればstrike_courseに変える
                {
                case 0:
                    message = "ball.wav";
                    break;
                case 1:
                    message = "ball_s1.wav";
                    break;
                case 2:
                    message = "ball_s2.wav";
                    break;
                case 3:
                    message = "ball_s3.wav";
                    break;
                case 4:
                    message = "ball_l1.wav";
                    break;

                default:
                    message = "ball.wav";
                    break;
                }
            }

            send(sock, message.c_str(), message.length(), 0);
            LOG_DEBUG.printf("play sound: %s", message.c_str());
            play_sound_f = false;
        }
    }
    close(sock);
}

/**********************************************************************************/

/**
 * @brief ボタンなどGUI表示
 * @param Mat &img
 * @param bool BSO_Only
 */
void print_gui(Mat &img, bool bso_only)
{
    Mat button_img;
    if (!bso_only)
    {
        for (int l = 0; l < 10; l++)
        {
            switch (button[l].button)
            {
            case button_long:
                button_img = cv::imread("button2.png", -1);
                break;
            case button_mini:
                button_img = cv::imread("button3.png", -1);
                break;
            case button_f:
                button_img = cv::imread("plus.png", -1);
                break;
            default:
                break;
            }
            putTranspPng(img, button_img, button[l].x, button[l].y);
            cv::putText(img, button_str[l], cv::Point(button[l].x, button[l].y + 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(20, 20, 20), 2);
        }
        for (int l = 0; l < 5; l++)
        {
            if (basez.batter_hight == l)
            {
                rectangle(img, Point(button[l].x, button[l].y), Point(button[l].x + button[l].x_size, button[l].y + button[l].y_size), Scalar(250, 0, 0), 2, 1);
            }
        }
    }

    Mat BSO_jpg = imread("BSO.jpg"); // BSO背景
    paste(img, BSO_jpg, 440, 10, 150, 140);

    for (int ball_c = 0; ball_c < (ball_cnt); ball_c++)
    {
        cv::circle(img, cv::Point(495 + (ball_c * 36), 34), 17, cv::Scalar(0, 200, 0), -1, LINE_AA);
    }
    for (int strike_c = 0; strike_c < (strike_cnt); strike_c++)
    {
        cv::circle(img, cv::Point(495 + (strike_c * 36), 80), 17, cv::Scalar(0, 200, 200), -1, LINE_AA);
    }
    for (int out_c = 0; out_c < (out_cnt); out_c++)
    {
        cv::circle(img, cv::Point(495 + (out_c * 36), 125), 17, cv::Scalar(0, 0, 200), -1, LINE_AA);
    }
}

/**
 * @brief 録画ON/OFF
 * @param int 1:BGR,2:BIN,3:DEPTH
 * @param bool on/off
 * @return
 * @sa
 */
void rokuga_onoff(int num, bool on_off)
{
    if (!v_rokuga_sw)
        return;
    if (on_off)
    {
        time_t now = time(NULL);
        struct tm *pnow = localtime(&now);
        char buff[128] = "";
        switch (num)
        {
        case rec_BGR:
            if (!writer2.isOpened())
            { // カラー画像
                LOG_NONE.printf("rokuga on BGR");
                rokuga_flg = true;
                sprintf(buff, "/home/pi/kyouyuu/LOG/ball_BGR%d_%02d_%02d_%02d%02d%02d.mp4", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
                writer2.open(buff, fourcc2, 30, cv::Size(640, 400)); // カラー画像
                break;
            }
        case rec_bin:
            if (!writer3.isOpened())
            { // 2値画像
                LOG_NONE.printf("rokuga on BIN");
                rokuga_flg = true;
                sprintf(buff, "/home/pi/kyouyuu/LOG/ball_BIN%d_%02d_%02d_%02d%02d%02d.mp4", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
                writer3.open(buff, fourcc3, 117, cv::Size(640, c_rect_y_h)); // 距離フィルター後の２値画像
                break;
            }
        case rec_depth:
            if (!writer5.isOpened())
            { // 深度画像
                LOG_NONE.printf("rokuga on depth");
                rokuga_flg2 = true;
                sprintf(buff, "/home/pi/kyouyuu/LOG/ball_depth%d_%02d_%02d_%02d%02d%02d.mp4", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
                writer5.open(buff, fourcc5, 117, cv::Size(640 - (c_rect_x * 2), c_rect_y_h)); // 深度画像
            }
        }
        //  動画ファイルを書き出すためのオブジェクトを宣言する
    }
    else
    {
        switch (num)
        {
        case rec_BGR:
            if (writer2.isOpened() && rokuga_flg)
            {
                rokuga_flg = false;
                waitKey(1000);
                writer2.release(); // カラー画像
                LOG_NONE.printf("rokuga off BGR");
            }
            break;
        case rec_bin:
            if (writer3.isOpened() && rokuga_flg)
            {
                rokuga_flg = false;
                waitKey(1000);
                writer3.release(); // 距離フィルター後の２値画像
                LOG_NONE.printf("rokuga off BIN");
            }
            break;
        case rec_depth:
            if (writer5.isOpened() && rokuga_flg2)
            {
                rokuga_flg2 = false;
                waitKey(1000);
                writer5.release(); // 深度画像
                LOG_NONE.printf("rokuga off depth");
            }
            break;
        }
    }
}

//全体終了処理
void exit_strike(bool ex)
{
    LOG_DEBUG.printf("exit_strike");
    if (ex)
    {
        rokuga_onoff(rec_BGR, false);
        rokuga_onoff(rec_bin, false);
        rokuga_onoff(rec_depth, false);
        Judgment_mode = false;
        ball_detection_phase = invalid;
        end_flg_img_key = false;
        end_flg_rgb_read = false;
        end_flg_mono_read = false;
        sleep_sec(1);
        end_flg_Judgment_main_get = false;
        isActJudgment_main = false;
        end_flg_img_key = false;
        state = true;
    }
}

// depフィールドを基準にソートするための比較関数
bool compareByDep(const ball_cours &a, const ball_cours &b)
{
    return a.dep < b.dep;
}
// timeフィールドを基準にソートするための比較関数
bool compareByTime(const ball_cours &a, const ball_cours &b)
{
    return a.time < b.time;
}

/**
 * @brief どのボタンがクリックされたか *
 * @param int クリックされたx座標 *
 * @param int クリックされたy座標 *
 * @return int ボタン番号 *
 * @sa */
int button_event(int x, int y)
{
    x = x * 0.8;
    y = y * 0.83;
    int ret = -1;
    for (int l = 0; l < 10; l++)
    {
        auto &btn = button[l];
        if (btn.x <= x && (btn.x + btn.x_size) >= x && btn.y <= y && (btn.y + btn.y_size) >= y)
        {
            for (auto &b : button)
            {
                b.click = false;
            }
            btn.click = true;
            ret = l;
            break;
        }
    }
    return ret;
}

/**
 * @brief 判定モード初期化
 * @param () 引数の説明
 * @return
 * @sa 参照すべき関数を書けばリンクが貼れる
 * @detail 詳細な説明
 */
void Judgment_init()
{
    Start_of_judgment = false;
    strike_last = false;
    Judgment_mode = false;
    ball_detection_phase = invalid;
    // basez.zone_flg = false;
    Judgment_start = false;
    basez.batter_hight = 1;
    knee_point = Point2i(0, 0);
    shoulder_point = Point2i(0, 0);
    hip_point = Point2i(0, 0);
    wrist_point = Point2i(0, 0);
    basez.dot_hi_err = c_dot_hi_err;
    straightLineDistanceToBase_i = 1;
    ball_detection_f = false;
    ball_clear_sleep = false;
    LOG_DEBUG.printf("Judgment_init");
    ball_clear();
    // zone_il_top = imread("basezone.png"); // ゾーンイラスト トップビュー
    zone_il_top = imread("homebase_yoko_no_zone.png"); // basezone.png"); // ゾーンイラスト トップビュー
    // sleep_sec(1);
    // snap_shot = true;
}

/**
 * @brief oak-d 初期化 起動
 * @param () 引数の説明
 * @return TRUE
 * @sa 参照すべき関数を書けばリンクが貼れる
 * @detail 詳細な説明
 */
int depthai_setup()
{
    LOG_NONE.printf("depthai_setup");
    return (true);
}

/**
 * @brief Function to solve Perspective-n-Point problem using OpenCV.
 *
 * This function calculates the rotation and translation vectors between a 3D object and a 2D image.
 * It uses the solvePnP function from OpenCV library.
 *
 * @param void
 * @return void
 */
void solve_pnp()
{
    if (!basez.zone_flg)
        return;
    Mat rvec_front;
    Mat tvec_front;
    Mat rotation_front;
    Mat world_position_front_cam;
    Mat cameraMatrix_Front = (Mat_<double>(3, 3) << c_focal_length, 0, 320, 0, c_focal_length, 200, 0, 0, 1);
    std::vector<Point3f> front_object_pts;
    std::vector<Point2f> front_image_pts;

    // Fill front object points(x-y-z order in cms)
    // It is square of side 12.8cms on Z=0 plane
    front_object_pts.push_back(Point3f(0, 0, 0));      // b
    front_object_pts.push_back(Point3f(-200, 0, 400)); // nw
    front_object_pts.push_back(Point3f(-200, 0, 200)); // sw
    front_object_pts.push_back(Point3f(200, 0, 200));  // se
    front_object_pts.push_back(Point3f(200, 0, 400));  // ne

    // Corresponding Image points detected in the same order as object points
    front_image_pts.push_back(basez.baseCoordinateBase);
    front_image_pts.push_back(basez.baseCoordinateFront_l);
    front_image_pts.push_back(basez.baseCoordinateBack_l);
    front_image_pts.push_back(basez.baseCoordinateBack_r);
    front_image_pts.push_back(basez.baseCoordinateFront_r);

    // Detected points in image matching the 3-D points in the same order
    //  Get rvec and tvec using Solve PnP
    solvePnP(front_object_pts, front_image_pts, cameraMatrix_Front, Mat(4, 1, CV_64FC1, Scalar(0)), rvec_front, tvec_front, false, 0);

    //std::cout << "rvec_front=" << rvec_front << std::endl
    //          << std::endl;
    PLOG_VERBOSE.printf(" rvec_1:%.2f", basez.ardn(rvec_front.at<double>(0)));
    PLOG_VERBOSE.printf(" rvec_2:%.2f", basez.ardn(rvec_front.at<double>(1)));
    PLOG_VERBOSE.printf(" rvec_3:%.2f", basez.ardn(rvec_front.at<double>(2)));
    if (!imu_use)
    {
        basez.base_roll = -rvec_front.at<double>(2);
        basez.base_pitch = rvec_front.at<double>(0);
    }
    if (no_base_mode)
    {
        basez.base_yaw = 0;
    }
    else
    {
        basez.base_yaw = rvec_front.at<double>(1);
    }
    if (ground_mode)
        basez.base_roll = 0;
}
// 線形回帰を行う関数
void linearRegression(const std::vector<double> &x, const std::vector<double> &y, double &slope, double &intercept)
{
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    int n = x.size();

    for (int i = 0; i < n; ++i)
    {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i] * y[i];
        sum_xx += x[i] * x[i];
    }

    slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    intercept = (sum_y - slope * sum_x) / n;
}

// 外れ値を検出する関数
void detectOutliers(std::vector<ball_cours> &balls, int camera_no)
{
    std::vector<double> x, y;
    for (const auto &ball : balls)
    {
        if (ball.camera_no == camera_no)
        {
            x.push_back(static_cast<double>(ball.time));
            y.push_back(ball.dep_cm);
        }
    }

    double slope, intercept;
    linearRegression(x, y, slope, intercept);

    // 残差の標準偏差を計算
    double sum_residual_squared = 0;
    for (size_t i = 0; i < x.size(); ++i)
    {
        double predicted_y = slope * x[i] + intercept;
        double residual = y[i] - predicted_y;
        sum_residual_squared += residual * residual;
    }
    double std_dev_residual = std::sqrt(sum_residual_squared / x.size());

    // 外れ値の検出（2シグマ法）
    for (auto &ball : balls)
    {
        if (ball.camera_no == camera_no)
        {
            double predicted_y = slope * static_cast<double>(ball.time) + intercept;
            double residual = ball.dep_cm - predicted_y;
            // LOG_DEBUG.printf("predicted_y:%.2f,residual:%.2f,std_dev_residual:%.2f", predicted_y, residual, std_dev_residual);
            if (std::abs(residual) > 2 * std_dev_residual)
            {
                ball.filter = false;
                LOG_DEBUG.printf("撮影時間ハズレ predicted_y:%.2f,residual:%.2f,std_dev_residual:%.2f", predicted_y, residual, std_dev_residual);
            }
            else
            {
                ball.filter = true;
            }
        }
    }
}

//サイドビューのゾーン表示
void drawsideviewZone()
{
    // 共通の色と線の太さを定義
    cv::Scalar color(0, 200, 200);
    int thickness = 10;
    int lineType = LINE_AA;

    // 矩形を描画
    cv::rectangle(zone_il_top, cv::Point(side_view_add_x_zone(0, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_hi_cm, -22)), cv::Point(side_view_add_x_zone(22, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_low_cm, -22)), color, thickness, lineType);
    cv::rectangle(zone_il_top, cv::Point(side_view_add_x_zone(0, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_hi_cm, 22)), cv::Point(side_view_add_x_zone(22, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_low_cm, 22)), color, thickness, lineType);
    cv::rectangle(zone_il_top_over, cv::Point(side_view_add_x_zone(0, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_hi_cm, -22)), cv::Point(side_view_add_x_zone(22, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_low_cm, -22)), color, thickness, lineType);
    cv::rectangle(zone_il_top_over, cv::Point(side_view_add_x_zone(0, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_hi_cm, 22)), cv::Point(side_view_add_x_zone(22, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_low_cm, 22)), color, thickness, lineType);

    // 線を描画
    cv::line(zone_il_top, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_hi_cm, 0)), cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_low_cm, 0)), color, thickness, lineType);
    cv::line(zone_il_top, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_hi_cm, 0)), cv::Point(side_view_add_x_zone(0, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_hi_cm, -22)), color, thickness, lineType);
    cv::line(zone_il_top, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_hi_cm, 0)), cv::Point(side_view_add_x_zone(0, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_hi_cm, 22)), color, thickness, lineType);
    cv::line(zone_il_top, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_low_cm, 0)), cv::Point(side_view_add_x_zone(0, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_low_cm, -22)), color, thickness, lineType);
    cv::line(zone_il_top, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_low_cm, 0)), cv::Point(side_view_add_x_zone(0, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_low_cm, 22)), color, thickness, lineType);
    cv::line(zone_il_top, cv::Point(side_view_add_x_zone(22, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_low_cm, -22)), cv::Point(side_view_add_x_zone(22, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_low_cm, 22)), color, thickness, lineType);
    cv::line(zone_il_top, cv::Point(side_view_add_x_zone(22, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_hi_cm, -22)), cv::Point(side_view_add_x_zone(22, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_hi_cm, 22)), color, thickness, lineType);
    cv::line(zone_il_top_over, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_hi_cm, 0)), cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_low_cm, 0)), color, thickness, lineType);
    cv::line(zone_il_top_over, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_hi_cm, 0)), cv::Point(side_view_add_x_zone(0, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_hi_cm, -22)), color, thickness, lineType);
    cv::line(zone_il_top_over, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_hi_cm, 0)), cv::Point(side_view_add_x_zone(0, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_hi_cm, 22)), color, thickness, lineType);
    cv::line(zone_il_top_over, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_low_cm, 0)), cv::Point(side_view_add_x_zone(0, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_low_cm, -22)), color, thickness, lineType);
    cv::line(zone_il_top_over, cv::Point(side_view_add_x_zone(-22, 0), c_base_center_y - side_view_add_y(basez.zone_low_cm, 0)), cv::Point(side_view_add_x_zone(0, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_low_cm, 22)), color, thickness, lineType);
    cv::line(zone_il_top_over, cv::Point(side_view_add_x_zone(22, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_low_cm, -22)), cv::Point(side_view_add_x_zone(22, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_low_cm, 22)), color, thickness, lineType);
    cv::line(zone_il_top_over, cv::Point(side_view_add_x_zone(22, -22), c_base_haba_tate_u - side_view_add_y(basez.zone_hi_cm, -22)), cv::Point(side_view_add_x_zone(22, 22), c_base_haba_tate_d - side_view_add_y(basez.zone_hi_cm, 22)), color, thickness, lineType);
}

// ゾーンイラストにボール履歴を表示
// ret :ボール数
void zone_il_print(int function, Point2i ball_point, bool strike)
{
    if (basez.right_batter)
    {
        zone_il_r.copyTo(zone_il);
    }
    else
    {
        zone_il_r.copyTo(zone_il);
    }
    // zone_il = (basez.right_batter) ? zone_il_r.clone() : zone_il_l.clone();
    switch (function)
    {
    case 0: //初期化
        for (int i = 0; i < 20; i++)
        {
            ball_il_position[i].num = 99;
        }
        break;
    case 1: //追加
        for (int i = 0; i < 20; i++)
        {
            if (ball_il_position[i].num == 99)
            {
                ball_il_position[i].num = i + 1;
                ball_il_position[i].ball_position = ball_point;
                ball_il_position[i].strike = strike;
                break;
            }
        }
        break;
    case 2: //表示
        for (int i = 0; i < 20; i++)
        {
            if (ball_il_position[i].num == 99)
            {
                break;
            }
            else
            {
                Scalar bcolor = (ball_il_position[i].strike) ? Scalar(0, 255, 255) : bcolor = Scalar(0, 255, 0);
                circle(zone_il, ball_il_position[i].ball_position, 12, bcolor, 3, LINE_AA, 0);
                putText(zone_il, to_string(ball_il_position[i].num), Point(ball_il_position[i].ball_position.x - 7, ball_il_position[i].ball_position.y + 7), cv::FONT_HERSHEY_SIMPLEX, 0.7, bcolor, 2);
            }
        }
        break;
    default:
        break;
    }
}
void zone_il_print(int function)
{
    zone_il_print(function, Point2i(0, 0), true);
}
/**********************************************************************************/

#define LOW 0
#define HIGH 1
#define AutoJudgh 5
#define JudghTrigger 6
#define KneeDetection 16
#define LED_sw 26
#define MOTORSW 20
#define AutoJudghOn (digitalRead(AutoJudgh) == LOW)
#define AutoJudghOff (digitalRead(AutoJudgh) == HIGH)
#define JudghTriggerOn (digitalRead(JudghTrigger) == LOW)
#define JudghTriggerOff (digitalRead(JudghTrigger) == HIGH)
#define KneeDetectionOn (digitalRead(KneeDetection) == LOW)
#define KneeDetectionOff (digitalRead(KneeDetection) == HIGH)
// digitalRead(5)
#if (raspi5 == true)
/**
 * @brief Sets the pin mode for a specific pin.
 *
 * @param pin_n The pin number to set the mode for.
 * @param pin_m The mode to set the pin to.
 *
 * @return None.
 *
 * @note This function is used to configure the direction of a pin.
 *       It can be used to set a pin as an input or output.
 *
 * @warning The behavior of this function may vary depending on the specific hardware and library being used.
 */
bool pinMode(int pin_n, int pin_m)
{
    int ret;
    switch (pin_n)
    {
    case AutoJudgh:
        if (pin_m == 1)
        {
            ret = gpiod_line_request_input(sw_5_AutoJudgh, "AutoJudgh");
        }
        else
        {
            ret = gpiod_line_request_output(sw_5_AutoJudgh, "AutoJudgh", pin_m);
        }
        break;
    case JudghTrigger:
        if (pin_m == 1)
        {
            ret = gpiod_line_request_input(sw_6_JudghTrigger, "JudghTrigger");
        }
        else
        {
            ret = gpiod_line_request_output(sw_6_JudghTrigger, "JudghTrigger", pin_m);
        }
        break;
    case LED_sw:
        if (pin_m == 1)
        {
            ret = gpiod_line_request_input(sw_26_LED_sw, "LED_sw");
        }
        else
        {
            ret = gpiod_line_request_output(sw_26_LED_sw, "LED_sw", pin_m);
        }
        break;
    case MOTORSW:
        if (pin_m == 1)
        {
            ret = gpiod_line_request_input(sw_20_MOTORSW, "motor");
        }
        else
        {
            ret = gpiod_line_request_output(sw_20_MOTORSW, "motor", pin_m);
        }
        break;
    case KneeDetection:
        if (pin_m == 1)
        {
            ret = gpiod_line_request_input(sw_16_KneeDetection, "KneeDetection");
        }
        else
        {
            ret = gpiod_line_request_output(sw_16_KneeDetection, "KneeDetection", pin_m);
        }
        break;
    }
    return (ret);
}
/**
 * @brief Reads the value of a digital input pin.
 * @param pin_n The pin number to read from.
 * @return The value of the pin (0 for low, 1 for high).
 * @note This function assumes that the necessary GPIO setup has been performed.
 */
int digitalRead(int pin_n)
{
    int ret;
    switch (pin_n)
    {
    case AutoJudgh:
        // sw_5_AutoJudgh = gpiod_chip_get_line(chip, 5);
        ret = (gpiod_line_get_value(sw_5_AutoJudgh));
        break;
    case JudghTrigger:
        // sw_6_JudghTrigger = gpiod_chip_get_line(chip, 6);
        ret = (gpiod_line_get_value(sw_6_JudghTrigger));
        break;
    case LED_sw:
        ret = (gpiod_line_get_value(sw_26_LED_sw));
        break;
    case MOTORSW:
        ret = (gpiod_line_get_value(sw_20_MOTORSW));
        break;
    case KneeDetection:
        ret = (gpiod_line_get_value(sw_16_KneeDetection));
        break;
    }
    return (ret);
}
#endif

// 共通のアイテム取得関数を定義
PyObject *getDictItem(PyObject *dict, const char *key)
{
    PyObject *item = PyDict_GetItemString(dict, key);
    if (item)
    {
        Py_INCREF(item); // アイテムの参照カウントを増やす
    }
    return item;
}

int getDictItemAsLong(PyObject *dict, const char *key)
{
    PyObject *item = getDictItem(dict, key);
    if (item && PyLong_Check(item))
    {
        int value = PyLong_AsLong(item);
        Py_DECREF(item); // アイテムの参照カウントを減らす
        return value;
    }
    return -1; // エラーハンドリング（適宜調整）
}

/**
 * @brief main
 * @param () 引数の説明
 * @return
 * @sa 参照すべき関数を書けばリンクが貼れる
 * @detail 詳細な説明
 */
int main(int argc, char *argv[])
{
    // OpenCVのログレベルを設定
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
    // OpenCVのマルチスレッド設定
    cv::setNumThreads(std::thread::hardware_concurrency());
#if raspi == true
    chdir("/home/pi/depthaiC++/depthai-core-example-main/build");
#endif
    // 時間初期化
    tv_sec_init = get_sec();
    tv_m_sec_init = get_m_sec();

    /**********************************************************************************/

    /**********************************************************************************/
    /*このコードは、strike_config.jsonという名前のファイルから設定を読み込んでいます。
まず、std::ifstreamを使用して、strike_config.jsonファイルを開いています。その後、json::parseを使用して、ファイルの内容を解析し、conf_jsonという変数に格納しています。
その後、conf_jsonからさまざまな設定値を取得しています。例えば、c_kekka_print_time、c_rez_x_r、c_rez_y_rなどの変数に値を代入しています。これらの変数は、conf_json内の対応するキーの値に基づいて初期化されます。
このコードは、設定ファイルから読み込んだ値を適切な変数に割り当てる
ための手順を示しています。これにより、プログラムは実行時に設定を使用することができます。
このコードの改善点としては、設定ファイルの読み込みエラーの処理が欠けていることが挙げられます。ファイルが存在しない場合や形式が正しくない場合に備えて、エラーハンドリングを追加することをお勧めします。
また、設定値を取得するために連続したコード行を使用しているため、コードの可読性を向上させるためにループや関数を使用することも考慮してください。
以上が、この選択されたコードの概要です。設定ファイルから値を読み込むための基本的な手順を示しています。*/
    std::ifstream conf_f("strike_config.json");
    conf_json = json::parse(conf_f);
    // config.json 設定
    c_kekka_print_time = conf_json["c_kekka_print_time"];
    c_rez_x_r = conf_json["c_rez_x_r"];
    c_rez_y_r = conf_json["c_rez_y_r"];
    dep_l_en_bias = conf_json["dep_l_en_bias"];
    dep_l_kin_bias = conf_json["dep_l_kin_bias"];
    float hfov_bias = conf_json["hfov_bias"];
    float vfov_bias = conf_json["vfov_bias"];
    int c_setConfidenceThreshold = conf_json["c_setConfidenceThreshold"];
    int c_setConfidenceThreshold_B = conf_json["c_setConfidenceThreshold_B"];
    int c_setLeftRightCheckThreshold = conf_json["c_setLeftRightCheckThreshold"];
    C_monorgb_hi_x = conf_json["C_monorgb_hi_x"];
    c_bias_x_ork_d = conf_json["c_bias_x_oak-d"];
    c_bias_x_ork_d_lite = conf_json["c_bias_x_oak-d_lite"];
    C_monorgb_hi_y = conf_json["C_monorgb_hi_y"];
    c_bias_y_ork_d = conf_json["c_bias_y_oak-d"];
    c_bias_y_ork_d_lite = conf_json["c_bias_y_oak-d_lite"];
    c_rnd_filter = conf_json["c_rnd_filter"];
    base_katamuki_hosei = conf_json["base_katamuki_hosei"];
    c_horizontalDistanceToBase_cm = conf_json["horizontalDistanceToBase_cm"];
    c_speed_lim_min = conf_json["c_speed_lim_min"];
    int setBilateralFil = conf_json["setBilateralFilterSigma"];
    tsuuka_filter = conf_json["tsuuka_filter"];
    rnd_pitch_keisuu = conf_json["rnd_pitch_keisuu"];
    int fps_mono = conf_json["fps_mono"];
    int Isp3aFps = conf_json["Isp3aFps"];
    int fps_Rgb = conf_json["fps_Rgb"];
    int Isp3aFps_Rgb = conf_json["Isp3aFps_Rgb"];
    exp_time_mono = conf_json["exp_time_mono"];
    sens_iso_mono = conf_json["sens_iso_mono"];
    exp_time_mono_B = conf_json["exp_time_mono_B"];
    sens_iso_mono_B = conf_json["sens_iso_mono_B"];
    focus_mono = conf_json["focus_mono"];

    knee_keisuu = conf_json["knee_keisuu"];
    base_detection_maker = conf_json["base_detection_maker"];
    fov_ork_d_lite = conf_json["fov_oak-d_lite"];
    fov_ork_d = conf_json["fov_oak-d"];
    float hfov_oak_d_lite = conf_json["c_hfov_oak-d_lite"];
    float hfov_oak_d = conf_json["c_hfov_oak-d"];
    float hfov_oak_d_lite_rgb = conf_json["c_hfov_oak-d_lite_rgb"];
    float hfov_oak_d_rgb = conf_json["c_hfov_oak-d_rgb"];
    float vfov_oak_d_lite_rgb = conf_json["c_vfov_oak-d_lite_rgb"];
    float vfov_oak_d_rgb = conf_json["c_vfov_oak-d_rgb"];
    vfov_oak_d_lite_mono = conf_json["c_vfov_oak-d_lite_mono"];
    vfov_oak_d_mono = conf_json["c_vfov_oak-d_mono"];
    camera_center_x_oak_d_lite = conf_json["camera_center_x_oak_d_lite"];
    camera_center_y_oak_d_lite = conf_json["camera_center_y_oak_d_lite"];
    camera_center_x_oak_d = conf_json["camera_center_x_oak_d"];
    camera_center_y_oak_d = conf_json["camera_center_y_oak_d"];

    c_zone_hi_per = conf_json["c_zone_hi_per"];
    c_zone_low_per = conf_json["c_zone_low_per"];
    c_zone_low_cm = (float)c_zone_low_per * 1.85;
    c_zone_hi_cm = (float)c_zone_hi_per * 1.85;

    camera_p_home_dis = conf_json["camera_p_home_dis"];
    camera_pitch_keisuu = conf_json["c_camera_pitch_keisuu"];
    second_camera_x_bias = conf_json["second_camera_x_bias"];
    second_camera_y_bias = conf_json["second_camera_y_bias"];
    second_camera_pitch_bias = conf_json["second_camera_pitch_bias"];
    second_camera_yaw_bias = conf_json["second_camera_yaw_bias"];
    first_camera_pitch_bias = conf_json["first_camera_pitch_bias"];
    first_camera_yaw_bias = conf_json["first_camera_yaw_bias"];
    c_second_camera_high_bias = conf_json["second_camera_high_bias"];
    c_second_camera_length_bias = conf_json["second_camera_length_bias"];

    no_base_mode = conf_json["no_base_mode"];
    ground_mode = conf_json["ground_mode"];
    basez.ground_mode = ground_mode;
    base_depth_bias = conf_json["base_depth_bias"];
    base_depth_bias_s = conf_json["base_depth_bias_s"];
    base_depth_bias_hi = conf_json["base_depth_bias_hi"];
    base_depth_bias_hi_s = conf_json["base_depth_bias_hi_s"];
    base_depth_bias_720p = conf_json["base_depth_bias_720p"];
    base_depth_bias_s_720p = conf_json["base_depth_bias_s_720p"];
    base_depth_bias_hi_720p = conf_json["base_depth_bias_hi_720p"];
    base_depth_bias_hi_s_720p = conf_json["base_depth_bias_hi_s_720p"];
    p720p_on = conf_json["720p_on"];
    p720p_2_on = conf_json["720p_2_on"];
    float c_rect_x_hi_bias_1 = conf_json["c_rect_x_hi_bias_1"];
    float c_rect_x_hi_bias_2 = conf_json["c_rect_x_hi_bias_2"];
    snapshot_cnt = conf_json["snapshot_cnt"];

    /**********************************************************************************/

    /**********************************************************************************/
    // PLOGd
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
    /**********************************************************************************/

    /**********************************************************************************/
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    char buff[128] = "";
    // sprintf(buff, "strike_log_%d_%02d_%02d_%02d_%02d.log", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min);
    sprintf(buff, "/home/pi/kyouyuu/LOG/strike_log_%d_%02d_%02d.log", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday);
    plog::Severity v_plog_level = conf_json["plog_level"];
    plog::init(v_plog_level, buff, 100000000, 5); // Step2: initialize the logger.
    PLOG_NONE << "";                              // long macro
    PLOG_NONE << "";                              // long macro
    PLOG_NONE << "";                              // long macro
    PLOG_NONE << "";                              // long macro
    PLOG_NONE << "";                              // long macro
    PLOG_NONE << "Hello log!";                    // long macro
    LOG_NONE.printf("start main");
    /**********************************************************************************/

    /**********************************************************************************/
    second_camera = false;
    // デバイスリストを取得
    auto availableDevices = dai::Device::getAllAvailableDevices();
#if (double_camera == true)
    if (availableDevices.size() < 2)
    {
        std::cerr << "2台以上のデバイスが接続されていません。" << std::endl;
        printf(" %d台\n", availableDevices.size());
        second_camera = true;
        return -1;
    }
#endif

    // デバイスを選択
    dai::DeviceInfo deviceInfo1 = availableDevices[0];
    // デバイスを初期化
    dai::Device device(deviceInfo1);
    // パイプラインを構築
    dai::Pipeline pipeline;

#if (double_camera == true)
    // デバイスを選択
    dai::DeviceInfo deviceInfo2 = availableDevices[1];
    // デバイスを初期化
    dai::Device device_B(deviceInfo2);
    // パイプラインを構築
    dai::Pipeline pipeline_B;
#endif

    //!  Create pipeline
    // dai::Pipeline pipeline;
    // Define sources and outputs
    monoLeft = pipeline.create<dai::node::MonoCamera>();
    monoRight = pipeline.create<dai::node::MonoCamera>();
    camRgb = pipeline.create<dai::node::ColorCamera>();
    depth = pipeline.create<dai::node::StereoDepth>();
    /* V2->V3
    xout = pipeline.create<dai::node::XLinkOut>();
    // xout_mono = pipeline.create<dai::node::XLinkOut>();
    xoutRgb = pipeline.create<dai::node::XLinkOut>();
    // xoutconfidenceMap = pipeline.create<dai::node::XLinkOut>();
    // xoutrectifiedLeft = pipeline.create<dai::node::XLinkOut>();
    controlIn_mono = pipeline.create<dai::node::XLinkIn>();
    controlIn_Rgb = pipeline.create<dai::node::XLinkIn>();
    controlIn_script = pipeline.create<dai::node::XLinkIn>();
    configIn = pipeline.create<dai::node::XLinkIn>();
    pipeline.setXLinkChunkSize(0); // チャンクサイズ設定
    */
    manip_l = pipeline.create<dai::node::ImageManip>();
    manip_r = pipeline.create<dai::node::ImageManip>();
    script = pipeline.create<dai::node::Script>();
    // auto logger = pipeline.create<dai::node::SystemLogger>();
    // logger->setRate(1.0f);

#if (double_camera == true)
    // camera_B Define sources and outputs
    monoLeft_B = pipeline_B.create<dai::node::MonoCamera>();
    monoRight_B = pipeline_B.create<dai::node::MonoCamera>();
    // camRgb_B = pipeline_B.create<dai::node::ColorCamera>();
    depth_B = pipeline_B.create<dai::node::StereoDepth>();
    /* V2->V3
    xout_B = pipeline_B.create<dai::node::XLinkOut>();
    // xoutRgb_B = pipeline_B.create<dai::node::XLinkOut>();
    controlIn_mono_B = pipeline_B.create<dai::node::XLinkIn>();
    // controlIn_Rgb_B = pipeline_B.create<dai::node::XLinkIn>();
    // controlIn_script_B = pipeline_B.create<dai::node::XLinkIn>();
    configIn_B = pipeline_B.create<dai::node::XLinkIn>();
    pipeline_B.setXLinkChunkSize(0); // チャンクサイズ設定
    */
    manip_l_B = pipeline_B.create<dai::node::ImageManip>();
    manip_r_B = pipeline_B.create<dai::node::ImageManip>();
    manip_hanten = pipeline.create<dai::node::ImageManip>();
    manip_hanten_B = pipeline_B.create<dai::node::ImageManip>();
    // script_B = pipeline_B.create<dai::node::Script>();
#endif

    // Properties
    if (p720p_on)
    {
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    }
    else
    {
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    }
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoLeft->setIsp3aFps(Isp3aFps);
    monoRight->setIsp3aFps(Isp3aFps);
    monoLeft->setFps((int)(fps_mono)*0.90);
    monoRight->setFps((int)(fps_mono)*0.90);

    camRgb->setPreviewSize(640, 400);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setFps(fps_Rgb);
    camRgb->setIsp3aFps(Isp3aFps_Rgb);

    auto depthPresetMode = dai::node::StereoDepth::PresetMode::FAST_ACCURACY; //(V3)
    depth->setDefaultProfilePreset(depthPresetMode);                       // HIGH_ACCURACY/HIGH_DENSITY/DEFAULT/FACE/HIGH_DETAIL/ROBOTICS);
    /* V2->V3
    depth->initialConfig.setConfidenceThreshold(c_setConfidenceThreshold); // 視差計算の信頼度しきい値(小さくすると検知が厳しくなる)
    */
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)

    // (V2)auto medianFilter = dai::MedianFilter::KERNEL_7x7;
    // (V2) depth->initialConfig.setMedianFilter(medianFilter);
    dai::StereoDepthConfig::MedianFilter medianFilter = dai::StereoDepthConfig::MedianFilter::KERNEL_7x7; //(V3)
    depth->initialConfig->setMedianFilter(medianFilter); //(V3)
    depth->setLeftRightCheck(lr_check);              // LR方向とRL方向の両方の視差を計算して結合し、それらを結合します。オクルージョン処理を改善するために、無効な視差値を破棄します
    depth->setExtendedDisparity(extended_disparity); // フル解像度とダウンスケールされた画像を組み合わせて、視差範囲が0-95から0-190に増加しました。短距離のオブジェクトに適しています。現在、サブピクセルの視差と互換性がありません
    depth->setSubpixel(subpixel);                    // サブピクセル補間（5小数ビット）で視差を計算します。長距離射撃に適しています。現在、拡張された視差と互換性がありません
    // サブピクセル精度の小数点以下のビット数を設定
    // 例: 3ビットに設定 (0.125ピクセル単位の精度)
    // depth->setSubpixelFractionalBits(3);  // ★ここが設定する場所です
    // depth->initialConfig.setBilateralFilterSigma(setBilateralFil); // パラメータの値が大きいほど、ピクセル近傍内のより遠い色が混合され、半等しい色の領域が大きくなります。0..65535
    // (V2) depth->initialConfig.setLeftRightCheckThreshold(c_setLeftRightCheckThreshold);
    depth->initialConfig->setLeftRightCheckThreshold(c_setLeftRightCheckThreshold);
    // depth->setOutputDepth(false);
    // depth->setOutputDisparity(true);

#if (double_camera == true)
    // camera_B Properties
    if (p720p_2_on)
    {
        monoLeft_B->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoRight_B->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    }
    else
    {
        monoLeft_B->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
        monoRight_B->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    }
    monoLeft_B->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight_B->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoLeft_B->setIsp3aFps(Isp3aFps);
    monoRight_B->setIsp3aFps(Isp3aFps);
    monoLeft_B->setFps(fps_mono);
    monoRight_B->setFps(fps_mono);

    /*
    camRgb_B->setPreviewSize(640, 400);
    camRgb_B->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb_B->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb_B->setInterleaved(false);
    camRgb_B->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb_B->setFps(fps_Rgb);
    camRgb_B->setIsp3aFps(Isp3aFps_Rgb);
    */

    depth_B->setDefaultProfilePreset(depthPresetMode);                         // HIGH_DENSITY);
    /* V2->V3
    depth_B->initialConfig.setConfidenceThreshold(c_setConfidenceThreshold_B); // 視差計算の信頼度しきい値(小さくすると検知が厳しくなる)
    */
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    // (V2) depth_B->initialConfig.setMedianFilter(medianFilter);
    depth_B->initialConfig->setMedianFilter(medianFilter); //(V3)
    depth_B->setLeftRightCheck(lr_check);              // LR方向とRL方向の両方の視差を計算して結合し、それらを結合します。オクルージョン処理を改善するために、無効な視差値を破棄します
    depth_B->setExtendedDisparity(extended_disparity); // フル解像度とダウンスケールされた画像を組み合わせて、視差範囲が0-95から0-190に増加しました。短距離のオブジェクトに適しています。現在、サブピクセルの視差と互換性がありません
    depth_B->setSubpixel(subpixel);                    // サブピクセル補間（5小数ビット）で視差を計算します。長距離射撃に適しています。現在、拡張された視差と互換性がありません
    // サブピクセル精度の小数点以下のビット数を設定
    // 例: 3ビットに設定 (0.125ピクセル単位の精度)
    // depth_B->setSubpixelFractionalBits(3);  // ★ここが設定する場所です
    // depth_B->initialConfig.setBilateralFilterSigma(setBilateralFil); // パラメータの値が大きいほど、ピクセル近傍内のより遠い色が混合され、半等しい色の領域が大きくなります。0..65535
    // (V2) depth_B->initialConfig.setLeftRightCheckThreshold(c_setLeftRightCheckThreshold);
    depth_B->initialConfig->setLeftRightCheckThreshold(c_setLeftRightCheckThreshold); //(V3)
    // depth_B->setOutputDepth(false);
    // depth_B->setOutputDisparity(true);
#endif

#if (detNN == false)
    //  Crop range
    //
    float x_temp = 0.0f;
    float y_temp = 0.0f;
    if (p720p_on)
    {
        x_temp = c_rect_x_hi_bias_720p; //720pでのcrop範囲
        y_temp = 0.0f;
    }
    else
    {
        x_temp = 0.0f;
        y_temp = 0.0f;
    }
    float ax = adjust_ratio(640, c_rect_x_hi + x_temp);
    int ax_width = 640 - (int)(ax * 2 * 640);
    dai::Point2f topLeft(ax + c_rect_x_hi_bias_1, 0.0f + y_temp);
    dai::Point2f bottomRight(1 - ax + c_rect_x_hi_bias_1, c_rect_y_lo - y_temp);

    // dai::Rect(topLeft, bottomRight) でクロップ領域を定義し、normalizedCoords を true に設定 V2->V3
    manip_l->initialConfig->addCrop(dai::Rect(topLeft, bottomRight), true); // V2->V3
    manip_r->initialConfig->addCrop(dai::Rect(topLeft, bottomRight), true); // V2->V3
    manip_l->setMaxOutputFrameSize(monoRight->getResolutionHeight() * monoRight->getResolutionWidth() * 1);
    manip_r->setMaxOutputFrameSize(monoRight->getResolutionHeight() * monoRight->getResolutionWidth() * 1);
    // Resize image
    //manip_l->initialConfig.setResize(ax_width, 400);
    //manip_r->initialConfig.setResize(ax_width, 400);
    // setOutputSize(uint32_t w, uint32_t h, ResizeMode mode) は、出力画像の幅(w)、高さ(h)、およびリサイズモードを設定します。
    // ◦ dai::ImageManipConfig::ResizeMode::STRETCH
    manip_l->initialConfig->setOutputSize(ax_width, 400, dai::ImageManipConfig::ResizeMode::STRETCH); // アスペクト比の固定(V2->V3)
    manip_r->initialConfig->setOutputSize(ax_width, 400, dai::ImageManipConfig::ResizeMode::STRETCH); // アスペクト比の固定(V2->V3)
    if (ground_mode)
    { //反転
        manip_l->initialConfig->addFlipVertical();       // V2->V3
        manip_l->initialConfig->addFlipHorizontal();     // V2->V3
        manip_r->initialConfig->addFlipVertical();       // V2->V3
        manip_r->initialConfig->addFlipHorizontal();     // V2->V3
    }
    //(V2) manip_hanten->initialConfig->setResize((16 * (int)((640 - (c_rect_x_hi + x_temp) * 2 * 640) / 16)), 400 - y_temp * 400); // 640x400 V2->V3
    //(V2) manip_hanten->initialConfig->setKeepAspectRatio(false);                                                                  // アスペクト比の固定 V2->V3
    manip_hanten->initialConfig->setOutputSize(
        (16 * (int)((640 - (c_rect_x_hi + x_temp) * 2 * 640) / 16)), // 幅の計算
        (400 - y_temp * 400),                                        // 高さの計算
        dai::ImageManipConfig::ResizeMode::STRETCH                   // アスペクト比を維持せず引き伸ばす
    );                                                               // V3向け修正
#if (double_camera == true)
       // camera_B
    dai::Point2f topLeft_B(c_rect_x_hi + c_rect_x_hi_bias_2, 0.0f);
    dai::Point2f bottomRight_B(1 - c_rect_x_hi + c_rect_x_hi_bias_2, c_rect_y_lo);
    // dai::Rect(topLeft, bottomRight) でクロップ領域を定義し、normalizedCoords を true に設定 V2->V3
    manip_l_B->initialConfig->addCrop(dai::Rect(topLeft, bottomRight), true); // V2->V3
    manip_r_B->initialConfig->addCrop(dai::Rect(topLeft, bottomRight), true); // V2->V3
    manip_l_B->setMaxOutputFrameSize(monoRight->getResolutionHeight() * monoRight->getResolutionWidth() * 1);
    manip_r_B->setMaxOutputFrameSize(monoRight->getResolutionHeight() * monoRight->getResolutionWidth() * 1);
    // Resize image
    // manip_l_B->initialConfig.setResize((int)(16 * 16), 400);
    // manip_r_B->initialConfig.setResize((int)(16 * 16), 400);
    manip_l_B->initialConfig->setOutputSize(16*16, 400, dai::ImageManipConfig::ResizeMode::STRETCH); // アスペクト比の固定(V2->V3)
    manip_r_B->initialConfig->setOutputSize(16*16, 400, dai::ImageManipConfig::ResizeMode::STRETCH); // アスペクト比の固定(V2->V3)
    /*
    manip_l_B->initialConfig.setVerticalFlip(true); //反転
    manip_l_B->initialConfig.setHorizontalFlip(true);
    manip_r_B->initialConfig.setVerticalFlip(true); //反転
    manip_r_B->initialConfig.setHorizontalFlip(true);
    */
    manip_hanten_B->initialConfig->addFlipVertical();    // 反転 V2->V3
    manip_hanten_B->initialConfig->addFlipHorizontal();  // V2->V3
    // アスペクト比を固定せずに指定されたサイズに引き伸ばす (STRETCHモード) [2]
    manip_hanten_B->initialConfig->setOutputSize(
        (int)(16 * 16), // 例えば、setResizeのコメントアウトからサイズを推測
        400,            // 例えば、setResizeのコメントアウトからサイズを推測
        dai::ImageManipConfig::ResizeMode::STRETCH
    );
    //
    #endif
#endif

    /*
    auto config = depth->initialConfig.get();
    config.postProcessing.speckleFilter.enable = false; // FPS半減
    config.postProcessing.speckleFilter.speckleRange = 50;
    config.postProcessing.temporalFilter.enable = false; // FPS半減
    config.postProcessing.spatialFilter.enable = false;  // FPS60くらい
    config.postProcessing.spatialFilter.holeFillingRadius = 2;
    config.postProcessing.spatialFilter.numIterations = 1;
    config.postProcessing.thresholdFilter.minRange = 400;
    config.postProcessing.thresholdFilter.maxRange = 15000;
    config.postProcessing.decimationFilter.decimationFactor = 1;
    depth->initialConfig.set(config);
    */

    /* V2->V3
    xout->setStreamName("disparity");
    xoutRgb->setStreamName("rgb");
    xoutRgb->input.setBlocking(false);
    xoutRgb->input.setQueueSize(1);

    controlIn_mono->setStreamName("control_mono");
    controlIn_Rgb->setStreamName("control_Rgb");
    controlIn_script->setStreamName("control_script");
    configIn->setStreamName("config");
    */
    // xout_mono->setStreamName("mono");
    // xoutconfidenceMap->setStreamName("confidenceMap");
    // xoutrectifiedLeft->setStreamName("rectifiedLeft");

#if (double_camera == true)
    /* V2->V3
    xout_B->setStreamName("disparity_B");
    // xoutRgb_B->input.setBlocking(false);
    // xoutRgb_B->input.setQueueSize(1);
    // xoutRgb_B->setStreamName("rgb_B");

    controlIn_mono_B->setStreamName("control_mono_B");
    // controlIn_Rgb_B->setStreamName("control_Rgb_B");
    // controlIn_script_B->setStreamName("control_script_B");
    configIn_B->setStreamName("config_B");
    */
    // camera_B
#endif

#if (detNN == true)
    manip_l->initialConfig.setResize(416, 416);
    manip_r->initialConfig.setResize(416, 416);
#endif
    script->setScript(R"(
    # Initially send still event
    while True:
        ctrl = node.io['control_script'].get().getData()
        frame = node.io['frames'].get()
        node.io['stream1'].send(frame)
    )");

#if (false) // (double_camera == true)
    // camera_B
    script_B->setScript(R"(
    # Initially send still event
    while True:
        ctrl = node.io['control_script_B'].get().getData()
        frame = node.io['frames_B'].get()
        node.io['stream1_B'].send(frame)
    )");
#endif

#if (detNN == true)
    // NN
    std::string nnPath("tiny-yolo-v4_openvino_2021.2_6shave.blob");
    auto detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();
    /* V2->V3
    auto nnOut = pipeline.create<dai::node::XLinkOut>();
    detectionNetwork->setConfidenceThreshold(0.5f);
    */
    nnOut->setStreamName("detections");
    // Network specific settings
    detectionNetwork->setNumClasses(80);
    detectionNetwork->setCoordinateSize(4);
    detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    detectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    detectionNetwork->setIouThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);
    detectionNetwork->setNumInferenceThreads(2);
    detectionNetwork->input.setBlocking(false);
    //
#endif

    // Linking
    monoLeft->out.link(manip_l->inputImage);
    monoRight->out.link(manip_r->inputImage);
    // monoLeft->out.link(xout_mono->input);
    // depth->disparity.link(xout->input);
    camRgb->preview.link(script->inputs["frames"]);
    /* V2->V3
    controlIn_script->out.link(script->inputs["control_script"]);
    controlIn_mono->out.link(monoRight->inputControl);
    controlIn_mono->out.link(monoLeft->inputControl);
    controlIn_Rgb->out.link(camRgb->inputControl);
    configIn->out.link(manip_l->inputConfig);
    configIn->out.link(manip_r->inputConfig);
    script->outputs["stream1"].link(xoutRgb->input);
    */
    // depth->disparity.link(xout->input);
    //  depth->confidenceMap.link(xoutconfidenceMap->input);

    // ↓FPSを上げるにはコメントアウト
    // depth->rectifiedLeft.link(xoutrectifiedLeft->input);

#if (double_camera == true)
    // camera_B Linking
    monoLeft_B->out.link(manip_l_B->inputImage);
    monoRight_B->out.link(manip_r_B->inputImage);
    /* V2->V3
    controlIn_mono_B->out.link(monoRight_B->inputControl);
    controlIn_mono_B->out.link(monoLeft_B->inputControl);
    // controlIn_Rgb_B->out.link(camRgb_B->inputControl);
    configIn_B->out.link(manip_l_B->inputConfig);
    configIn_B->out.link(manip_r_B->inputConfig);
    */
    // camRgb_B->preview.link(script_B->inputs["frames_B"]);
    // controlIn_script_B->out.link(script_B->inputs["control_script_B"]);
    // script_B->outputs["stream1_B"].link(xoutRgb_B->input);
#endif

#if (detNN == false)
    manip_l->out.link(depth->left);
    manip_r->out.link(depth->right);
    depth->disparity.link(manip_hanten->inputImage);
    //depth->depth.link(manip_hanten->inputImage);
    /* V2->V3
    manip_hanten->out.link(xout->input);
    // depth_B->disparity.link(xout_B->input);
    manip_hanten_B->out.link(xout_B->input);
    */
#if (double_camera == true)
    // camera_B
    manip_l_B->out.link(depth_B->left);
    manip_r_B->out.link(depth_B->right);
    depth_B->disparity.link(manip_hanten_B->inputImage);
    // depth_B->depth.link(manip_hanten_B->inputImage);
    #endif
#else
    /* V2->V3
    depth->disparity.link(xout->input);
    depth->rectifiedLeft.link(xoutrectifiedLeft->input);
    */
    detectionNetwork->out.link(nnOut->input);
    manip_l->out.link(detectionNetwork->input);
    camRgb->preview.link(manip_l->inputImage);
    manip_r->out.link(detectionNetwork->input);
    camRgb->preview.link(manip_r->inputImage);
#endif
    // detectionNetwork->out.link(nnOut->input);
    // manip->out.link(detectionNetwork->input);
    // camRgb->preview.link(manip->inputImage);
    /*
    detNN=falseの場合
     +----------+
     |          |out      +----------+           +----------+      +----------+
     | monoLeft +-------->|          |disparity  |          |out   |          | "disparity"
     |          |     left| depth    |---------->|manip     |----->|xout      |----->
     +----------+  +----->|          | inputImage|          | input|          |
                   | right+----------+           +----------+      +----------+
     +----------+  |
     |          +--+                          0.3f,0.9f -> xy:256,360
     | monoRight|
     |          |
     +----------+

     +----------+        +----------+        +----------+
     |          |preview |          |outputs |          | "rgb"
     |camRgb    |------->|script    |------->|xoutRgb   |----->
     |          |   input|          |   input|          |
     +----------+        +----------+        +----------+

    //蜈医↓逕ｻ蜒上し繧､繧ｺ繧貞ｰ上＆縺上＠縺ｦ縺翫￥
     +----------+           +----------+
     |          |out        |          |out
     | monoLeft +---------->|manip     |----+
     |          | inputImage|          |    |      +----------+          +----------+
     +----------+           +----------+    +----->|          |disparity |          |
                                               left|depth     |--------->|xout      |----->
     +----------+           +----------+    +----->|          |     input|          |
     |          |out        |          |out | right+----------+          +----------+
     | monoLeft +---------->|manip     |----+
     |          | inputImage|          |
     +----------+           +----------+

    //蜈医↓逕ｻ蜒上し繧､繧ｺ繧貞ｰ上＆縺上＠縺ｦ縺翫￥(荳贋ｸ句渚霆｢)
     +----------+           +----------+
     |          |out        |          |out
     | monoLeft +---------->|manip     |----+                              縺薙％縺ｧ蜿崎ｻ｢
     |          | inputImage|          |    |      +----------+           +----------+      +----------+
     +----------+           +----------+    +----->|          |disparity  |          |out   |          |
                                               left|depth     |---------->|manip     |----->|xout      |----->
     +----------+           +----------+    +----->|          | inputImage|          | input|          |
     |          |out        |          |out | right+----------+           +----------+      +----------+
     | monoLeft +---------->|manip     |----+
     |          | inputImage|          |
     +----------+           +----------+
     */

    /**********************************************************************************/

    /**********************************************************************************/
    // 接続カメラの判別　＝＞　fovの設定
    fov = 0;
    fov_2 = 0;
    c_bias_x = 0;
    c_bias_y = 0;
    for (auto &sensor : device.getCameraSensorNames())
    {
        std::cout << "1_Socket: " << sensor.first << " - " << sensor.second << ", ";
        string input = sensor.second;
        std::stringstream ss(input);
        std::vector<std::string> tokens;
        std::string token;

        while (ss >> token)
        {
            tokens.push_back(token);
        }

        string b = "OV9282";
        string c = "IMX378";
        for (const auto &t : tokens)
        {
            std::cout << t << std::endl;
            if ((t == b) || (t == c))
            {
                printf("SET oak-d\n");
                fov = fov_ork_d;
                fov_2 = fov_ork_d * 2;
                basez.hfov = hfov_oak_d;
                basez.vfov_mono = vfov_oak_d_mono;
                basez.hfov_rgb = hfov_oak_d_rgb;
                basez.vfov_rgb = vfov_oak_d_rgb;
                basez.camera.horizontal_fov = basez.hfov;
                basez.camera.vertical_fov = basez.vfov_mono;
                c_bias_x = (c_bias_x == 0) ? c_bias_x_ork_d : c_bias_x;
                c_bias_y = (c_bias_y == 0) ? c_bias_y_ork_d : c_bias_y;
                basez.c_bias_x = c_bias_x;
                basez.c_bias_y = c_bias_y;
                basez.camera_center_x = camera_center_x_oak_d;
                basez.camera_center_y = camera_center_y_oak_d;
                imu_use = true;
                basez.imu_use = true;
            }
            else
            {
                printf("SET oak-d-Lite\n");
                fov = (fov == 0) ? fov_ork_d_lite : fov;
                basez.hfov = hfov_oak_d_lite;
                basez.hfov_rgb = hfov_oak_d_lite_rgb;
                basez.vfov_rgb = vfov_oak_d_lite_rgb;
                basez.vfov_mono = vfov_oak_d_lite_mono;
                basez.camera.vertical_fov = basez.vfov_mono;
                basez.camera.horizontal_fov = basez.hfov;
                c_bias_x = (c_bias_x == 0) ? c_bias_x_ork_d_lite : c_bias_x;
                c_bias_y = (c_bias_y == 0) ? c_bias_y_ork_d_lite : c_bias_y;
                basez.c_bias_x = c_bias_x;
                basez.c_bias_y = c_bias_y;
                basez.camera_center_x = camera_center_x_oak_d_lite;
                basez.camera_center_y = camera_center_y_oak_d_lite;
                imu_use = false;
                basez.imu_use = false;
            }
        }
        if (ground_mode)
        {
            basez.camera_center_x = 320 + (320 - basez.camera_center_x);
            basez.camera_center_y = 200 + (200 - basez.camera_center_y);
        }
    }
    for (auto &sensor : device_B.getCameraSensorNames())
    {
        std::cout << "2_Socket: " << sensor.first << " - " << sensor.second << ", ";
        string input = sensor.second;
        std::stringstream ss(input);
        std::vector<std::string> tokens;
        std::string token;

        while (ss >> token)
        {
            tokens.push_back(token);
        }
    }
#if (c_imu == true)
    if (imu_use)
    {
        /* V2->V3
            auto imu = pipeline.create<dai::node::IMU>();
            auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
            xlinkOut->setStreamName("imu");
        */
        // グローバルで宣言したimu変数を初期化
        imu = pipeline.create<dai::node::IMU>(); // NEW (v3): ノードタイプを明示

        // enable ACCELEROMETER_RAW at 500 hz rate
        imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 30);
        // enable GYROSCOPE_RAW at 400 hz rate
        imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 30);
        // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
        // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
        imu->setBatchReportThreshold(1);
        // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
        // if lower or equal to batchReportThreshold then the sending is always blocking on device
        // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
        imu->setMaxBatchReports(10);

        /*
    // enable ROTATION_VECTOR at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 10);
    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(10);
    */

        // Link plugins IMU -> XLINK
        /* V2->V3
        imu->out.link(xlinkOut->input);
        */
    }
#endif
    device.startPipeline(pipeline);
#if (double_camera == true)
    //camera_B
    device_B.startPipeline(pipeline_B);
#endif

    basez.fov = fov;
    basez.fov_2 = fov_2;
    std::cout << std::endl;
    auto eeprom = device.readCalibration2().getEepromData();
    std::cout << "1_Product name: " << eeprom.productName << ", board name: " << eeprom.boardName << std::endl;
    auto eeprom_B = device_B.readCalibration2().getEepromData();
    std::cout << "2_Product name: " << eeprom_B.productName << ", board name: " << eeprom_B.boardName << std::endl;
    bool firstTs = false;
    //事前計算
    basez.precalc_func();
    /**********************************************************************************/

#if (detNN == true)
// (V2)     qDet = device.getOutputQueue("detections", 4, false);
#endif
    // Manual exposure/focus set step
    sens_iso_mono = clamp(sens_iso_mono, sens_min, sens_max);
    sens_iso_mono_B = clamp(sens_iso_mono_B, sens_min, sens_max);
    exp_time_Rgb = clamp(exp_time_Rgb, exp_min, exp_max);
    sens_iso_Rgb = clamp(sens_iso_Rgb, sens_min, sens_max);
    LOG_INFO.printf("Setting manual exposure  MONO , time: %d, iso: %d", exp_time_mono, sens_iso_mono);
    LOG_INFO.printf("Setting manual exposure  MONO_B , time: %d, iso: %d", exp_time_mono_B, sens_iso_mono_B);
    autoex_mono = true;

    LOG_INFO.printf("FPS=%.1f", monoLeft->getFps());
    PLOG_INFO.printf("usbspeed:%d", device.getUsbSpeed());
#if (double_camera == true)
    if ((int)device.getUsbSpeed() == 4 && (int)device_B.getUsbSpeed() == 4)
#else
    if ((int)device.getUsbSpeed() == 4)
#endif
    {
#if (double_camera == true)
        thread th_a(Judgment_main, std::ref(device), std::ref(device_B));
        thread th_d(mono_read_f, std::ref(device));
        thread th_e(mono_read_s, std::ref(device_B));
#else
        thread th_a(Judgment_main, std::ref(device));
        thread th_d(mono_read_f, std::ref(device));
#endif
        thread th_c(img_key, std::ref(device));
#if (double_camera == true)
        th_d.detach();
        th_e.detach();
#endif
        //th_e.detach();
        th_c.detach();
        th_a.join();
    }
    else
    {
        printf("Please USB cable CHECK!!!");
    }
    PLOG_NONE.printf("END");
    PLOG_NONE.printf("");
    PLOG_NONE.printf("");
    PLOG_NONE.printf("");
    if (shutdown_f)
    {
#if (raspi5 == true)
        pigpio_stop(gpio);
#endif
        system("sudo shutdown -h now");
    }
}
/**********************************************************************************/

/**
 * @brief ボール検知
 * @param (dai::Device &device) 引数の説明
 * @return TRUE
 * @sa 参照すべき関数を書けばリンクが貼れる
 * @detail 詳細な説明
 *  判定に必要最小の処理のみにしてFPSを稼ぐ
 */
int Judgment_main(dai::Device &device, dai::Device &device_B)
{
    LOG_NONE.printf("start Judgment_main");
    uint64_t time_cnt = get_sec();
    int i_ball;                                                                                                   // ボールを検知した深度画像上の距離
    int codeBallPosition;                                                                                         // ボール検出結果
    int ball_dep_cnt = 0;                                                                                         // onaji kyori kaunto
    int bat_dot_suu, bat_dot_suu_old;                                                                             //, bat_dot_suu_old2;                                                           // 打者検知用
    int bat_dot_suu_right_top, bat_dot_suu_right_bottom, bat_dot_suu_old_right_top, bat_dot_suu_old_right_bottom; //(審判視点)左打者検知用
    int bat_dot_suu_left_top, bat_dot_suu_left_bottom, bat_dot_suu_old_left_top, bat_dot_suu_old_left_bottom;     //(審判視点)右打者検知用
    // まず、uint64_t型のbat_t_old変数を宣言しています。この変数は、前回打者が検知された時刻を保持するために使用されます。
    uint64_t bat_t_old = UINT64_MAX; // 0xFFFFFFFFFFFFFFFF
    // basez.dep_l_en = c_dep_l_en;
    // basez.dep_l_kin = c_dep_l_kin;
    float rnd_pitch_bias = conf_json["rnd_pitch_bias"];
    // カメラ設定変更
    // qconfidenceMap = device.getOutputQueue("confidenceMap", 1, false);
    // qrectifiedLeft = device.getOutputQueue("rectifiedLeft", 1, false);
    mono_read_lock_f = true;
    mono_read_lock_s = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // controlQueue_mono = device.getInputQueue(controlIn_mono->getStreamName()); //V2
    controlQueue_mono_right = monoRight->inputControl.createInputQueue(); // V3
    controlQueue_mono_left = monoLeft->inputControl.createInputQueue(); // V3
    // ctrl_mono.setAutoExposureLimit(1000);
    // ctrl_mono.setAutoWhiteBalanceLock(true);
    ctrl_mono.setAutoFocusMode(dai::CameraControl::AutoFocusMode::OFF); //Members:    OFF    AUTO    MACRO    CONTINUOUS_VIDEO    CONTINUOUS_PICTURE    EDOF
    // ctrl_mono.setManualExposure(exp_time_mono, sens_iso_mono);
    ctrl_mono.setManualFocus(focus_mono);
    // controlQueue_mono->send(ctrl_mono);
    ctrl_mono.setAutoExposureLock(false);
    ctrl_mono.setAutoExposureEnable();
    /* ISP関連設定（モノクロ用）
    ctrl_mono.setSharpness(0);  // シャープネス処理無効化
    ctrl_mono.setBrightness(0); // 明るさ補正無効化
    ctrl_mono.setSharpness(0);
    ctrl_mono.setBrightness(0);
    */
    mono_read_check_f = false;
    mono_read_check_s = false;
    controlQueue_mono_right->send(std::make_shared<dai::CameraControl>(ctrl_mono)); //V3
    controlQueue_mono_left->send(std::make_shared<dai::CameraControl>(ctrl_mono)); // V3
#if (double_camera == true)
    // controlQueue_mono_B = device_B.getInputQueue(controlIn_mono_B->getStreamName()); // V2
    controlQueue_mono_B_right->send(std::make_shared<dai::CameraControl>(ctrl_mono)); // V3
    controlQueue_mono_B_left->send(std::make_shared<dai::CameraControl>(ctrl_mono));  // V3
    // ctrl_mono_B.setAutoExposureLimit(1000);
    // ctrl_mono_B.setAutoWhiteBalanceLock(true);
    ctrl_mono_B.setAutoFocusMode(dai::CameraControl::AutoFocusMode::OFF); //Members:    OFF    AUTO    MACRO    CONTINUOUS_VIDEO    CONTINUOUS_PICTURE    EDOF
    // ctrl_mono_B.setManualExposure(exp_time_mono_B, sens_iso_mono);
    ctrl_mono_B.setManualFocus(focus_mono);
    // controlQueue_mono_B->send(ctrl_mono);
    ctrl_mono_B.setAutoExposureLock(false);
    ctrl_mono_B.setAutoExposureEnable();
    /* ISP関連設定（モノクロ用）
    ctrl_mono_B.setSharpness(0);  // シャープネス処理無効化
    ctrl_mono_B.setBrightness(0); // 明るさ補正無効化
    ctrl_mono_B.setSharpness(0);
    ctrl_mono_B.setBrightness(0);
    */
    controlQueue_mono_B_right->send(std::make_shared<dai::CameraControl>(ctrl_mono)); // V3
    controlQueue_mono_B_left->send(std::make_shared<dai::CameraControl>(ctrl_mono));  // V3
#endif
    mono_read_lock_f = false;
    mono_read_check_f = true;
    mono_read_lock_s = false;
    mono_read_check_s = true;
    //! ボールコースの初期化
    ball_clear();
    basez.ball_noise_clear();
    zone_il_print(0);
    auto buffer = dai::Buffer(); // dummy kakidashi you
    float center_cm_y_old = 0;
    float x, y, z;
    double t, prevTime = 0;
    const float minSpeed = 5.0;             // 最小速度（km/h換算で約20km/h）
    const float maxSpeed = 46.0;            // 最大速度（km/h換算で約165km/h）
    const float minAllowedDeviation = 5.0;  // 最小許容誤差（cm）
    const float maxAllowedDeviation = 50.0; // 最大許容誤差（cm）
    bool mono_frame_flip_first = true;      // true:first,false:second
    while (isActJudgment_main)
    {
        while (end_flg_Judgment_main_get)
        {
            if (!Judgment_mode)
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 100msスリープ
            std::this_thread::sleep_for(std::chrono::microseconds(1));     // 200msスリープ
            /*このコードは、C++で書かれた画像処理やフレームレートの管理に関連する一部です。特に、デプス画像の取得とフレームレートの計算、そして条件に基づいた処理の実行を行っています。
            最初に、get_sec()関数を呼び出して現在の秒数を取得し、cnt_tに代入します。この関数は、現在の時刻を秒単位で返します。その後、state変数をfalseに設定します。*/
            if (mono_frame_taken_img_key_f //
                // &&mono_frame_flip_first //
            )
            {
                mono_frame_taken_img_key_f = false;
                rnd_filter_syori_flg = true;
                mono_read_lock_f = true; // mono画像取得中フラグを立てる
                if (p720p_on)
                    cv::divide(frame_mono_f_tmp, 2.0, frame_mono_org); // すべての画素値を2で割る
                else
                    frame_mono_f_tmp.copyTo(frame_mono_org);
                frame_mono_org.copyTo(frame_mono);
                mono_read_lock_f = false; // mono画像取得中フラグを下げる
                cnt_t = get_sec();
                fps_cnt++;
                ts_msec = std::chrono::duration_cast<std::chrono::milliseconds>(ts_org_f.time_since_epoch()).count();
                mono_frame_flip_first = false;

                basez.camera_high = basez.c_camera_high_no_base;
                //fps_cnt++;
                second_camera_length_bias_org = 0;
                second_camera_sht = false;
                first_mono_act = true;
                second_camera_y_bias_org = 0;
                basez.isSecondCamera = false;
                basez.camera.pitch = camera_pitch_org + first_camera_pitch_bias;
                basez.camera.yaw = basez.camera_yaw_org + first_camera_yaw_bias;
            }
            else if (                      //
                mono_frame_taken_img_key_s //
                // && !mono_frame_flip_first //
            )
            {
                mono_frame_taken_img_key_s = false;
                rnd_filter_syori_flg = true;
                mono_read_lock_s = true; // mono画像取得中フラグを立てる
                if (p720p_2_on)
                    cv::divide(frame_mono_s_tmp, 2.0, frame_mono_org); // すべての画素値を2で割る
                else
                    frame_mono_s_tmp.copyTo(frame_mono_org);
                frame_mono_org.copyTo(frame_mono);
                mono_read_lock_s = false; // mono画像取得中フラグを下げる
                cnt_t = get_sec();
                fps_cnt++;
                second_camera_length_bias = second_camera_length_bias_org;
                ts_msec = std::chrono::duration_cast<std::chrono::milliseconds>(ts_org_s.time_since_epoch()).count();
                ts_msec += initialTs_ms_f - initialTs_ms_s;
                mono_frame_flip_first = true;

                basez.camera_high = basez.c_camera_high_no_base + c_second_camera_high_bias;
                //fps_cnt++;
                second_camera_length_bias_org = c_second_camera_length_bias;
                second_camera_sht = true;
                second_mono_act = true;
                second_camera_y_bias_org = 0;
                basez.isSecondCamera = true;
                basez.camera.pitch = camera_pitch_org + second_camera_pitch_bias;
                basez.camera.yaw = basez.camera_yaw_org + second_camera_yaw_bias;
            }
            else
            {
                continue;
            }
            state = false;

            // LOG_INFO.printf("tmtm  %.1f",get_sec());
            //test_measurement_modeがtrueの場合、つまりテスト測定モードが有効である場合にのみ、内部の処理が実行されます。このモードでは、本番のボール検知中は処理を実行しないようにしています。time_cntとcnt_tが異なり、かつボール検知フラグ(ball_detection_f)がfalseの場合、フレームレート(fps_cnt_fix)を計算し、特定の条件下でFPSが40未満であれば警告ログを出力します。また、自動露出が有効(autoex_mono == true)であれば、それを無効にします。
            if (true) //(test_measurement_mode)
            {         //本番ボール検知中は処理を実行しない
                if (((int)time_cnt != (int)cnt_t) && (!ball_detection_f))
                {
                    fps_cnt_fix = (float)fps_cnt / (int)(cnt_t - time_cnt);
                    if ((int)(time_cnt / 3) != (int)(cnt_t / 3))
                    {
                        if (test_measurement_mode)
                            printf("fps:%d\n", fps_cnt_fix);
                        // LOG_WARNING_IF((fps_cnt_fix < 40) && (Judgment_mode)).printf("fps low:%d", fps_cnt_fix);
                        if (Judgment_mode && fps_cnt_fix < 100)
                        {
                            LOG_DEBUG.printf("FPS error! %d", fps_cnt_fix);
                            // exit_strike(true);
                        }
                        /*
                        if (autoex_mono)
                        {
                            mono_read_lock = true;
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                            ctrl_mono.setAutoExposureLock(true);
                            controlQueue_mono->send(ctrl_mono);
                            autoex_mono = false;
                            mono_read_lock = false;
                        }*/
                    }
                    time_cnt = cnt_t;
                    fps_cnt = 0;
                    state = true;
                }
            }

#if (detNN == true)
            // 処理領域を設定
            dai::Point2f topLeft(640 * c_rect_x_hi, 0.0f);
            dai::Point2f rectSize(640 - 640 * c_rect_x_hi * 2, 400 * c_rect_y_lo);
            cv::Rect roi(topLeft.x, topLeft.y, rectSize.x, rectSize.y);
            // 領域を切り取りコピー
            clip_img = cv::Mat(frame_mono_tmp, roi).clone();
            clip_img.copyTo(frame_mono);
#else
            // clip_img = frame_mono.clone();

            // 深度画像からカメラピッチ角を算出するよう
            // frame_mono_tmp = frame_mono.clone();
#endif

            if (Judgment_mode || test_measurement_mode)
            {
                /*
                if (!second_camera_sht)
                    printf(" first_camera ");
                if (second_camera_sht)
                    printf(" second_camera ");
                printf("ts_msec_old:%llu,ts_msec:%llu sa:%d\n", ts_msec_old, ts_msec, ts_msec - ts_msec_old);
                ts_msec_old = ts_msec;
                */
                // ボール検出プレ処理
                if (basez.zone_flg) // ゾーン検出時のみ実施
                {
                    /*このコードは、C++で書かれた深度画像から特定の条件に基づいて地面を排除し、特定のオブジェクト（ボール）を検出するための前処理を行っています。主に、深度画像(frame_mono)が空でない場合に、画像内の各ピクセルに対して特定の処理を適用しています。
                    まず、test_measurement_modeがtrueの場合、つまりテスト測定モードが有効である場合、temp_rowsはbasez.jyuushin.y - 20に設定されます。これは、ベース付近のバイアスを調整しないための処理であり、ベースの距離測定を正確に行うためです。テスト測定モードが無効の場合、temp_rowsはframe_monoの行数（高さ）に設定されます。
                    その後、temp_rowsの数だけループを回し、各行に対して処理を行います。各行の先頭画素へのポインタを取得し(src変数)、その行の各ピクセルに対してループを回します。このループ内で、各ピクセルの深度値(aa)を取得し、その深度値から高さ(hi)を計算します。計算には、カメラの高さ(basez.camera_high)、カメラのピッチ角度(basez.camera.pitch)、およびピクセルの深度値から距離を計算する関数(basez.c_dis_cm)が使用されます。
                    特定の条件（ピクセルの深度値が設定された範囲外であるか、計算された高さがフィルター値よりも大きい場合）に該当するピクセルは、その深度値を0に設定します。これにより、地表から一定高さまでのオブジェクトを検知しないようにし、
                    特定のオブジェクトのみを検出することが可能になります。*/
                    int i = 0;
                    int kyori_filter = -35; //身体を検知しないように身体の厚み分は残しておいたほうがよい
                    int filter_en, filter_kin;
                    // 距離に応じた深度バイアス 地面を排除
                    if (!frame_mono.empty() && !ground_mode && rnd_filter_f) // && !clip_img_o2.empty()) // セグメント例外の防止
                    {
                        float cos_or_sin_iy;
                        float hi_limit_d;
                        int aa;
                        int ab;
                        int temp_rows; //フィルタリングするy軸の下限
                        // ボール高さ計測のときはベース付近のバイアス調整しない(ベース距離測定のため)　ただし　ベース検知なしモード以外
                        if (test_measurement_mode && !no_base_mode)
                        {
                            temp_rows = basez.jyuushin.y - 20;
                            // temp_rows = c_rect_zone_y;
                        }
                        else
                        {
                            temp_rows = frame_mono.rows;
                        }

                        // 地表から一定高さまでは検知しないようにゼロ設定する
                        switch (ball_detection_phase)
                        {
                        case far_away:
                            if (!test_measurement_mode)
                            {
                                filter_en = dep_l_en;
                                filter_kin = basez.c_dis_dep_i(dep_base_en + 40);
                            }
                            else
                            {
                                filter_en = dep_l_en - 1;
                                filter_kin = dep_l_kin;
                            }
                            break;
                        case beyond_the_base:
                            // filter_en = basez.c_dis_dep_i(basez.c_dis_cm(dep_base_en) + 120);
                            if (second_camera_sht)
                            {
                                filter_en = basez.c_dis_dep_i(last_dep_cm_old_s - kyori_filter);
                                LOG_DEBUG.printf("last_dep_cm_old_s:%.1f", last_dep_cm_old_s);
                                // filter_en = basez.c_dis_dep_i(450);
                                if (basez.center_temp_old_s.x == 0)
                                    filter_en = dep_l_en;
                            }
                            else
                            {
                                filter_en = basez.c_dis_dep_i(last_dep_cm_old_f - kyori_filter);
                                LOG_DEBUG.printf("last_dep_cm_old_f:%.1f", last_dep_cm_old_f);
                                // filter_en = basez.c_dis_dep_i(450);
                                if (basez.center_temp_old_f.x == 0)
                                    filter_en = dep_l_en;
                            }
                            filter_kin = dep_base_kin - 1;
                            break;
                        case on_the_base:
                            if (second_camera_sht)
                            {
                                filter_en = basez.c_dis_dep_i(last_dep_cm_old_s - kyori_filter);
                                LOG_DEBUG.printf("last_dep_cm_old_s:%.1f", last_dep_cm_old_s);
                                // filter_en = basez.c_dis_dep_i(260);
                                if (basez.center_temp_old_s.x == 0)
                                    filter_en = dep_l_en;
                            }
                            else
                            {
                                filter_en = basez.c_dis_dep_i(last_dep_cm_old_f - kyori_filter);
                                LOG_DEBUG.printf("last_dep_cm_old_f:%.1f", last_dep_cm_old_f);
                                // filter_en = basez.c_dis_dep_i(260);
                                if (basez.center_temp_old_f.x == 0)
                                    filter_en = dep_l_en;
                            }
                            filter_kin = dep_l_kin;
                            break;
                        case passing_the_base:
                            if (second_camera_sht)
                            {
                                filter_en = basez.c_dis_dep_i(last_dep_cm_old_s - kyori_filter);
                                LOG_DEBUG.printf("last_dep_cm_old_s:%.1f", last_dep_cm_old_s);
                                // filter_en = basez.c_dis_dep_i(220);
                                if (basez.center_temp_old_s.x == 0)
                                    filter_en = dep_l_en;
                            }
                            else
                            {
                                filter_en = basez.c_dis_dep_i(last_dep_cm_old_f - kyori_filter);
                                LOG_DEBUG.printf("last_dep_cm_old_f:%.1f", last_dep_cm_old_f);
                                // filter_en = basez.c_dis_dep_i(220);
                                if (basez.center_temp_old_f.x == 0)
                                    filter_en = dep_l_en;
                            }
                            filter_kin = dep_l_kin;
                            break;
                        default:
                            filter_en = dep_l_en;
                            filter_kin = dep_l_kin;
                            break;
                        }
                        //身体の誤検知対応
                        //filter_en = clamp(filter_en, basez.c_dis_dep_i(260), basez.c_dis_dep_i(500)); // 260cm以上500cm以下

                        int left_limit = basez.jyuushin.x - c_rect_x - 80;
                        int right_limit = basez.jyuushin.x - c_rect_x + 80;
                        // #pragma omp parallel for
                        const int filter_en_i_def = basez.c_dis_dep_i(basez.c_dis_cm(dep_base_en) + 40);
                        const int filter_kin_i_def = basez.c_dis_dep_i(basez.c_dis_cm(dep_base_kin) - 20);

                        for (int iy = 0; iy < temp_rows; iy++)
                        {
                            // iy行目の先頭画素のポインタを取得
                            ushort *src = frame_mono.ptr<ushort>(iy);
                            ushort *src_org = frame_mono_org.ptr<ushort>(iy);
                            //hi_limit_d = basez.camera_high - (c_rnd_filter + ((float)((400 - iy) * (400 - iy)) * rnd_pitch_keisuu)); // カメラからボールまでの垂直距離 地面から16cm上まで検知する
                            if (ground_mode)
                            {
                                cos_or_sin_iy = abs(sin(basez.camera.pitch - 1.54 - basez.y_r_mono(iy))); //
                            }
                            else
                            {
                                cos_or_sin_iy = abs(cos(basez.camera.pitch - basez.y_r_mono(iy) + rnd_pitch_bias)); //
                            }
                            //距離フィルター敷居値
                            // float y_r = basez.camera.pitch - (iy - basez.camera_center_y) * (basez.rdn(basez.vfov_mono) / 400);
                            float y_r = basez.camera.pitch;
                            int filter_kin_i = basez.c_dis_dep_i(basez.c_dis_cm(filter_kin) / sin(basez.camera.pitch - basez.y_r_mono(iy)));
                            int filter_en_i = basez.c_dis_dep_i(basez.c_dis_cm(filter_en) / sin(basez.camera.pitch - basez.y_r_mono(iy)));
#if (false)
                            if (iy % 20 == 0)
                            {
                                LOG_DEBUG.printf(" iy:%d cos_or_sin_iy:%.2f", iy, basez.ardn(cos_or_sin_iy));
                                LOG_DEBUG.printf(" kakudo:%.2f", basez.ardn(y_r));
                                LOG_DEBUG.printf(" kin kyori:%.1f,en kyori:%.1f", basez.c_dis_cm(dep_l_kin) / sin(y_r), basez.c_dis_cm(dep_l_en) / sin(y_r));
                                LOG_DEBUG.printf(" kin:%d,en:%d", filter_kin_i, filter_en_i);
                            }
#endif
                            // #pragma omp parallel for
                            for (int ix = 0; ix < frame_mono.cols; ix++)
                            {
                                /*
                                if (!second_camera_sht)
                                {
                                    src[ix] = (p720p_on) ? (int)(base_depth_bias_hi_720p * (src[ix] / 2)) : (int)(base_depth_bias_hi * src[ix]); //720pの時は半分にする
                                }
                                else
                                {
                                    src[ix] = (p720p_2_on) ? (int)(base_depth_bias_hi_s_720p * (src[ix] / 2)) : (int)(base_depth_bias_hi_s * src[ix]); //距離の調整係数
                                }
                                */

                                //if (ix == 199 && iy==159)
                                //printf(" %d,%.0f\n",src[ix],basez.c_dis_cm(src[ix]));
                                //ダブルカメラののりしろ部分は削除
#if (double_camera == true)
                                if (                                                                                      //
                                    iy < abs(second_camera_y_bias) || iy >= (frame_mono.rows - abs(second_camera_y_bias)) //
                                )
                                {
                                    src[ix] = 0;
                                    src_org[ix] = 0;
                                    continue;
                                }
                                if (                                                                                      //
                                    ix < abs(second_camera_x_bias) || ix >= (frame_mono.cols - abs(second_camera_x_bias)) //
                                )
                                {
                                    src[ix] = 0;
                                    src_org[ix] = 0;
                                    continue;
                                }
                                if (src[ix] > 0)
                                { //距離バイアス補正
                                    // src[ix] = (second_camera_sht) ? basez.c_dis_dep_i(basez.c_dis_cm(src[ix] + base_depth_bias_s + second_camera_length_bias)) : basez.c_dis_dep_i(basez.c_dis_cm(src[ix] + base_depth_bias + second_camera_length_bias)); //原因不明だが、距離バイアス設定するとうまく行く
                                    if (!second_camera_sht)
                                    {
                                        src[ix] = (p720p_on) ? basez.c_dis_dep_i(basez.c_dis_cm(src[ix]) + base_depth_bias_720p) : basez.c_dis_dep_i(basez.c_dis_cm(src[ix]) + base_depth_bias);
                                        src_org[ix] = (p720p_on) ? basez.c_dis_dep_i(basez.c_dis_cm(src[ix]) + base_depth_bias_720p) : basez.c_dis_dep_i(basez.c_dis_cm(src[ix]) + base_depth_bias);
                                    }
                                    else
                                    {
                                        src[ix] = (p720p_2_on) ? basez.c_dis_dep_i(basez.c_dis_cm(src[ix]) + base_depth_bias_s_720p) : basez.c_dis_dep_i(basez.c_dis_cm(src[ix]) + base_depth_bias_s);
                                        src_org[ix] = (p720p_2_on) ? basez.c_dis_dep_i(basez.c_dis_cm(src[ix]) + base_depth_bias_s_720p) : basez.c_dis_dep_i(basez.c_dis_cm(src[ix]) + base_depth_bias_s);
                                    }
                                }
#else
                                //if (src[ix] > 0)
                                //    src[ix] = basez.c_dis_dep_i(basez.c_dis_cm(src[ix] + base_depth_bias)); //原因不明だが、距離バイアス設定するとうまく行く
#endif
                                aa = src[ix];
                                //距離フィルター
#if (true)
                                //打者側は距離フィルタ緩和
                                bool tmp_flg_a = (ix < 0.25 * frame_mono.cols && right_bat_f && basez.c_dis_cm(src[ix]) > basez.c_dis_cm(dep_base_en));
                                bool tmp_flg_b = (ix > 0.75 * frame_mono.cols && left_bat_f && basez.c_dis_cm(src[ix]) > basez.c_dis_cm(dep_base_en));
                                if (tmp_flg_a || tmp_flg_b)
                                {
                                    filter_en_i = filter_en_i_def;
                                    filter_kin_i = filter_kin_i_def;
                                }
                                if (!isWithinRange(aa, filter_en_i, filter_kin_i))
                                {
                                    if (!tmp_flg_a && !tmp_flg_b)
                                    {
                                        src[ix] = 0;
                                        continue;
                                    }
                                }
#endif
#if (true)
                                /* 両サイドで遠いピクセルは検知しない
                                //if ((ix < left_limit) || (ix > right_limit))
                                {
                                    //if (aa <= basez.precalc[ix].side_filter) // basez.c_dis_dep_i(basez.c_dis_cm(dep_l_kin) + 2.51 * (((basez.jyuushin.x - c_rect_x) - ix) * ((basez.jyuushin.x - c_rect_x) - ix))))
                                    //上部サイドのカット
                                    if ((ix <= frame_mono.cols / 2 - iy) || (ix >= frame_mono.cols / 2 + iy))
                                    {
                                        //if (iy==300 && ix%10==0)
                                        //printf(" aa:%d>%d ix %d zero\n",aa,basez.precalc[ix].side_filter,ix);
                                        if (src[ix] < dep_base_en - 10)
                                            src[ix] = 0;
                                        continue;
                                    }
                                }*/
#endif
                                //
                                //高さ算出
                                float hi, ball_depth;
                                if (ground_mode)
                                {
                                    hi = basez.camera_high + cos_or_sin_iy * basez.c_dis_cm(aa, iy);
                                }
                                else
                                {
                                    hi = basez.camera_high - cos_or_sin_iy * basez.c_dis_cm(aa, iy); // / basez.precalc[ix + c_rect_x].cos_x_r;
                                    ball_depth = basez.c_dis_cm(aa) * basez.precalc[iy].sin_y_r;
                                }
                                //
                                // if (ball_depth > basez.c_dis_cm(dep_base_en))
                                // d = c_rnd_filter - ball_depth - basez.c_dis_cm(dep_base_en);
                                if (!isWithinRange(hi, c_rnd_filter, 200))
                                {
                                    src[ix] = 0;
                                    src_org[ix] = 0;
                                }
                                //

#if (false)
                                //
                                int xxx = 192 / 2;
                                if (iy > 200 && iy % 10 == 0 && ix == xxx)
                                {
                                    //if (second_camera_sht)
                                    //    printf("s:");
                                    printf(" y %d  hi:%.0f, hi_limit:%.1f\n", iy, hi, cos_or_sin_iy);
                                }
                                //
#endif
                            }
                        }
                    }
                    //

                    //２値化(frame_mono -> binary_mask_distance)
                    //このC++コードは、ボールの検出と追跡に関連する処理を行っています。主に、画像からボールを検出し、その動きを追跡し、特定の条件下で様々なアクションを実行するためのロジックが含まれています。
                    //まず、inRange関数を使用して、frame_mono画像を二値化し、binary_mask_distanceというマスクを生成します。これは、特定の距離範囲内のオブジェクトを検出するためのフィルタリング手法です。
                    //次に、Ball_findメソッドを呼び出して、二値化された画像内でボールを検出します。このメソッドは、ボールが見つかったかどうか、およびその他の情報を返します。
                    //検出されたボールに対して、いくつかの条件をチェックし、それに基づいて異なるアクションを実行します。例えば、ボールが特定の距離を移動したかどうか、前回の検出からの時間が一定以内かどうかなどの条件です。
                    //ボールが特定の条件を満たした場合、その位置や検出時間などの情報を記録し、ボールの軌道を追跡します。また、ボールが打者に近い場所にあるかどうかをチェックし、その場合は特定の処理をスキップするなどのロジックが含まれています。
                    //このコードは、ボールの動きをリアルタイムで追跡し、その情報を基にさまざまな判断を行うためのものです。画像処理と条件分岐を駆使して、ボールの位置や動きを正確に把握し、必要な情報を収集・分析することが目的です。
                    inRange(frame_mono, 1, 255, binary_mask_distance); // 距離フィルター
                    // 打者検出
                    inRange(frame_mono_org, dep_base_en - 10, dep_base_kin + 20, frame_mono_batter); // 距離フィルター

                    rnd_filter_syori_flg = false;

                    // ↓範囲を絞り込んで高速化できないか。。。。。。(最初は上部だけとか、ボールに移動位置推測で限定など)
                    codeBallPosition = basez.findBall(&binary_mask_distance, &strike, &frame_mono, &frame_mono_batter, ball_detection_phase, filter_en, filter_kin, &i_ball);

                    // ボール検知エラー連続時はリセット
                    if (codeBallPosition == ball_none) // && 300 < basez.ball_position_old.d) //!= base_shortly)
                    {
                        if (ball_detection_phase == beyond_the_base || ball_detection_phase == on_the_base)
                        {
                            ball_err_renzoku_cnt++;
                            // matfilesave("/home/pi/kyouyuu/LOG/ball_rireki9", &binary_mask_distance); // 途中失敗したときのボール映像
                            // LOG_DEBUG_IF(ball_detection_phase != far_away).printf("ボール検知なし ball_err_renzoku_cnt++:%d,camera:#%d", ball_err_renzoku_cnt, second_camera_sht + 1);
                            LOG_DEBUG_IF((ball_detection_phase != far_away || ball_detection_phase != invalid)).printf("camera:#%d ボール検知なし", second_camera_sht + 1);
                            LOG_DEBUG_IF((ball_detection_phase != far_away || ball_detection_phase != invalid)).printf("\n");
                        }
                    }
                    if (codeBallPosition == shape_none)
                    {
                        codeBallPosition = ball_none; // ボール検知なし
                        // shapeなし
                        LOG_DEBUG_IF(ball_detection_phase != far_away && ball_detection_phase != invalid && !Start_of_judgment).printf("図形検知なし shape_none");
                    }

                    //判定開始から少しの時間はノイズテーブルを作らせるため、ボール記録は少し待つ
                    if ((ball_kiroku_machi + 500) > get_m_sec() //
                        // || (second_camera_sht && ball_detection_phase == far_away)
                    )
                    {
                        LOG_DEBUG_IF(get_m_sec() % 100 == 0).printf("ball_kiroku_machi:%d", ball_kiroku_machi - get_m_sec());
                        codeBallPosition = ball_none;
                    }

                    bool kalman_filter_check = false;
                    if ((codeBallPosition != ball_none)) // && !test_measurement_mode) // (ボールが検知された　＆　テスト測定モードでない)→ボール記録処理
                    {
                        LOG_DEBUG_IF((codeBallPosition != ball_none) && !test_measurement_mode).printf("***");
                        LOG_DEBUG_IF((codeBallPosition != ball_none) && !test_measurement_mode).printf("cnt:%d/%d Ball_find ret %d", ball_detection_cnt, ball_find_cnt, codeBallPosition);

#if (true) //拡張カルマンフィルタ

                        float w = basez.ball_position.w; //原点を０としたX軸方向の位置（CM）
                        float h = basez.ball_position.h; //地平面を０としたy軸方向の高さ（CM)
                        float d = basez.ball_position.d; //原点を０としたZ軸方向の距離（CM)
                        double t = (double)ts_msec;      //計測時間(ms)
                        const float g = 980.0;           // 重力加速度 (cm/s^2)

                        do
                        {
                            if (isFirstObservation)
                            {
    #if (false)
                                // 初回観測: 状態ベクトルを初期化
                                kf.statePost = (Mat_<float>(9, 1) << basez.ball_position.w, basez.ball_position.h, basez.ball_position.d, 0, 0, 0, 0, -g, 0);
                                prevTime = (double)ts_msec;
                                isFirstObservation = false;
                                LOG_DEBUG.printf("calman_filter Initialized with first observation. w=%.1f, h=%.1f, d=%.1f", basez.ball_position.w, basez.ball_position.h, basez.ball_position.d);
    #endif
                                break;
                            }
                            // 時間差 (秒)
                            double dt = (t - prevTime) / 1000.0;
                            if (!isWithinRange(t - prevTime, -800, 800))
                            {
                                LOG_DEBUG.printf("calman_filter Invalid time difference: %.0f ms", t - prevTime);
                                break;
                            }
                            if (!isWithinRange(d, basez.ball_position_old.d - 100, basez.ball_position_old.d + 100))
                            {
                                LOG_DEBUG.printf("calman_filter Invalid distance difference: %.0f cm", d - basez.ball_position_old.d);
                                break;
                            }
                            // 状態遷移行列を更新
                            // 必要な要素を手動で設定
                            kf.transitionMatrix.at<float>(0, 3) = dt;
                            kf.transitionMatrix.at<float>(0, 6) = 0.5 * dt * dt;
                            kf.transitionMatrix.at<float>(1, 4) = dt;
                            kf.transitionMatrix.at<float>(1, 7) = 0.5 * dt * dt;
                            kf.transitionMatrix.at<float>(2, 5) = dt;
                            kf.transitionMatrix.at<float>(2, 8) = 0.5 * dt * dt;
                            kf.transitionMatrix.at<float>(3, 6) = dt;
                            kf.transitionMatrix.at<float>(4, 7) = dt;
                            kf.transitionMatrix.at<float>(5, 8) = dt;
                            kf.transitionMatrix.at<float>(6, 8) = -g * dt; // 重力の影響

                            // 予測ステップ
                            Mat prediction = kf.predict(); //予測状態を計算します
                            float pred_w = prediction.at<float>(0);
                            float pred_h = prediction.at<float>(1);
                            float pred_d = prediction.at<float>(2);
                            float pred_vw = prediction.at<float>(3);
                            float pred_vh = prediction.at<float>(4);
                            float pred_vd = prediction.at<float>(5);

                            // 結果を表示
                            LOG_DEBUG.printf("calman_filter Current position: w=%.1f, h=%.1f, d=%.1f, dt(ms) =%.0f", w, h, d, dt * 1000);
                            LOG_DEBUG.printf("calman_filter prediction position: w=%.1f, h=%.1f, d=%.1f, dt(ms) =%.0f", pred_w, pred_h, pred_d, dt * 1000);
                            float kalman_leng = sqrt(pow((w - pred_w), 2) + pow((w - pred_w), 2));
                            LOG_DEBUG.printf("calman_filter prediction leng: %.1f", kalman_leng);

                            // 観測データを取得し更新ステップ
                            Mat measurement = (Mat_<float>(3, 1) << w, h, d);
                            Mat estimated = kf.correct(measurement); //計測値から予測される状態を更新します measurement	計測されたシステムパラメータ

                            LOG_DEBUG.printf("calman_filter correct: vw=%.1f, vh=%.1f, vd=%.1f", estimated.at<float>(3), estimated.at<float>(4), estimated.at<float>(5));
                            if (ball_cours_cnt == 0)
                                kf.statePost = (Mat_<float>(9, 1) << w, h, d, estimated.at<float>(3), estimated.at<float>(4), estimated.at<float>(5), 0, -g, 0);

                            /* 次の予測
                            kf.transitionMatrix.at<float>(0, 3) = 0.01; // 次の予測を(100)ms後に設定
                            kf.transitionMatrix.at<float>(1, 4) = 0.01;
                            kf.transitionMatrix.at<float>(2, 5) = 0.01;

                            Mat nextPrediction = kf.predict();
                            float next_w = nextPrediction.at<float>(0);
                            float next_h = nextPrediction.at<float>(1);
                            float next_d = nextPrediction.at<float>(2);
                            float next_vx = prediction.at<float>(3);
                            float next_vy = prediction.at<float>(4);
                            float next_vz = prediction.at<float>(5);

                            // 予測を表示
                            LOG_DEBUG.printf("Predicted 10ms later: w=%.1f, h=%.1f, d=%.1f", next_w, next_h, next_d);
                            LOG_DEBUG.printf("Predicted 10ms later: vw=%.1f, vh=%.1f, vd=%.1f", next_vx, next_vy, next_vz);
                            */

                            // 時間更新
                            prevTime = t;
                            break;
                        } while (true);
#else
                        kalman_filter_check = false;
#endif
                    }
                    //このコードは、ボールの検知と追跡、およびその情報の記録に関連する処理を行っています。特に、野球やクリケットのようなスポーツにおいて、ボールの位置や速度を計測するシステムで使用される可能性があります。
                    // まず、if文でボールが検知され、かつテスト測定モードでない場合に、ボールの記録処理を開始します。この条件は、実際のゲーム中にのみデータを記録するために重要です。
                    // 次に、複数のブール変数を用いて、ボールの検知が初めてでないこと、ボールが一定の距離を移動したこと、ボールが過度に移動していないこと、そして前回の検知から適切な時間内であることを確認します。これらの条件は、ボールの追跡が正確に行われていることを保証するために重要です。
                    //ボールが適切な条件下で検知された場合、検知回数をインクリメントし、様々なボールの情報（位置、速度など）を更新します。特に、ボールが特定の領域を通過したかどうかを判断し、その情報を記録します。また、ボールの速度を計測するために、時間と位置の情報を更新します。
                    //さらに、打者に近い場所でボールが検知された場合、その検知を無視する処理があります。これは、打者の動きによって誤検知が発生することを防ぐためです。
                    //最後に、ボールのコース情報を記録し、判定処理を開始する条件をチェックします。これにより、ボールが特定の条件を満たした場合にのみ、詳細な分析や判定が行われるようになっています。
                    if ((codeBallPosition != ball_none) && !test_measurement_mode) // (ボールが検知された　＆　テスト測定モードでない)→ボール記録処理
                    {
                        //移動体抽出
                        bool ball_shinkou = false;
                        if (kalman_filter_check)
                        {
                            ball_shinkou = true;
                        }
                        else
                        {
                            for (int i = basez.c_dis_dep_i(basez.ball_position.d) + 3; i > clamp((basez.c_dis_dep_i(basez.ball_position.d) - 50), 0, 254); i--)
                            {
                                int ts_sa = (int)((!second_camera_sht) ? (ts_msec - ts_msec_fast) : (ts_msec - ts_msec_fast_2) - ball[i].time);
                                float w_sa = ball[i].w_cm - basez.ball_position.w;
                                float h_sa = ball[i].h_cm - basez.ball_position.h;
                                float d_sa = ball[i].dep_cm - basez.ball_position.d;
                                float check = sqrt(w_sa * w_sa + h_sa * h_sa);
                                if (ball[i].h_cm != 0)
                                {
                                    LOG_DEBUG.printf(" check i:%d,kyori:%.3f,ts_sa:%d", i, check, ts_sa);
                                    // if (isWithinRange(abs(check), 0.5, 15.0) && isWithinRange(ts_sa, 0, 1500) && isWithinRange(d_sa, -30, 80))
                                    if (isWithinRange(abs(check), 0.5, 50.0) && isWithinRange(d_sa, -30, 80))
                                    {
                                        /*if (d_sa < 0 && second_camera_sht == second_camera_sht_old)
                                        {
                                            LOG_DEBUG.printf(" check_NG kyori gyakuten %.1f,%.1f", ball[i].dep_cm, basez.ball_position.d);
                                            ball_clear();ball_position.d
                                            break;
                                        }*/
                                        ball_shinkou = true;
                                        cntShinkou++;
                                        ball_detection_t_old = get_m_sec(); // 次のボール検知まで100msの猶予
                                        LOG_DEBUG.printf(" check_OK:%d,%.3f", i, check);
                                        break;
                                    }
                                    //if (ts_sa > 500)
                                    //    break;
                                }
                                // else{
                                //LOG_DEBUG_IF().printf("ball[%d].h_cm=0!!!",i);
                                //}
                            }

                            /*if (ball_detection_phase == far_away)
                            {
                                ball_shinkou = true;
                                LOG_DEBUG.printf(" check_OK");
                                ball_detection_t_old = get_m_sec(); //時間変数が整数型だったことを失念していたバグ対応
                            }
                            else*/
                            if (!ball_shinkou)
                            {
                                ball_detection_t_old = get_m_sec(); // 次のボール検知まで700msの猶予
                                LOG_DEBUG.printf(" check_NG w:%.1f,h:%.1f,d:%.1f,", basez.ball_position.w, basez.ball_position.h, basez.ball_position.d);
                                if (ball_detection_phase == far_away) // 遠方でエラーの場合はさっさとクリア
                                {
                                    // ball_clear();
                                }
                            }
                        }
                        ball_find_cnt++;
                        ball_detection_cnt++;
                        //
                        switch (ball_detection_phase)
                        {
                        case far_away:
                            LOG_DEBUG.printf(" ball_detection_phase:%d", ball_detection_phase);
                            // 最初のif文は、ボールが特定の基準点(dep_base_kin) よりも手前にあり、かつまだ検知されていない（ball_detection_cnt == 0）場合に、様々な変数を更新します。具体的には、ボールの検知回数(ball_detection_cnt)を1に設定し、その前の検知回数(ball_detection_cnt_old) を0にリセットします。また、ボールの最初の長さ(ball_first_len_cm) を計算し、検知フェーズ(ball_detection_phase) を1に設定します。その他、ボールの中心位置や球速測定のための変数も初期化されます。このブロックの最後に、LOG_NONE_IFマクロを使用して、テスト測定モードでない場合にボールの初回検知をログに記録します。
                            if ( //
                                // basez.ball_position.d > (basez.c_dis_cm(dep_base_en) + 0)                         //
                                isWithinRange(basez.ball_position.d, basez.c_dis_cm(dep_base_en) + 50, c_dep_cm_en - 1) //
                                && ((second_camera_sht && basez.ball_position.d < last_dep_cm_old_s) || (!second_camera_sht && basez.ball_position.d < last_dep_cm_old_f)) && ((second_camera_sht && basez.ball_position.h < last_ball_h_old_s) || (!second_camera_sht && basez.ball_position.h < last_ball_h_old_f))
                                // || ball_shinkou                                                            //
                            )
                            {
                                // if (ball_shinkou && cntShinkou > 0)
                                if (ball_cours_cnt == 0)
                                {
                                    LOG_NONE_IF(!test_measurement_mode).printf(" ボール初回検知:%d", (int)ball_first_len_cm);
                                    // 球速測定で初期化（最初の検知は捨てる）
                                }
                                // if (ball_cours_cnt > 1 && ball_shinkou)
                                {
                                    ball_detection_phase = beyond_the_base;
                                }
                                ball_detection_t_old = get_m_sec(); //時間変数が整数型だったことを失念していたバグ対応
                                ball_err_renzoku_cnt = 0;
#if (true)
                                // 初回観測: 状態ベクトルを初期化
                                const float g = 980.0; // 重力加速度 (cm/s^2)
                                kf.statePost = (Mat_<float>(9, 1) << basez.ball_position.w, basez.ball_position.h, basez.ball_position.d, 0, 0, 0, 0, -g, 0);
                                prevTime = (double)ts_msec;
                                isFirstObservation = false;
                                LOG_DEBUG.printf("calman_filter Initialized with first observation. w=%.1f, h=%.1f, d=%.1f", basez.ball_position.w, basez.ball_position.h, basez.ball_position.d);
#endif
                                first_dep_cm = basez.ball_position.d;
                                first_time = get_m_sec();
                                ts_msec_fast = ts_msec;
                                ts_msec_fast_2 = ts_msec;
                                dep_threshold = straightLineDistanceToBase_i; //検知距離の更新
                            }
                            else
                            {
                                ball_detection_cnt = ball_detection_cnt_old; //ボール記録しない
                                LOG_DEBUG.printf(" ボール記録なし! ball_shinkou:%d,cntShinkou:%d,last_dep_cm_old:%.1f,basez.ball_position.d:%.1f", ball_shinkou, cntShinkou, (second_camera_sht) ? last_dep_cm_old_s : last_dep_cm_old_f, basez.ball_position.d);
                            }
                            break;
                        case beyond_the_base:
                            /* code */
                            // 外側のif文は、ボールがベースの投手側より内側に入ったかどうか（i_ball >= dep_base_en）、ボールの検知回数が3回以上であるか（ball_detection_cnt >= 3）、そしてボールの検知フェーズが3未満であるか（ball_detection_phase < 3）を評価します。これらの条件がすべてtrueである場合、内側のif文が評価されます。
                            if (ball_shinkou                                                                                                                                //
                                && (((second_camera_sht && basez.ball_position.d < last_dep_cm_old_s) || (!second_camera_sht && basez.ball_position.d < last_dep_cm_old_f)) //
                                        && ((second_camera_sht && basez.ball_position.h < last_ball_h_old_s) || (!second_camera_sht && basez.ball_position.h < last_ball_h_old_f)) ||
                                    basez.center.y > basez.center_temp_old.y))
                            {
                                LOG_DEBUG.printf(" ball_err_renzoku_cnt:%d", ball_err_renzoku_cnt);
                                ball_err_renzoku_cnt = 0;
                                if (
                                    isWithinRange(basez.ball_position.d, basez.c_dis_cm(dep_base_kin) - 15, basez.c_dis_cm(dep_base_en)) && ((second_camera_sht && basez.ball_position.d < last_dep_cm_old_s) || (!second_camera_sht && basez.ball_position.d < last_dep_cm_old_f))
                                    //basez.ball_position.d <= basez.c_dis_cm(dep_base_en) + 15 //　ベース投手側端よりうちに入ったか
                                    //&& basez.ball_position.d >= basez.c_dis_cm(dep_base_kin)  //　ベース捕手側端より遠くか
                                    // || (ball_detection_cnt >= 5) // ベース通過しないときの救済
                                    // && last_dep_cm_old > basez.ball_position.d - 15 // ボールが近づいているか
                                )
                                {
                                    second_dep_cm = basez.ball_position.d;
                                    second_time = get_m_sec();
                                    snap_speed = (first_dep_cm - second_dep_cm) / (second_time - first_time) * 36;
                                    //if(isWithinRange(snap_speed, 18, 165)) //
                                    if (true) // (codeBallPosition == base_shortly)
                                    {
                                        ball_detection_phase = on_the_base;
                                        LOG_DEBUG.printf(" ball_detection_phase:1->2");
                                    }
                                }
                                /*
                                if (                                                                         //
                                    basez.ball_position.d <= horizontalDistanceToBase_cm - 10                //　ベース中心よりうちに入ったか
                                    && ball_detection_cnt > 5 && basez.ball_position.d < (first_dep_cm - 80) //
                                    )                                                                        //
                                {
                                    ball_zone_s = true;
                                    LOG_NONE.printf(" ベース通過 The ball went over the base:%.1fcm", basez.ball_position.d);
                                    LOG_DEBUG.printf(" ball_detection_phase:2->3");
                                    ball_detection_phase = passing_the_base;
                                    // ベース通過したらスナップショット
                                }
                                */
                            }
                            else
                            {
                                if (ball_shinkou && cntShinkou > 1 //
                                    // && (last_dep_cm_old - 15) < basez.ball_position.d //
                                )
                                {
                                    ball_detection_cnt = ball_detection_cnt_old; //ボール記録しない
                                    LOG_DEBUG.printf(" ボール距離の後戻り! ball_shinkou:%d,cntShinkou:%d,last_dep_cm_old:%.1f,basez.ball_position.d:%.1f", ball_shinkou, cntShinkou, (second_camera_sht) ? last_dep_cm_old_s : last_dep_cm_old_f, basez.ball_position.d);
                                    LOG_DEBUG.printf("ボール記録なし!");
                                }
                            }
                            break;
                        case on_the_base:
                            // 最初に検知さらたときから規定以上の距離だったか
                            // 内側のif文では、ボールが最初に検知された時から規定以上の距離を移動したかどうかを判断します。これは、ボールの最初の長さ（ball_first_len_cm）から特定のフィルタ値（tsuuka_filter）を引いた値が、現在のボールの距離（basez.c_dis_cm(i_ball)）よりも大きいかどうかをチェックすることで行われます。
                            // この条件がtrueである場合、ボールがベースを通過したと判断され、ball_zone_s変数がtrueに設定されます。また、この事象をログに記録するためにLOG_NONE.printf関数が呼び出され、"ベース通過 The ball went over the base:%d"というメッセージと共に、ボールの位置（i_ball）が出力されます。最後に、ボールの検知フェーズを3に設定することで、ボールがベースを通過したことを示すフェーズに移行します。
                            if (ball_shinkou //
                                && ((second_camera_sht && basez.ball_position.d < last_dep_cm_old_s) || (!second_camera_sht && basez.ball_position.d < last_dep_cm_old_f)) && ((second_camera_sht && basez.ball_position.h < last_ball_h_old_s) || (!second_camera_sht && basez.ball_position.h < last_ball_h_old_f))
                                // && last_dep_cm_old > basez.ball_position.d - 15 // ボールが近づいているか
                            )
                            {
                                LOG_DEBUG.printf(" ball_err_renzoku_cnt:%d", ball_err_renzoku_cnt);
                                ball_err_renzoku_cnt = 0;
                                trd_dep_cm = basez.ball_position.d;
                                trd_time = get_m_sec();
                                float snap_speed_2 = (second_dep_cm - trd_dep_cm) / (trd_time - second_time) * 36;
                                LOG_DEBUG.printf(" snap_speed %.1f,%.1f", snap_speed, snap_speed_2);
                                if (  //
                                    ( //
                                        isWithinRange(basez.ball_position.d, horizontalDistanceToBase_cm - 100, horizontalDistanceToBase_cm - 5)
                                        // basez.ball_position.d <= horizontalDistanceToBase_cm - 5 //　ベース中心よりうちに入ったか
                                        && basez.ball_position.d < (first_dep_cm - 50) //
                                        // && last_dep_cm_old > basez.ball_position.d - 15 // ボールが近づいているか
                                        ) //
                                          /*
                                    || (                                                                         //
                                           ball_detection_cnt > 5 && basez.ball_position.d < (first_dep_cm - 80) //
                                           )                                                                     //
*/
                                          // && isWithinRange(snap_speed_2, 18, 165) //
                                          // && isWithinRange(snap_speed_2, snap_speed * 0.7, snap_speed * 1.3)                        //
                                )
                                {
                                    ball_zone_s = true;
                                    LOG_NONE.printf(" ベース通過 The ball went over the base:%.1fcm", basez.ball_position.d);
                                    LOG_DEBUG.printf(" ball_detection_phase:2->3");
                                    ball_detection_phase = passing_the_base;
                                    // ベース通過したらスナップショット
                                }
                            }
                            else
                            {
                                ball_detection_cnt = ball_detection_cnt_old; //ボール記録しない
                                LOG_DEBUG.printf(" norecord! ball_shinkou:%d,cntShinkou:%d,last_dep_cm_old:%.1f,basez.ball_position.d:%.1f", ball_shinkou, cntShinkou, (second_camera_sht) ? last_dep_cm_old_s : last_dep_cm_old_f, basez.ball_position.d);
                            }
                            break;

                        case passing_the_base:
                            LOG_DEBUG.printf(" ball_err_renzoku_cnt:%d", ball_err_renzoku_cnt);
                            ball_err_renzoku_cnt = 0;
                            if (!(ball_shinkou                                                                                                                               //
                                  && ((second_camera_sht && basez.ball_position.d < last_dep_cm_old_s) || (!second_camera_sht && basez.ball_position.d < last_dep_cm_old_f)) //
                                  )
                                // && last_dep_cm_old > basez.ball_position.d - 15 // ボールが近づいているか
                            )
                            {
                                ball_detection_cnt = ball_detection_cnt_old; //ボール記録しない
                                LOG_DEBUG.printf(" norecord! ball_shinkou:%d,cntShinkou:%d,last_dep_cm_old:%.1f,basez.ball_position.d:%.1f", ball_shinkou, cntShinkou, (second_camera_sht) ? last_dep_cm_old_s : last_dep_cm_old_f, basez.ball_position.d);
                            }
                            break;

                        default:
                            LOG_DEBUG.printf("ball_detection_phase:&d", ball_detection_phase);
                            break;
                        }

                        // 打者に近い場所かのチェック
                        // このコードのセクションは、打者の姿勢を分析し、特定の点が打者に近いかどうかを判断するためのものです。C++言語で書かれており、OpenCVライブラリを使用しています。このコードは、打者の体の特定の部位間の線分に対して、ある点が十分に近いかどうかをチェックすることで、ボールが打者に近い位置にあるかどうかを判断します。
                        // bat_pose_flgがtrueである場合、つまり打者の姿勢が検出された場合にのみ、この処理が実行されます。
                        if (bat_pose_flg)
                        {
                            // 次に、打者の体の特定の部位を表す点のペアを含むstd::vectorが定義されます。これらの点のペアは、打者の手首から肘、肩から肘、肩から腰、そして膝から腰までの線分を表します。
                            std::vector<std::pair<cv::Point, cv::Point>> points = {
                                {cv::Point(wrist_x, wrist_y), cv::Point(elbow_x, elbow_y)},
                                {cv::Point(shoulder_x, shoulder_y), cv::Point(elbow_x, elbow_y)},
                                {cv::Point(shoulder_x, shoulder_y), cv::Point(hip_x, hip_y)},
                                {cv::Point(knee_x, knee_y), cv::Point(hip_x, hip_y)}};

                            // forループを使用して、これらの点のペア（線分）を一つずつ取り出し、isPointNearLine関数を呼び出して、特定の点（basez.hoseimae_center）が各線分に対して指定された距離（この場合は10）以内にあるかどうかをチェックします。isPointNearLine関数は、点が線分に対して指定された距離以内にあるかどうかを判断するためのものです。
                            for (const auto &point_pair : points)
                            {
                                if (isPointNearLine(basez.hoseimae_center, point_pair.first, point_pair.second, 10))
                                {
                                    // もしisPointNearLine関数がtrueを返した場合、つまり点が打者のいずれかの線分に十分に近い場合、ボールが打者に近いと判断され、ball_detection_cntをball_detection_cnt_oldに設定して、ボールの検知カウントを更新せずに記録をスキップします。そして、LOG_DEBUG.printfを使用してデバッグログに「打者近辺エラー」というメッセージを出力し、ループから抜け出します。
                                    ball_detection_cnt = ball_detection_cnt_old; //ボール記録しない
                                    LOG_DEBUG.printf(" 打者近辺エラー");
                                    break;
                                }
                            }
                        }

                        // ボールコースの記録
                        // このコードブロックは、ボールの検出とその軌道の記録に関連する処理を行っています。具体的には、ボールが検出されたかどうかを判断し、検出された場合にはその情報を記録します。
                        // まず、if文でball_detection_cnt（現在のボール検出カウント）とball_detection_cnt_old（前回のボール検出カウント）を比較しています。これらが異なる場合、新たにボールが検出されたと判断し、処理を続行します。
                        if (ball_detection_cnt != ball_detection_cnt_old)
                        {
                            int ball_dep = basez.c_dis_dep_i(basez.ball_position.d);
                            int old_dep = ball[ball_dep].dep;
                            LOG_DEBUG_IF(!test_measurement_mode && old_dep == ball_dep).printf(" ボールコース記録オーバーライト");
                            // ボールが検出された場合、まずball_detection_cnt_oldを更新し、次にball配列のball_cours_cnt番目の要素にボールの情報を格納します。この情報には、ボールの深度(dep)、ストライクかどうか(strike)、ボールの画面上の位置(x, y)、補正後の位置(hosei_x, hosei_y)、ボールの半径(radius)、検出カウント(cnt)、検出時刻(time)、ボールまでの距離(dep_cm)、ボールの横位置(w_cm)、ボールの高さ(h_cm)が含まれます。
                            ball_detection_cnt_old = ball_detection_cnt;
                            ball[ball_dep].strike = strike; // exitしないと
                            ball[ball_dep].x = basez.hoseimae_center.x;
                            ball[ball_dep].y = basez.hoseimae_center.y;
                            ball[ball_dep].hosei_x = basez.hosei_center.x;
                            ball[ball_dep].hosei_y = basez.hosei_center.y;
                            ball[ball_dep].radius = basez.radius;
                            ball[ball_dep].cnt = ball_dep_cnt;
                            ball[ball_dep].time = (!second_camera_sht) ? (ts_msec - ts_msec_fast) : (ts_msec - ts_msec_fast_2);
                            ball[ball_dep].dep = ball_dep;
                            ball[ball_dep].dep_cm = basez.ball_position.d; //basez.c_dis_cm(i_ball);
                            ball[ball_dep].w_cm = basez.ball_position.w;   //basez.ballCenter_cm_x - basez.zone_c_dis_w_cm;
                            ball[ball_dep].h_cm = basez.ball_position.h;   //(basez.ballCenter_cm_y >= 0) ? basez.ballCenter_cm_y : 0;
                            if (second_camera_sht)
                            {
                                ball[ball_dep].camera_no = 2;
                            }
                            else
                            {
                                ball[ball_dep].camera_no = 1;
                            }
                            ball[ball_dep].filter = true;

                            // ball_cours_xyz配列には、ボールの横位置(w_cm)をx座標として、y座標とz座標を0として記録します。これは、ボールの軌道を3次元空間で追跡するための準備ですが、現在のコードではy座標とz座標は使用されていません。
                            ball_cours_xyz.at<float>(ball_cours_cnt, 0) = ball[ball_dep].w_cm;
                            ball_cours_xyz.at<float>(ball_cours_cnt, 1) = 0; // int((ball[ball_dep].h_cm * 0.01);
                            ball_cours_xyz.at<float>(ball_cours_cnt, 2) = 0; // ball_cours_cnt*0;

                            // LOG_DEBUG_IFマクロを使用して、テスト測定モードでない場合にボールの情報をログに記録します。これには、ボールの画面上の位置、ボールまでの距離、ボールの横位置、ボールの高さ、検出カウント、検出フェーズ、ボールのインデックスが含まれます。
                            //LOG_DEBUG_IF(!test_measurement_mode).printf("");
                            // if (ball_cours_cnt > 3)
                            //     led(50);
                            LOG_DEBUG_IF(!test_measurement_mode).printf(" ボールコース記録");
                            LOG_DEBUG_IF(!test_measurement_mode).printf("   ボール画面位置(x,y):(%d,%d)", (int)basez.center.x, (int)basez.center.y);
                            LOG_DEBUG_IF(!test_measurement_mode).printf("   ボールまでの距離 dep:%d,dep(cm):%.1fcm", ball_dep, basez.ball_position.d);
                            LOG_DEBUG_IF(!test_measurement_mode).printf("   ボール横位置(width(cm),(ベース相対):%.1fcm,(%.1fcm)", basez.ball_position.w, ball[ball_dep].w_cm);
                            LOG_DEBUG_IF(!test_measurement_mode).printf("   ボール高さ(high(cm)):%.1fcm", basez.ball_position.h);
                            LOG_DEBUG_IF(!test_measurement_mode).printf("    ball_detection_cnt:%d,ball_detection_phase:%d", ball_detection_cnt, ball_detection_phase);
                            LOG_DEBUG_IF(!test_measurement_mode).printf("    ball_cours_cnt:    %d", ball_cours_cnt);
                            LOG_DEBUG_IF(!test_measurement_mode && !second_camera_sht).printf("  by first camera:%d", ts_msec - ts_msec_fast);
                            LOG_DEBUG_IF(!test_measurement_mode && second_camera_sht).printf("  by second camera:%d", ts_msec - ts_msec_fast_2);
                            LOG_DEBUG_IF(!test_measurement_mode).printf("");
                            last_dep_cm_old = basez.ball_position.d;
                            if (second_camera_sht)
                            {
                                if (ts_msec > ts_msec_2_old)
                                {
                                    last_dep_cm_old_s = basez.ball_position.d;
                                    last_ball_h_old_s = basez.ball_position.h;
                                }
                                else
                                {
                                    last_dep_cm_old_s = basez.ball_position.d + 50;
                                    last_ball_h_old_s = basez.ball_position.h + 20;
                                }
                                ts_msec_2_old = ts_msec;
                            }
                            else
                            {
                                if (ts_msec > ts_msec_1_old)
                                {
                                    last_dep_cm_old_f = basez.ball_position.d;
                                    last_ball_h_old_f = basez.ball_position.h;
                                }
                                else
                                {
                                    last_dep_cm_old_f = basez.ball_position.d + 50;
                                    last_ball_h_old_f = basez.ball_position.h + 20;
                                }
                                ts_msec_1_old = ts_msec;
                            }
                            LOG_DEBUG_IF(!test_measurement_mode).printf(" last_dep_cm_old_s:%.0f,last_dep_cm_old_f:%.0f", last_dep_cm_old_s, last_dep_cm_old_f);

                            //ボールイメージ出力
                            // paste(ball_rireki,binary_mask_distance(Rect((int)basez.center.x-basez.radius_temp-c_rect_x,(int)basez.center.y-basez.radius_h,basez.radius_temp*2,basez.radius_h*2)),(int)basez.center.x-basez.radius_temp,(int)basez.center.y-basez.radius_h,basez.radius_temp*2,basez.radius_h*2);
                            // 切り出し領域の計算(xの半径は＋１すると縦線になってしまうので０にしている)
                            int x = static_cast<int>(basez.center.x - (basez.radius_temp + 0) - c_rect_x);
                            int y = static_cast<int>(basez.center.y - (basez.radius_h + 1));
                            int width = (basez.radius_temp + 0) * 2;
                            int height = (basez.radius_h + 1) * 2;
                            // ball_rirekiの範囲を取得
                            int max_width = ball_rireki.cols;
                            int max_height = ball_rireki.rows;
                            // 切り出し領域がball_rirekiの範囲内に収まるように調整
                            x = std::max(0, std::min(x, max_width - 1));
                            y = std::max(0, std::min(y, max_height - 1));
                            width = std::min(width, max_width - x);
                            height = std::min(height, max_height - y);
                            // 切り出し領域が有効な場合のみ処理を実行
                            if (width > 0 && height > 0)
                            {
                                /*
                                cv::Rect roi(x, y, width, height);
                                cv::Mat src = binary_mask_distance(roi);
                                // pasteを呼び出す
                                paste(ball_rireki, src, x + c_rect_x, y);
                                */
                                circle(ball_rireki, Point(basez.center.x, basez.center.y), width / 4, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
                            }
                            else
                            {
                                circle(ball_rireki, Point(basez.center.x, basez.center.y), 3, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
                            }

                            // 最後に、ball_cours_cntをインクリメントして次のボールの記録に備え、ball_detection_fフラグをtrueに設定し、ball_detection_t_oldとball_detection_t_oldに現在時刻を記録します。これにより、ボールが検出されたこととその時刻が記録されます。
                            if (ball_cours_cnt < 255)
                            {
                                ball_cours_cnt++;
                            }
                            else
                            {
                                ball_cours_cnt = 0;
                            }
                            ball_detection_f = true;
                            basez.ball_cours_cnt = ball_cours_cnt;             // ボールコースのカウントを更新
                            basez.updateOldballPosition(ball_detection_phase); // ボールの位置を更新
                            if (old_dep == ball_dep)
                            {
                                // if (ball_detection_phase == beyond_the_base)
                                {
                                    ball_err_renzoku_cnt++;
                                    // LOG_DEBUG.printf("ball_err_renzoku_cnt++:%d\n", ball_err_renzoku_cnt);
                                }
                            }
                            else
                            {
                                ball_err_renzoku_cnt = 0;
                            }
                        }
                        else
                        {
                            if (ball_detection_phase != far_away)
                            {
                                ball_err_renzoku_cnt++;
                                LOG_DEBUG.printf(" ball_err_renzoku_cnt++:%d", ball_err_renzoku_cnt);
                            }
                            LOG_DEBUG.printf("ボール記録なし!\n");
                        }
                    }
                    if (                                                                       //
                        (ball_err_renzoku_cnt > 10 && ball_detection_phase == beyond_the_base) //
                        || (ball_err_renzoku_cnt > 15 && ball_detection_phase == on_the_base)  //
                        // || ball_err_renzoku_cnt > 10                                          //
                        // q&& !Start_of_judgment && ball_detection_phase != far_away
                    )
                    {
                        LOG_DEBUG.printf("★★★★★★★★★★　ball_err_renzoku_cnt_5over:%d　★★★★★★★★★★", ball_err_renzoku_cnt);
                        ball_clear();
                    }

                    // このコードブロックは、打者が動いたかどうかを判定し、必要に応じてゾーンをリセットする処理を行っています。この処理は、打者がすでに検知されている場合に限り実行されます。また、毎フレームごとにこの処理を行う必要はありません。
                    // if文の条件では、bat_flg（打者が検知されているかどうかのフラグ）またはJudgment_mode（判定モードが有効かどうかのフラグ）がtrueであり、かつ現在時刻（get_sec()関数で取得）がbat_t_oldと異なる場合に、ブロック内の処理を実行します。これは、打者が検知されており、かつ新たな時刻になった場合にのみ処理を行うことを意味します。
                    if (          //
                        (bat_flg) // || Judgment_mode) // 打者の影が検知されている
                        // && (get_sec() != bat_t_old) //　一定時間経過済
                        && (ball_cours_cnt < 1) //　ボール検知中ではない
                        && AutoJudghOff         //　自動検知モードでない
                        && second_camera_sht    //1番カメラは深度画像が不安定なため
                    )
                    {
                        // cv::Rect roiは、画像内の関心領域（Region of Interest, ROI）を定義しています。この領域は、打者が存在する可能性のある画像の一部分を指定しています。
                        cv::Rect roi(5, 10, frame_mono_batter.cols - 10, 200);
                        // bat_dot_suu変数には、frame_mono_batter(roi)関数を用いて、ROI内の非ゼロピクセル（つまり、打者によって占められていると考えられるピクセル）の数をカウントし、代入しています。
                        bat_dot_suu = countNonZero(frame_mono_batter(roi));
                        // LOG_DEBUG.printfを用いて、現在のbat_dot_suuと前回のbat_dot_suu_oldの値をログに出力しています。
                        LOG_DEBUG.printf("打者dot %d,old:%d", bat_dot_suu, bat_dot_suu_old);
                        // 次に、if文でbat_dot_suuがbat_dot_suu_oldの10%未満であるかどうかを判定しています。これは、打者が大きく動いたか、画像から消えたことを示す可能性があります。
                        if ( //
                            bat_dot_suu < 600
                            // ((bat_dot_suu < (int)(0.1 * bat_dot_suu_old)) || (bat_dot_suu < 100)) //
                            // && (bat_dot_suu_old > 1000)
                        )
                        {
                            // この条件が真の場合、Judgment_init()関数を呼び出して判定関連の変数を初期化し、bat_flgとbat_pose_flgをfalseに設定しています。これにより、打者の検知状態がリセットされます。
                            LOG_DEBUG.printf("打者リセット:%d,%d", bat_dot_suu, (int)(0.1 * bat_dot_suu_old));
                            Judgment_init();
                            bat_flg = false;
                            basez.bat_flg = false;
                            bat_pose_flg = false;
                            right_bat_f = false;
                            left_bat_f = false;
                            basez.batter_hight = 1;
                        }
                    }

                    //! ボールが特定のゾーン周辺に入ってこなかった場合に、一定時間が経過した後にコースデータをクリアする処理を行っています。この処理は、ボールが検出されていない(ball_detection_fがfalse)、かつコースカウント(ball_cours_cnt)が2未満の場合にのみ実行されます。これは、ボールがゾーンに近づいていない、またはゾーンに入っていない状態を示しています。
                    if (                                   //
                        ball_detection_phase != far_away   //
                        && ball_detection_phase != invalid //
                    )
                    {
                        // 判定の開始フラグ(Start_of_judgment)と別のフラグ(pr_flg)が共にfalseである場合にのみ、内部の処理が実行されます。これは、特定の条件下でのみコースデータのクリアを行うことを意味しています。←　判定が開始されようとしているときは実行しないため
                        if (!Start_of_judgment && !pr_flg)
                        {
                            // 、現在の時刻を取得するget_m_sec()関数の戻り値が、ball_detection_t_old（ボール検出時刻）に800ミリ秒を加えた値よりも大きいかどうかをチェックしています。これは、ボールが検出されてから一定時間が経過したかどうかを判断するための条件です。条件が真の場合、ball_clear()関数を呼び出してコースデータをクリアし、ball_detection_tを現在の時刻で更新します。この処理により、ボールが一定時間ゾーン周辺に入ってこなかった場合に、古いコースデータをクリアして新しい状態にリセットすることができます。
                            if ((
                                    // (ball_detection_t_old + c_ball_interval) < get_m_sec() //
                                    (first_time + c_ball_timeout) < get_m_sec() //
                                    )                                           //時間変数が整数型だったことを失念していたバグ対応
                            )
                            {
                                if (                                                                                                            //
                                    ball_detection_cnt > 3 && (ball_detection_phase == on_the_base || ball_detection_phase == passing_the_base) //
                                    || ((ball_detection_phase == on_the_base) && (ball_detection_cnt > 3)))
                                {
                                    // ball 配列を vector に変換
                                    if ((Start_of_judgment == false)) // && (AutoJudgh_sw || JudghTrigger_sw))
                                    {
#if (true)
                                        std::vector<ball_cours> ballVector(std::begin(ball), std::end(ball));
                                        // if (isLinearArrangementWithNoise_xy(ballVector))
                                        if (filterLinearPoints3D(ballVector))
#else
                                        if (isLinearArrangementWithNoise(ball_rireki))
#endif
                                        {
                                            // ボール確認できなくても線形の履歴が確認できれば判定する
                                            ball_detection_phase = passing_the_base;
                                            // 次に、Start_of_judgment（判定開始フラグ）がfalseであるかどうかをチェックします。これは、判定処理がまだ開始されていないことを確認するための条件です。コメントアウトされた部分（ // && (AutoJudgh_sw || JudghTrigger_sw)）は、追加の条件が考慮されていた可能性を示していますが、現在は使用されていません。
                                            // 条件が満たされた場合、Start_of_judgmentをtrueに設定し、判定処理を開始します。ball_clear_sleepもtrueに設定され、これはおそらくボールの検出状態をリセットするためのフラグです。Judgment_machi_timeには、get_m_sec関数を呼び出して取得した現在時刻（ミリ秒単位）が設定されます。この時刻は、判定処理の開始時刻として使用される可能性があります。
                                            Start_of_judgment = true;
                                            ball_clear_sleep = true;
                                            Judgment_machi_time = get_m_sec() + c_Judgment_wait;
                                            // 最後に、LOG_NONE.printfを使用してログに「find ball &hantei start」というメッセージを出力します。これは、ボールが見つかり、判定処理が開始されたことを示すメッセージです。
                                            LOG_NONE.printf("find ball & hantei start ");
                                        }
                                        else
                                        {
                                            LOG_DEBUG.printf("ボール線形エラー");
                                            matfilesave("/home/pi/kyouyuu/LOG/ball_rireki2", &ball_rireki); // 途中失敗したときのボール映像
                                            ball_clear();
                                        }
                                    }
                                }
                                else
                                {
                                    // get_m_sec()関数は、現在の時刻をミリ秒単位で返す関数であり、timespec_get関数を使用してシステム時刻を取得し、秒とナノ秒をミリ秒に変換しています。ball_clear()関数は、コースデータを初期化するための関数で、ボールの各種情報をリセットし、コースカウントや検出フェーズなどの変数を初期化しています。
                                    LOG_DEBUG.printf("Judgment timeout");
                                    ball_clear();
                                }
                            }
                        }
                        else
                        {
                            // 、現在の時刻を取得するget_m_sec()関数の戻り値が、ball_detection_t_old（ボール検出時刻）に800ミリ秒を加えた値よりも大きいかどうかをチェックしています。これは、ボールが検出されてから一定時間が経過したかどうかを判断するための条件です。条件が真の場合、ball_clear()関数を呼び出してコースデータをクリアし、ball_detection_tを現在の時刻で更新します。この処理により、ボールが一定時間ゾーン周辺に入ってこなかった場合に、古いコースデータをクリアして新しい状態にリセットすることができます。
                            if ((first_time + 800) < get_m_sec()) //時間変数が整数型だったことを失念していたバグ対応
                            {
                                // get_m_sec()関数は、現在の時刻をミリ秒単位で返す関数であり、timespec_get関数を使用してシステム時刻を取得し、秒とナノ秒をミリ秒に変換しています。ball_clear()関数は、コースデータを初期化するための関数で、ボールの各種情報をリセットし、コースカウントや検出フェーズなどの変数を初期化しています。
                                LOG_DEBUG.printf("Regular clear");
                                ball_clear();
                            }
                        }
                    }
                }
                second_camera_sht_old = second_camera_sht;
            }
            else
            {
                // 打者検出
                inRange(frame_mono_org, dep_base_en - 10, dep_base_kin + 20, frame_mono_batter); // 距離フィルター
            }

            //打者有無検知
            if (basez.base_find_f) //(!Judgment_mode) //ボール検知中でない →　打者の有無を検知する
            {
                if (!bat_flg             //(!Judgment_start && rgb_frame_taken // カラー画像がとれたときのみ深度画像取得（パフォーマンス向上のため）
                    && AutoJudghOff      //
                    && second_camera_sht //fastカメラが安定しないため追加
                )
                {
                    /*このコードは、画像処理を用いてバッターボックス内の右打者と左打者を検知するためのロジックを実装しています。*/
                    /*まず、frame_mono_batterという二値化された画像（またはマスク）から、特定の領域（四角形）内の非ゼロピクセル（ドット）の数をカウントしています。これは、バッターボックス内の特定の位置におけるドットの密度を計算するために使用されます。*/
                    // int temp_x = frame_mono_batter.cols - (temp_x_jyuushin - c_bias_x);
                    int temp_x = frame_mono_batter.cols / 2;
                    /*四つのRectオブジェクト（top_left_rect、top_right_rect、bottom_left_rect、bottom_right_rect）が定義されており、これらはバッターボックスの上部左、上部右、下部左、下部右の領域を表しています。これらの領域は、バッターボックス内の特定の位置を示しており、それぞれの領域に対してcountNonZero関数を使用して非ゼロピクセルの数をカウントしています。*/
                    Rect top_left_rect(temp_x, 10, temp_x, 100);
                    Rect top_right_rect(0, 10, temp_x, 100);
                    Rect bottom_left_rect(temp_x, 100, temp_x, 100);
                    Rect bottom_right_rect(0, 100, temp_x, 100);

                    // バッターボックス内のドット数をカウント
                    int bat_dot_suu_right_top = countNonZero(Mat(frame_mono_batter, top_right_rect));
                    int bat_dot_suu_left_top = countNonZero(Mat(frame_mono_batter, top_left_rect));
                    int bat_dot_suu_right_bottom = 0; //countNonZero(Mat(frame_mono_batter, bottom_right_rect));
                    int bat_dot_suu_left_bottom = 0;  //countNonZero(Mat(frame_mono_batter, bottom_left_rect));
                    const int bat_dot_suu_right_max = (top_right_rect.width * top_right_rect.height) / 4;
                    const int bat_dot_suu_left_max = (top_left_rect.width * top_left_rect.height) / 4;
#if (true)
                    LOG_VERBOSE.printf("bat_dot_suu_right_top:%d", bat_dot_suu_right_top);
                    LOG_VERBOSE.printf("bat_dot_suu_right_bottom:%d", bat_dot_suu_right_bottom);
                    LOG_VERBOSE.printf("bat_dot_suu_left_top:%d", bat_dot_suu_left_top);
                    LOG_VERBOSE.printf("bat_dot_suu_left_bottom:%d", bat_dot_suu_left_bottom);
#endif

                    int bat_dot_suu_right = bat_dot_suu_right_top + bat_dot_suu_right_bottom;
                    int bat_dot_suu_left = bat_dot_suu_left_top + bat_dot_suu_left_bottom;
                    /*右打者検知条件（right_bat_f）は、右上と右下の領域内のドット数の合計が、右側の最大ドット数（bat_dot_suu_right_max）よりも大きく、かつ左上と左下の領域内のドット数の合計が右側の最大ドット数の半分未満である場合にtrueとなります。*/
                    right_bat_f = (                                //
                        (bat_dot_suu_right > bat_dot_suu_left * 2) //
                        && bat_dot_suu_right > 1000                //
                        // && ((bat_dot_suu_left < (bat_dot_suu_left_max / 3))) //
                    );

                    /*左打者検知条件（left_bat_f）は、左上と左下の領域内のドット数の合計が、左側の最大ドット数（bat_dot_suu_left_max）よりも大きく、かつ右上と右下の領域内のドット数の合計が左側の最大ドット数の半分未満である場合にtrueとなります。*/
                    left_bat_f = (                                 //
                        (bat_dot_suu_left > bat_dot_suu_right * 2) //
                        && bat_dot_suu_left > 1000                 //
                        // && ((bat_dot_suu_right < (bat_dot_suu_right_max / 3))) //
                    );

                    if (bat_dot_suu_right > bat_dot_suu_left)
                    {
                        bat_dot_suu = bat_dot_suu_right;
                    }
                    else
                    {
                        bat_dot_suu = bat_dot_suu_left;
                    }

                    // 打者検知ログ出力
                    LOG_DEBUG_IF(right_bat_f).printf("右打者検知 r:%d,l:%d", (bat_dot_suu_right_top + bat_dot_suu_right_bottom), (bat_dot_suu_left_top + bat_dot_suu_left_bottom));
                    LOG_DEBUG_IF(left_bat_f).printf("左打者検知 r:%d,l:%d", (bat_dot_suu_right_top + bat_dot_suu_right_bottom), (bat_dot_suu_left_top + bat_dot_suu_left_bottom));

                    /*
                    if (rectifiedLeft_f)
                    {
                        inrectifiedLeft = qrectifiedLeft->get<dai::ImgFrame>(); // inrectifiedLeft
                        frame_rectifiedLeft = inrectifiedLeft->getFrame();
                    }
                    */
                    //露出設定変更反映
                    if (monocam_cng)
                    {
                        mono_read_check_f = false;
                        mono_read_lock_f = true;
                        mono_read_check_s = false;
                        mono_read_lock_s = true;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        // dai::CameraControl ctrl_mono;
                        // ctrl_mono.setAutoExposureLock(false);
                        ctrl_mono.setManualExposure(exp_time_mono, sens_iso_mono);
                        controlQueue_mono_right->send(std::make_shared<dai::CameraControl>(ctrl_mono)); // V3
                        controlQueue_mono_left->send(std::make_shared<dai::CameraControl>(ctrl_mono));  // V3
                        LOG_INFO.printf("Setting manual exposure MONO , time: %d, iso: %d", exp_time_mono, sens_iso_mono);
#if (double_camera == true)
                        // ctrl_mono_B.setAutoExposureLock(false);
                        ctrl_mono_B.setManualExposure(exp_time_mono_B, sens_iso_mono_B);
                        controlQueue_mono_B_right->send(std::make_shared<dai::CameraControl>(ctrl_mono)); // V3
                        controlQueue_mono_B_left->send(std::make_shared<dai::CameraControl>(ctrl_mono));  // V3
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        mono_read_lock_f = false;
                        mono_read_lock_s = false;
                        LOG_INFO.printf("Setting manual exposure MONO_B , time: %d, iso: %d", exp_time_mono_B, sens_iso_mono_B);
#endif
                        monocam_cng = false;
                        mono_read_check_f = true;
                        mono_read_check_s = true;
                    }
                }

                // 打者がバッターボックスに立ったことを検知。しばらく動きがないと投球待と判定Judgment_start
                //このコードのセクションは、打者がバッターボックスに立っているかどうかを検知し、一定時間動きがなければ投球待ち状態（Judgment_start）と判定するロジックを含んでいます。C++で書かれており、特定の条件下で変数の値を更新し、ログを出力する処理を行っています。
                //まず、if (right_bat_f || left_bat_f)の条件分岐では、右打者（right_bat_f）または左打者（left_bat_f）がバッターボックスに立っているかどうかをチェックしています。どちらか一方がtrueであれば、打者がボックスに立っているとみなされます。
                if (!Judgment_start)
                {
                    if ((right_bat_f && !left_bat_f) || (!right_bat_f && left_bat_f)) //(左右どちらかのボックスには立っていない）
                    {
                        //この条件がtrueの場合、bat_flgをtrueに設定し、打者が右打者か左打者かに応じて、bat_dot_suu_oldにそれぞれのドット数の合計を代入します。これにより、以前のフレームと比較して動きがあったかどうかを後で判断できるようになります。
                        // LOG_DEBUG.printf("打者検知(ボックス内) r:%d,l:%d,right_bat_f:%d,left_bat_f:%d", (bat_dot_suu_right_top + bat_dot_suu_right_bottom), (bat_dot_suu_left_top + bat_dot_suu_left_bottom), right_bat_f, left_bat_f);
                        bat_flg = true;
                        basez.bat_flg = true;
                        bat_dot_suu_old = bat_dot_suu;
                        basez.right_batter = right_bat_f;
                        zone_il_print(2);

                        if (bat_f_time == UINT64_MAX || bat_f_time == 0)
                            bat_f_time = get_m_sec();
                    }
                    else
                    {
                        //条件がfalseの場合、つまり打者がボックスに立っていない場合は、bat_flgをfalseに設定し、bat_f_timeに現在のミリ秒を記録し、bat_dot_suu_oldを0にリセットします。これにより、次のフレームでの検知に備えます。
                        bat_flg = false;
                        basez.bat_flg = false;
                        bat_dot_suu_old = 0;
                        bat_f_time = UINT64_MAX;
                    }
                    //さらに、if ((get_m_sec() - bat_f_time) > 500)の条件分岐では、最後に打者が検知されてから500ミリ秒以上経過しているかどうかをチェックします。この時間が経過していれば、LOG_DEBUG.printfを使用してログに打者の検知を記録し、Judgment_startをtrueに設定してボール検知を開始します。また、basez.right_batterにright_bat_fの値を代入し、打者が右打者かどうかの情報を保持します。
                    if ((get_m_sec() - bat_f_time) >= 800 && bat_flg)
                    {
                        LOG_DEBUG.printf("打者検知(判定開始) r:%d,l:%d,right_bat_f:%d,left_bat_f:%d", (bat_dot_suu_right_top + bat_dot_suu_right_bottom), (bat_dot_suu_left_top + bat_dot_suu_left_bottom), right_bat_f, left_bat_f);
                        Judgment_start = true; // ボール検知開始
                        Judgment_mode_kaishimachi_cnt = 1;
                        bat_f_time = UINT64_MAX;
                    }
                }

                // バッターの高さに基づくゾーンの低めと高めの再設定
                // 最後に、batter_hightの値に基づいて、バッターのゾーンの低め（zone_low_cm）と高め（zone_hi_cm）の値が再設定されます。これは、バッターの身長に応じてストライクゾーンを調整するためのもので、c_zone_low_cmとc_zone_hi_cmはそれぞれゾーンのデフォルトの低めと高めの値を表しており、batter_hightの値に応じてこれらの値が調整されます。
                basez.zone_low_cm = c_zone_low_cm - (3 * basez.batter_hight) - 3.6; // 低め
                basez.zone_hi_cm = c_zone_hi_cm - (7 * basez.batter_hight) + 3.6;   // 高め
            }

            // 距離フィルター後の２値画像を動画ファイルへ書き出す
            if (rokuga_flg) // && ball_detection_f)
            {
                binary_mask_distance.convertTo(tmp_frame, CV_8U);
                cvtColor(tmp_frame, tmp_binary_mask_distance, COLOR_GRAY2BGR);
                writer3 << tmp_binary_mask_distance;
            }
            // 深度画像を動画ファイルへ書き出す
            if (rokuga_flg2 && ball_detection_f)
            {
                frame_mono.convertTo(tmp_frame_depth, 8);
                cvtColor(tmp_frame_depth, tmp_frame2_depth, COLOR_GRAY2BGR); // 深度画像
                writer5 << tmp_frame2_depth;
            }
        }
    }
    LOG_NONE.printf("END HANTEI MAIN");
    return true;
}
/**********************************************************************************/

/**
 * @brief 距離（mono）画像取得
 * @param
 * @param
 * @param
 * @return
 * r_image_orgにカラー画像を取得します。
 * @sa
 */
void mono_read_f(dai::Device &device3)
{
    LOG_NONE.printf("start mono_read_f()");
    // (V2)     std::shared_ptr<dai::DataOutputQueue> q;
    // q = device3.getOutputQueue("disparity", 32, true); //trueでバッファ少なすぎるとハングする(v2)
    // リンクは不要。代わりにノードから直接出力キューを作成 (v3)
    std::shared_ptr<dai::MessageQueue> q = manip_hanten->out.createOutputQueue(); // NEW (v3)
    // 時間同期の設定
    std::chrono::milliseconds syncPeriod(1000); // 100msごとに同期
    int numSamples = 10;                        // 同期に使うサンプル数
    bool random = false;                        // ランダムな揺らぎはなし
    device3.setTimesync(syncPeriod, numSamples, random);
    auto initialFrame = q->get<dai::ImgFrame>();
    auto initialTs_f = initialFrame->getTimestamp();
    initialTs_ms_f = std::chrono::duration_cast<std::chrono::milliseconds>(initialTs_f.time_since_epoch()).count();
    // リサイズ後の画像を格納する変数
    cv::Mat resizedImage;

    cv::Mat inDepth_getCvFrame_tmp;
    // inDepth_getCvFrame_tmp のサイズに合わせた ROI を作成（貼り付け先）
    int a = std::round(c_rect_x_bias_720p / 16.0) * 16.0;
    int b = std::floor((640 - (c_rect_x + c_rect_x_bias_720p) * 2) / 16.0) * 16.0;
    printf("roi x:%d,%d\n", a, b);
    cv::Rect roi(a, 0, b, 400);

    // (V2)     std::shared_ptr<dai::DataOutputQueue> qRgb;
    std::shared_ptr<dai::MessageQueue> qRgb; // NEW (v3 に合わせた宣言、または v2 API の戻り値型)
    std::shared_ptr<dai::Buffer> buffer2 = std::make_shared<dai::Buffer>(); // V2->V3
    qRgb = script->outputs["stream1"].createOutputQueue(1, false); // V2->V3
    // controlQueue_Rgb = device3.getInputQueue(controlIn_Rgb->getStreamName()); ///V2
    controlQueue_Rgb = camRgb->inputControl.createInputQueue(); // NEW (v3)
    // (V2) controlQueue_script = device3.getInputQueue(controlIn_script->getStreamName());
    controlQueue_script = script->inputs["control_script"].createInputQueue(); // V3
    LOG_INFO.printf("Setting manual exposure  Rgb , time: %d, iso: %d", exp_time_Rgb, sens_iso_Rgb);
    ctrl_Rgb.setAutoExposureEnable();
    ctrl_Rgb.setAutoWhiteBalanceLock(true);
    ctrl_Rgb.setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);
    controlQueue_Rgb->send(std::make_shared<dai::CameraControl>(ctrl_Rgb)); //V2->V3
    cond_flg = true;
    int snap_loop = 0;

    while (end_flg_mono_read)
    {
        //マイクロセックにしておかないとファーストカメラが取得できなくなる
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        if (mono_read_lock_f)
        {
            continue; //判定中以外は読み込ませる（打者検知のため）
        }
        inDepth_f = q->tryGet<dai::ImgFrame>();
        if (inDepth_f)
        {
            if (p720p_on)
            {
                inDepth_getCvFrame_tmp = inDepth_f->getCvFrame();
                inDepth_getCvFrame_tmp.copyTo(frame_mono_f_tmp(roi));
            }
            else
            {
                frame_mono_f_tmp = inDepth_f->getCvFrame();
            }
            if (camera_sel == 1 || camera_sel == 0)
            {
                ts_org_f = inDepth_f->getTimestamp();
                mono_frame_taken_img_key_f = true;
            }
        }

        // カラー画像取得
        if (!Judgment_mode || snap_shot)
        {
            try
            {
                if (snap_shot && !snap_shot_sumi)
                {
                    // バッファーをクリア
                    for (int l = 0; l < snapshot_cnt; l++) //
                    //if (snap_loop < 10)
                    {
                        mono_read_lock_f = true; // mono画像取得済状態をロック
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        controlQueue_script->send(buffer2); // このcontrol_scriptによりdepthaiのSCRIPTが動く ***
                        inRgb = qRgb->get<dai::ImgFrame>();
                        if (inRgb)
                        {
                            r_image_org = inRgb->getCvFrame();
                            snap_loop++;
                        }
                        else
                        {
                            l--;
                        }
                        mono_read_lock_f = false; // mono画像取得済状態をリリース
                    }
                    //else
                    //{
                    snap_shot_sumi = true;
                    snap_loop = 0;
                    LOG_DEBUG.printf(" qRgb buff clear");
                    //}
                }
                else if (!rgb_frame_taken)
                {
                    controlQueue_script->send(buffer2); // このcontrol_scriptによりdepthaiのSCRIPTが動く ***
                    inRgb = qRgb->tryGet<dai::ImgFrame>();
                    if (!inRgb)
                    {
                        continue;
                    }
                    r_image_org = inRgb->getCvFrame();
                    // 水平・垂直両方向に反転
                    if (ground_mode)
                        cv::flip(r_image_org, r_image_org, -1);
                    rgb_frame_taken = true;
                    rgb_frame_taken_img_key = true;
                    cond_flg = false;
                }
            }
            catch (...)
            {
            }
        }
    }
    LOG_NONE.printf("end mono_read_f()");
}

/**
 * @brief 距離（mono）画像取得
 * @param
 * @param
 * @param
 * @return
 * r_image_orgにカラー画像を取得します。
 * @sa
 */
void mono_read_s(dai::Device &device3)
{
    LOG_NONE.printf("start mono_read_s()");
    // (V2)     std::shared_ptr<dai::DataOutputQueue> q;
    // q = device3.getOutputQueue("disparity_B", 32, true); //trueでバッファ少なすぎるとハングする(v2)
    // リンクは不要。代わりにノードから直接出力キューを作成 (v3)
    std::shared_ptr<dai::MessageQueue> q = manip_hanten_B->out.createOutputQueue(); // NEW (v3)

    // 時間同期の設定
    std::chrono::milliseconds syncPeriod(1000); // 100msごとに同期
    int numSamples = 10;                        // 同期に使うサンプル数
    bool random = false;                        // ランダムな揺らぎはなし
    device3.setTimesync(syncPeriod, numSamples, random);
    auto initialFrame = q->get<dai::ImgFrame>();
    auto initialTs_s = initialFrame->getTimestamp();
    initialTs_ms_s = std::chrono::duration_cast<std::chrono::milliseconds>(initialTs_s.time_since_epoch()).count();

    // リサイズ後の画像を格納する変数
    cv::Mat resizedImage;

    cv::Mat inDepth_getCvFrame_tmp;
    // inDepth_getCvFrame_tmp のサイズに合わせた ROI を作成（貼り付け先）
    int a = std::round(c_rect_x_bias_720p / 16.0) * 16.0;
    int b = std::floor((640 - (c_rect_x + c_rect_x_bias_720p) * 2) / 16.0) * 16.0;
    printf("roi x:%d,%d\n", a, b);
    cv::Rect roi(a, 0, b, 400);
    usleep(100);

    while (end_flg_mono_read)
    {
        //マイクロセックにしておかないとファーストカメラが取得できなくなる
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        if (mono_read_lock_s)
        {
            continue; //判定中以外は読み込ませる（打者検知のため）
        }
        inDepth_s = q->tryGet<dai::ImgFrame>();
        if (inDepth_s)
        {
            if (p720p_2_on)
            {
                inDepth_getCvFrame_tmp = inDepth_s->getCvFrame();
                inDepth_getCvFrame_tmp.copyTo(frame_mono_s_tmp(roi));
            }
            else
            {
                frame_mono_s_tmp = inDepth_s->getCvFrame();
            }
            if (camera_sel == 2 || camera_sel == 0)
            {
                ts_org_s = inDepth_s->getTimestamp();
                paste(frame_mono_s_tmp, frame_mono_s_tmp, second_camera_x_bias, second_camera_y_bias, inDepth_s->getCvFrame().cols, inDepth_s->getCvFrame().rows);
                mono_frame_taken_img_key_s = true;
            }
        }
    }
    LOG_NONE.printf("end mono_read_s()");
}

/*
                 basez.camera_high = basez.c_camera_high_no_base + c_second_camera_high_bias;
                 paste(frame_mono_s_tmp, frame_mono_s_tmp, second_camera_x_bias, second_camera_y_bias, inDepth_B->getCvFrame().cols, inDepth_B->getCvFrame().rows);
                 ts_org = inDepth_B->getTimestamp() + offset;
                 mono_frame_taken_img_key = true;
                 //fps_cnt++;
                 second_camera_length_bias_org = c_second_camera_length_bias;
                 second_camera_sht = true;
                 second_mono_act = true;
                 second_camera_y_bias_org = 0;
                 basez.camera.pitch = camera_pitch_org + second_camera_pitch_bias;
                 basez.camera.yaw = basez.camera_yaw_org + second_camera_yaw_bias;
                 frame_mono_s_tmp.copyTo(frame_mono_org);
*/

/**
 * @brief キーイベント、カラー画像処理、など
 *          判定中に必要のない処理はすべてこちらに入れてFPSを稼ぐ
 * @param
 * @param
 * @param
 * @return
 * @sa
 */
void img_key(dai::Device &device2)
{
    LOG_NONE.printf("start img_key()");
    bool info_print = false;      // インフォメーション表示フラグ
    bool info_print2 = false;     // 画面中心距離表示フラグ
    float sqrt_imu_old = 0;       // 前回取得時の合計加速度
    Point2f base_center;          // base center
    bool kyuusoku_keisoku = true; // 球速測定表示するか
    int base_umekomi = conf_json["base_umekomi"];
    // Mat zone_3d_img = imread("zone_3d.png", -1);
    zone_il_r = imread("zone2_r.png"); // ゾーンイラスト
    zone_il_l = imread("zone2_l.png"); // ゾーンイラスト
    zone_il_r.copyTo(zone_il);         // ゾーンイラスト
    // zone_il_top = imread("basezone.png"); // ゾーンイラスト トップビュー
    zone_il_top = imread("homebase_yoko_no_zone.png"); // basezone.png"); // ゾーンイラスト トップビュー
    Mat strike_png = imread("strike.png", -1);
    Mat ball_png = imread("ball.png", -1);
    Mat ball_il = imread("ball_il.png", -1);
    Mat hanteityuu_png = imread("hanteity.png");
    Mat hsv_img;
    Mat tmp_frame_mono;
    Mat tmp_bin_img;
    Mat ball_out_il;
    float zone_c_dis_w_cm_old;
    float x_acc_bias = conf_json["x_acc_bias"];
    float y_acc_bias = conf_json["y_acc_bias"];
    float z_acc_bias = conf_json["z_acc_bias"];
    float wHazureCm = 0;
    float hHazureCm = 0;

#if (raspi5 == true)
    //プルアップ
    system("pinctrl set 5 ip pu");
    system("pinctrl set 6 ip pu");
    system("pinctrl set 16 ip pu");
    // デフォルトのGPIOチップをオープンします
    chip = gpiod_chip_open("/dev/gpiochip4");
    if (!chip)
    {
        std::cerr << "Error opening GPIO chip" << std::endl;
    }

    // LEDのGPIOラインをリクエストします
    sw_5_AutoJudgh = gpiod_chip_get_line(chip, AutoJudgh);
    sw_6_JudghTrigger = gpiod_chip_get_line(chip, JudghTrigger);
    sw_26_LED_sw = gpiod_chip_get_line(chip, LED_sw);
    sw_20_MOTORSW = gpiod_chip_get_line(chip, MOTORSW);
    sw_16_KneeDetection = gpiod_chip_get_line(chip, KneeDetection);
#endif

#if (raspi5 == true)
    gpio = pigpio_start(nullptr, "8888");
    bool JudghTrigger_sw_old;
    uint64_t JudghTrigger_sw_on_t;
    LOG_DEBUG.printf("pinMode(AutoJudgh):%d", pinMode(AutoJudgh, 1)); // 入力モード(1) 出力モード(0)
    LOG_DEBUG.printf("pinMode(JudghTrigger):%d", pinMode(JudghTrigger, 1));
    LOG_DEBUG.printf("pinMode(LED_sw):%d", pinMode(LED_sw, 0));   // 出力モード
    LOG_DEBUG.printf("pinMode(MOTORSW):%d", pinMode(MOTORSW, 0)); // 出力モード
    LOG_DEBUG.printf("pinMode(KneeDetection):%d", pinMode(KneeDetection, 1));
    LOG_DEBUG.printf("本体スイッチ状況");
    LOG_DEBUG.printf("KneeDetection:%d", digitalRead(KneeDetection));
    LOG_DEBUG.printf("JudghTrigger:%d", digitalRead(JudghTrigger));
    LOG_DEBUG.printf("AutoJudgh:%d", digitalRead(AutoJudgh));
    LOG_DEBUG_IF(AutoJudghOn).printf("AutoJudghOn");
    LOG_DEBUG_IF(AutoJudghOff).printf("AutoJudghOff");
    LOG_DEBUG_IF(JudghTriggerOn).printf("JudghTriggerOn");
    LOG_DEBUG_IF(JudghTriggerOff).printf("JudghTriggerOff");
    LOG_DEBUG_IF(KneeDetectionOn).printf("KneeDetectionOn");
    LOG_DEBUG_IF(KneeDetectionOff).printf("KneeDetectionOff");
    basez.knee_detection = KneeDetectionOn;
    if (AutoJudghOn)
        Judgment_mode_kaishimachi_cnt = 8; // 3秒後に自動判定開始
                                           //led(200, 150, 3);
                                           //motor(800);
#else
    wiringPiSetupGpio();
    bool JudghTrigger_sw_old;
    uint64_t JudghTrigger_sw_on_t;
    pinMode(JudghTrigge, INPUT);
    pinMode(AutoJudgh, , INPUT);
    pullUpDnControl(JudghTrigge, PUD_UP);
    pullUpDnControl(AutoJudgh, , PUD_UP);
#endif

    uint64_t calibration_start_t = 0;

    namedWindow("img-rinkaku", cv::WND_PROP_FULLSCREEN);
    setWindowProperty("img-rinkaku", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    // 固定サイズのウィンドウを作成
    int fixedWidth = 800;  // ウィンドウの幅
    int fixedHeight = 600; // ウィンドウの高さ

    // ウィンドウの作成とサイズ設定
    // cv::namedWindow("img-rinkaku", cv::WINDOW_NORMAL);
    cv::resizeWindow("img-rinkaku", fixedWidth, fixedHeight);
    setMouseCallback("img-rinkaku", on_mouse); //"win1"にon_mouseを登録．

    // Pythonの初期化
    Py_Initialize();
    _import_array();
    // カレントディレクトリを探す範囲にいれる-------------------------
    PyObject *sys = PyImport_ImportModule("sys");
    PyObject *path = PyObject_GetAttrString(sys, "path");
    PyList_Append(path, PyUnicode_DecodeFSDefault("."));
    // Pythonのモジュール名を指定
    PyObject *pName = PyUnicode_DecodeFSDefault("pose_cpp");
    // モジュールのインポート
    PyObject *pModule = PyImport_Import(pName);
    Py_DECREF(pName);
    // printf("PModule %d\n", pModule);
    // Pythonのモジュールから関数を取得
    // if (pModule != NULL)
    //{

    PyObject *pFunc = PyObject_GetAttrString(pModule, "Pose");
    printf("callable? %d\n", PyCallable_Check(pFunc));

    // グローバル辞書を取得
    PyObject *global_dict = PyModule_GetDict(pModule);

    //}
    //
    bool r_image_org_shrink_copyed = false;

#if (c_imu == true)
    // IMU
    if (imu_use)
    {
        bool firstTs = false;
        // (V2) imuQueue = device2.getOutputQueue("imu", 1, false);
        imuQueue = imu->out.createOutputQueue(1, false); // NEW (v3): IMUノードのoutポートから直接キューを作成
        auto baseTs = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>();
    }
#endif
    // カラー画像ベース作成
    Mat image_tmp = Mat::zeros(400, 640, CV_8UC3);
    image_tmp = cv::Scalar(16, 57, 15);
    image_tmp.copyTo(r_image_SARA);
    //r_image_SARA = image_tmp;
    uint16_t counter = 0;
    uint64_t old_sec = 0;
    float x_acc_old = 0;
    float y_acc_old = 0;
    float z_acc_old = 0;
    sleep_sec(1);
    // play ball sound
    play_sound_func_2(0);
    while (end_flg_img_key)
    {
        if (Judgment_mode)
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100msスリープ
        std::this_thread::sleep_for(std::chrono::milliseconds(1));      // 100msスリープ

        // RGB画像を深度画像とスケール合わせして、r_image_SARAに保存する
        // スナップショット時、RGB画像を深度画像とスケール合わせして、r_image_SARA＿snapに保存する
        if ((!Judgment_mode) || (snap_shot_sumi))
        {
            //１秒ごとの処理
            if (get_m_sec() > (old_sec + 999))
            {
                old_sec = get_m_sec();
                if (Judgment_mode_kaishimachi_cnt > 0)
                {
                    {
                        Judgment_mode_kaishimachi_cnt--;
                        LOG_DEBUG.printf(" kaishimachi:%d", Judgment_mode_kaishimachi_cnt);
                    }
                }
            }

            //if (Judgment_start)
            {
                // 背景ボーダ　クリア
                image_tmp = cv::Scalar(16, 57, 15);
                image_tmp.copyTo(r_image_SARA);
                // r_image_SARA = image_tmp;
            }
            // カラー画像をシュリンクして、モノ画像と位置合わせ
            if (!r_image_org.empty() && !cond_flg) //r_image_orgは取得したままのカラー画像
            {
                paste2(r_image_SARA, r_image_org, c_bias_x, c_bias_y, C_monorgb_hi_x, C_monorgb_hi_y);
                // pose検知ようにコピー
                r_image_SARA.copyTo(r_image_org_shrink); //r_image_org_shrinkはpose検知に利用する
                // r_image_org_shrink = r_image_SARA;
                rgb_frame_taken = false; // カラー画像取得済状態をリリース
            }
            else
            {
                continue;
            }
            RGB_image_acquisition = true; // カラー画像が取得できたらベース距離測定ように深度画層を取得
#if (detNN == true)
            inDet = qDet->get<dai::ImgDetections>();
#endif

            if (snap_shot)
            {
                snap_shot = false;
                snap_shot_sumi = false;
                LOG_INFO.printf("SNAP SHOT!!!");
            }
        }
        else if (snap_shot)
        {
            continue;
            // RGB_image_acquisition = false;
        }

#if (detNN == true)
        if (inDet)
        {
            detections = inDet->detections;
        }
        Judgment_mode

            // バッター検出
            basez.batter_top = 0;
        if (!r_image_SARA.empty())
        {
            displayFrame("img-rinkaku", &r_image_SARA, detections);
        }
#endif

#if (raspi == false) || (raspi == FALSE)
        bin_img_f = true;              // imshow("masked 2color", bin_img);//ベース（白）抽出
        r_image_f = true;              // imshow("img-rinkaku", r_image_SARA);//ストライクゾーン＆インフォメーション
        binary_mask_distance_f = true; // imshow("mask", binary_mask_distance);//距離２値画像
#else
        bin_img_f = true;                               // imshow("masked 2color", bin_img);//ベース（白）抽出
        r_image_f = false;                              // imshow("img-rinkaku", r_image_SARA);//ストライクゾーン＆インフォメーション
        binary_mask_distance_f = test_measurement_mode; // imshow("mask", binary_mask_distance);//距離２値画像
#endif
        frame_mono_f = test_measurement_mode; // cv::imshow("nama depth", frame_mono);//生距離画像

        if (!Judgment_mode)
        {
            //oak-dの場合加速度センサーからカメラピッチ、ローを取得
#if (c_imu == true)
            std::shared_ptr<dai::IMUData> imuData;
            std::vector<dai::IMUPacket, std::allocator<dai::IMUPacket>> imuPackets;
            if (imu_use && !ground_mode)
            {
                try
                {
                    imuData = imuQueue->get<dai::IMUData>();
                    imuPackets = imuData->packets;
                }
                catch (...)
                {
                }

                float x_acc;
                float y_acc;
                float z_acc;

                for (auto &imuPacket : imuPackets)
                {
                    auto &acceleroValues = imuPacket.acceleroMeter;
                    // auto &gyroValues = imuPacket.gyroscope;

                    /*
                    auto acceleroTs1 = acceleroValues.Device();
                    auto gyroTs1 = gyroValues.getTimestamp();
                    if (!firstTs)
                    {
                        baseTs = std::min(acceleroTs1, gyroTs1);
                        firstTs = true;
                    }
                    auto acceleroTs = acceleroTs1 - baseTs;
                    auto gyroTs = gyroTs1 - baseTs;
                    */

                    x_acc = acceleroValues.x + x_acc_bias; //1.31
                    y_acc = acceleroValues.y + y_acc_bias; //0.31
                    z_acc = acceleroValues.z + z_acc_bias; //0.29;
                    x_acc = x_acc * 0.1 + x_acc_old * 0.9;
                    y_acc = y_acc * 0.1 + y_acc_old * 0.9;
                    z_acc = z_acc * 0.1 + z_acc_old * 0.9;
                    x_acc_old = x_acc;
                    y_acc_old = y_acc;
                    z_acc_old = z_acc;
                    // printf(" x:%.2f y:%.2f z%.2f \n",x_acc,y_acc,z_acc);
                }
                float camera_pitch_old = basez.camera.pitch;
                camera_pitch_org = camera_pitch_old * 0.0 + 1.0 * clamp(abs(atan2(z_acc, sqrt(x_acc * x_acc + y_acc * y_acc)) + 1.54), basez.rdn(40), basez.rdn(80));
                //if (!test_measurement_mode)
                // basez.camera.pitch = camera_pitch_org;
                basez.camera_pitch_org = camera_pitch_org;
                //if (!test_measurement_mode)
                // basez.camera.pitch = basez.camera.pitch * camera_pitch_keisuu;
                //basez.camera.pitch = (basez.camera.pitch < 0) ? basez.rdn(60) : basez.camera.pitch;
                //basez.base_roll = atan2(y_acc, sqrt(x_acc * x_acc + z_acc * z_acc));// clamp(atan2(y_acc, sqrt(x_acc * x_acc + z_acc * z_acc)), basez.rdn(-15), basez.rdn(15));
                basez.base_roll = clamp(atan2(y_acc, sqrt(x_acc * x_acc + z_acc * z_acc)), basez.rdn(-15), basez.rdn(15));
                // printf(" angle:%.2f",basez.base_roll);
                basez.camera.roll = -basez.base_roll;
            }
#endif

            // ゾーン以外の画面パーツの描画
            //このコードは、画像処理とGUIの描画に関連する一連の操作を行っています。OpenCVライブラリを使用して、特定の画像(r_image_SARA)に対して複数のグラフィカルな要素を追加しています。具体的には、以下のような処理を行っています。
            //画面パーツの描画: print_gui関数を呼び出して、r_image_SARA画像にGUI要素を描画しています。この関数は、ボタンやその他のGUI要素を画像に追加するために使用されます。
            //イラスト画像のインポートと配置: paste関数を使用して、zone_ilとzone_il_topというイラスト画像をr_image_SARA画像に配置しています。これらの画像は、特定の座標に貼り付けられます。
            //サイドビューのゾーン描画: zone_il_topにhomebase_yoko_no_zone.png画像を読み込み、drawsideviewZone関数を呼び出してサイドビューのゾーンを描画しています。
            //矩形の描画: rectangle関数を使用して、r_image_SARA画像に矩形を描画しています。これらの矩形は、特定の座標とサイズで定義されています。
            //インフォメーションの描画: sprintfとputText関数を使用して、画像上にテキスト情報を描画しています。これには、カメラの設定やボールの位置などの情報が含まれます。
            //テストボール位置の表示: テスト測定モードが有効な場合、circle関数を使用して、ボールの位置を示す円を描画しています。
            //自動判定の表示: digitalRead関数を使用して、特定のピンの状態を読み取り、その結果に基づいてAutoというテキストを画像に描画しています。
            //このコードは、画像に対して複雑なGUI要素と情報を動的に描画するためのものです。OpenCVの関数を活用して、画像処理とグラフィカルな要素の追加を行っています。
            {
                print_gui(r_image_SARA, false);
                // イラスト画像のインポート
                paste(r_image_SARA, zone_il, 0, 210, 150, 185);
                paste(r_image_SARA, zone_il_top, 440, 213, 200, 187);
                // サイドビューのゾーン描画
                zone_il_top = imread("homebase_yoko_no_zone.png"); // basezone.png"); // ゾーンイラスト トップビュー
                drawsideviewZone();

                // 別の矩形を描画
                rectangle(r_image_SARA, Point(0, 210), Point(150 + 0, 185 + 210), Scalar(200, 200, 200), 3, 1);
                rectangle(r_image_SARA, Point(440, 213), Point(200 + 440, 187 + 213), Scalar(200, 200, 200), 3, 1);
                // インフォメーション描画
                if (info_print)
                {
                    sprintf(value_c, "v=%d,exp(m)=%d,iso(m)=%d", best_v_filter, exp_time_mono, sens_iso_mono); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "v=%d,exp(R)=%d,iso(R)=%d", best_v_filter, exp_time_Rgb, sens_iso_Rgb); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    // sprintf(value_c, "en=%d,kin=%d", dep_l_en, dep_l_kin); //変数の値も含めた表示したい文字列をchar型変数に格納
                    // cv::putText(r_image_SARA, value_c, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "FPS:%d", fps_cnt_fix); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(260, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                }
                //中心距離表示（キャリブレーション用）
                if (info_print2)
                {
                    float depcm = basez.depth_measure(Point(frame_mono.cols / 2, frame_mono.rows / 2), &frame_mono, false);
                    sprintf(value_c, "depthcm:%.1f,camera:%d", depcm, second_camera_sht + 1); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(160, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    float depcm_org = basez.depth_measure(Point(frame_mono_org.cols / 2, frame_mono_org.rows / 2), &frame_mono_org, false);
                    sprintf(value_c, "org depthcm:%.1f,camera:%d", depcm_org, second_camera_sht + 1); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(100, 110), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    printf("frame_mono.cols / 2, frame_mono.rows / 2: %d,%d depcm:%.1f\n", frame_mono.cols / 2, frame_mono.rows / 2, depcm);
                    printf("frame_mono_org.cols / 2, frame_mono_org.rows / 2: %d,%d depcm:%.1f\n", frame_mono_org.cols / 2, frame_mono_org.rows / 2, depcm_org);
                }
                if (test_measurement_mode)
                {
                    // テストボール位置表示
                    if ((basez.hoseimae_center.x != 0) && basez.last_strike)
                        circle(r_image_SARA, basez.worldToScreen(), 12, Scalar(255, 255, 0), 3, LINE_AA, 0);
                    if ((basez.hoseimae_center.x != 0) && !basez.last_strike)
                        circle(r_image_SARA, basez.worldToScreen(), 12, Scalar(0, 0, 255), 3, LINE_AA, 0);
                    circle(r_image_SARA, basez.bestMatch, 5, Scalar(255, 255, 0), 3, LINE_AA, 0);
                    // printf(" %d,%d\n",basez.worldToScreen().x,basez.worldToScreen().y);
                    int base_y = 50;
                    sprintf(value_c, "camera_yaw:%.1f", basez.ardn(basez.camera.yaw)); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(150, base_y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "camera_pitch:%.1f", basez.ardn(basez.camera.pitch)); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(150, base_y + 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "camera_roll:%.1f", basez.ardn(basez.camera.roll)); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    // sprintf(value_c, "focus:%d", focus_mono); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(150, base_y + 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "Ball soutai_w: %.1fcm", basez.ball_position.w); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(150, base_y + 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "Ball hi:    %.1fcm", basez.ball_position.h); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(150, base_y + 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "Balldep:    %.1fcm", (basez.ball_position.d)); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(150, base_y + 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "straightToBall_cm: %.1f", basez.straightLineDistanceToBall_cm); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(10, base_y + 180), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                    sprintf(value_c, "Mrk%d,Cam%d", base_detection_maker, camera_sel); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(400, base_y + 180), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(128, 128, 255), 2);
                }
                if (Judgment_mode_kaishimachi_cnt > 0)
                {
                    sprintf(value_c, "Judgment mode start in %d sec", Judgment_mode_kaishimachi_cnt); // 変数の値も含めた表示したい文字列をchar型変数に格納
                    cv::putText(r_image_SARA, value_c, cv::Point(50, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(20, 20, 255), 2);
                }
                r_image_f = true;
                // 自動判定か否かを表示
                if AutoJudghOn
                {
                    sprintf(value_c, "Auto");
                    cv::putText(r_image_SARA, value_c, cv::Point(180, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(20, 20, 255), 2);
                }
                if (isHorizontalError)
                {
                    sprintf(value_c, "Horizontal Error");
                    cv::putText(r_image_SARA, value_c, cv::Point(150, 180), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(28, 28, 255), 2);
                }
            }

            // ベース検知
            // if (rgb_frame_taken_img_key)
            {
                float tmp_r, tmp_x, tmp_y, tmp_x_old, tmp_y_old; // 画像傾き
                tmp_r = 0;
                tmp_x_old = 0;
                tmp_y_old = 0;
                tmp_x = 0;
                tmp_y = 0;

                cv::Rect roi(c_rect_x, c_rect_zone_y, 640 - ((c_rect_x)*2), 400 - c_rect_zone_y);
                mask_image_add = cv::Mat(r_image_SARA, roi);

                /* 深度画像からカメラピッチ角を算出するよう
                printf(" frame_mono?temp:%d,%d\n",frame_mono_tmp.cols,frame_mono_tmp.rows);
                if (!frame_mono_tmp.empty())
                basez.camera.pitch = basez.calc_camera_pich(&frame_mono_tmp);
                */
                bool base_find_f_tmp;
                if (true)
                {
                    basez.base_Binarization(&mask_image_add, &bin_img, best_v_filter);

                    // 距離計測
                    if (true) // (c_horizontalDistanceToBase_cm == 0) // ベースまでの直線距離(固定)
                    {
                        if (((counter % 30) == 0) || !basez.base_find_f)
                        {
                            base_find_f_tmp = basez.Base_find(&bin_img, mask_image_add, false);
                            /*
                            switch (base_detection_maker)
                            {
                            case 0:
                            {
                                base_find_f_tmp = basez.Base_find(&bin_img, mask_image_add, false);
                            }
                            break;
                            case 1:
                            {
                                base_find_f_tmp = basez.Base_find_maker(&bin_img, false);
                            }
                            break;
                            case 2:
                            {
                                base_find_f_tmp = basez.Base_find(&bin_img, mask_image_add, false);
                            }
                            break;
                            case 3:
                            case 4:
                            {
                                base_find_f_tmp = basez.Base_find(&bin_img, mask_image_add, false);
                            }
                            break;
                            default:
                                break;
                            }
                            */
                            //if (base_find_f_tmp)
                            //    solve_pnp();
                        }
                    }
                    else
                    {
                        base_find_f_tmp = true;
                    }
                }
                if (true) //(base_find_f_tmp)
                {
                    // ベース距離測定
                    base_center = basez.jyuushin;
                    LOG_VERBOSE.printf(" base_center:%.0f,%.0f", base_center.x, base_center.y);
                    basez.base_center = base_center;
                    int check_y = (int)base_center.y;            //                    +c_bias_y;                                   //, 5, (c_rect_y_lo * 400 - 5));
                    int check_x = (int)base_center.x - c_bias_x; // - (c_rect_x_hi * 640), 5, (c_rect_x_hi * 2 * 640 - 5));
                    int dep_i_heikin = 0;
                    int dep_cnt = 0;
                    int dep_cnt_base = 0;
                    if (!no_base_mode)
                    {
                        // LOG_DEBUG.printf("depth %d",mask_image.depth());
                        for (int iy = clamp(check_y - 2, 0, frame_mono.rows - 1); iy <= clamp(check_y + 2, 0, frame_mono.rows - 1); iy++)
                        {
                            ushort *src = frame_mono.ptr<ushort>(iy);
                            for (int ix = clamp(check_x - 30, 0, frame_mono.cols - 1); ix <= clamp(check_x + 30, 0, frame_mono.cols - 1); ix++)
                            {
                                int dep_tmp = (int)src[ix];
                                // LOG_DEBUG.printf(" basedep:%d", dep_tmp);
                                if ((dep_tmp != 0)) // && (dep_tmp >= c_dep_base_en) && (dep_tmp <= c_dep_l_kin))
                                {
                                    // printf("base距離:%.1f\n", (basez.c_dis_cm(dep_tmp)));
                                    dep_i_heikin += dep_tmp;
                                    dep_cnt++;
                                }
                                dep_cnt_base++;
                            }
                        }
                        if (dep_cnt != 0)
                        {
                            if (base_detection_maker == 3 || base_detection_maker == 4)
                            {
                                straightLineDistanceToBase_i = (int)((float)dep_i_heikin / dep_cnt);
                            }
                            else
                            {
                                straightLineDistanceToBase_i = 0.2 * (int)((float)dep_i_heikin / dep_cnt) + 0.8 * base_dep_i_old;
                                base_dep_i_old = straightLineDistanceToBase_i;
                            }
                        }
                        else
                        {
                            straightLineDistanceToBase_i = 128;
                            LOG_DEBUG_IF(dep_i_heikin == 0).printf("base dep cnt error");
                        }
                    }

                    // printf("base直線距離:%.1f\n", basez.c_dis_cm(straightLineDistanceToBase_i));
                    float high;
                    float width;
                    float h_distance;
                    int base_dep_dummy;
                    Point2i base_katamuki_hoseigo;
                    base_katamuki_hoseigo = basez.katamuki_hosei(basez.jyuushin, straightLineDistanceToBase_i, &straightLineDistanceToBase_i);
                    // LOG_DEBUG.printf(" _i:%.0fcm", basez.c_dis_cm(straightLineDistanceToBase_i));
                    if (c_horizontalDistanceToBase_cm == 0 && !no_base_mode) // ベースまでの直線距離(固定)
                    {
                        if (straightLineDistanceToBase_i != 0)
                        {
                            basez.straightLineDistanceToBase_cm = 1.0 * basez.c_dis_cm(straightLineDistanceToBase_i) + 0.0 * old_straightLinelDistanceToBase_cm;
                            old_straightLinelDistanceToBase_cm = basez.straightLineDistanceToBase_cm;
                            // basez.calc_h_d_ball(basez.straightLineDistanceToBase_cm, basez.jyuushin.x, basez.jyuushin.y, &h_distance, &high, &width, true); // 水平距離、垂直高さの算出
                            basez.calc_h_d_ball(basez.straightLineDistanceToBase_cm, base_katamuki_hoseigo.x, base_katamuki_hoseigo.y, &h_distance, &high, &width, true); // 水平距離、垂直高さの算出
                            Dimensions base_dm;
                            basez.screenToWorld(base_katamuki_hoseigo, basez.straightLineDistanceToBase_cm);
                            h_distance = base_dm.d;
                        }
                    }
                    else
                    {                                                                                                                                   //baseまでの距離を設定
                        basez.calc_h_d_ball(basez.straightLineDistanceToBase_cm, basez.jyuushin.x, basez.jyuushin.y, &h_distance, &high, &width, true); // 水平距離、垂直高さの算出
                        h_distance = basez.c_horizontalDistanceToBase_cm;
                    }
                    // LOG_DEBUG.printf(" straightLineDistanceToBase_i:%.0fcm", basez.straightLineDistanceToBase_cm);
                    //  ベース位置設定
                    // basez.calc_h_d_ballのコールでベースの座標 basez.horizontalDistanceToBase_cm,base_dep_cmなどが設定されている
                    straightLineDistanceToBase_cm = basez.straightLineDistanceToBase_cm;
                    straightLineDistanceToBase_i = basez.straightLineDistanceToBase_i;
                    horizontalDistanceToBase_cm = basez.horizontalDistanceToBase_cm;
                    horizontalDistanceToBase_i = basez.horizontalDistanceToBase_i;
                    dep_base_en = basez.c_dis_dep_i(basez.horizontalDistanceToBase_cm + 21.6 + 3.6);
                    dep_base_kin = basez.c_dis_dep_i(basez.horizontalDistanceToBase_cm - 21.6 - 3.6);
                    dep_l_en = basez.c_dis_dep_i(basez.c_dis_cm(dep_base_en) + dep_l_en_bias);
                    dep_l_kin = clamp(basez.c_dis_dep_i(basez.c_dis_cm(dep_base_kin) - dep_l_kin_bias), 10, 999);
                    basez.dep_base_en = dep_base_en;
                    basez.dep_base_kin = dep_base_kin;
                    basez.dep_l_en = dep_l_en;
                    basez.dep_l_kin = dep_l_kin;
                    if (counter % 30 == 0)
                    {
                        if (base_detection_maker == 1)
                        {
                            base_find_f_tmp = basez.Base_find_maker(&bin_img, false);
                        }
                        else
                        {
                            if (!bin_img.empty())
                            {
                                base_find_f_tmp = basez.Base_find(&bin_img, mask_image_add, false);
                                // matfilesave("/home/pi/kyouyuu/LOG/base_bin_img", &bin_img);
                            }
                        }

                        //ベース平面傾き算出
                        //basez.base_yaw = -(1.54 - calculateAngle(basez.jyuushin,basez.baseCoordinateBase));
                        // printf(" aaa:%.2f\n",basez.ardn(basez.base_yaw));
                        if (base_find_f_tmp && !no_base_mode)
                            solve_pnp();
                    }
                    /*
                    if ((basez.camera_high > 180) || (basez.camera_high < 120))
                    {
                        base_find_f_tmp=false;
                        LOG_INFO.printf("カメラ高さエラー %.1f",basez.camera_high);
                    }*/
                    if (base_find_f_tmp) // 距離設定後にもう一度ゾーン設定するためbase_findコール
                    {
                        LOG_INFO_IF(!basez.zone_flg).printf("ホームベース検知 zone_flg:%d", basez.zone_flg);
                        if (!basez.zone_flg || ((basez.zone_c_dis_w_cm < (zone_c_dis_w_cm_old - 5)) || (basez.zone_c_dis_w_cm > (zone_c_dis_w_cm_old + 5))))
                        {
                            basez.zone_flg = true;

                            LOG_INFO.printf("ホームベース横位置:%.1fcm,カメラからの直線距離:%.1fcm,ゾーン下:%.1fcm,ゾーン上:%.1fcm", basez.zone_c_dis_w_cm, basez.straightLineDistanceToBase_cm, basez.zone_low_cm, basez.zone_hi_cm);
                            LOG_INFO.printf("ホームベース水平距離(dep):%d,カメラからの水平距離%.1fcm", basez.horizontalDistanceToBase_i, basez.horizontalDistanceToBase_cm);
                            LOG_INFO.printf("ホームベース角度(x-y):%.1f", basez.base_roll * 180 / 3.14);
                            LOG_INFO.printf("ホームベース角度(x-z):%.1f", basez.base_yaw * 180 / 3.14);
                            LOG_INFO.printf("検知距離 MAX,MIN:%.0f,%.0f", (basez.c_dis_cm(dep_base_en) + dep_l_en_bias), (basez.c_dis_cm(dep_base_kin) - dep_l_kin_bias));
                            if (no_base_mode)
                            {
                                LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("カメラピッチ %.2f°", basez.ardn(basez.camera.pitch));
                                LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("カメラ高さ  %.1fcm", basez.camera_high);
                                LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("カメラロール  %.2f°", basez.ardn(basez.camera.roll));
                                LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("カメラヨー  %.2f°", basez.ardn(basez.camera.yaw));
                            }
                            else
                            {
                                LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("カメラピッチ %.2f°", basez.ardn(basez.camera.pitch));
                                LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("カメラ高さ  %.1fcm", basez.camera_high);
                                LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("ベースロール  %.2f°", basez.ardn(basez.base_roll));
                                LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("ベースヨー  %.2f°", basez.ardn(basez.base_yaw));
                            }
                        }
                        zone_c_dis_w_cm_old = basez.zone_c_dis_w_cm;
                        // ベース中心の距離を表示
                        if (test_measurement_mode)
                        {
                            sprintf(value_c, "dep=%.1fcm(%.1fcm)", basez.horizontalDistanceToBase_cm, basez.horizontalDistanceToBase_cm);
                            cv::putText(r_image_SARA, value_c, cv::Point(220, 250), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 20, 20), 2);
                        }
                    }
                }
                else
                {                       // ベース検知できなかった場合
                    if (basez.zone_flg) //すでにゾーン検知されているか
                        calibration_start_t = get_sec() + 3;
                    else
                        calibration_start_t = get_sec() + 1; //ゼロにしたらだめ
                    straightLineDistanceToBase_i = 1;
                    basez.zone_flg = false;
                    LOG_VERBOSE_IF(calibration_start_t < (get_sec() + 1)).printf("base not find ");
                }
            }
            if (isHorizontalError && !((isWithinRange(basez.camera.pitch, 0.9, 1.4)) && (isWithinRange(basez.camera.roll, -0.18, 0.18))))
            {
                Judgment_mode_kaishimachi_cnt = 2;
                isHorizontalError = false;
            }
            else
            {
                isHorizontalError = !((isWithinRange(basez.camera.pitch, 0.9, 1.4)) && (isWithinRange(basez.camera.roll, -0.18, 0.18)));
            }

            // 判定開始初期処理
            if ((Judgment_start) && basez.zone_flg && !pr_flg)
            {
                //打者の身長に基づくゾーン高さ調整
                //if ((!bat_flg) && KneeDetectionOn)
                if (AutoJudghOff && KneeDetectionOn && bat_flg) // && rgb_frame_taken_img_key)
                {
                    // imwrite("bat.jpg", r_image_org);
                    // thread th_p(python3 pose_cpose);
                    // th_p.join();
                    // LOG_DEBUG.printf("python3 pose_cpp.py");
                    // int ret = system("python3 pose.py");
                    // 打者json作成
                    // 下記参照
                    // https://qiita.com/kokawa2003/items/bd6dccd7c2d5f1bf2773
                    // Pythonスクリプトに渡す引数（タプル）を設定
                    /*
                    PyObject *pUnicodeStr = PyUnicode_DecodeFSDefault("");
                    PyObject *pArgs = PyTuple_Pack(0, pUnicodeStr);
                    // RUN
                    */

                    /*pythonコールの初期化
                    このコードは、C++でPythonのコードを呼び出すための初期化処理を行っています。具体的には、NumPy配列を作成し、Pythonの関数に渡すためのタプルを作成しています。
                    まず、`r_image_SARA`という変数からチャンネル数を取得し、`npy_intp`型の配列`dimensions`を作成しています。この配列は、NumPy配列の次元とサイズを指定するために使用されます。次に、`r_image_SARA`のデータを使用して、`PyArray_SimpleNewFromData`関数を呼び出してNumPy配列を作成しています。
                    その後、タプルを作成するために`PyTuple_New`関数を使用しています。このタプルは、Pythonの関数に渡す引数を格納するために使用されます。ここでは、1つの要素を持つタプルを作成しています。その要素の位置0には、先ほど作成したNumPy配列`pdemo2`を設定しています。
                    このコードは、C++とPythonの間でデータをやり取りするための準備を行っています。このような処理は、C++でPythonの機能を利用する場合に一般的です。*/
                    int ch = r_image_org_shrink.channels();
                    npy_intp dimensions[3] = {r_image_org_shrink.rows, r_image_org_shrink.cols, ch};
                    int d = r_image_org_shrink.dims + 1;
                    PyObject *pdemo2 = PyArray_SimpleNewFromData(d, &dimensions[0], NPY_UINT8, r_image_org_shrink.data);
                    // Tupleの作成    -----------------------------------------------
                    PyObject *pArgs = PyTuple_New(1);  // Tupleを新しく作成する
                    PyTuple_SetItem(pArgs, 0, pdemo2); // Tupleの0の位置にpdemo2を設定
                    //---------------------------------------------------------------

                    // RUN
                    PyObject_CallObject(pFunc, pArgs);
                    Py_DECREF(pArgs);
                    //

                    // 身体の各部位位置変数を取得
                    int right_shoulder_visibility = getDictItemAsLong(global_dict, "right_shoulder_visibility");
                    int left_shoulder_visibility = getDictItemAsLong(global_dict, "left_shoulder_visibility");
                    // int right_hip_visibility = getDictItemAsLong(global_dict, "right_hip_visibility");
                    // int left_hip_visibility = getDictItemAsLong(global_dict, "left_hip_visibility");
                    // int right_knee_visibility = getDictItemAsLong(global_dict, "right_knee_visibility");
                    // int left_knee_visibility = getDictItemAsLong(global_dict, "left_knee_visibility");
                    bat_pose_flg = false;

                    if (                                                                    //
                        (right_shoulder_visibility > 70) || (left_shoulder_visibility > 70) //
                        // && (right_hip_visibility > 70) || (left_hip_visibility > 70) //
                        // && (right_knee_visibility > 70) || (left_knee_visibility > 70) //
                    )
                    {
                        int right_knee_z = getDictItemAsLong(global_dict, "right_knee_z");
                        int left_knee_z = getDictItemAsLong(global_dict, "left_knee_z");

                        int right_knee_x = getDictItemAsLong(global_dict, "right_knee_x");
                        int right_knee_y = getDictItemAsLong(global_dict, "right_knee_y");
                        int right_shoulder_x = getDictItemAsLong(global_dict, "right_shoulder_x");
                        int right_shoulder_y = getDictItemAsLong(global_dict, "right_shoulder_y");
                        int right_hip_x = getDictItemAsLong(global_dict, "right_hip_x");
                        int right_hip_y = getDictItemAsLong(global_dict, "right_hip_y");
                        int right_wrist_x = getDictItemAsLong(global_dict, "right_wrist_x");
                        int right_wrist_y = getDictItemAsLong(global_dict, "right_wrist_y");
                        int right_elbow_x = getDictItemAsLong(global_dict, "right_elbow_x");
                        int right_elbow_y = getDictItemAsLong(global_dict, "right_elbow_y");

                        int left_knee_x = getDictItemAsLong(global_dict, "left_knee_x");
                        int left_knee_y = getDictItemAsLong(global_dict, "left_knee_y");
                        int left_shoulder_x = getDictItemAsLong(global_dict, "left_shoulder_x");
                        int left_shoulder_y = getDictItemAsLong(global_dict, "left_shoulder_y");
                        int left_hip_x = getDictItemAsLong(global_dict, "left_hip_x");
                        int left_hip_y = getDictItemAsLong(global_dict, "left_hip_y");
                        int left_wrist_x = getDictItemAsLong(global_dict, "left_wrist_x");
                        int left_wrist_y = getDictItemAsLong(global_dict, "left_wrist_y");
                        int left_elbow_x = getDictItemAsLong(global_dict, "left_elbow_x");
                        int left_elbow_y = getDictItemAsLong(global_dict, "left_elbow_y");

                        if ((right_knee_z < left_knee_z) && right_bat_f)
                        {
                            knee_x = right_knee_x;
                            knee_y = right_knee_y;
                            shoulder_x = right_shoulder_x;
                            shoulder_y = right_shoulder_y;
                            hip_x = right_hip_x;
                            hip_y = right_hip_y;
                            wrist_x = right_wrist_x;
                            wrist_y = right_wrist_y;
                            elbow_x = right_elbow_x;
                            elbow_y = right_elbow_y;
                        }
                        else if ((right_knee_z > left_knee_z) && left_bat_f)
                        {
                            knee_x = left_knee_x;
                            knee_y = left_knee_y;
                            shoulder_x = left_shoulder_x;
                            shoulder_y = left_shoulder_y;
                            hip_x = left_hip_x;
                            hip_y = left_hip_y;
                            wrist_x = left_wrist_x;
                            wrist_y = left_wrist_y;
                            elbow_x = left_elbow_x;
                            elbow_y = left_elbow_y;
                        }
                        else
                        {
                            bat_pose_flg = false;
                            knee_x = 0;
                            knee_y = 0;
                            knee_point = Point2i(0, 0);
                        }
                        if (                                              // 立っている人か？
                            shoulder_y < 200 && (shoulder_y + 30 < hip_y) //
                            && (hip_y + 30 < knee_y)                      //
                            && (knee_y > 230) && (elbow_y < hip_y)        //
                            && (wrist_y < hip_y)                          //
                        )
                        {
                            // 手首が肘より上がっていなければ、もしくは肘が胸の高さより上がっていなければ投球待ちとみなさない
                            if (((wrist_y > (elbow_y - 10)) && (elbow_y > (shoulder_y + 10))) || wrist_y == 0 || elbow_y == 0 || shoulder_y == 0)
                            {
                                bat_pose_flg = false;
                                bat_pose_flg_cnt = 0;
                            }
                            else
                            {
                                if (bat_pose_flg_cnt > 2)
                                {
                                    bat_pose_flg = true;
                                    bat_pose_flg_cnt = 0;
                                    Judgment_mode_kaishimachi_cnt = 0;
                                    LOG_DEBUG.printf("打撃姿勢検知");
                                }
                                else
                                {
                                    bat_pose_flg_cnt++;
                                }
                            }
                            //
                            knee_point = Point(knee_x, knee_y);
                            shoulder_point = Point(shoulder_x, shoulder_y);
                            hip_point = Point(hip_x, hip_y);
                            wrist_point = Point(wrist_x, wrist_y);
                            elbow_point = Point(elbow_x, elbow_y);
                            // matfilesave("/home/pi/kyouyuu/LOG/bat_image", &frame_mono);
                            //if (bat_flg) //((knee_x > c_rect_x) && bat_flg)
                            //{
                            basez.knee_find(knee_x, knee_keisuu * knee_y, shoulder_x, shoulder_y, wrist_x, wrist_y, hip_x, hip_y, &frame_mono);
                            // basez.Base_find(&bin_img, mask_image_add, false);
                            LOG_DEBUG_IF(bat_pose_flg).printf("打者身長による調整");
                            //}
                        }
                        else
                        {
                            bat_pose_flg = false;
                            knee_x = 0;
                            knee_y = 0;
                            knee_point = Point2i(0, 0);
                        }
                    }
                    else
                    {
                        bat_pose_flg = false;
                        knee_x = 0;
                        knee_y = 0;
                        knee_point = Point2i(0, 0);
                    }
                    LOG_VERBOSE.printf("knee:x %d,%d", knee_x, knee_y);
                }
                else
                {
                    basez.batter_hight = 1;
                }

                //判定開始待ちカウントが０以下になったら判定開始
                //且つ、打者が検知されている場合＆＆打者姿勢検知モードが有効の場合
                //or 打者検知モードが無効だが、判定開始スイッチが押されている場合
                // LOG_DEBUG.printf(" KneeDetectionOff:%d,Judgment_mode_kaishimachi_cnt:%d,Judgment_mode:%d", KneeDetectionOff, Judgment_mode_kaishimachi_cnt, Judgment_mode);
                if (Judgment_mode_kaishimachi_cnt <= 0     //
                    && !Judgment_mode                      //
                    && ((bat_pose_flg && KneeDetectionOn)  //
                        || (JudghTriggerOn || AutoJudghOn) //
                        || (KneeDetectionOff && bat_flg)   //
                        )                                  //
                    && !isHorizontalError)
                {
                    usleep(500);
                    //! ゾーン検出後しばらく安定したらボール検出スタート
                    LOG_DEBUG.printf("Start_of_judgment:%d,basez.zone_flg:%d,Judgment_mode:%d", Start_of_judgment, basez.zone_flg, Judgment_mode);
                    LOG_DEBUG.printf("camera_yaw_mutch:%d", basez.camera_yaw_mutch);
                    if (basez.zone_flg && basez.camera_yaw_mutch && !Judgment_mode)
                    {
                        ball_clear_sleep = false;
                        ball_detection_cnt = 99; //ボールクリアを実行させるため
                        ball_clear();
                        basez.ball_noise_clear();
                        // solve_pnp();
                        //背景クリア
                        LOG_NONE.printf("--- judge start ---");
                        LOG_INFO.printf("dep_base:%d, %dcm", straightLineDistanceToBase_i, (int)basez.straightLineDistanceToBase_cm);
                        LOG_INFO.printf("kyori_base:%d, %dcm", horizontalDistanceToBase_i, (int)basez.horizontalDistanceToBase_cm);
                        LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("dep_base_en:%d, %dcm", dep_base_en, (int)basez.c_dis_cm(dep_base_en));
                        LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("dep_base_kin:%d, %dcm", dep_base_kin, (int)basez.c_dis_cm(dep_base_kin));
                        LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("dep_l_en:%d, %dcm", dep_l_en, (int)basez.c_dis_cm(dep_l_en));
                        LOG_INFO_IF(straightLineDistanceToBase_i != 1).printf("dep_l_kin:%d, %dcm", dep_l_kin, (int)basez.c_dis_cm(dep_l_kin));
                        // zone_print(&r_image_SARA); // ゾーン描画
                        paste(r_image_SARA, hanteityuu_png, 270, 100, 100, 50);
                        // rokuga_onoff(rec_bin, true);
                        // rokuga_onoff(rec_depth, true);
                        Judgment_mode = true;
                        ball_kiroku_machi = get_m_sec(); //noiseテーブル記録のために設定
                        ball_detection_phase = far_away;
                        basez.dot_hi_err = c_dot_hi_err;
                        Judgment_start = false;
                        // snap_shot = true; // 撮影開始
                        //ジャッジ開始の点灯
                        led(100);
                        motor(100);
                        // 間あけて、あらためて露出ロック
                        // dai::CameraControl ctrl_mono;
                        /* ロックするとモノカメラが不安定に？。。。
                        mono_read_check = false;
                        mono_read_lock = true;
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        ctrl_mono.setAutoFocusMode(dai::CameraControl::AutoFocusMode::OFF); //Members:    OFF    AUTO    MACRO    CONTINUOUS_VIDEO    CONTINUOUS_PICTURE    EDOF
                        ctrl_mono.setAutoExposureLock(true);
                        controlQueue_mono->send(ctrl_mono);
                        usleep(200);
                        ctrl_mono_B.setAutoFocusMode(dai::CameraControl::AutoFocusMode::OFF); //Members:    OFF    AUTO    MACRO    CONTINUOUS_VIDEO    CONTINUOUS_PICTURE    EDOF
                        ctrl_mono_B.setAutoExposureLock(true);
                        controlQueue_mono_B->send(ctrl_mono_B);
                        mono_read_lock = false;
                        mono_read_check = true;
                        */
                    }
                }
            }
        }

        // Judgment_start = false;  // 判定開始指示フラグ TRUE判定モードに変わる 判定モード開始されるとFALE
        syuuryousyori = false;                    // 終了処理の開始指示フラグ 各種フラグを初期化される
        if (AutoJudghOn && !AutoJudgh_sw_keyboad) // LOW->スイッチON
        {
            if (!Judgment_mode)
            {
                if ((basez.zone_flg) && (!Judgment_start))
                {
                    // ボール検知開始
                    Judgment_start = true;
                    Judgment_mode_kaishimachi_cnt = 2;
                    LOG_DEBUG.printf("AutoJudghOn:%d,Judgment_mode_kaishimachi_cnt:%d", AutoJudghOn, Judgment_mode_kaishimachi_cnt);
                }
            }
            if (!AutoJudgh_sw)
            {
                LOG_NONE.printf("自動測定モード auto sokutei ON!");
                AutoJudgh_sw = true;
                Judgment_mode_kaishimachi_cnt = 5;
                LOG_NONE.printf("AutoJudgh_sw_ON");
                printf("AutoJudgh_sw_ON:%d \n", digitalRead(AutoJudgh));
            }
        }
        else
        {
            if (AutoJudgh_sw)
            {
                LOG_NONE.printf("手動測定モード auto sokutei OFF");
                AutoJudgh_sw = false;
                Judgment_init();
                LOG_NONE.printf("AutoJudgh_sw_OFF");
                printf("AutoJudgh_sw_OFF \n");
            }
        }
        if JudghTriggerOn // LOW->スイッチON
        {
            if (!JudghTrigger_sw)
            {
                if (!Judgment_mode && !AutoJudgh_sw)
                {
                    if (basez.zone_flg)
                    {
                        Judgment_start = true;
                        Judgment_mode_kaishimachi_cnt = 0;
                    }
                }
                JudghTrigger_sw = true;
                LOG_NONE.printf("JudghTrigger_sw_ON");
                play_sound_func_2(0);
                printf("JudghTrigger_sw_ON \n");
            }
        }
        else // スイッチOFFの場合
        {
            if (JudghTrigger_sw)
            {
                if (!AutoJudgh_sw)
                {
                    ball_clear_sleep = true;
                    Judgment_machi_time = get_m_sec(); // - c_Judgment_wait;
                    Start_of_judgment = true;
                    LOG_NONE.printf("判定開始 hantei start ");
                    ball_zone_s = false;
                }
                JudghTrigger_sw = false;
                JudghTrigger_sw_on_t = get_sec();
                LOG_NONE.printf("JudghTrigger_sw_OFF");
                printf("JudghTrigger_sw_OFF \n");
                pr_time = pr_time - c_kekka_print_time - 1;

                if (!pr_flg)
                {
                    syuuryousyori = true;
                }
                // キャリブレーション開始
                if ((JudghTrigger_sw_on_t + 0.1) < get_sec())
                {
                    JudghTrigger_sw_on_t = DBL_MAX;
                    calibration_start_t = 0;
                }
            }
        }

        //! 判定＆結果表示
        if (Start_of_judgment)
        {
            //! 判定
            if (!(Judgment_machi_time <= get_m_sec())) // 0.*秒後に判定開始
            {
                if (!r_image_org_shrink_copyed)
                {
                    r_image_tmp.copyTo(r_image_SARA); // しばらく表示画像を判定結果表示画像に置き換え
                    r_image_org_shrink_copyed = true;
                    if (!snap_shot)
                        snap_shot = true; // カラー画像保存
                    continue;
                }
            }
            else
            {
                Judgment_machi_time = UINT64_MAX;
                Judgment_mode = false;
                ball_detection_phase = invalid;
                mono_read_check_f = false; //モノカメラ監視の対応
                mono_read_check_s = false; //モノカメラ監視の対応
                LOG_NONE.printf("hantei kaishi %d:", ball_cours_cnt - 1);
                zone_il_top = imread("homebase_yoko_no_zone.png"); // basezone.png"); // ゾーンイラスト サイドビュー

                //!最新カラー画像の取得(最大200msでexit)
                uint64_t check_t = get_m_sec();
                r_image_org_shrink.copyTo(r_image_tmp);
                //else
                //{
                //    r_image_SARA.copyTo(r_image_tmp);
                //}

                LOG_NONE.printf("find ball & start judgement");
                // rokuga_onoff(rec_bin, false);
                // rokuga_onoff(rec_depth, false);
                strike_last = false;

//撮影時間による直線回帰とハズレ値の除去
#if (true)
                std::vector<ball_cours> balls(std::begin(ball), std::end(ball));
                // カメラ0のデータに対してハズレ値検出
                detectOutliers(balls, 0);
                // カメラ1のデータに対してハズレ値検出
                detectOutliers(balls, 1);
                copy(begin(balls), end(balls), begin(ball)); // 退避
#endif

                // ボールの軌道を計算する
                // 軌道計算中にボール／ストライクの判定も実施する
                // 回帰直線算出用
                ball_cours ball_temp[256];
                float ball_x[256];
                float ball_y[256];
                int ball_dep[256];
                float ball_x_cm[256];
                float ball_y_cm[256];
                float ball_dep_cm[256];
                float ball_dep_cm2[256];
                uint64_t ball_time[256];

                copy(begin(ball), end(ball), begin(ball_temp)); // 退避
                int loop_dst;
                loop_dst = 0;
                for (int loop = dep_l_kin; loop > dep_l_en; loop--)
                {
                    if (isWithinRange(ball[loop].dep, 1, 254) && ball[loop].filter) //時間フィルターの判断追加
                    {
                        ball_dep_cm[loop_dst] = ball[loop].dep_cm;
                        ball_dep_cm2[loop_dst] = (float)(400 - ball[loop].dep_cm); // 画面上のｘ、ｙ座標は距離に対して逆数の関係になる
                        ball_x_cm[loop_dst] = ball[loop].w_cm;
                        ball_y_cm[loop_dst] = ball[loop].h_cm;
                        ball_dep[loop_dst] = ball[loop].dep;
                        ball_x[loop_dst] = (float)ball[loop].x;
                        ball_y[loop_dst] = (float)ball[loop].y;
                        ball_time[loop_dst] = ball[loop].time;
                        loop_dst++;
                    }
                    else
                    {
                        LOG_DEBUG_IF(ball[loop].x != 0).printf("depハズレerr_ball[%d].x:%d,ball[%d].y:%d,ball[%d].dep:%d,ball[%d].filter:%d", loop, ball[loop].x, loop, ball[loop].y, loop, ball[loop].dep, loop, ball[loop].filter);
                    }
                }

                // 球筋CSVデータ(raw)書き出し
                writeBallDataToCSV(ball, dep_l_kin, dep_l_en, 0);

                QuadraticCoefficients coeffX, coeffY;
                float per_hazure;
                robustQuadraticRegression(ball_dep_cm, ball_x_cm, ball_y_cm, loop_dst, coeffX, coeffY, &per_hazure);
                LOG_DEBUG.printf("ロバスト係数 X a:%.6f,b:%.3f,c:%.0f", coeffX.a, coeffX.b, coeffX.c);
                LOG_DEBUG.printf("ロバスト係数 Y a:%.6f,b:%.3f,c:%.0f", coeffY.a, coeffY.b, coeffY.c);
                LOG_DEBUG.printf("per_hazure:%.1f", per_hazure);
                if (per_hazure > 0.5)
                { // 終了処理
                    if (AutoJudgh_sw)
                        matfilesave("/home/pi/kyouyuu/LOG/ball_rireki4", &ball_rireki); // 途中失敗したときのボール映像
                    LOG_DEBUG.printf("ロバスト Error");
                    ball_clear();
                    // pr_time = get_sec();
                    // pr_flg = true;
                    syuuryousyori = true;
                    Start_of_judgment = false; // pr_flgをtrueにしてからでないと、ball_clearが実行されてしまう場合あり
                }
                else
                {
                    double xa_keisuu_t[3], xa_keisuu[3];
                    double ya_keisuu_t[3], ya_keisuu[3];
                    double xa_keisuu_cm_t[3], xa_keisuu_cm[3];
                    double ya_keisuu_cm_t[3], ya_keisuu_cm[3];
                    double keisuu_time_t[3], keisuu_time[3];

                    //
                    xa_keisuu_cm[0] = coeffX.c;
                    xa_keisuu_cm[1] = coeffX.b;
                    xa_keisuu_cm[2] = coeffX.a;
                    ya_keisuu_cm[0] = coeffY.c;
                    ya_keisuu_cm[1] = coeffY.b;
                    ya_keisuu_cm[2] = coeffY.a;

                    int dep_loop = 0;
                    int loop_init = 0;
                    double x_init = 0;
                    double y_init = 0;
                    double x_init_cm = 0;
                    double y_init_cm = 0;
                    double init_time = 0;
                    for (int loop = 0; loop < 256; loop++)
                    {
                        if (ball[loop].dep != 0 && ball[loop].dep != 999)
                        {
                            loop_init = loop;
                            x_init = ball[loop].hosei_x;
                            y_init = ball[loop].hosei_y;
                            x_init_cm = ball[loop].w_cm;
                            y_init_cm = ball[loop].h_cm;
                            init_time = ball[loop].time;
                            break;
                        }
                    }
                    int loop_dst2 = 0;
                    int dep_max = 0;
                    float dep_dst2_cm;
                    int dep_dst;
                    float w_bias = 0; // 5角柱の手前の範を補

                    // ワンバウンドチェック(ホームベースから70cm後ろでワンバウンドするようならゾーンを通過していてもボール判定する)
                    int hosyumae_x = 120;
                    int hosyumae_y = (int)ya_keisuu_cm[0] + ya_keisuu_cm[1] * (basez.straightLineDistanceToBase_cm - hosyumae_x) + ya_keisuu_cm[2] * ((basez.straightLineDistanceToBase_cm - hosyumae_x) * (basez.straightLineDistanceToBase_cm - hosyumae_x));
                    LOG_INFO.printf("捕球高さ:%d,kyori:%.1f", hosyumae_y, basez.straightLineDistanceToBase_cm - hosyumae_x);

                    float radius_temp = 0;
                    for (int loop = dep_l_en; loop < dep_l_kin; loop++)
                    {
                        // printf(" %d\n", loop);
                        ball[loop].cnt = 1;
                        dep_dst2_cm = basez.c_dis_cm(loop);
                        double dep_dst2_cm_temp = 400 - dep_dst2_cm;
                        x_init = xa_keisuu[0] + xa_keisuu[1] * dep_dst2_cm_temp + xa_keisuu[2] * (dep_dst2_cm_temp * dep_dst2_cm_temp);
                        ball[loop].hosei_x = (int)x_init;
                        y_init = ya_keisuu[0] + ya_keisuu[1] * dep_dst2_cm_temp + ya_keisuu[2] * (dep_dst2_cm_temp * dep_dst2_cm_temp);
                        ball[loop].hosei_y = (int)y_init;
                        x_init_cm = xa_keisuu_cm[0] + xa_keisuu_cm[1] * dep_dst2_cm + xa_keisuu_cm[2] * (dep_dst2_cm * dep_dst2_cm);
                        ball[loop].w_cm = x_init_cm;
                        y_init_cm = ya_keisuu_cm[0] + ya_keisuu_cm[1] * dep_dst2_cm + ya_keisuu_cm[2] * (dep_dst2_cm * dep_dst2_cm);
                        ball[loop].h_cm = (y_init_cm >= 0) ? y_init_cm : 0;
                        // init_time = keisuu_time[0] + keisuu_time[1] * dep_dst2_cm; // + keisuu_time[2] * (dep_dst2_cm * dep_dst2_cm);
                        ball[loop].time = ball_temp[loop].time;
                        ball[loop].dep_cm = dep_dst2_cm;
                        ball[loop].dep = loop;
                        ball[loop].z = ball_temp[loop].z;
                        if (ball_temp[loop].radius != 0) //radius補完のため
                            radius_temp = ball_temp[loop].radius;
                        ball[loop].radius = radius_temp;
                        ball[loop].strike = ball_temp[loop].strike;
                        ball[loop].ball_right = true; //
                        ball[loop].ball_left = true;  //
                        ball[loop].ball_lo = true;    //
                        ball[loop].ball_hi = true;    //
                        ball[loop].camera_no = ball_temp[loop].camera_no;

                        // ストライク判定
                        w_bias = 0;
                        if (basez.horizontalDistanceToBase_cm > dep_dst2_cm)
                        {
                            w_bias = clamp((basez.horizontalDistanceToBase_cm - dep_dst2_cm), 0, 22); // 5角柱の手前の範を補
                        }
                        // ストライク判定
                        if (isWithinRange(dep_dst2_cm, basez.c_dis_cm(basez.dep_base_kin), basez.c_dis_cm(basez.dep_base_en)))
                        {
                            // ルールより1cmストライクゾーンの幅を狭める
                            float zone_haba_bias = 3.6;                                                                // ストライクゾーンの幅を狭める
                            ball[loop].ball_right = ((c_zone_r_cm - w_bias + 3.6 - zone_haba_bias) < ball[loop].w_cm); // 左打者側に外れた
                            ball[loop].ball_left = ((c_zone_l_cm + w_bias - 3.6 + zone_haba_bias) > ball[loop].w_cm);  // 右打者側に外れた
                            ball[loop].ball_lo = ((basez.zone_low_cm - 3.6) > ball[loop].h_cm);                        // ボールロー
                            ball[loop].ball_hi = ((basez.zone_hi_cm + 3.6) < ball[loop].h_cm);                         // ボールハイ
                        }
                        else
                        {
                            ball[loop].ball_right = true; // 左打者側に外れた
                            ball[loop].ball_left = true;  // 右打者側に外れた
                            ball[loop].ball_lo = true;    // ボールロー
                            ball[loop].ball_hi = true;    // ボールハイ
                            ball[loop].strike = false;
                        }
                        if (ball[loop].ball_left || ball[loop].ball_lo || ball[loop].ball_right || ball[loop].ball_hi)
                        {
                            ball[loop].strike = false;
                        }
                        else
                        {
                            //if (dep_dst2 != loop_tmp)
                            ball[loop].strike = true;
                        }
                        // ワンバウンド想定時はボール
                        if (hosyumae_y < 1)
                        {
                            ball[loop].strike = false;
                            ball[loop].ball_lo = true;
                        }

                        if (loop > 254)
                            break;
                    }

                    // 判定結果の表示
                    r_image_tmp.copyTo(r_image_tmp_over);
                    int dep_last;
                    float x, y, x_hi, y_hi;
                    int radius_last = 0;
                    x_last = 0, y_last = 0;
                    bool zone_tsuuka = false;
                    // 初期はゾーンハズレにしておいて、ゾーン通過方向のみfalseにする
                    bool ball_hi = true;
                    bool ball_right = true;
                    bool ball_lo = true;
                    bool ball_left = true;

                    int old_dep_cm = 999;
                    x_hi = (float)(c_in_lo_x - c_out_hi_x) / 43.2;
                    if (basez.zone_hi_cm - basez.zone_low_cm != 0)
                        y_hi = ((float)(c_in_lo_y - c_out_hi_y) / (basez.zone_hi_cm - basez.zone_low_cm));
                    float ball_zone_dis_min = 9999999; // ゾーンからの最短距離
                    float ball_zone_dis;               // ゾーンからの距離
                    bool zone_print_f = false;
                    for (int loop = basez.c_dis_dep_i(450); loop < dep_l_kin; loop++)
                    {
                        // if (ball[loop].cnt == 999)
                        if (!isWithinRange(ball[loop].dep, 1, 254))
                        {
                            continue;
                        }
                        // 色設定
                        if (ball[loop].strike)
                        {
                            bcolor = Scalar(255, 255, 0);
                            strike_last = true;
                        }
                        else
                        {
                            bcolor = Scalar(128, 128, 255);
                        }
                        // ボール表示
                        if (true) //(!zone_tsuuka) // || (ball[loop].strike && !strike_last))
                        {
                            // if ((ball[loop].dep_cm <= basez.c_dis_cm(basez.dep_base_en)) && (ball[loop].dep_cm >= basez.c_dis_cm(basez.dep_base_kin)))
                            if ((ball[loop].dep_cm <= (basez.c_dis_cm(basez.dep_base_en) - 11)) && !zone_tsuuka) // (ball[loop].dep_cm >= (basez.c_dis_cm(basez.dep_base_kin) + 30)))
                            {
                                ball_zone_dis = sqrt((ball[loop].w_cm * ball[loop].w_cm) + ((ball[loop].h_cm - 70) * (ball[loop].h_cm - 70)));
                                if ((ball_zone_dis_min > ball_zone_dis)) // || (ball[loop].strike))
                                {
                                    radius_last = ball[loop].radius;
                                    center_last = basez.worldToScreen(ball[loop].w_cm, ball[loop].h_cm, ball[loop].dep_cm);
                                    center_last_3d = Dimensions(ball[loop].w_cm, ball[loop].h_cm, ball[loop].dep_cm);
                                    dep_last = ball[loop].dep;
                                    // ストライクゾーンイラスト座標への変換
                                    x = (float)ball[loop].w_cm;
                                    y = (float)ball[loop].h_cm - basez.zone_low_cm;
                                    x = x * x_hi;
                                    y = y * y_hi;
                                    x = c_center_x - x;
                                    y = c_in_lo_y - y;
                                    x_last = (int)x;
                                    y_last = (int)y;

                                    LOG_NONE.printf("ボール位置 横:%.1fcm,高さ:%.1fcm,直線距離:%.1fcm,水平距離%.1fcm", ball[loop].w_cm, ball[loop].h_cm, ball[loop].dep_cm, basez.horizontalDistanceToBase_cm);
                                    wHazureCm = 0;
                                    hHazureCm = 0;
                                    if ((21.6 + 3.6) < ball[loop].w_cm)
                                        wHazureCm = ball[loop].w_cm - (21.6 + 3.6);
                                    if (-(21.6 + 3.6) > ball[loop].w_cm)
                                        wHazureCm = ball[loop].w_cm + (21.6 + 3.6);
                                    if ((basez.zone_low_cm - 3.6) > ball[loop].h_cm) // ボールロー
                                        hHazureCm = ball[loop].h_cm - (basez.zone_low_cm - 3.6);
                                    if ((basez.zone_hi_cm + 3.6) < ball[loop].h_cm) // ボールハイ
                                        hHazureCm = ball[loop].h_cm - (basez.zone_hi_cm + 3.6);
                                    LOG_NONE.printf(" w外れ%.1f,h外れ%.1f,距離%.1f,ベース距離%.1f,判定%d", wHazureCm, hHazureCm, ball[loop].dep_cm, basez.horizontalDistanceToBase_cm, strike_last);
                                    zone_tsuuka = true; // ボールを連続表示するならコメントアウトする
                                }
                                ball_zone_dis_min = ball_zone_dis;
                            }
                        }
                        // ボール軌跡表示
                        // int circle_size = clamp((float)(ball[loop].dep - dep_l_en) / 15 + 2, 1, 30);
                        float temp_num = 600 - basez.c_dis_cm(ball[loop].dep);
                        int circle_size = (temp_num * temp_num) / 18000;
                        if ((ball[loop].dep_cm <= (old_dep_cm - 3)) && (ball[loop].dep_cm <= basez.c_dis_cm(dep_l_en)))
                        {
                            old_dep_cm = ball[loop].dep_cm;
                            Point2i ball_temp = basez.worldToScreen(ball[loop].w_cm, ball[loop].h_cm, ball[loop].dep_cm);
                            circle_size = clamp(circle_size - (int)(0.02 * (110 - ball[loop].h_cm)), 1, 30); // 低いボールは少し小さく
                            if (ball[loop].strike)
                            {
                                //ストライクゾーン内の表示
                                circle(r_image_tmp_over, ball_temp, circle_size, Scalar(80, 250, 250), -1, LINE_AA, 0);
                            }
                            else
                            {
                                //ボールの表示
                                circle(r_image_tmp_over, ball_temp, circle_size, Scalar(100, 180 + (basez.c_dis_dep_i(ball[loop].dep_cm) - dep_l_en), 100), -1, LINE_AA, 0);
                            }
                            //ストライクゾーンの表示
                            if (ball[loop].dep_cm < basez.straightLineDistanceToBase_cm && !zone_print_f)
                            {
                                zone_print(&r_image_tmp_over);
                                zone_print_f = true;
                            }
                        }
                        if (!ball[loop].ball_right)
                            ball_right = false;
                        if (!ball[loop].ball_left)
                            ball_left = false;
                        if (!ball[loop].ball_hi)
                            ball_hi = false;
                        if (!ball[loop].ball_lo)
                            ball_lo = false;
                    }

                    if ((center_last.x > 10) && (center_last.y > 10))
                    {
                        putTranspPng(r_image_tmp, ball_il, center_last.x - 10, center_last.y - 10, 20, 20);
                        putTranspPng(r_image_tmp_over, ball_il, center_last.x - 10, center_last.y - 10, 20, 20);
                    }

                    //ボール位置
                    strike_course = c_strike_center;
                    if (center_last_3d.h > basez.zone_hi_cm - 15)
                        strike_course = c_strike_hi;
                    if (center_last_3d.h < basez.zone_low_cm + 15)
                        strike_course = c_strike_lo;
                    if ((center_last_3d.w < -18 || center_last_3d.w > 18)) // && ball[loop].h_cm < basez.zone_low_cm -18)
                        strike_course = c_strike_nice;
                    /* ボールコール音声が準備できていないため
                    if (ball_right) // 左打者側に外れた
                        strike_course = c_ball_left;
                    if (ball_left) // 右打者側に外れた
                        strike_course = c_ball_right;
                    if (ball_lo) // ボールロー
                        strike_course = c_ball_lo;
                    if (ball_hi) // ボールハイ
                        strike_course = c_ball_hi;
                    */

                    bool il_reset = false;
                    if (strike_last)
                    {
                        LOG_NONE.printf("STRIKE!!! r:%d,x:%d,y:%d", radius_last, (int)center_last.x, (int)center_last.y);
                        LOG_NONE.printf(" strike_course:%d", strike_course);
                        bcolor = Scalar(0, 255, 255);
                        strike_cnt++;
                        if (strike_cnt > 2)
                        {
                            strike_cnt = 0;
                            ball_cnt = 0;
                            ball_no = 0;
                            il_reset = true;
                            out_cnt++;
                            if (out_cnt > 2)
                            {
                                out_cnt = 0;
                            }
                        }
                        putTranspPng(r_image_tmp, strike_png, 250, 30, 140, 50);
                        putTranspPng(r_image_tmp_over, strike_png, 250, 30, 140, 50);
                    }
                    else
                    {
                        LOG_NONE.printf("Ball---   r:%d,x:%d,y:%d,RiLeHiLo:%d,%d,%d,%d", radius_last, (int)center_last.x, (int)center_last.y, ball_right, ball_left, ball_hi, ball_lo);

                        int r_l = 60;
                        int h_l = 60;
                        if (ball_right)
                        {
                            ball_out_il = imread("yajirushi_r.png", -1);
                            putTranspPng(r_image_tmp, ball_out_il, center_last.x + 17, center_last.y - 17, 35, 35);
                            putTranspPng(r_image_tmp_over, ball_out_il, center_last.x + 17, center_last.y - 17, 35, 35);
                        }
                        if (ball_left)
                        {
                            ball_out_il = imread("yajirushi_l.png", -1);
                            putTranspPng(r_image_tmp, ball_out_il, center_last.x - 50, center_last.y - 17, 35, 35);
                            putTranspPng(r_image_tmp_over, ball_out_il, center_last.x - 50, center_last.y - 17, 35, 35);
                        }
                        if (ball_hi)
                        {
                            ball_out_il = imread("yajirushi_h.png", -1);
                            putTranspPng(r_image_tmp, ball_out_il, center_last.x - 17, center_last.y - 50, 35, 35);
                            putTranspPng(r_image_tmp_over, ball_out_il, center_last.x - 17, center_last.y - 50, 35, 35);
                        }
                        if (ball_lo)
                        {
                            ball_out_il = imread("yajirushi_lo.png", -1);
                            putTranspPng(r_image_tmp, ball_out_il, center_last.x - 17, center_last.y + 15, 35, 35);
                            putTranspPng(r_image_tmp_over, ball_out_il, center_last.x - 17, center_last.y + 15, 35, 35);
                        }
                        //外れ距離の表示
                        if (wHazureCm != 0 && (ball_left || ball_right))
                        {
                            sprintf(value_c, "%.1f", abs(wHazureCm)); // 変数の値も含めた表示したい文字列をchar型変数に格納
                            // cv::putText(r_image_tmp, value_c, cv::Point(center_last.x - 17, center_last.y - 17), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(28, 28, 255), 3);
                            cv::putText(r_image_tmp_over, value_c, cv::Point(center_last.x - 17, center_last.y - 17), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(28, 28, 255), 3);
                        }
                        int yBias = (hHazureCm > 0) ? 20 : -20;
                        if (hHazureCm != 0 && (ball_lo || ball_hi))
                        {
                            sprintf(value_c, "%.1f", abs(hHazureCm)); // 変数の値も含めた表示したい文字列をchar型変数に格納
                            // cv::putText(r_image_tmp, value_c, cv::Point(center_last.x - 17, yBias - 17), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(28, 28, 255), 3);
                            cv::putText(r_image_tmp_over, value_c, cv::Point(center_last.x - 17, center_last.y + yBias), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(28, 28, 255), 3);
                        }

                        bcolor = Scalar(0, 255, 0);
                        ball_cnt++;
                        if (ball_cnt > 3)
                        {
                            strike_cnt = 0;
                            ball_cnt = 0;
                            ball_no = 0;
                            il_reset = true;
                        }
                        putTranspPng(r_image_tmp, ball_png, 260, 30, 120, 50);
                        putTranspPng(r_image_tmp_over, ball_png, 260, 30, 120, 50);
                    }
                    addWeighted(r_image_tmp, 0.3, r_image_tmp_over, 0.7, 0, r_image_tmp);

                    // ゾーンイラストへの追加
                    if ((x_last > 5) && (y_last > 5))
                    {
                        LOG_VERBOSE.printf(" zone_il x:%d,y:%d", x_last, y_last);
                        zone_il_print(1, Point2i(x_last, y_last), strike_last);
                        zone_il_print(2);
                    }

                    // サイドビュー描画
                    float x_old = 0;
                    float y_old = 0;
                    int dep_old = 99;
                    int y_bias = 0;       // onaji kyori bias
                    int y_bias_base = 0;  // onaji kyori bias base
                    int doukyori_cnt = 0; // 同じ距離か
                    int y_old_cnt;
                    bool speed_keisoku_sumi = false;
                    int kyuusoku = 0;
                    float kyori = 0;
                    double jikan1844 = 0;
                    LOG_NONE.printf("ホームベース横位置:%.1fcm,カメラからの直線距離:%.1fcm,ゾーン下:%dcm,ゾーン上:%dcm", basez.zone_c_dis_w_cm, basez.straightLineDistanceToBase_cm, (int)basez.zone_low_cm, (int)basez.zone_hi_cm);
                    LOG_NONE.printf("ベース中心水平距離:%d,%.1f cm", horizontalDistanceToBase_i, horizontalDistanceToBase_cm);
                    LOG_NONE_IF(straightLineDistanceToBase_i != 1).printf("ベース投手側距離:%d,%.1f cm", dep_base_en, basez.c_dis_cm(dep_base_en));
                    LOG_NONE_IF(straightLineDistanceToBase_i != 1).printf("ベース捕手側距離:%d,%.1f cm", dep_base_kin, basez.c_dis_cm(dep_base_kin));
                    LOG_NONE.printf(" ball_cours,cnt,dep,dep_cm,  x,  y,  z,hosei_x,hosei_y, w_cm, h_cm,radius,strike,time,camera_no");
                    zone_il_top.copyTo(zone_il_top_over);

                    old_dep_cm = 0;
                    int speed_cnt = 999;
                    int speed_cnt_2 = 999;
                    float kyori_total = 0;
                    uint64_t time_sa_total = 0, time_sa = 0;
                    float kyori_total_old = 0;
                    int time_sa_tmp = 0;
                    uint64_t time_sa_total_old = 0;
                    float kyori_total_2 = 0;
                    uint64_t time_sa_total_2 = 0, time_sa_2 = 0;
                    float kyori_total_old_2 = 0;
                    int time_sa_tmp_2 = 0;
                    uint64_t time_sa_total_old_2 = 0;
                    for (int top_i = dep_l_kin; top_i >= basez.c_dis_dep_i(450); top_i--)
                    {
                        // if (ball[top_i].cnt != 999)
                        if (isWithinRange(ball[top_i].dep, 1, 254))
                        {
                            dep_old = ball[top_i].dep;
                            //if (ball[top_i].radius != 0)
                            //{
                            /*
                                if (ball[top_i].cnt == 0)
                                {
                                    y_bias = 0;
                                    y_bias_base = 0;
                                    y_old_cnt = 0;
                                }
                                else
                                {
                                    if (y_old_cnt != 0)
                                    {
                                        y_bias = ball[top_i].cnt * y_bias_base;
                                        y_old_cnt = ball[top_i].cnt;
                                    }
                                    else
                                    {
                                        y_old_cnt = ball[top_i].cnt;
                                        y_bias_base = (float)(basez.c_dis_cm(ball[top_i].dep - 1) - basez.c_dis_cm(ball[top_i].dep)) / (ball[top_i].cnt + 2);
                                        y_bias = ball[top_i].cnt * y_bias_base;
                                    }
                                }
                                */
                            float home_haba = 43.2;
                            float dot_p_cm_tate = (float)(c_base_haba_tate_d - c_base_haba_tate_u) / home_haba;
                            float dot_p_cm_yoko = (float)(c_base_haba_yoko_r - c_base_haba_yoko_l) / home_haba;
                            float top_y_sdw = c_base_center_y + (ball[top_i].w_cm * dot_p_cm_tate);        // サイドビュー（ボールシャドーのY座標）
                            float top_y = top_y_sdw - side_view_add_y(ball[top_i].h_cm, ball[top_i].w_cm); // サイドビュー（ボールのY座標）
                            float top_x = side_view_add_x(ball[top_i].dep_cm, ball[top_i].w_cm);           // サイドビュー（ボールのX座標）

                            LOG_NONE_IF(ball[top_i].camera_no != 0).printf(" ball_cours,  %1d,%3d, %3.1f,%03d,%03d,%03d,    %03d,    %03d,%03.1f,%03.1f,    %02d, %1d,%1d,%1d,%1d,%1d,%03" PRIu64 ",%1d,filter:%1d", //
                                                                           ball[top_i].cnt,
                                                                           ball[top_i].dep,
                                                                           ball[top_i].dep_cm, //
                                                                           ball[top_i].x,
                                                                           ball[top_i].y,
                                                                           ball[top_i].z, //
                                                                           ball[top_i].hosei_x,
                                                                           ball[top_i].hosei_y, //
                                                                           ball[top_i].w_cm,
                                                                           ball[top_i].h_cm, //
                                                                           ball[top_i].radius,
                                                                           ball[top_i].strike, //
                                                                           ball[top_i].ball_right,
                                                                           ball[top_i].ball_left,
                                                                           ball[top_i].ball_hi,
                                                                           ball[top_i].ball_lo,   //
                                                                           ball[top_i].time,      //
                                                                           ball[top_i].camera_no, //
                                                                           ball[top_i].filter     //
                            );
                            if (x_old == 0)
                                x_old = top_x;
                            if (y_old == 0)
                                y_old = top_y;
                            if (ball[top_i].strike)
                            {
                                bcolor = Scalar(20, 230, 230);
                                bcolor_shadow = Scalar(0, 40, 220);
                            }
                            else
                            {
                                bcolor = Scalar(20, 230, 20);
                                bcolor_shadow = Scalar(30, 30, 30);
                            }
                            x = ((float)top_x);
                            if ((top_y < top_y_sdw) && (ball[top_i].dep_cm >= (old_dep_cm + 10)))
                            {
                                old_dep_cm = ball[top_i].dep_cm;
                                circle(zone_il_top, Point((int)x, (int)top_y), dot_p_cm_yoko * 3.6, bcolor, -1, LINE_AA, 0);                 //
                                circle(zone_il_top_over, Point((int)x, (int)top_y), dot_p_cm_yoko * 3.6, bcolor, -1, LINE_AA, 0);            // 非透明
                                circle(zone_il_top_over, Point((int)x, (int)top_y_sdw), dot_p_cm_yoko * 3.6, bcolor_shadow, -1, LINE_AA, 0); // ベースのセンターの座標からどれだけ離れているかから座標算出
                            }
                            x_old = x;
                            y_old = top_y;

                            // snap速度算出
                            if (ball[top_i].camera_no == 1)
                            {
                                if (speed_cnt == 999)
                                {
                                    kyori_total_old = ball[top_i].dep_cm;
                                    time_sa_total_old = ball[top_i].time;
                                    speed_cnt = 0;
                                }
                                else
                                {
                                    time_sa_tmp = (int)(time_sa_total_old - ball[top_i].time);
                                    LOG_VERBOSE.printf(" time_sa_tmp:%d", time_sa_tmp);
                                    if ((time_sa_tmp != 0) && isWithinRange(time_sa_tmp, 5, 30))
                                    {
                                        kyori_total = kyori_total + (ball[top_i].dep_cm - kyori_total_old);
                                        LOG_VERBOSE.printf(" %.1f,%.1f,kyori sa(cm):%.1f", ball[top_i].dep_cm, kyori_total_old, (ball[top_i].dep_cm - kyori_total_old));
                                        time_sa_total = time_sa_total + time_sa_tmp;
                                        LOG_VERBOSE.printf(" %" PRIu64 ",%" PRIu64 ",timr sa:%" PRIu64, time_sa_total_old, ball[top_i].time, time_sa_tmp);
                                        speed_cnt++;
                                        LOG_VERBOSE.printf(" speed_cnt:%d", speed_cnt);
                                    }
                                    kyori_total_old = ball[top_i].dep_cm;
                                    time_sa_total_old = ball[top_i].time;
                                }
                            }
#if (double_camera == true)
                            if (ball[top_i].camera_no == 2)
                            {
                                if (speed_cnt_2 == 999)
                                {
                                    kyori_total_old_2 = ball[top_i].dep_cm;
                                    time_sa_total_old_2 = ball[top_i].time;
                                    speed_cnt_2 = 0;
                                }
                                else
                                {
                                    time_sa_tmp_2 = (int)(time_sa_total_old_2 - ball[top_i].time);
                                    LOG_DEBUG.printf(" time_sa_tmp_2:%d", time_sa_tmp_2);
                                    if ((time_sa_tmp_2 != 0) && isWithinRange(time_sa_tmp_2, 5, 30))
                                    {
                                        kyori_total = kyori_total + (ball[top_i].dep_cm - kyori_total_old_2);
                                        LOG_DEBUG.printf(" %.1f,%.1f,sa:%.1f", ball[top_i].dep_cm, kyori_total_old_2, (ball[top_i].dep_cm - kyori_total_old_2));
                                        time_sa_total = time_sa_total + time_sa_tmp_2;
                                        LOG_DEBUG.printf(" %" PRIu64 ",%" PRIu64 ",sa:%" PRIu64, time_sa_total_old_2, ball[top_i].time, time_sa_tmp_2);
                                        speed_cnt_2++;
                                        LOG_DEBUG.printf(" speed_cnt_2:%d", speed_cnt_2);
                                    }
                                    kyori_total_old_2 = ball[top_i].dep_cm;
                                    time_sa_total_old_2 = ball[top_i].time;
                                }
                            }

#endif
                            //}
                        }
                    }

                    // 速度算出
                    speed_cnt = (speed_cnt == 999) ? 0 : speed_cnt;
#if (double_camera == true)
                    speed_cnt_2 = (speed_cnt_2 == 999) ? 0 : speed_cnt_2;
                    kyori = abs(kyori_total / (speed_cnt + speed_cnt_2));
                    time_sa = abs(time_sa_total / (speed_cnt + speed_cnt_2));
                    LOG_DEBUG.printf(" speed_cnt:%d(%d + %d)", speed_cnt + speed_cnt_2, speed_cnt, speed_cnt_2);
#else
                    kyori = abs(kyori_total / speed_cnt);
                    time_sa = abs(time_sa_total / speed_cnt);
                    LOG_DEBUG.printf(" speed_cnt:%d", speed_cnt);
#endif
                    //LOG_DEBUG.printf(" e_time,first_time:%d,%d", e_time, first_time);
                    //LOG_DEBUG.printf(" kyori(e_dep(cm),s_dep(cm)):%.1f,%.1f", basez.c_dis_cm(e_dep), basez.c_dis_cm(s_dep));
                    if (kyori != 0)
                    {
                        kyuusoku = (int)(((kyori * sin(basez.camera.pitch)) / time_sa) * 36);
                        speed_keisoku_sumi = true;
                    }
                    LOG_NONE_IF(kyori != 0).printf(" kyori(cm) %.1f", kyori);
                    LOG_NONE_IF(kyori != 0).printf(" jikan sa(s) %d", time_sa);
                    LOG_NONE_IF(kyori != 0).printf(" SPEED(k) %d", kyuusoku);

                    // サイドビューのゾーン描画
                    drawsideviewZone();
                    addWeighted(zone_il_top, 0.3, zone_il_top_over, 0.7, 0, zone_il_top);

                    if (kyuusoku_keisoku)
                    {
                        if (kyuusoku != 0)
                        {
                            cv::rectangle(r_image_tmp, cv::Point(10, 10), cv::Point(210, 80), cv::Scalar(50, 50, 50), -1, 1);
                            sprintf(value_c, "%dk", kyuusoku); // 変数の値も含めた表示したい文字列をchar型変数に格納
                            cv::putText(r_image_tmp, value_c, cv::Point(30, 67), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 200, 200), 8);
                        }
                    }

                    pr_time = get_sec();
                    pr_flg = true;
                    Start_of_judgment = false; // pr_flgをtrueにしてからでないと、ball_clearが実行されてしまう場合あり
                    ball_no++;
                    if (!r_image_tmp.empty())
                    {
                        namedWindow("base & ball", cv::WND_PROP_FULLSCREEN);
                        setWindowProperty("base & ball", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
                        // イラスト画像のインポート
                        paste(r_image_tmp, zone_il, 0, 210, 150, 185);
                        // topビュー画像のインポート
                        paste(r_image_tmp, zone_il_top, 440, 213, 200, 187);
                        rectangle(r_image_tmp, Point(0, 210), Point(150 + 0, 185 + 210), Scalar(200, 200, 200), 3, 1);
                        rectangle(r_image_tmp, Point(440, 213), Point(200 + 440, 187 + 213), Scalar(200, 200, 200), 3, 1);
                        print_gui(r_image_tmp, true);
                        resize(r_image_tmp, resize_frame, Size(c_rez_x_r, c_rez_y_r));

                        imshow("base & ball", resize_frame);
                        wnd_dis_f_base = true;
                        // waitKey(1); //c_Judgment_wait + 100);
                        int led_a, led_b, led_c;
                        if (strike_last)
                        {
                            //strike call
                            led_a = 100;
                            led_b = 40;
                            led_c = 5;
                            motor(1000);
                        }
                        else
                        {
                            //ball call
                            led_a = 800;
                            led_b = 0;
                            led_c = 1;
                        }
                        led(led_a, led_b, led_c);
                        play_sound_f = true;
                        play_sound_func_2(1);
                        matfilesave("/home/pi/kyouyuu/LOG/r_image_SARA", &resize_frame);
                        matfilesave("/home/pi/kyouyuu/LOG/ball_rireki", &ball_rireki);
                        LOG_INFO.printf("play sound");

                        // 球筋CSVデータ書き出し
                        writeBallDataToCSV(ball, dep_l_kin, dep_l_en, 1);

                        if (il_reset)
                        {
                            zone_il_print(0);
                        } // ゾーンイラスト初期化
                    }
                }
                syuuryousyori = true;
                ball_clear_sleep = false;
            }
        }

        // 判定終了後 後片付け
        if (pr_flg)
        {
            if (((pr_time + 1) < get_sec()))
                pr_flg = false;
        }
        if (((pr_time + c_kekka_print_time) < get_sec()))
        {
#if (raspi == false) || (raspi == FALSE)
            if (!zone_il.empty())
            {
                imshow("zone il", zone_il);
            }
#endif

            if (getWindowProperty("base & ball", WND_PROP_VISIBLE) && wnd_dis_f_base)
            {
                destroyWindow("base & ball");
                wnd_dis_f_base = false;
                LOG_INFO.printf("hantei window DESTROY");
            }

#if (raspi == false) || (raspi == FALSE)
            if (getWindowProperty("zone il_top", WND_PROP_VISIBLE))
                destroyWindow("zone il_top");
#endif
        }

        if (syuuryousyori)
        {
            // 終了処理
            snap_shot = false;
            LOG_DEBUG.printf("hantei end process");
            Start_of_judgment = false;
            strike_last = false;
            Judgment_mode = false;
            ball_detection_phase = invalid;
            Judgment_init();
            basez.zone_flg = false;
            straightLineDistanceToBase_i = 1;
            ball_detection_f = false;
            zone_il_top = imread("homebase_yoko_no_zone.png"); // basezone.png"); // ゾーンイラスト トップビュー
            Judgment_mode_kaishimachi_cnt = 3;
            syuuryousyori = false;
            ball_clear_sleep = false;
            r_image_org_shrink_copyed = false;
            /* ↓ハングする可能性あり？
            mono_read_check = false;
            mono_read_lock = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ctrl_mono.setAutoExposureLock(false);
            // ctrl_mono.setAutoExposureEnable();
            controlQueue_mono->send(ctrl_mono);
            usleep(200);
            ctrl_mono_B.setAutoExposureLock(false);
            // ctrl_mono_B.setAutoExposureEnable();
            controlQueue_mono_B->send(ctrl_mono_B);
            autoex_mono = true;
            mono_read_lock = false;
            mono_read_check = true;
            */
        }

        // キャリブレーション
        if (!Judgment_mode && !no_base_mode)
        {
            LOG_VERBOSE.printf("calibration_start_t:%d,get_sec:%d,basez.zone_flg:%d", calibration_start_t, get_sec(), basez.zone_flg);
            // if (((calibration_start_t < get_sec()) && !basez.base_find_f) || (calibration_start_t == 0))
            if (calibration_start_t <= get_sec() || (calibration_start_t == 0))
            {
                LOG_VERBOSE_IF(basez.zone_flg).printf("calibration start");
                if (!mask_image_add.empty() && !bat_flg) //打者検知中はキャリブレーションしない
                {
                    calibrate = basez.Base_calibration2(&mask_image_add, &best_v_filter);
                }
                calibration_start_t = get_sec() + 3;
            }
        }

        // マウスイベント処理
        // 二度押しの抑止？
        if (click_time < (get_m_sec() - 100))
        {
            if ((mouse_x <= mouse_x_old + 5) && (mouse_y <= mouse_y_old + 5) && (mouse_x >= mouse_x_old - 5) && (mouse_y >= mouse_y_old - 5))
            {
                mouse_event = NULL;
            }
            else
            {
                mouse_x_old = 0;
                mouse_y_old = 0;
                click_time = get_m_sec();
            }
        }
        switch (mouse_event)
        {
        case EVENT_LBUTTONDOWN:
            LOG_DEBUG.printf("EVENT_LBUTTONDOWN %d,%d", mouse_x, mouse_y);
            break;
        case EVENT_LBUTTONUP:
            LOG_DEBUG.printf("EVENT_LBUTTONUP %d,%d", mouse_x, mouse_y);
            /*
            if ((mouse_x > 300) && (mouse_x < 500) && (mouse_y < 50)) // 真ん中上クリック
            {
                key = 'q'; // 終了
                isActJudgment_main = false;
                end_flg_Judgment_main_get = false;
                end_flg_rgb_read = false;
                end_flg_img_key = false;
                shutdown_f = true;
                LOG_DEBUG.printf("KEY = :q", key);
                break;
            }*/
            if ((mouse_x > 300) && (mouse_x < 500) && (mouse_y < 80)) // 真ん中少し下クリック
            {
                key = 'q'; // 終了
                LOG_DEBUG.printf("KEY = :q", key);
                break;
            }
            if ((mouse_x > 300) && (mouse_x < 500) && (mouse_y > 300)) // 真ん中下クリック
            {
                key = 'c'; // キャリブレーション
                LOG_DEBUG.printf("KEY = :c", key);
                break;
            }

            if ((mouse_x > 600) && (mouse_x < 800) && (mouse_y > 250)) // 右下クリック
            {
                key = ' '; // 判定モード
                LOG_DEBUG.printf("KEY = :' '", key);
                break;
            }
            if ((mouse_x < 200) && (mouse_y > 250)) // 左下クリック
            {
                key = 'd'; // 打者身長判定モード
                LOG_DEBUG.printf("KEY = :d", key);
                break;
            }
            switch (button_event(mouse_x, mouse_y))
            {
            case 0:
                LOG_DEBUG.printf("180cm-");
                basez.batter_hight = 0;
                break;
            case 1:
                LOG_DEBUG.printf("170cm-");
                basez.batter_hight = 1;
                break;
            case 2:
                LOG_DEBUG.printf("160cm-");
                basez.batter_hight = 2;
                break;
            case 3:
                LOG_DEBUG.printf("150cm-");
                basez.batter_hight = 3;
                break;
            case 4:
                LOG_DEBUG.printf("140cm-");
                basez.batter_hight = 4;
                break;
            case 5:
                LOG_DEBUG.printf("clear");
                ball_cnt = 0;
                strike_cnt = 0;
                ball_no = 0;
                // zone_il_top = imread("basezone.png"); // ゾーンイラストトップビュー初期化
                zone_il_top = imread("homebase_yoko_no_zone.png"); // basezone.png"); // ゾーンイラスト トップビュー
                break;
            case 6:
                LOG_DEBUG.printf("ball");
                ball_cnt++;
                if (ball_cnt > 3)
                {
                    ball_cnt = 0;
                    ball_no = 0;
                }
                break;
            case 7:
                LOG_DEBUG.printf("strike");
                strike_cnt++;
                if (strike_cnt > 2)
                {
                    strike_cnt = 0;
                    ball_no = 0;
                }
                break;
            case 8:
                LOG_DEBUG.printf("OUT");
                out_cnt++;
                if (out_cnt > 2)
                {
                    out_cnt = 0;
                    ball_no = 0;
                }
                break;
            case 9:
                LOG_DEBUG.printf("RESET");
                ball_cnt = 0;
                strike_cnt = 0;
                out_cnt = 0;
                ball_no = 0;
                // zone_il_top = imread("basezone.png"); // ゾーンイラストトップビュー初期化
                zone_il_top = imread("homebase_yoko_no_zone.png"); // basezone.png"); // ゾーンイラスト トップビュー
                break;
            default:
                break;
            }
            break;
        default:
            // mouse_x_old = 0;
            // mouse_y_old = 0;
            break;
        }
        mouse_event = 0;

        // キー入力
        if (key == ' ')
        {
            AutoJudgh_sw_keyboad = !AutoJudgh_sw_keyboad;
            LOG_DEBUG.printf("press space key AutoJudgh_sw:%d", AutoJudgh_sw_keyboad);
            if (Judgment_mode)
            {
                Judgment_init();
                AutoJudgh_sw = !AutoJudgh_sw;
            }
            else
            {
                // rokuga_onoff(rec_bin, true);
                // rokuga_onoff(rec_depth, true);
                ball_clear();
                solve_pnp();
                paste(r_image_SARA, hanteityuu_png, 270, 100, 100, 50);
                // Judgment_mode = true;
                Judgment_start = true;
                Judgment_mode_kaishimachi_cnt = 1;
                basez.dot_hi_err = c_dot_hi_err;
                // 間あけて、あらためて露出ロック
                // mono_read_lock = true;
                // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // dai::CameraControl ctrl_mono;
                // ctrl_mono.setAutoExposureLock(true);
                // controlQueue_mono->send(ctrl_mono);
                // mono_read_lock = false;
            }
        }
        if (key == 'r') //rndフィルター
            rnd_filter_f = (rnd_filter_f) ? false : true;
        if (key == 'd') // 打者身長検知モード
        {
            Judgment_init();
        }
        if (key == 'q' || key == 'Q')
        {
            exit_strike(true);
            break;
        }
        if (key == 'c' || key == 'C')
        {
            rgb_read_lock = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            controlQueue_Rgb->send(std::make_shared<dai::CameraControl>(ctrl_Rgb));
            rgb_read_lock = false;
            autoex_rgb = true;
            calibration_start_t = 0;
            if (key == 'c')
            {
                base_detection_maker = (base_detection_maker >= 4) ? 0 : (base_detection_maker + 1);
            }
            else
            {
                base_detection_maker = (base_detection_maker <= 0) ? 4 : (base_detection_maker - 1);
            }
            basez.base_detection_maker = base_detection_maker;
            printf("base_detection_maker:%d\n", base_detection_maker);
        }
        if (key == 't' || key == 'T')
        {
            camera_sel = (camera_sel == 2) ? 0 : camera_sel + 1;
            LOG_INFO.printf("camera_sel:%d", camera_sel);
        }
        //カメラ高さ
        if (key == '[' || key == '{')
        {
            basez.c_camera_high_no_base += key == '[';
            basez.c_camera_high_no_base -= key == '{';
            basez.camera.position = Eigen::Vector3f(0.0f, basez.c_camera_high_no_base, 0.0f); // カメラ位置を更新
        }
        if (key == 'i' || key == 'o' || key == 'k' || key == 'l')
        {
            Judgment_mode = false;
            ball_detection_phase = invalid;
            if (key == 'i')
                exp_time_mono -= EXP_STEP;
            exp_time_mono_B -= EXP_STEP;
            if (key == 'o')
                exp_time_mono += EXP_STEP;
            exp_time_mono_B += EXP_STEP;
            if (key == 'k')
                sens_iso_mono -= ISO_STEP;
            sens_iso_mono_B -= ISO_STEP;
            if (key == 'l')
                sens_iso_mono += ISO_STEP;
            sens_iso_mono_B += ISO_STEP;

            exp_time_mono = clamp(exp_time_mono, exp_min, exp_max); // clampir→値をann範囲内に収める関数
            sens_iso_mono = clamp(sens_iso_mono, sens_min, sens_max);
            exp_time_mono_B = clamp(exp_time_mono_B, exp_min, exp_max); // clampir→値をann範囲内に収める関数
            sens_iso_mono_B = clamp(sens_iso_mono_B, sens_min, sens_max);
            monocam_cng = true;
        }
        if (key == 'I' || key == 'O' || key == 'K' || key == 'L')
        {
            Judgment_mode = false;
            ball_detection_phase = invalid;
            if (key == 'I')
                exp_time_Rgb -= EXP_STEP;
            if (key == 'O')
                exp_time_Rgb += EXP_STEP;
            if (key == 'K')
                sens_iso_Rgb -= ISO_STEP;
            if (key == 'L')
                sens_iso_Rgb += ISO_STEP;
            exp_time_Rgb = clamp(exp_time_Rgb, exp_min, exp_max); // clampir→値をann範囲内に収める関数
            sens_iso_Rgb = clamp(sens_iso_Rgb, sens_min, sens_max);
            LOG_INFO.printf("Setting manual exposure Rgb , time: %d, iso: %d", exp_time_Rgb, sens_iso_Rgb);
            rgb_read_lock = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ctrl_Rgb.setManualExposure(exp_time_Rgb, sens_iso_Rgb);
            controlQueue_Rgb->send(std::make_shared<dai::CameraControl>(ctrl_Rgb));
            rgb_read_lock = false;
        }
        if (key == 'y' || key == 'u' || key == 'h' || key == 'j')
        {
            Judgment_mode = false;
            ball_detection_phase = invalid;
            if (key == 'y')
                basez.camera_high_key_bias -= 1;
            if (key == 'u')
                basez.camera_high_key_bias += 1;
            if (key == 'h')
                basez.base_kyori_cm_key_bias -= 1;
            if (key == 'j')
                basez.base_kyori_cm_key_bias += 1;

            LOG_INFO.printf("camera_high,horizontalDistanceToBase_cm:%d,%d", basez.camera_high + basez.camera_high_key_bias, basez.horizontalDistanceToBase_cm + basez.base_kyori_cm_key_bias);
        }
        if (key == 'v' || key == 'V')
        {
            Judgment_mode = false;
            ball_detection_phase = invalid;
            if (key == 'v')
                best_v_filter = best_v_filter - 2;
            if (key == 'V')
                best_v_filter = best_v_filter + 2;

            best_v_filter = clamp(best_v_filter, 0, 255); // clampir→値をann範囲内に収める関数
            LOG_INFO.printf("v filter %d", best_v_filter);
            calibration_start_t = get_sec() + 10;
        }
        if (key == 'w') // インフォメーション表示
        {
            info_print = !info_print;
        }
        if (key == 'W') // インフォメーション表示
        {
            info_print2 = !info_print2;
        }

        // 録画ON/OFF
        if (key == 'x' || key == 'z')
        {
            if (key == 'x')
            {
                // rokuga_onoff(rec_depth, true);f
                printf("depth rokuga mode on\n");
                v_rokuga_sw = true;
                rokuga_onoff(rec_depth, true);
                rokuga_onoff(rec_bin, true);
            }
            if (key == 'z')
            {
                rokuga_onoff(rec_bin, false);
                rokuga_onoff(rec_depth, false);
                rokuga_onoff(rec_BGR, false);
                v_rokuga_sw = false;
                printf("rokuga mode all off\n");
            }
        }
        // 録画ON/OFF
        if (key == 's' || key == 'a')
        {
            if (key == 's')
            {
                v_rokuga_sw = true;
                rokuga_onoff(rec_BGR, true);
            }
        }
        if (key == '4' || key == '6')
        {
            Judgment_mode = false;
            if (key == '4')
                focus_Rgb -= 2;
            if (key == '6')
                focus_Rgb += 2;
            focus_Rgb = clamp(focus_Rgb, 0, 255);
            rgb_read_lock = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ctrl_Rgb.setManualFocus(focus_Rgb);
            controlQueue_Rgb->send(std::make_shared<dai::CameraControl>(ctrl_Rgb));
            rgb_read_lock = false;
            printf("focus_Rgb %d", focus_Rgb);
        }
        if (key == '1' || key == '3')
        {
            Judgment_mode = false;
            ball_detection_phase = invalid;
            if (key == '1')
                focus_mono -= 5;
            if (key == '3')
                focus_mono += 5;
            focus_mono = clamp(focus_mono, 0, 255);
            // dai::CameraControl ctrl_mono;
            mono_read_lock_f = true; // mono画像取得済状態をロック
            mono_read_lock_s = true; // mono画像取得済状態をロック
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ctrl_mono.setManualFocus(focus_mono);
            controlQueue_mono_right->send(std::make_shared<dai::CameraControl>(ctrl_mono)); // V3
            controlQueue_mono_left->send(std::make_shared<dai::CameraControl>(ctrl_mono));  // V3
            ctrl_mono_B.setManualFocus(focus_mono);
    controlQueue_mono_B_right->send(std::make_shared<dai::CameraControl>(ctrl_mono)); // V3
    controlQueue_mono_B_left->send(std::make_shared<dai::CameraControl>(ctrl_mono));  // V3
    mono_read_lock_f = false;
    mono_read_lock_s = false;
    printf("focus_mono %d", focus_mono);
        }
        if (key == '2' || key == '8')
        {
            Judgment_mode = false;
            ball_detection_phase = invalid;
            if (key == '8')
                basez.knee++;
            if (key == '2')
                basez.knee--;
            basez.knee = clamp(basez.knee, 0, 120);
        }
        if (key == 'n' || key == 'N')
        {
            snap_shot_onoff = false;
            printf("snap shot off");
        }
        if (key == 'b')
        {
            if (!test_measurement_mode)
            {
                test_measurement_mode = true;
                LOG_DEBUG.printf("bin ON");
                basez.test_measurement_mode = true;
            }
            else
            {
                test_measurement_mode = false;
                basez.test_measurement_mode = false;
                LOG_DEBUG.printf("bin OFF");
                if (getWindowProperty("nama depth", WND_PROP_VISIBLE) && wnd_dis_f_mask)
                {
                    destroyWindow("nama depth");
                    wnd_dis_f_mask = false;
                }
                if (getWindowProperty("mask", WND_PROP_VISIBLE) && wnd_dis_f_nama_depth)
                {
                    destroyWindow("mask");
                    wnd_dis_f_nama_depth = false;
                }
            }
            binary_mask_distance_f = test_measurement_mode; // imshow("mask", binary_mask_distance);//距離２値画像
            frame_mono_f = test_measurement_mode;           // cv::imshow("nama depth", frame_mono);//生距離画像
        }
        /*
        if (key == 'B')
        {
            if (!confidenceMap_f)
            {
                confidenceMap_f = true;
                rectifiedLeft_f = true;
                printf("confidenceMap/rectifiedLeft ON\n");
            }
            else
            {
                confidenceMap_f = false;
                rectifiedLeft_f = false;
                printf("confidenceMap/rectifiedLeft OFF\n");
                if (getWindowProperty("rectifiedLeft", WND_PROP_VISIBLE) && wnd_dis_f_rectifiedLeft)
                {
                    destroyWindow("rectifiedLeft");
                    wnd_dis_f_rectifiedLeft = false;
                }
            }
        }
        */
        if (key == '5')
        {
            kyuusoku_keisoku != kyuusoku_keisoku;
        } // イメージ表示
        if (bin_img_f)
        {
            cvtColor(bin_img(Rect(40, 0, bin_img.cols - 80, bin_img.rows)), tmp_bin_img, COLOR_GRAY2BGR);
            // ベース検知画像（カラーからモノに変換後）の埋め込み
            if (((base_umekomi != 0) && test_measurement_mode)) // && !no_base_mode)
                paste(r_image_SARA, Mat(tmp_bin_img, Rect(0, 0, tmp_bin_img.cols, tmp_bin_img.rows - 50)), (int)c_rect_x + 40, (int)c_rect_zone_y, (int)(640 - (c_rect_x * 2)) - 80, (int)(400 - c_rect_zone_y - 50));
            if (base_center.x != 0)
            {
                if (no_base_mode)
                {
                    circle(r_image_SARA, base_center, 10, Scalar(0, 15, 255), 3, LINE_AA, 0); // ベース中心表示
                }
                else
                {
                    circle(r_image_SARA, base_center, 3, Scalar(0, 15, 255), 1, LINE_AA, 0); // ベース中心表示
                }
            }
            //  ベース中心線描画
            // line(r_image_SARA, Point(319, c_rect_zone_y), Point(319, 350), Scalar(100, 255, 100), 2, LINE_AA);
            // line(r_image_SARA, Point(294, c_rect_zone_y), Point(294, 350), Scalar(100, 255, 100), 1, LINE_AA);
            // line(r_image_SARA, Point(344, c_rect_zone_y), Point(344, 350), Scalar(100, 255, 100), 1, LINE_AA);
            // ベースの隅の基準点
            //
            float dx = 0.07;
            float dy = 0.13;
            if (!no_base_mode)
            {
                circle(r_image_SARA, basez.target_nw, 3, Scalar(250, 250, 250), 1, LINE_AA, 0);
                circle(r_image_SARA, basez.target_ne, 3, Scalar(250, 250, 250), 1, LINE_AA, 0);
                circle(r_image_SARA, basez.target_sw, 3, Scalar(250, 250, 250), 1, LINE_AA, 0);
                circle(r_image_SARA, basez.target_se, 3, Scalar(250, 250, 250), 1, LINE_AA, 0);
                circle(r_image_SARA, basez.target_bs, 3, Scalar(250, 250, 250), 1, LINE_AA, 0);
                circle(r_image_SARA, Point(basez.target_bs.x, (int)(basez.target_sw.y * 1.05)), 18, Scalar(250, 50, 50), 1, LINE_AA, 0);
            }
            //ベースエリア目印
            if (!Judgment_mode)
                rectangle(r_image_SARA, Point(250, (int)c_rect_zone_y), Point((640 - 250), 350), Scalar(100, 100, 100), 1, cv::LINE_4);
            // ゾーン輪郭
            if (!no_base_mode)
            {
                line(r_image_SARA, basez.baseCoordinateFront_r, basez.baseCoordinateFront_l, Scalar(255, 100, 100), 1, LINE_AA);
                line(r_image_SARA, basez.baseCoordinateFront_r, basez.baseCoordinateBack_r, Scalar(255, 100, 100), 1, LINE_AA);
                line(r_image_SARA, basez.baseCoordinateFront_l, basez.baseCoordinateBack_l, Scalar(255, 100, 100), 1, LINE_AA);
                line(r_image_SARA, basez.baseCoordinateBack_r, basez.baseCoordinateBase, Scalar(255, 100, 100), 1, LINE_AA);
                line(r_image_SARA, basez.baseCoordinateBack_l, basez.baseCoordinateBase, Scalar(255, 100, 100), 1, LINE_AA);
            }
            // 打者の膝,肩表示
            if (knee_point != Point(0, 0) && bat_flg)
            {
                circle(r_image_SARA, knee_point, 5, Scalar(50, 200, 200), 2, LINE_AA, 0);
                circle(r_image_SARA, shoulder_point, 5, Scalar(50, 200, 200), 2, LINE_AA, 0);
                circle(r_image_SARA, hip_point, 5, Scalar(50, 200, 200), 2, LINE_AA, 0);
                circle(r_image_SARA, wrist_point, 5, Scalar(50, 50, 200), 2, LINE_AA, 0);
                circle(r_image_SARA, elbow_point, 5, Scalar(50, 200, 200), 2, LINE_AA, 0);
                line(r_image_SARA, hip_point, knee_point, Scalar(30, 150, 150), 2, LINE_AA);
                line(r_image_SARA, hip_point, shoulder_point, Scalar(30, 150, 150), 2, LINE_AA);
                line(r_image_SARA, wrist_point, elbow_point, Scalar(30, 150, 150), 2, LINE_AA);
                line(r_image_SARA, elbow_point, shoulder_point, Scalar(30, 150, 150), 2, LINE_AA);
            }
            bin_img_f = false;
            zone_print(&r_image_SARA); // ゾーン描画
        }
        if (r_image_f)
        {
            if (!r_image_SARA.empty())
            {
                // フレームをリサイズ
                resize(r_image_SARA, resize_frame, Size(c_rez_x_r, c_rez_y_r));
                imshow("img-rinkaku", resize_frame);
                r_image_f = false;
                // ★★★ ホームベース部分の拡大表示機能の追加 ★★★
                if (!AutoJudgh_sw)
                {
                    int padding_x = 50; // ベース周囲に追加するパディング (ピクセル)
                    int padding_y = 30;

                    // ホームベースのROIを計算 (中心座標と幅/高さから)
                    int roi_x = static_cast<int>(basez.jyuushin.x - basez.base_w / 2 - padding_x);
                    int roi_y = static_cast<int>(basez.jyuushin.y - basez.base_h / 2 - padding_y);
                    int roi_width = basez.base_w + 2 * padding_x;
                    int roi_height = basez.base_h + 2 * padding_y;

                    // ROIが画像境界内にあることを確認し、調整
                    roi_x = std::max(0, roi_x);
                    roi_y = std::max(0, roi_y);
                    roi_width = std::min(roi_width, r_image_SARA.cols - roi_x);
                    roi_height = std::min(roi_height, r_image_SARA.rows - roi_y);

                    if (roi_width > 0 && roi_height > 0)
                    {
                        cv::Rect base_roi(roi_x, roi_y, roi_width, roi_height);
                        cv::Mat base_region = r_image_SARA(base_roi);
                        cv::Mat zoomed_base_frame;
                        cv::resize(base_region, zoomed_base_frame, cv::Size(640, 400)); // 640x400に拡大
                        cv::imshow("Zoomed Base", zoomed_base_frame);
                    }
                    else
                    {
                        // ROIが不正な場合、ウィンドウが開いていれば閉じる
                        if (cv::getWindowProperty("Zoomed Base", cv::WND_PROP_VISIBLE))
                        {
                            cv::destroyWindow("Zoomed Base");
                        }
                    }
                }
                else
                {
                    // ズーム表示OFFまたはベース未検出の場合、ウィンドウが開いていれば閉じる
                    if (cv::getWindowProperty("Zoomed Base", cv::WND_PROP_VISIBLE))
                    {
                        cv::destroyWindow("Zoomed Base");
                    }
                }
            }
        }
        if (frame_mono_f)
        {
            if (!frame_mono.empty() && !rnd_filter_syori_flg)
            {
                frame_mono.convertTo(tmp_frame_mono, 8);
                cv::imshow("nama depth", tmp_frame_mono);
                wnd_dis_f_mask = true;
            }
            frame_mono_f = false;
        }
        if (binary_mask_distance_f)
        {
            if (!binary_mask_distance.empty() && !rnd_filter_syori_flg)
            {
                imshow("mask", binary_mask_distance);
                wnd_dis_f_nama_depth = true;
            }
            binary_mask_distance_f = false;
        }
        /*
        if (!Judgment_mode && (confidenceMap_f || rectifiedLeft_f))
        {
            if (!frame_rectifiedLeft.empty())
                imshow("rectifiedLeft", frame_rectifiedLeft);
        }
        */
        // カラー画像を動画ファイルへ書き出す
        if (rokuga_flg)
        {
            if (!r_image_SARA.empty())
                writer2 << r_image_SARA;
        }
        key = cv::waitKey(1);
        if (key == key_old)
            key = 0;
        key_old = key;

        counter++;
        rgb_frame_taken_img_key = false;
    }

    // Pythonの終了
    Py_Finalize();
#if (raspi5 == true)
    // リソースを解放します
    gpiod_line_release(sw_5_AutoJudgh);
    gpiod_line_release(sw_6_JudghTrigger);
    gpiod_line_release(sw_26_LED_sw);
    gpiod_line_release(sw_20_MOTORSW);
    gpiod_line_release(sw_16_KneeDetection);
    gpiod_chip_close(chip);
#endif
    LOG_INFO.printf("END IMG KEY");
}
/**********************************************************************************/
