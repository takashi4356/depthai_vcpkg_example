#include <opencv2/opencv.hpp>
#include <iostream>

int main(void)
{   
    // 動画ファイルのパスの文字列を格納するオブジェクトを宣言する
    std::string filepath = "Mountain.mp4";
	// 動画ファイルを取り込むためのオブジェクトを宣言する
	cv::VideoCapture video;
	// 動画ファイルを開く
	video.open(filepath);
	if (video.isOpened() == false) {
		// 動画ファイルが開けなかったときは終了する
		return 0;
	}
	cv::VideoWriter writer; // 動画ファイルを書き出すためのオブジェクトを宣言する
    
	int    width, height, fourcc; // 作成する動画ファイルの設定
   fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // ビデオフォーマットの指定( ISO MPEG-4 / .mp4)
	double fps;
	width  = (int)video.get(cv::CAP_PROP_FRAME_WIDTH);	// フレーム横幅を取得
	height = (int)video.get(cv::CAP_PROP_FRAME_HEIGHT);	// フレーム縦幅を取得
	fps    = video.get(cv::CAP_PROP_FPS);				// フレームレートを取得

	writer.open("CloneVideo.mp4", fourcc, fps, cv::Size(width, height));
    cv::Mat image ;// 画像を格納するオブジェクトを宣言する
	while (1) {
		video >> image; // videoからimageへ1フレームを取り込む
		if (image.empty() == true) break; // 画像が読み込めなかったとき、無限ループを抜ける
		cv::imshow("showing", image); // ウィンドウに動画を表示する
		writer << image;  // 画像 image を動画ファイルへ書き出す
		if (cv::waitKey(1) == 'q') break; //qを押すと終了
	}
       return 0;
}