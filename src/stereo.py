import cv2
import depthai as dai
import numpy as np

# DepthAIパイプラインの設定
pipeline = dai.Pipeline()

# 左右モノカメラの設定
mono_left = pipeline.create(dai.node.MonoCamera)
mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

mono_right = pipeline.create(dai.node.MonoCamera)
mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# 出力の設定
xout_left = pipeline.create(dai.node.XLinkOut)
xout_left.setStreamName("left")
mono_left.out.link(xout_left.input)

xout_right = pipeline.create(dai.node.XLinkOut)
xout_right.setStreamName("right")
mono_right.out.link(xout_right.input)

# ROIの定義
x, y, w, h = 192, 0, 256, 400
roi = (x, y, w, h)

# デバイスに接続してパイプラインを開始
with dai.Device(pipeline) as device:
    # 出力キューの取得
    q_left = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    q_right = device.getOutputQueue(name="right", maxSize=4, blocking=False)

    numDisparities_b=16*3
    # ステレオマッチングの設定
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=numDisparities_b,
        blockSize=5,
        P1=8 * 5 * 0**2,
        P2=32 * 5 * 0**2,
        #P1=8 * 3 * 5**2,
        #P2=32 * 3 * 5**2,
        disp12MaxDiff=5,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=128,
        mode=cv2.STEREO_SGBM_MODE_SGBM
        #disp12MaxDiff	1-10	左右視差チェックの許容誤差
        #spreFilterCap	1-63	前処理フィルタのクリップ値
        #uniquenessRatio	5-15	マッチングの一意性を保証する比率
        # speckleWindowSize = 100  # スぺキュル除去ウィンドウサイズ
        # speckleRange = 32       # スぺキュル判定範囲
        # mode=cv2.STEREO_SGBM_MODE_HH4
        # mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        # cv2.StereoSGBM_MODE_HH = 1  # 高精度モード（上下方向も考慮）
        # cv2.StereoSGBM_MODE_SGBM = 0  # 基本モード
    )

    while True:
        # 左右の画像を取得
        in_left = q_left.get()
        in_right = q_right.get()

        frame_left = in_left.getCvFrame()
        frame_right = in_right.getCvFrame()

        # ステレオマッチングを実行
        disparity = stereo.compute(frame_left[y:y+h, x-numDisparities_b:x+w+numDisparities_b], frame_right[y:y+h, x-numDisparities_b:x+w+numDisparities_b])[y:y+h, numDisparities_b:numDisparities_b+w]

        # 視差を正規化して表示用に変換
        disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # 距離情報の計算
        focal_length = 389  # ピクセル単位の焦点距離（カメラに依存）
        baseline = 7.5  # センチメートル単位のステレオカメラのベースライン

        # 視差から距離を計算（ゼロ除算を避ける）
        #depth = np.zeros_like(disparity, dtype=np.float32)
        #mask = disparity > 0
        #depth[mask] = (focal_length * baseline) / disparity[mask]

        # 結果の表示
        cv2.imshow("Left Camera", frame_left)
        cv2.imshow("Disparity", disparity_normalized)
        #cv2.imshow("Depth", depth / np.max(depth))

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
