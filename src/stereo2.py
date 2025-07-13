import cv2
import numpy as np
import time

import fcntl
import v4l2
import os

def set_camera_params(device='/dev/video0', width=640, height=240, pixelformat=v4l2.V4L2_PIX_FMT_MJPEG, fps=210):
    # デバイスをオープン
    fd = os.open(device, os.O_RDWR)

    # フォーマット設定
    fmt = v4l2.v4l2_format()
    fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    fmt.fmt.pix.width = width
    fmt.fmt.pix.height = height
    fmt.fmt.pix.pixelformat = pixelformat
    fmt.fmt.pix.field = v4l2.V4L2_FIELD_NONE
    fcntl.ioctl(fd, v4l2.VIDIOC_S_FMT, fmt)

    # フレームレート設定
    parm = v4l2.v4l2_streamparm()
    parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    parm.parm.capture.timeperframe.numerator = 1
    parm.parm.capture.timeperframe.denominator = fps
    fcntl.ioctl(fd, v4l2.VIDIOC_S_PARM, parm)

    # デバイスをクローズ
    os.close(fd)

    print(f"Camera set to {width}x{height} at {fps} fps")


# 設定を適用
# set_camera_params()

# カメラのインデックスを指定
camera_index = 0

# カメラをオープン
#cap = cv2.VideoCapture(camera_index)
cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)  # V4L2 を明示的に指定

# MJPG を指定
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

# 解像度と fps を設定
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 210)  # fps を明示的に設定
# カメラが正常にオープンされたか確認
if not cap.isOpened():
    print("カメラを開けませんでした")
    exit()

# フレームレート計測用の変数
frame_count = 0
start_time = time.time()
fps = 0

# SGBM（Semi-Global Block Matching）の設定
min_disparity = 0
num_disparities = 16 * 5  # 視差の範囲（16の倍数）
block_size = 3  # マッチングブロックサイズ（奇数）
stereo = cv2.StereoSGBM_create(
    minDisparity=min_disparity,
    numDisparities=num_disparities,
    blockSize=block_size,
    P1=8 * 3 * block_size**2,  # スムージングパラメータ1
    P2=32 * 3 * block_size**2,  # スムージングパラメータ2
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

# 距離計算用のパラメータ（調整が必要）
focal_length = 800.0  # ピクセル単位の焦点距離（カメラキャリブレーションで取得）
baseline = 7.5  # ステレオカメラのベースライン（cm）

while True:
    # フレームを取得
    ret, frame = cap.read()

    if not ret:
        print("フレームの取得に失敗しました")
        break

    # フレームを左右に分割
    height, width = frame.shape[:2]
    mid = width // 2
    frame_left = frame[:, :mid]
    frame_right = frame[:, mid:]

    # グレースケールに変換
    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)


    # 視差マップを計算
    disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

    # 視差マップを正規化して表示用に変換
    disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    disparity_normalized = disparity_normalized.astype(np.uint8)

    # 距離画像を計算（ゼロ除算を避けるために条件付きで計算）
    depth_map = np.zeros_like(disparity, dtype=np.float32)
    mask = disparity > min_disparity
    depth_map[mask] = (focal_length * baseline) / disparity[mask]

    depth_normalized = cv2.normalize(depth_map, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    depth_normalized = depth_normalized.astype(np.uint8)


    # フレームレート計測
    frame_count += 1
    current_time = time.time()
    if (current_time - start_time) >= 1.0:
        fps = frame_count / (current_time - start_time)
        frame_count = 0
        start_time = current_time

    # FPSを視差マップに描画
    cv2.putText(disparity_normalized, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    #print(f"FPS: {fps:.2f}")

    # 結果を表示
    cv2.imshow("Left Camera", gray_left)
    cv2.imshow("Right Camera", gray_right)
    cv2.imshow("Disparity Map", disparity_normalized)
    
    cv2.imshow("Depth Map", depth_normalized)

    # 'q'キーで終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# リソースを解放
cap.release()
cv2.destroyAllWindows()
