import depthai as dai
import cv2

# パイプラインの作成
pipeline = dai.Pipeline()

# カメラノードの設定
cam = pipeline.create(dai.node.MonoCamera)
cam.setBoardSocket(dai.CameraBoardSocket.LEFT)  # 左側モノクロカメラを選択
cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# フォーカス制御の設定
control = pipeline.create(dai.node.XLinkIn)
control.setStreamName('control')

# カメラコントロールリンク
cam.initialControl.setManualFocus(250)  # 初期フォーカス値（範囲: 0-255）
control.out.link(cam.inputControl)

# 出力設定
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName('video')
cam.out.link(xout.input)

# デバイス接続
with dai.Device(pipeline) as device:
    control_queue = device.getInputQueue('control')
    video_queue = device.getOutputQueue('video', maxSize=4, blocking=False)

    # フォーカス値変更の例（実行中に動的に変更可能）
    def set_focus(value: int):
        ctrl = dai.CameraControl()
        ctrl.setManualFocus(value)
        control_queue.send(ctrl)

    # 初期フォーカス設定（例: 150）
    _focus=250
    set_focus(_focus)

    while True:
        frame = video_queue.get().getCvFrame()
        cv2.imshow("OAK-D Mono Camera", frame)

        # キー操作でフォーカス調整
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('+'):
            _focus=min(255,_focus+10)
            set_focus(_focus)
        elif key == ord('-'):
            _focus=max(0,_focus-10)
            set_focus(_focus)
