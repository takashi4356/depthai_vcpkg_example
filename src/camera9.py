#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)

xout.setStreamName("disparity")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutRgb = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")

# Properties
camRgb.setPreviewSize(640, 400)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

# Linking
camRgb.preview.link(xoutRgb.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queue will be used to get the disparity frames from the outputs defined above
    q = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

    # Output queue will be used to get the rgb frames from the output defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    # 重ね合わせる位置を指定（ここではframeの中央に配置）
    x_offset = 75
    y_offset = 35
    scale_percent_h = (480 / 640) * 100  # 50%にリサイズ
    scale_percent_v = (320 / 400) * 100  # 50%にリサイズ

    while True:
        inDisparity = q.get()  # blocking call, will wait until a new data has arrived
        frame = inDisparity.getFrame()
        # Normalization for better visualization
        #frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)
        #cv2.imshow("disparity", frame)

        frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        cv2.imshow("disparity_color", frame)

        inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived

        # Retrieve 'bgr' (opencv format) frame
        frame2 = inRgb.getCvFrame()
        #cv2.imshow("rgb", frame2)

        # frame2をリサイズ
        width = int(frame2.shape[1] * scale_percent_h / 100)
        height = int(frame2.shape[0] * scale_percent_v / 100)
        dim = (width, height)

        # 画像のリサイズ
        frame2_resized = cv2.resize(frame2, dim, interpolation=cv2.INTER_AREA)
        cv2.imshow("rgb", frame2_resized)

        # ROI（Region of Interest）を設定
        roi = frame[y_offset:y_offset+frame2_resized.shape[0], x_offset:x_offset+frame2_resized.shape[1]]

        # アルファブレンディングの割合を設定
        alpha = 0.8
        beta = 1.0 - alpha

        # 画像を重ね合わせる
        blended_part = cv2.addWeighted(roi, alpha, frame2_resized, beta, 0.0)

        # 重ね合わせた部分を元の画像に戻す
        frame[y_offset:y_offset+frame2_resized.shape[0], x_offset:x_offset+frame2_resized.shape[1]] = blended_part

        # 結果を表示
        cv2.imshow('Blended Image', frame)

        key = cv2.waitKey(10)
        if key == ord('q'):
            break
        if key == ord("4"):
            x_offset = x_offset-1
        if key == ord("6"):
            x_offset = x_offset+1
        if key == ord("8"):
            y_offset = y_offset-1
        if key == ord("2"):
            y_offset = y_offset+1
        if key == ord("a"):
            scale_percent_h = scale_percent_h-1
        if key == ord("d"):
            scale_percent_h = scale_percent_h+1
        if key == ord("w"):
            scale_percent_v = scale_percent_v-1
        if key == ord("s"):
            scale_percent_v = scale_percent_v+1
    print("x_offset",x_offset)
    print("y_offset",y_offset)
    print("scale_percent_h",scale_percent_h,scale_percent_h*6.4)
    print("scale_percent_v",scale_percent_v,scale_percent_v*4)
