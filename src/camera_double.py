import cv2
import depthai as dai
import numpy as np

def create_pipeline(resolution):
    pipeline = dai.Pipeline()
    
    # ���m�N���J�����ƃX�e���I�[�x�ݒ�
    cam_left = pipeline.create(dai.node.MonoCamera)
    cam_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # �𑜓x�ݒ�
    if resolution == '720p':
        cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    elif resolution == '400p':
        cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    # �[�x�ݒ�
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    
    # �p�C�v���C���ڑ�
    cam_left.out.link(stereo.left)
    cam_right.out.link(stereo.right)
    
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName('depth')
    stereo.depth.link(xout_depth.input)

    return pipeline

# 2��̃f�o�C�X������
pipeline1 = create_pipeline('720p')
pipeline2 = create_pipeline('400p')
flg1=1
flg2=1

with dai.Device(pipeline1, usb2Mode=False) as device1, \
     dai.Device(pipeline2, usb2Mode=False) as device2:

    # �L���[�ݒ�
    q1 = device1.getOutputQueue(name='depth', maxSize=4, blocking=False)
    q2 = device2.getOutputQueue(name='depth', maxSize=4, blocking=False)

    # �ʒu�I�t�Z�b�g�ϐ�
    offset_x, offset_y = 0, 0

    def colorize_depth(depth_frame, colormap):
        depth_frame = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
        return cv2.applyColorMap(depth_frame.astype(np.uint8), colormap)

    def overlay_images(img1, img2):
        h, w = img1.shape[:2]
        canvas = img1.copy()
        
        # �I�t�Z�b�g�v�Z
        x_start = max(0, offset_x)
        y_start = max(0, offset_y)
        x_end = min(w, img2.shape[1] + offset_x)
        y_end = min(h, img2.shape[0] + offset_y)

        # �A���t�@�u�����f�B���O
        roi = canvas[y_start:y_end, x_start:x_end]
        img2_roi = img2[max(0, -offset_y):y_end-y_start, 
                       max(0, -offset_x):x_end-x_start]
        
        if roi.shape == img2_roi.shape:
            cv2.addWeighted(roi, 0.3, img2_roi, 0.7, 0, roi)
        
        return canvas

    while True:
        # �[�x�摜�擾
        depth1 = q1.get().getFrame()
        depth2 = q2.get().getFrame()

        # �𑜓x����
        # depth2 = cv2.resize(depth2, (depth1.shape[1], depth1.shape[0]))
        # ������640x400�Ƀ��T�C�Y
        depth1_resized = cv2.resize(depth1, (640, 400))
        depth2_resized = cv2.resize(depth2, (640, 400))
        depth2_flipped = cv2.flip(depth2_resized, -1)

        # �F�ϊ�
        color1 = colorize_depth(depth1_resized, cv2.COLORMAP_JET)
        color2 = colorize_depth(depth2_flipped, cv2.COLORMAP_HOT)

        # �摜�d�ˍ��킹
        overlay = overlay_images(color1, color2)
        cv2.imshow('Depth Overlay', overlay)
        '''
        if flg1==1:
            cv2.imshow('Depth Overlay', color1)
        if flg2==1:
            cv2.imshow('Depth Overlay', color2)
        '''

        # �L�[����
        key = cv2.waitKey(1)
        if key == ord('q'):  # ESC�ŏI��
            break
        elif key in [ord('w'), ord('s'), ord('a'), ord('d')]:
            offset_y += -1 if key == ord('w') else 1 if key == ord('s') else 0
            offset_x += -1 if key == ord('a') else 1 if key == ord('d') else 0
        elif key == ord('r'):
            flg1=1
            flg2=0
        elif key == ord('t'):
            flg1=0
            flg2=1

cv2.destroyAllWindows()
