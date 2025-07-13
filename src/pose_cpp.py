import cv2
import os
import sys
#import daemon
import mediapipe as mp

'''
このコードは、与えられた画像から人のポーズを検出し、ポーズの関節位置と可視性をJSONファイルに書き込む関数Poseを定義しています。以下にコードの概要を説明します。
まず、bat.jsonという名前のファイルを書き込みモードで開きます。次に、mp_pose.Poseを使用してポーズ検出を行います。min_detection_confidenceとmin_tracking_confidenceは、検出と追跡の信頼度の閾値を設定するパラメータです。
画像の書き込み可能フラグを無効にし、画像をRGB形式に変換します。そして、pose.processを使用してポーズ検出を実行し、結果をresultsに格納します。
次に、mp_drawing.draw_landmarksを使用して、検出されたポーズの骨格を画像上に描画します。この関数は、画像、ポーズのランドマーク、およびポーズの接続情報を引数として受け取ります。
results.pose_landmarksが存在する場合、各関節の位置と可視性をJSON形式のテキストに追加します。例えば、results.pose_landmarks.landmark[25]は左膝の関節を表し、そのx座標、y座標、z座標、可視性を計算してテキストに追加します。
最後に、テキストをファイルに書き込み、ファイルを閉じます。
コメントアウトされたコードは、ポーズの関節位置を画像上に円で表示するためのものです。必要に応じてコメントを外して使用することができます。
この関数は、与えられた画像からポーズの関節位置を抽出し、JSONファイルに保存するためのものです。
'''
def Pose(image):
    #f = open('bat.json', 'w') 
    jikkou_flg=False
    # Webカメラから入力
    #cap = cv2.VideoCapture(0)
    global right_knee_visibility
    global left_knee_visibility
    global right_shoulder_visibility
    global left_shoulder_visibility
    global right_knee_z
    global left_knee_z

    global right_knee_x
    global right_knee_y
    global right_shoulder_x
    global right_shoulder_y
    global right_hip_x
    global right_hip_y
    global right_wrist_x
    global right_wrist_y
    global right_elbow_x
    global right_elbow_y

    global left_knee_x
    global left_knee_y
    global left_shoulder_x
    global left_shoulder_y
    global left_hip_x
    global left_hip_y
    global left_wrist_x
    global left_wrist_y
    global left_elbow_x
    global left_elbow_y

    with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:
    #  while cap.isOpened():
    #    success, image = cap.read()
        #image = cv2.imread('bat.jpg')
        #cv2.imshow('MediaPipe Pose', image)
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)
        # 画像をファイルとして書き出し
        # cv2.imwrite('bat.jpg', image)
        
        # 検出されたポーズの骨格をカメラ画像に重ねて描画
        #image.flags.writeable = True
        #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS
        )
        #,landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        #print (image.shape[1])
        #print (image.shape[0])
        #text=""
        if results.pose_landmarks:
            right_knee_visibility = int(results.pose_landmarks.landmark[26].visibility*100)
            left_knee_visibility = int(results.pose_landmarks.landmark[25].visibility*100)
            right_shoulder_visibility = int(results.pose_landmarks.landmark[12].visibility*100)
            left_shoulder_visibility = int(results.pose_landmarks.landmark[11].visibility*100)
            right_knee_z = int(results.pose_landmarks.landmark[26].z*100)
            left_knee_z = int(results.pose_landmarks.landmark[25].z*100)

            right_knee_x = int(results.pose_landmarks.landmark[26].x*image.shape[1])
            right_knee_y = int(results.pose_landmarks.landmark[26].y*image.shape[0])
            right_shoulder_x = int(results.pose_landmarks.landmark[12].x*image.shape[1])
            right_shoulder_y = int(results.pose_landmarks.landmark[12].y*image.shape[0])
            right_hip_x = int(results.pose_landmarks.landmark[24].x*image.shape[1])
            right_hip_y = int(results.pose_landmarks.landmark[24].y*image.shape[0])
            right_wrist_x = int(results.pose_landmarks.landmark[16].x*image.shape[1])
            right_wrist_y = int(results.pose_landmarks.landmark[16].y*image.shape[0])
            right_elbow_x = int(results.pose_landmarks.landmark[14].x*image.shape[1])
            right_elbow_y = int(results.pose_landmarks.landmark[14].y*image.shape[0])

            left_knee_x = int(results.pose_landmarks.landmark[25].x*image.shape[1])
            left_knee_y = int(results.pose_landmarks.landmark[25].y*image.shape[0])
            left_shoulder_x = int(results.pose_landmarks.landmark[11].x*image.shape[1])
            left_shoulder_y = int(results.pose_landmarks.landmark[11].y*image.shape[0])
            left_hip_x = int(results.pose_landmarks.landmark[23].x*image.shape[1])
            left_hip_y = int(results.pose_landmarks.landmark[23].y*image.shape[0])
            left_wrist_x = int(results.pose_landmarks.landmark[15].x*image.shape[1])
            left_wrist_y = int(results.pose_landmarks.landmark[15].y*image.shape[0])
            left_elbow_x = int(results.pose_landmarks.landmark[13].x*image.shape[1])
            left_elbow_y = int(results.pose_landmarks.landmark[13].y*image.shape[0])

            '''
            text = ("{\n")
            text = text + ("\"left_knee_x\":"+str(int(results.pose_landmarks.landmark[25].x*image.shape[1]))+",\n")
            text = text + ("\"left_knee_y\":"+ str(int(results.pose_landmarks.landmark[25].y*image.shape[0]))+ ",\n")
            text = text + ("\"left_knee_z\":"+ str(int(results.pose_landmarks.landmark[25].z*100))+ ",\n")
            text = text + ("\"left_knee_visibility\":"+ str(int(results.pose_landmarks.landmark[25].visibility*100))+ ",\n")
            text = text + ("\"right_knee_x\":"+ str(int(results.pose_landmarks.landmark[26].x*image.shape[1]))+ ",\n")
            text = text + ("\"right_knee_y\":"+ str(int(results.pose_landmarks.landmark[26].y*image.shape[0]))+ ",\n")
            text = text + ("\"right_knee_z\":"+ str(int(results.pose_landmarks.landmark[26].z*100))+ ",\n")
            text = text + ("\"right_knee_visibility\":"+ str(int(results.pose_landmarks.landmark[26].visibility*100))+",\n")
            text = text + ("\"left_hip_x\":"+str(int(results.pose_landmarks.landmark[23].x*image.shape[1]))+",\n")
            text = text + ("\"left_hip_y\":"+ str(int(results.pose_landmarks.landmark[23].y*image.shape[0]))+ ",\n")
            text = text + ("\"left_hip_z\":"+ str(int(results.pose_landmarks.landmark[23].z*100))+ ",\n")
            text = text + ("\"left_hip_visibility\":"+ str(int(results.pose_landmarks.landmark[23].visibility*100))+ ",\n")
            text = text + ("\"right_hip_x\":"+ str(int(results.pose_landmarks.landmark[24].x*image.shape[1]))+ ",\n")
            text = text + ("\"right_hip_y\":"+ str(int(results.pose_landmarks.landmark[24].y*image.shape[0]))+ ",\n")
            text = text + ("\"right_hip_z\":"+ str(int(results.pose_landmarks.landmark[24].z*100))+ ",\n")
            text = text + ("\"right_hip_visibility\":"+ str(int(results.pose_landmarks.landmark[24].visibility*100))+",\n")
            text = text + ("\"left_shoulder_x\":"+str(int(results.pose_landmarks.landmark[11].x*image.shape[1]))+",\n")
            text = text + ("\"left_shoulder_y\":"+ str(int(results.pose_landmarks.landmark[11].y*image.shape[0]))+ ",\n")
            text = text + ("\"left_shoulder_z\":"+ str(int(results.pose_landmarks.landmark[11].z*100))+ ",\n")
            text = text + ("\"left_shoulder_visibility\":"+ str(int(results.pose_landmarks.landmark[11].visibility*100))+ ",\n")
            text = text + ("\"right_shoulder_x\":"+ str(int(results.pose_landmarks.landmark[12].x*image.shape[1]))+ ",\n")
            text = text + ("\"right_shoulder_y\":"+ str(int(results.pose_landmarks.landmark[12].y*image.shape[0]))+ ",\n")
            text = text + ("\"right_shoulder_z\":"+ str(int(results.pose_landmarks.landmark[12].z*100))+ ",\n")
            text = text + ("\"right_shoulder_visibility\":"+ str(int(results.pose_landmarks.landmark[12].visibility*100))+",\n")
            text = text + ("\"left_wrist_x\":"+str(int(results.pose_landmarks.landmark[15].x*image.shape[1]))+",\n")
            text = text + ("\"left_wrist_y\":"+str(int(results.pose_landmarks.landmark[15].y*image.shape[0]))+",\n")
            text = text + ("\"left_wrist_z\":"+ str(int(results.pose_landmarks.landmark[15].z*100))+ ",\n")
            text = text + ("\"left_wrist_visibility\":"+ str(int(results.pose_landmarks.landmark[15].visibility*100))+ ",\n")
            text = text + ("\"right_wrist_x\":"+str(int(results.pose_landmarks.landmark[16].x*image.shape[1]))+",\n")
            text = text + ("\"right_wrist_y\":"+str(int(results.pose_landmarks.landmark[16].y*image.shape[0]))+",\n")
            text = text + ("\"right_wrist_z\":"+ str(int(results.pose_landmarks.landmark[16].z*100))+ ",\n")
            text = text + ("\"right_wrist_visibility\":"+ str(int(results.pose_landmarks.landmark[16].visibility*100))+ ",\n")
            text = text + ("\"left_elbow_x\":"+str(int(results.pose_landmarks.landmark[13].x*image.shape[1]))+",\n")
            text = text + ("\"left_elbow_y\":"+str(int(results.pose_landmarks.landmark[13].y*image.shape[0]))+",\n")
            text = text + ("\"left_elbow_z\":"+ str(int(results.pose_landmarks.landmark[13].z*100))+ ",\n")
            text = text + ("\"left_elbow_visibility\":"+ str(int(results.pose_landmarks.landmark[13].visibility*100))+ ",\n")
            text = text + ("\"right_elbow_x\":"+str(int(results.pose_landmarks.landmark[14].x*image.shape[1]))+",\n")
            text = text + ("\"right_elbow_y\":"+str(int(results.pose_landmarks.landmark[14].y*image.shape[0]))+",\n")
            text = text + ("\"right_elbow_z\":"+ str(int(results.pose_landmarks.landmark[14].z*100))+ ",\n")
            text = text + ("\"right_elbow_visibility\":"+ str(int(results.pose_landmarks.landmark[14].visibility*100))+ "\n")
            text = text + ("}\n")
        '''
        else:
            right_knee_visibility = int(0)
            left_knee_visibility = int(0)
            right_shoulder_visibility = int(0)
            left_shoulde_visibility = int(0)
        #f.writelines(text)

        '''
            cv2.circle(image, #left
                    center=(int(results.pose_landmarks.landmark[25].x*image.shape[1]), int(results.pose_landmarks.landmark[25].y*image.shape[0])),
                    radius=10,
                    color=(255, 0, 0),
                    thickness=3,
                    lineType=cv2.LINE_4,
                    shift=0)
            cv2.circle(image, #right
                    center=(int(results.pose_landmarks.landmark[26].x*image.shape[1]), int(results.pose_landmarks.landmark[26].y*image.shape[0])),
                    radius=10,
                    color=(0, 255, 0),
                    thickness=3,
                    lineType=cv2.LINE_4,
                    shift=0)
            cv2.imshow('MediaPipe Pose', image)
            cv2.waitKey()
        '''
        #cv2.imshow('MediaPipe Pose', image)
        #cv2.waitKey()
        #print("/buld/pose_cpp.py")
        #f.close()


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

#image = cv2.imread('bat.jpg')
#Pose(image)
