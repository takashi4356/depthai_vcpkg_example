import cv2
import os
import sys
#import daemon
import mediapipe as mp

def Pose():
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_pose = mp.solutions.pose
    f = open('bat.json', 'w') 
    jikkou_flg=False
    # Webカメラから入力
    #cap = cv2.VideoCapture(0)
    with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:
    #  while cap.isOpened():
    #    success, image = cap.read()
        image = cv2.imread('bat.jpg')
        #cv2.imshow('MediaPipe Pose', image)
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)
        
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
        if results.pose_landmarks:
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
            text = text + ("\"right_wrist_visibility\":"+ str(int(results.pose_landmarks.landmark[16].visibility*100))+ "\n")
            text = text + ("}\n")
            f.writelines(text)

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
        f.close()

Pose()

