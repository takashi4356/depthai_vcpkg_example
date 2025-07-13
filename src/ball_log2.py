import re
import cv2
import numpy as np
from datetime import datetime, timedelta

def parse_log_line(line):
    match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}).*中心 \((\d+),(\d+).*半径 \((\d+\.\d+),(\d+\.\d+)\)', line)
    if match:
        timestamp = datetime.strptime(match.group(1), '%Y-%m-%d %H:%M:%S.%f')
        x = int(match.group(2))
        y = int(match.group(3))
        radius_x = float(match.group(4))
        radius_y = float(match.group(5))
        radius = int((radius_x + radius_y) / 2)  # 平均半径を使用
        return timestamp, x, y, radius
    return None

def draw_circles_from_log(log_file, video_file, start_time, end_time):
    cap = cv2.VideoCapture(video_file)
    if not cap.isOpened():
        raise ValueError(f"動画ファイルを読み込めません: {video_file}")
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    with open(log_file, 'r') as file:
        logs = file.readlines()
    
    valid_logs = []
    for log in logs:
        parsed = parse_log_line(log)
        if parsed and start_time <= parsed[0] <= end_time:
            valid_logs.append(parsed)
    
    if not valid_logs:
        print("指定された時間範囲内にログがありません。")
        return

    # 動画の開始時間を設定（ログの先頭を0msとする）
    video_start_time = valid_logs[0][0]
    cap.set(cv2.CAP_PROP_POS_MSEC, 0)  # 動画の再生位置を0msに設定

    current_index = 0
    current_frame_index = 0  # 初期フレームインデックス
    
    while True:
        # 動画のフレームを読み込む
        cap.set(cv2.CAP_PROP_POS_FRAMES, current_frame_index)
        ret, frame = cap.read()
        
        if not ret:
            break
        
        frame = cv2.resize(frame, (640, 400))

        # 現在のフレームの時間を計算
        current_time = video_start_time + timedelta(seconds=current_frame_index / fps)

        # ログエントリを描画
        while current_index < len(valid_logs) and valid_logs[current_index][0] <= current_time:
            timestamp, x, y, radius = valid_logs[current_index]
            scaled_x = int(x * (640 / width))
            scaled_y = int(y * (400 / height))
            scaled_radius = int(radius * (640 / width))

            cv2.circle(frame, (scaled_x, scaled_y), scaled_radius, (0, 255, 0), 2)
            current_index += 1

        timestamp_text = current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        cv2.putText(frame, timestamp_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.putText(frame, f"Log {current_index}/{len(valid_logs)}", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow('Video with Circles', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('n'):  # 次のフレームへ移動
            current_frame_index += 1
            if current_frame_index >= cap.get(cv2.CAP_PROP_FRAME_COUNT):
                current_frame_index = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) - 1
            current_index = min(current_index + 1, len(valid_logs) - 1)  # 次のログエントリへ進む
        elif key == ord('p'):  # 前のフレームへ移動
            if current_frame_index > 0:
                current_frame_index -= 1
                current_index = max(current_index - 1, 0)  # 前のログエントリへ戻る
    
    cap.release()
    cv2.destroyAllWindows()

# 使用例
log_file = '/home/pi/kyouyuu/LOG/strike_log_2025_01_05.log'
video_file = '/home/pi/kyouyuu/LOG/ANMR0014.mp4'
start_time = datetime(2025, 1, 5, 14, 25, 35)
end_time = datetime(2025, 1, 5, 14, 26, 0)

draw_circles_from_log(log_file, video_file, start_time, end_time)

# video_file = '/home/pi/kyouyuu/LOG/ANMR0014.mp4'
