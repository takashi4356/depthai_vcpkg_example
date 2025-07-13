import re
import cv2
import numpy as np
from datetime import datetime

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

def draw_circles_from_log(log_file, image_file, start_time, end_time):
    img = cv2.imread(image_file)
    img = cv2.resize(img, (640, 400))  # 画像サイズを640x400に固定
    if img is None:
        raise ValueError(f"画像ファイルを読み込めません: {image_file}")
    
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

    current_index = 0
    while True:
        timestamp, x, y, radius = valid_logs[current_index]
        scaled_x = int(x * (640 / img.shape[1]))
        scaled_y = int(y * (400 / img.shape[0]))
        scaled_radius = int(radius)

        temp_img = img.copy()
        cv2.circle(temp_img, (scaled_x, scaled_y), scaled_radius, (0, 255, 0), 2)

        timestamp_text = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        cv2.putText(temp_img, timestamp_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        cv2.putText(temp_img, f"Log {current_index + 1}/{len(valid_logs)}", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow('Circles', temp_img)

        key = cv2.waitKey(0)
        if key == ord('q'):
            break
        elif key == ord('n') and current_index < len(valid_logs) - 1:
            current_index += 1
        elif key == ord('p') and current_index > 0:
            current_index -= 1
    
    cv2.destroyAllWindows()

# 使用例
log_file = '/home/pi/kyouyuu/LOG/strike_log_2025_01_23.log'
image_file = '/home/pi/kyouyuu/LOG/r_image_SARA_2025_01_23_123556_11579.jpg'
start_time = datetime(2025, 1, 23, 12, 00, 42)
end_time = datetime(2025, 1, 23, 18, 49, 0)

draw_circles_from_log(log_file, image_file, start_time, end_time)
