from gpiozero import Button
import subprocess
import signal
import sys
import psutil

print("プログラムを終了します")
sys.exit(0)

PIN_TO_MONITOR = 6

# 実行するファイルのパス
EXECUTABLE_PATH = "/home/pi/depthaiC++/depthai-core-example-main/build/./myapp"

def is_process_running(process_name):
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == process_name:
            return True
    return False

def run_executable():
    process_name = EXECUTABLE_PATH.split("/")[-1]
    if not is_process_running(process_name):
        print(f"{process_name} を実行します")
        # コマンド `sudo -E nice -n -20` を実行
        subprocess.Popen(["sudo", "-E", "nice", "-n", "-20", EXECUTABLE_PATH])
    else:
        print(f"{process_name} はすでに実行中です")

def signal_handler(sig, frame):
    print("プログラムを終了します")
    sys.exit(0)

# Ctrl+Cで終了
signal.signal(signal.SIGINT, signal_handler)

button = Button(PIN_TO_MONITOR, pull_up=True)

print(f"GPIO {PIN_TO_MONITOR} を監視中")

# 実行
button.when_pressed = run_executable

# プログラムを常駐させる
signal.pause()
