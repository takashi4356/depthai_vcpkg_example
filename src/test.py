import depthai as dai
import cv2
import threading

def create_pipeline():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    xout_video = pipeline.create(dai.node.XLinkOut)

    xout_video.setStreamName("video")

    # Properties
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    # Linking
    cam_rgb.video.link(xout_video.input)

    return pipeline

def run_device(device_id):
    # Create and start pipeline
    pipeline = create_pipeline()
    with dai.Device(pipeline, device_id) as device:
        video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)

        while True:
            video_frame = video_queue.get()
            frame = video_frame.getCvFrame()

            cv2.imshow(f"Camera {device_id}", frame)

            if cv2.waitKey(1) == ord('q'):
                break

# Get the list of connected devices
found_devices = dai.Device.getAllAvailableDevices()
if len(found_devices) < 2:
    print("Two DepthAI devices are required")
    exit(1)

# Create and start threads for each device
threads = []
for i, device_info in enumerate(found_devices[:2]):
    device_id = device_info.getMxId()
    thread = threading.Thread(target=run_device, args=(device_id,))
    thread.start()
    threads.append(thread)

# Join threads
for thread in threads:
    thread.join()

cv2.destroyAllWindows()
