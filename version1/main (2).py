from argparse import ArgumentParser
import numpy as np  # 确保在顶部导入 numpy
import cv2
import serial
import struct
import time
from face_detection import FaceDetector
from mark_detection import MarkDetector
from pose_estimation import PoseEstimator
from utils import refine
from collections import deque

# Parse arguments from user input.
parser = ArgumentParser()
parser.add_argument("--video", type=str, default=None,
                    help="Video file to be processed.")
parser.add_argument("--cam", type=int, default=0,
                    help="The webcam index.")
args = parser.parse_args()

print("OpenCV version: {}".format(cv2.__version__))

# KCF 追踪器变量
tracker = None
init_tracking = False
tracking_fail_count = 0


# Kalman 滤波器初始化（每个角度各一个）
def create_kalman():
    kf = cv2.KalmanFilter(2, 1)
    kf.transitionMatrix = np.array([[1, 1], [0, 1]], np.float32)
    kf.measurementMatrix = np.array([[1, 0]], np.float32)
    kf.processNoiseCov = np.eye(2, dtype=np.float32) * 0.03
    kf.measurementNoiseCov = np.array([[1]], np.float32) * 1
    kf.errorCovPost = np.eye(2, dtype=np.float32)
    return kf

kf_pitch = create_kalman()
kf_yaw = create_kalman()
kf_roll = create_kalman()
def run():
    # Before estimation started, there are some startup works to do.

    # Initialize the video source from webcam or video file.
    video_src = ("/dev/camera")
    cap = cv2.VideoCapture(video_src)
    print(f"Video source: {video_src}")

    # Get the frame size. This will be used by the following detectors.
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Setup a face detector to detect human faces.
    face_detector = FaceDetector("assets/face_detector.onnx")

    # Setup a mark detector to detect landmarks.
    mark_detector = MarkDetector("assets/face_landmarks.onnx")

    # Setup a pose estimator to solve pose.
    pose_estimator = PoseEstimator(frame_width, frame_height)

    # Measure the performance with a tick meter.
    tm = cv2.TickMeter()
    # 初始化串口（修改为你实际使用的端口，比如 /dev/ttyUSB0）
    ser=None
    try:
        ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
        print("Serial port opened.")
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        ser = None

    # Now, let the frames flow.
    while True:
        
        # Read a frame.
        frame_got, frame = cap.read()
        if not frame_got or frame is None:  # ✅ 加入安全检查
            print("⚠️ Warning: Frame not captured or is None!")
            cap.release()
            time.sleep(1)  # 稍作延时，避免系统摄像头资源还未释放

            cap = cv2.VideoCapture(video_src)
            if not cap.isOpened():
                print("❌ Failed to reopen camera. Retrying in 2 seconds...")
                time.sleep(2)
            else:
                print("✅ Camera reinitialized.")
            continue

        # If the frame comes from webcam, flip it so it looks like a mirror.
        # if video_src == 0:
        frame = cv2.flip(frame, 2)

        # Step 1: Get faces from current frame.
        faces, _ = face_detector.detect(frame, 0.7)
        # Any valid face found?
        if len(faces) > 0:
            tm.start()

            # Step 2: Detect landmarks. Crop and feed the face area into the
            # mark detector. Note only the first face will be used for
            # demonstration.
            face = refine(faces, frame_width, frame_height, 0.15)[0]
            x1, y1, x2, y2 = face[:4].astype(int)
            patch = frame[y1:y2, x1:x2]

            # Run the mark detection.
            marks = mark_detector.detect([patch])[0].reshape([68, 2])

            # Convert the locations from local face area to the global image.
            marks *= (x2 - x1)
            marks[:, 0] += x1
            marks[:, 1] += y1

            # Step 3: Try pose estimation with 68 points.
            pose = pose_estimator.solve(marks)
            rvec, tvec = pose

            # 将旋转向量转换为旋转矩阵
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # 计算欧拉角
            sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
            singular = sy < 1e-6

            if not singular:
                pitch = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                yaw = np.arctan2(-rotation_matrix[2, 0], sy)
                roll = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            else:
                pitch = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                yaw = np.arctan2(-rotation_matrix[2, 0], sy)
                roll = 0

            # 弧度转角度
            pitch = np.degrees(pitch)
            yaw = np.degrees(yaw)
            roll = np.degrees(roll)
            if pitch>20:
                pitch=20
            if pitch< -20:
                pitch=-20
            if -2 <= pitch <= 5 :
                pitch+=10
            if -10<=yaw<=10:
                yaw=0

            if yaw>30 :
                yaw =30
            if yaw<-30 :
                yaw =-30
                
	     # 输出姿态角
            print(f"Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Roll: {roll:.2f}")
            if ser:
                try:
                    # 构造帧：包头 + 3个float + 包尾
                    send_frame = bytearray()
                    send_frame.append(0xAA)  # 包头

                    send_frame += struct.pack('<f', pitch)  # float 小端
                    send_frame += struct.pack('<f', yaw)
                    send_frame += struct.pack('<f', roll)

                    send_frame.append(0xEE)  # 包尾

                    ser.write(send_frame)
                    time.sleep(0.05)  # 每50ms发送一次

                except serial.SerialException as e:
                    print(f"Serial write error: {e}")
            tm.stop()

            # All done. The best way to show the result would be drawing the
            # pose on the frame in realtime.

            # Do you want to see the pose annotation?
         #    pose_estimator.visualize(frame, pose, color=(0, 255, 0))
	     # # 可选：在画面上显示姿态角
         #    cv2.putText(frame, f"Pitch: {pitch:.1f}", (10, 50),
         #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
         #    cv2.putText(frame, f"Yaw: {yaw:.1f}", (10, 70),
         #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
         #    cv2.putText(frame, f"Roll: {roll:.1f}", (10, 90),
         #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
         #    # Do you want to see the axes?
         #    pose_estimator.draw_axes(frame, pose)
         #
         #    # Do you want to see the marks?
         #    mark_detector.visualize(frame, marks, color=(0, 255, 0))
         #
         #    # Do you want to see the face bounding boxes?
         #    face_detector.visualize(frame, faces)

        # Draw the FPS on screen.
        # cv2.rectangle(frame, (0, 0), (90, 30), (0, 0, 0), cv2.FILLED)
        # cv2.putText(frame, f"FPS: {tm.getFPS():.0f}", (10, 20),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

        # Show preview.
            # ✅ 安全显示（防止崩溃）
        # try:
        #     cv2.imshow("Preview", frame)
        # except Exception as e:
        #     print(f"⚠️ imshow error: {e}")
        if cv2.waitKey(1) == 27:
            break



if __name__ == '__main__':
    run()
