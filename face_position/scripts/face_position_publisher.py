#!/usr/bin/env python

import rospy
from face_position.msg import FacePosition
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2

def publish_face_position():
    # ROS 노드 초기화
    rospy.init_node('face_position_publisher', anonymous=True)
    pub = rospy.Publisher('face_position', FacePosition, queue_size=10)
    rate = rospy.Rate(30)  

    BaseOptions = python.BaseOptions
    FaceDetector = vision.FaceDetector
    FaceDetectorOptions = vision.FaceDetectorOptions
    VisionRunningMode = vision.RunningMode

    # 콜백 함수 정의
    def result_callback(result: vision.FaceDetectorResult, output_image: mp.Image, timestamp_ms: int):
        # Mediapipe 결과를 처리하여 얼굴 위치를 퍼블리시
        if result.detections:
            height, width, _ = output_image.numpy_view().shape
            for detection in result.detections:
                bbox = detection.bounding_box
                x_center = (bbox.origin_x + bbox.width / 2) / width
                y_center = (bbox.origin_y + bbox.height / 2) / height

                # 얼굴 크기로 Z 값을 계산 (박스 높이 기준)
                bbox_height = bbox.height / height  # 정규화된 높이
                z_estimation = 1.0 / bbox_height  # 높이가 클수록 z 값이 작아짐
                
                # ROS 메시지 생성 및 퍼블리시
                face_msg = FacePosition(x=x_center, y=y_center, z=z_estimation)
                pub.publish(face_msg)

                rospy.loginfo(f"Face Position: x={x_center:.2f}, y={y_center:.2f}, z={z_estimation:.2f}")

    # Mediapipe 얼굴 감지 초기화
    options = FaceDetectorOptions(
        base_options=BaseOptions(model_asset_path='/home/gunminy/catkin_ws/src/face_position/scripts/detector.tflite'),
        running_mode=VisionRunningMode.LIVE_STREAM,
        result_callback=result_callback)

    with FaceDetector.create_from_options(options) as detector:
        # 카메라 열기
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            rospy.logerr("Failed to open camera")
            return

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue

            # 프레임 전처리 및 Mediapipe 이미지 생성
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            # 얼굴 감지 요청
            timestamp_ms = int(rospy.Time.now().to_sec() * 1000)
            detector.detect_async(mp_image, timestamp_ms)

            # 디버깅을 위한 비디오 출력
            cv2.imshow("Face Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        publish_face_position()
    except rospy.ROSInterruptException:
        pass
