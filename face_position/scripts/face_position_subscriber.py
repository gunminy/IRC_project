#!/usr/bin/env python

import rospy
from face_position.msg import FacePosition
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.msg import KinematicsPose
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math

# 전역 변수 초기화
joint4_service_name = '/goal_joint_space_path_to_kinematics_pose'
set_joint4_pose_service = None
set_gripper_pose_service = None
latest_msg = None  # 최신 메시지를 저장할 변수

init_r =0.2
init_theta = math.radians(0) 
init_phi = math.radians(30) 
r = init_r  # radius
theta = init_theta # azimuthal angle in radians
phi = init_phi  # polar angle in radians
pitch = math.radians(-10)   # 회전축 Y (rad) joint4 각도
path_time = 0.01
callback_count_per_second = 0
srvcall_count_per_second = 0
        
delta = 0.01

def spherical_to_cartesian(r, theta, phi):
    x = r * math.sin(phi) * math.cos(theta)
    y = r * math.sin(phi) * math.sin(theta)
    z = r * math.cos(phi)
    return x, y, z

def face_position_callback(msg):
    """
    콜백 함수: 최신 메시지를 전역 변수에 저장
    """
    global latest_msg, callback_count_per_second
    latest_msg = msg  # 메시지를 전역 변수에 저장
    callback_count_per_second += 1

def init_robot_pose():
    global set_joint4_pose_service
    global r, theta, phi, pitch
    try:  
        # Joint4의 목표 자세 설정
        target_joint4_pose = KinematicsPose()
        target_joint4_pose.pose = Pose()

        r = init_r  # radius
        theta = init_theta # azimuthal angle in radians
        phi = init_phi  # polar angle in radians
        pitch = math.radians(-10)   # 회전축 Y (rad) joint4 각도

        x_pos, y_pos, z_pos = spherical_to_cartesian(r, theta, phi)
        target_joint4_pose.pose.position.x = x_pos # Gripper의 X 좌표
        target_joint4_pose.pose.position.y = y_pos # Gripper의 Y 좌표
        target_joint4_pose.pose.position.z = z_pos # Gripper의 Z 좌표

        # 오일러 각도를 쿼터니언으로 변환
        quat = quaternion_from_euler(0, pitch, 0)
        target_joint4_pose.pose.orientation.x = quat[0]
        target_joint4_pose.pose.orientation.y = quat[1]
        target_joint4_pose.pose.orientation.z = quat[2]
        target_joint4_pose.pose.orientation.w = quat[3]

        # 서비스 호출: Joint4 설정
        planning_group = "arm"
        end_effector_name = "joint4"
        response = set_joint4_pose_service(planning_group, end_effector_name, target_joint4_pose, 1.5)
        if response.is_planned:
            rospy.loginfo("Joint4 motion planned successfully!")
        else:
            rospy.logwarn("Joint4 motion planning failed.")
        

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def process_message():
    """
    전역 변수에 저장된 최신 메시지를 처리
    """
    global latest_msg, delta, set_joint4_pose_service, path_time,  srvcall_count_per_second
    global r, theta, phi, pitch

    if callback_count_per_second == 0:
        return  # 처리할 메시지가 없으면 대기

    # 메시지 복사 및 초기화
    msg = latest_msg

    try:  
        # Joint4의 목표 자세 설정
        target_joint4_pose = KinematicsPose()
        target_joint4_pose.pose = Pose()

        # r = 1.0  # radius
        # theta = math.radians(45)  # azimuthal angle in radians
        # phi = math.radians(30)  # polar angle in radians
        if msg.x > 0.55:
            theta = theta - 0.04 * abs(0.55-msg.x)
        elif msg.x < 0.35:
            theta = theta + 0.04 * abs(0.45 -msg.x)

        if msg.y > 0.55 :
            if phi <= math.radians(10):
                if r > 0.2:
                    r = r - 0.001
                    pitch = pitch + 0.005
            elif phi < math.radians(80):
                phi = phi + 0.04 * abs(0.55 - msg.y)
            
        elif msg.y < 0.45 :
            if phi > math.radians(10):
                phi  = phi - 0.04 * abs(0.45 -msg.y)
            elif phi <= math.radians(10):
                if r < 0.29: 
                    r = r + 0.001
                    pitch = pitch - 0.005

        
        if msg.z > 3.5:
            if phi < math.radians(80):
                phi = phi + 0.003
        elif msg.z < 2.0 :
            if phi > math.radians(10):
                phi = phi - 0.003
        
        r = r + 0.0002* ((msg.z - 2.5) / math.sin(phi))
        if r > 0.29 :
            r = 0.29
        elif r < 0.2 :
            r = 0.20
        
        x_pos, y_pos, z_pos = spherical_to_cartesian(r, theta, phi)
        target_joint4_pose.pose.position.x = x_pos # Gripper의 X 좌표
        target_joint4_pose.pose.position.y = y_pos # Gripper의 Y 좌표
        target_joint4_pose.pose.position.z = z_pos # Gripper의 Z 좌표
        # print(f"x_pos: {x_pos}, y_pos: {y_pos}, z_pos: {z_pos}")

        # 오일러 각도를 쿼터니언으로 변환
        quat = quaternion_from_euler(0, pitch, 0)
        target_joint4_pose.pose.orientation.x = quat[0]
        target_joint4_pose.pose.orientation.y = quat[1]
        target_joint4_pose.pose.orientation.z = quat[2]
        target_joint4_pose.pose.orientation.w = quat[3]

        # 서비스 호출: Joint4 설정
        planning_group = "arm"
        end_effector_name = "joint4"
        response = set_joint4_pose_service(planning_group, end_effector_name, target_joint4_pose, path_time)
        if response.is_planned:
            srvcall_count_per_second += 1
        else:
            rospy.logwarn("Joint4 motion planning failed.")      
            # if r < 0.29 :
            #     r += 0.001
            # elif r > 0.2 :
            #      r -= 0.001
            # # init_robot_pose()
        

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    """
    ROS 노드 초기화 및 메인 루프 실행
    """
    global set_joint4_pose_service, set_gripper_pose_service, callback_count_per_second, srvcall_count_per_second

    rospy.init_node('manipulator_controller', anonymous=True)

    # 서비스 초기화
    rospy.loginfo("Waiting for the Joint4 and Gripper services...")
    rospy.wait_for_service(joint4_service_name)
    set_joint4_pose_service = rospy.ServiceProxy(joint4_service_name, SetKinematicsPose)
    rospy.loginfo(f"Service {joint4_service_name} is available!")

    # 토픽 구독
    rospy.Subscriber('face_position', FacePosition, face_position_callback)

    # 메인 루프
    rate = rospy.Rate(60)  # 60Hz 루프 주기
    rospy.loginfo("Manipulator Controller is running...")

    init_robot_pose()

    # 프레임레이트 계산용 변수 초기화
    last_time = rospy.Time.now()
    loop_count = 0

    while not rospy.is_shutdown():
        process_message()

        # 프레임레이트 계산
        loop_count += 1
        current_time = rospy.Time.now()
        elapsed_time = (current_time - last_time).to_sec()  # 경과 시간 (초)

        if elapsed_time >= 1.0:  # 1초마다 프레임레이트 계산
            framerate = loop_count / elapsed_time
            rospy.loginfo(f"Actual framerate: {framerate:.2f} Hz callback_count_per_second : {callback_count_per_second}, srvcall_count : {srvcall_count_per_second}")
            loop_count = 0
            callback_count_per_second = 0
            srvcall_count_per_second = 0
            last_time = current_time

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
