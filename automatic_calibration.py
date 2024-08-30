import cv2
from pyorbbecsdk import Config
from pyorbbecsdk import OBError
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import Pipeline, FrameSet
from pyorbbecsdk import VideoStreamProfile
from examples.utils import frame_to_bgr_image
import argparse
import os
import copy
import numpy as npq
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R
# segment anything
# from segment_anything import build_sam, SamPredictor
import cv2
import numpy as np
import matplotlib.pyplot as plt
import gc
import urx
import json
import time
# from arm_test.owl_sam import load_owlvit2,owl2_inference,draw_boxes_on_image
# from urx_test import get_homogeneous_transformation
from SimpleHandEye.SimpleHandEye.interfaces.apriltag import ApriltagTracker
from SimpleHandEye.SimpleHandEye.solvers import OpenCVSolver
ESC_KEY = 27
def euler_to_rot_matrix(rx, ry, rz):
    """ Convert Euler angles to rotation matrix. """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(rx), -np.sin(rx)],
                    [0, np.sin(rx), np.cos(rx)]])
    R_y = np.array([[np.cos(ry), 0, np.sin(ry)],
                    [0, 1, 0],
                    [-np.sin(ry), 0, np.cos(ry)]])
    R_z = np.array([[np.cos(rz), -np.sin(rz), 0],
                    [np.sin(rz), np.cos(rz), 0],
                    [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R
def get_homogeneous_transformation(pos, orient):
    tx, ty, tz = pos
    rx, ry, rz = orient
    # Convert Euler angles to rotation matrix
    R = euler_to_rot_matrix(rx, ry, rz)
    # Create homogeneous transformation matrix
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = [tx, ty, tz]
    return T
def draw_boxes_on_image(image, boxes, color=(0, 255, 0), thickness=2):
    """
    在图像上绘制 bounding boxes
    参数:
    - image: 输入图像，必须是一个 NumPy 数组
    - boxes: bounding boxes，必须是 Nx4 的 NumPy 数组或 Python 列表，其中每个 box 由 [x_min, y_min, x_max, y_max] 表示
    - color: bounding box 的颜色，默认为绿色 (B, G, R)
    - thickness: bounding box 线条的粗细，默认为 2
    返回:
    - 带有绘制 bounding boxes 的图像
    """
    # 确保输入图像是副本，以免修改原图像
    # 遍历所有 bounding boxes 并绘制
    for box in boxes:
        x_min, y_min, x_max, y_max = map(int, box)
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, thickness)
    return image
def save_poses_to_file(base_to_hand, cam_to_tag, filename="pose_data.txt"):
    data = {
        'base_to_hand': base_to_hand.tolist(),
        'cam_to_tag': cam_to_tag.tolist()
    }
    # 打开文件并追加写入一行
    with open(filename, 'a') as f:
        f.write(json.dumps(data) + '\n')
def is_valid_pose(pose):
    if pose.shape != (4, 4):  # 检查矩阵是否是4x4
        return False
    if not np.isfinite(pose).all():  # 检查是否有非数值
        return False
    if np.linalg.det(pose[:3, :3]) == 0:  # 检查旋转矩阵行列式是否为零
        return False
    return True
def read_poses_from_file(filename="pose_data.txt"):
    base_to_hand_list = []
    cam_to_tag_list = []
    with open(filename, 'r') as f:
        for line in f:
            data = json.loads(line)
            base_to_hand = np.array(data['base_to_hand'])
            cam_to_tag = np.array(data['cam_to_tag'])
            # 验证数据是否有效
            if not is_valid_pose(base_to_hand) or not is_valid_pose(cam_to_tag):
                print(f"Invalid data found, skipping this entry.\nbase_to_hand:\n{base_to_hand}\ncam_to_tag:\n{cam_to_tag}")
                continue
            base_to_hand_list.append(base_to_hand)
            cam_to_tag_list.append(cam_to_tag)
    return base_to_hand_list, cam_to_tag_list
def connect_to_robot(ip):
    while True:
        try:
            rob = urx.Robot(ip)
            
            return rob
        except Exception as e:
            print(f"连接机器人失败: {e}")
            time.sleep(2)  # 等待2秒后重试
def is_robot_connected(rob):
    try:
        if rob.is_program_running() or rob.robotmode() not in ["DISCONNECTED", "POWER_OFF"]:
            return True
        else:
            return False
    except:
        return False
def get_robot_pose(rob):
    # if not is_robot_connected(rob):
    #     print("连接检测失败，尝试重新连接机器人")
    #     rob = connect_to_robot(ip)
    #     print("机器人重新连接成功")
    try:
        trans = rob.get_pose()  # 获取当前末端位姿
        return trans
    except Exception as e:
        print(f"读取位姿失败: {e}")
        return None
    
def setup_camera():
    config = Config()
    pipeline = Pipeline()
    profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    try:
        color_profile = profile_list.get_video_stream_profile(1280, 0, OBFormat.RGB, 30)
    except OBError as e:
        print(e)
        color_profile = profile_list.get_default_video_stream_profile()
        print("color profile: ", color_profile)
    config.enable_stream(color_profile)
    pipeline.start(config)
    return pipeline

def get_camera_parameters(pipeline):
    camera_param = pipeline.get_camera_param()
    K = np.array([[camera_param.rgb_intrinsic.fx, 0, camera_param.rgb_intrinsic.cx],
                  [0, camera_param.rgb_intrinsic.fy, camera_param.rgb_intrinsic.cy],
                  [0, 0, 1]])
    D = np.array([camera_param.rgb_distortion.k1, camera_param.rgb_distortion.k2,
                  camera_param.rgb_distortion.p1, camera_param.rgb_distortion.p2,
                  camera_param.rgb_distortion.k3], dtype=np.float32)
    print(camera_param)
    return K, D

def process_frames(pipeline, tag_pose_tracker, rob):
    while True:
        try:
            frames = pipeline.wait_for_frames(1)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            
            color_image = frame_to_bgr_image(color_frame)
            cv2.imshow("Color Viewer", color_image)
            trans = get_robot_pose(rob)
            pos = trans.pos._data
            rotation = R.from_rotvec(trans.orient.rv._data)
            euler_angles = rotation.as_euler('xyz', degrees=False)
            base_T_hand = get_homogeneous_transformation(pos, euler_angles)
            # print("base_T_hand", base_T_hand)
            
            cam_T_tag = tag_pose_tracker.getPoseAndCorners(color_image, tag_id=0)
            
            # if cam_T_tag is not None:
            #     process_detected_tag(cam_T_tag, color_image)
            # else:
            #     print("No tag detected")
                
            if color_image is None:
                print("failed to convert frame to image")
                continue
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            # elif key == ord('s'):
            #     if cam_T_tag is not None:
            
            # cv2.imshow("Color Viewer", color_image)
            # time.sleep(1)
            save_poses_to_file(base_T_hand, cam_T_tag['pose'])
            print("Pose data saved to file.")
            break
        except KeyboardInterrupt:
            break

def process_detected_tag(cam_T_tag, color_image):
    corners = cam_T_tag['corners']
    # print("corners", corners)
    left_top = corners[1].astype(int)
    right_bottom = corners[3].astype(int)
    bounding_box = [left_top[0], left_top[1], right_bottom[0], right_bottom[1]]
    bounding_boxes = np.array([bounding_box])
    pose = cam_T_tag['pose']
    # print("cam_T_tag", pose)
    color_image = draw_boxes_on_image(color_image, bounding_boxes)
def move_robot_to_points(rob, points,pipeline, tag_pose_tracker):
    for point in points:
        
        rob.movej(point, acc=0.5, vel=0.2)
        time.sleep(1)
        process_frames(pipeline, tag_pose_tracker, rob)
        time.sleep(2)  # 等待机器人到达位置并稳定
        print(f"移动到点 {point}.")
     
def main():
    ip = "192.168.51.254"
    rob = connect_to_robot(ip)
    rob.set_tcp((0, 0, 0, 0, 0, 0))
    rob.set_payload(1, (0, 0, 0.9))
    pipeline = setup_camera()
    K, D = get_camera_parameters(pipeline)
    tag_pose_tracker = ApriltagTracker(tag_size=0.06, intrinsic_matrix=K, distortion_coeffs=D)
    points = [
    [0.5307509899139404, -1.3241446653949183, 1.8042359352111816, -0.21860248247255498, 2.146289825439453, -3.344113890324728],  # 点1
    [0.49460431933403015, -1.479098145161764, 2.125046730041504, -0.30116397539247686, 2.3091392517089844, -3.119495217000143],  # 点2
    [0.335248202085495, -2.150712315236227, 2.4691989421844482, -0.30072051683534795, 2.14618182182312, -3.1194830576526087],  # 点3
    [0.11963851749897003, -2.2922681013690394, 2.6508772373199463, -0.6133821646319788, 1.91532564163208, -3.8268588224994105]  # 点4
    ]

    # for i in range(10):
    move_robot_to_points(rob, points, pipeline, tag_pose_tracker)

    matrices1,matrices2=read_poses_from_file("pose_data.txt")
    solver = OpenCVSolver(type='AX=YB')
    X,Y = solver.solve(matrices1, matrices2)
    print("X=")
    print(X)
    print("Y=")
    print(Y)
    pipeline.stop()


if __name__ == "__main__":
    main()

    