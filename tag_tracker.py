# ******************************************************************************
#  Copyright (c) 2023 Orbbec 3D Technology, Inc
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http:# www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# ******************************************************************************
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
import math
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
    - boxes: bounding boxes,必须是 Nx4 的 NumPy 数组或 Python 列表，其中每个 box 由 [x_min, y_min, x_max, y_max] 表示
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
def get_robot_pose(rob,ip):
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
def homogeneous_to_rot_trans(T):
    # 提取平移向量
    tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
    # 提取旋转矩阵
    R = T[0:3, 0:3]
    # 计算旋转角度 θ
    trace_R = np.trace(R)
    theta = math.acos((trace_R - 1) / 2)
    if theta < 1e-6:
        # 如果旋转角度接近0，旋转向量为零向量S
        return np.array([0, 0, 0]), np.array([tx, ty, tz])
    # 计算旋转向量
    rx = (R[2, 1] - R[1, 2]) / (2 * math.sin(theta))
    ry = (R[0, 2] - R[2, 0]) / (2 * math.sin(theta))
    rz = (R[1, 0] - R[0, 1]) / (2 * math.sin(theta))
    rot_vector = theta * np.array([rx, ry, rz])
    # print(tx,ty,tz,rot_vector)
    return np.append(np.array([tx, ty, tz]),rot_vector)
def main():
    ip = "192.168.51.254"
    rob = connect_to_robot(ip)  # 确保成功连接机器人
    rob.set_tcp((0, 0, 0.25, 0, 0, 0))
    # rob.set_payload(1, (0, 0, 0.9))
    print("机器人连接成功")
    config = Config()
    pipeline = Pipeline()
    profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    try:
        color_profile: VideoStreamProfile = profile_list.get_video_stream_profile(1280, 0, OBFormat.RGB, 30)
    except OBError as e:
        print(e)
        color_profile = profile_list.get_default_video_stream_profile()
        print("color profile: ", color_profile)
    config.enable_stream(color_profile)
    pipeline.start(config)
    camera_param = pipeline.get_camera_param()
    K = np.array(
    [[camera_param.rgb_intrinsic.fx, 0, camera_param.rgb_intrinsic.cx],
    [0, camera_param.rgb_intrinsic.fy, camera_param.rgb_intrinsic.cy],
    [0, 0, 1]])
    D = np.array([camera_param.rgb_distortion.k1, camera_param.rgb_distortion.k2, camera_param.rgb_distortion.p1, camera_param.rgb_distortion.p2, camera_param.rgb_distortion.k3], dtype=np.float32)
    # print(camera_param)
     # model2, processor2 = load_owlvit2(device="cuda:0")
    tag_pose_tracker = ApriltagTracker(tag_size=0.06, # put your tag size here
                          intrinsic_matrix=K,
                          distortion_coeffs=D)
    while True:
        try:
            frames: FrameSet = pipeline.wait_for_frames(100)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            # covert to RGB format
            color_image = frame_to_bgr_image(color_frame)
            # topk_boxes=owl2_inference(model2, processor2,color_image,texts,1280,720)
            # print("boxes",topk_boxes)
            trans = get_robot_pose(rob,ip)  # get current transformation matrix (tool to base)
            pos = trans.pos._data  # [x, y, z]
            # orient = trans.orient.rv._data  # (rx, ry, rz)
            rotation = R.from_rotvec(trans.orient.rv._data )
            euler_angles = rotation.as_euler('xyz', degrees=False)
            base_T_hand = get_homogeneous_transformation(pos, euler_angles)
            # print("base_T_hand",base_T_hand)
            cam_T_tag = tag_pose_tracker.getPoseAndCorners(color_image, tag_id=0)
            if cam_T_tag is not None:
                corners=cam_T_tag['corners']
                # print("corners",corners)
                left_top = corners[1].astype(int)       # 第2个点
                right_bottom = corners[3].astype(int)    # 第4个点
                # 构建边界框
                bounding_box = [left_top[0], left_top[1], right_bottom[0], right_bottom[1]]
                # print("bounding_box",bounding_box)
                # 将其转换为 Nx4 的 NumPy 数组
                bounding_boxes = np.array([bounding_box])
                pose=cam_T_tag['pose']
                # print("cam_T_tag",pose)
                color_image=draw_boxes_on_image(color_image,bounding_boxes)
                Y = np.array(
                    [[-9.99978664e-01, -6.09311932e-04 ,-6.50387202e-03, -3.57293585e-01],
                    [ 2.65102085e-03,  8.72110881e-01 ,-4.89301118e-01 , 5.29347377e-02],
                    [ 5.97023457e-03 ,-4.89307920e-01, -8.72090658e-01 , 6.74492540e-01],
                    [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00  ,1.00000000e+00]]
                )
                base_T_tag = np.dot(Y, pose)
                # print("base_T_tag",base_T_tag)
                
                

            else:
                print("No tag detected")
                pose=None
            # print("cam_T_tag",cam_T_tag)
            if color_image is None:
                print("failed to convert frame to image")
                # continue
            cv2.imshow("Color Viewer", color_image)
            ur5_order = homogeneous_to_rot_trans(base_T_tag)
            # ur5_order[2] += 0.15
            print("go to the tag",ur5_order)
            rob.movel(ur5_order, acc=0.1, vel=0.1, relative=False)
            key = cv2.waitKey(1) 
            if key == ord('q') or key == ESC_KEY:
                break
            elif key == ord('s'):  # 按下's'键保存pose数据
                            if cam_T_tag is not None:
                                save_poses_to_file(base_T_hand,pose)
                                print("Pose data saved to file.")
        except KeyboardInterrupt:
            break
    pipeline.stop()


if __name__ == "__main__":
    main()




    # matrices1,matrices2=read_poses_from_file("pose_data.txt")
    # solver = OpenCVSolver(type='AX=YB')
    # X,Y = solver.solve(matrices1, matrices2)
    # print("X=")
    # print(X)
    # print("Y=")
    # print(Y)

    ##十组计算出的XY的结果
    # X=
    # [[-1.94978556e-02  1.25506568e-03 -9.99809111e-01  1.56594930e-02]
    #  [-9.94760632e-01  1.00349616e-01  1.95253716e-02  7.11611882e-03]
    #  [ 1.00354966e-01  9.94951446e-01 -7.08112388e-04  1.73658952e-01]
    #  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
    # Y=
    # [[-9.99978664e-01 -6.09311932e-04 -6.50387202e-03 -3.57293585e-01]
    #  [ 2.65102085e-03  8.72110881e-01 -4.89301118e-01  5.29347377e-02]
    #  [ 5.97023457e-03 -4.89307920e-01 -8.72090658e-01  6.74492540e-01]
    #  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

    