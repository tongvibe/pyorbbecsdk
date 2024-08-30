import cv2
from pyorbbecsdk import Config
from pyorbbecsdk import OBError
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import Pipeline, FrameSet
from pyorbbecsdk import VideoStreamProfile
from examples.utils import frame_to_bgr_image
import numpy as npq
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R
import cv2
import numpy as np
import matplotlib.pyplot as plt
import urx
import time
from SimpleHandEye.SimpleHandEye.interfaces.apriltag import ApriltagTracker
from SimpleHandEye.SimpleHandEye.solvers import OpenCVSolver
import math
from vibe_xgripper_v1 import *
import zmq
import numpy as np

context = zmq.Context()

sub_socket = context.socket(zmq.SUB)
sub_socket.connect("tcp://192.168.50.252:5556")  
sub_socket.setsockopt_string(zmq.SUBSCRIBE, "result")



ESC_KEY = 27
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


def pick_and_up(rob, gripper):
    print("Start to grasp")
    rob.translate_tool((0,0,0.1), acc=0.1, vel=0.1, wait=True, threshold=None)#参考系是夹爪的坐标，系需要改底层
    
    time.sleep(1)
    gripper.move_percentage(80, syncronous=True)
    time.sleep(1)
    rob.movel((0,0,0.1,0,0,0), acc=0.1, vel=0.1, relative=True)
    # rob.movel((0.103, -0.565, 0.196, 1.661, 0.0173, 0.0522), acc=0.1, vel=0.1, relative=False)
    # rob.movej((1.1793664693832397, -1.6889475027667444, 1.363736629486084, 0.25970005989074707, 1.6684476137161255, -3.1294618288623255), acc=0.1, vel=0.1)
    time.sleep(1)
def release_items(gripper):
    print("Release the grasp")
    gripper.move_percentage(0)
    time.sleep(1)

def main():
    ip = "192.168.51.254"
    rob = connect_to_robot(ip)  # 确保成功连接机器人
    rob.set_tcp((0, 0, 0.25, 0, 0, 0))
    rob.set_payload(1, (0, 0, 0.9))
    print("机器人连接成功")
    # rob.movel((0.103, -0.565, 0.196, 1.661, 0.0173, 0.0522), acc=0.1, vel=0.1, relative=False)
    # rob.movej((1.1793664693832397, -1.6889475027667444, 1.363736629486084, 0.25970005989074707, 1.6684476137161255, -3.1294618288623255), acc=0.01, vel=0.1)
    print("初始化机器人")

    HOST = "192.168.51.253"
    PORT = 8886 # Port to listen on (non-privileged ports are > 1023)
    gripper = Vibe_XGripper_V1(HOST, PORT, zeroing_on_boot=False)
    print("夹爪连接成功")
    gripper.move_percentage(0)
    print("初始化夹爪")

    # pipeline = setup_camera()
    # K, D = get_camera_parameters(pipeline)
    # tag_pose_tracker = ApriltagTracker(tag_size=0.06, intrinsic_matrix=K, distortion_coeffs=D)
    while True:
        try:
            # frames: FrameSet = pipeline.wait_for_frames(100)
            # if frames is None:
            #     continue
            # color_frame = frames.get_color_frame()
            # if color_frame is None:
            #     continue
            # # covert to RGB format
            # color_image = frame_to_bgr_image(color_frame)
            # pose = tag_pose_tracker.getPose(color_image, tag_id=0)
            # #稍微调整一下，base的xyz，大概x+=0.023 ，y-0.02
            # Y = np.array(
            #     [[-0.99691752, -0.04396394, -0.06498177, -0.335 ],  #-0.3301283   -0.35202257
            #     [ 0.00679575,  0.77674946, -0.62977305,  0.04730107],###0.05730
            #     [ 0.07816186, -0.62827339, -0.77405637,  0.66619011],
            #     [ 0.    ,      0.     ,     0.     ,     1.        ]]    
            # )

            # if pose is None:
            #     print("No tag detected")
            #     time.sleep(0.5)
            #     continue
            # else:
            #     base_T_tag = np.dot(Y, pose)
            # # print("base_T_tag",base_T_tag)
            
        
            # if color_image is None:
            #     print("failed to convert frame to image")
            #     # continue
            # cv2.imshow("Color Viewer", color_image)
            # ur5_order = homogeneous_to_rot_trans(base_T_tag)
            # print("go to the tag",ur5_order)

            topic,message = sub_socket.recv_multipart()
            
            result=np.frombuffer(message, dtype=np.float64)
            print("result",result)


            rob.movel(result, acc=0.01, vel=0.05, relative=False)
            pick_and_up(rob, gripper)
            time.sleep(0.5)
            release_items(gripper)
            time.sleep(0.5)







            key = cv2.waitKey(1) 
            if key == ord('q') or key == ESC_KEY:
                break
        except KeyboardInterrupt:
            break




    # while True:
    #     try:
    #         frames: FrameSet = pipeline.wait_for_frames(100)
    #         if frames is None:
    #             continue
    #         color_frame = frames.get_color_frame()
    #         if color_frame is None:
    #             continue
    #         # covert to RGB format
    #         color_image = frame_to_bgr_image(color_frame)
    #         cam_T_tag = tag_pose_tracker.getPoseAndCorners(color_image, tag_id=0)
    #         if cam_T_tag is not None:
    #             corners=cam_T_tag['corners']
    #             # print("corners",corners)
    #             left_top = corners[1].astype(int)       # 第2个点
    #             right_bottom = corners[3].astype(int)    # 第4个点
    #             # 构建边界框
    #             bounding_box = [left_top[0], left_top[1], right_bottom[0], right_bottom[1]]
    #             # print("bounding_box",bounding_box)
    #             # 将其转换为 Nx4 的 NumPy 数组
    #             bounding_boxes = np.array([bounding_box])
    #             pose=cam_T_tag['pose']
    #             # pose = tag_pose_tracker.getPose(color_image,tag_id=0)
    #             # print("cam_T_tag",pose)
    #             color_image=draw_boxes_on_image(color_image,bounding_boxes)
    #             # Y = np.array(
    #             #         [[-0.99689304, -0.04609481, -0.06387128, -0.35202257],
    #             #         [ 0.0044808,   0.77638688, -0.63024069,  0.05642099],
    #             #         [ 0.07863965, -0.62856875, -0.77376814,  0.66547692],
    #             #         [ 0.        ,  0.         , 0.        ,  1.        ]]
    #             # )

    #             #稍微调整一下，base的xyz，大概x+=0.023 ，y-0.02
    #             Y = np.array(
    #                [[-0.99691752, -0.04396394, -0.06498177, -0.335 ],  #-0.3301283   -0.35202257
    #                 [ 0.00679575,  0.77674946, -0.62977305,  0.04730107],###0.05730
    #                 [ 0.07816186, -0.62827339, -0.77405637,  0.66619011],
    #                 [ 0.    ,      0.     ,     0.     ,     1.        ]]    
    #             )


                
    #             base_T_tag = np.dot(Y, pose)
    #             # print("base_T_tag",base_T_tag)
                
    #         else:
    #             print("No tag detected")
    #             pose=None
    #         # print("cam_T_tag",cam_T_tag)
    #         if color_image is None:
    #             print("failed to convert frame to image")
    #             # continue
    #         cv2.imshow("Color Viewer", color_image)
    #         ur5_order = homogeneous_to_rot_trans(base_T_tag)
    #         print("go to the tag",ur5_order)
    #         rob.movel(ur5_order, acc=0.1, vel=0.1, relative=False)
    #         key = cv2.waitKey(1) 
    #         if key == ord('q') or key == ESC_KEY:
    #             break
    #     except KeyboardInterrupt:
    #         break
    # pipeline.stop()


if __name__ == "__main__":
    main()


    