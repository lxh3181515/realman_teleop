import numpy as np
import time
import argparse
import cv2
from multiprocessing import shared_memory, Array, Lock
from multiprocessing import Process, Queue
import threading

import zmq
import time
import json

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from teleop.open_television.tv_wrapper import TeleVisionWrapper
from teleop.image_server.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter

from teleop.robot_control.robot_hand_inspire import Inspire_Controller

import pickle
import socket


THUMB_INDEX_DISTANCE_MIN = 0.05  # Assuming a minimum Euclidean distance is 5 cm between thumb and index.
THUMB_INDEX_DISTANCE_MAX = 0.09  # Assuming a maximum Euclidean distance is 9 cm between thumb and index.
LEFT_MAPPED_MIN  = 0.0           # The minimum initial motor position when the gripper closes at startup.
RIGHT_MAPPED_MIN = 0.0           # The minimum initial motor position when the gripper closes at startup.
# The maximum initial motor position when the gripper closes before calibration (with the rail stroke calculated as 0.6 cm/rad * 9 rad = 5.4 cm).
LEFT_MAPPED_MAX = LEFT_MAPPED_MIN + 1000.0 
RIGHT_MAPPED_MAX = RIGHT_MAPPED_MIN + 1000.0



def matrix_to_rpy(matrix):
    """Extract RPY (ZYX convention) from a 4x4 homogeneous matrix."""
    R = matrix[:3, :3]  # 提取旋转矩阵部分

    # 判断是否接近奇异点（pitch = ±90°）
    if abs(R[2, 0]) != 1:
        pitch = -np.arcsin(R[2, 0])  # 负号是因为右手坐标系
        roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))
    else:
        # Gimbal Lock 情况
        yaw = 0
        if R[2, 0] == -1:
            pitch = np.pi / 2
            roll = yaw + np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = -yaw + np.arctan2(-R[0, 1], -R[0, 2])
    
    return roll, pitch, yaw  # 单位：弧度


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_dir', type = str, default = './utils/data', help = 'path to save data')
    parser.add_argument('--frequency', type = int, default = 30.0, help = 'save data\'s frequency')

    parser.add_argument('--record', action = 'store_true', help = 'Save data or not')
    parser.add_argument('--no-record', dest = 'record', action = 'store_false', help = 'Do not save data')
    parser.set_defaults(record = False)

    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1'], default='G1_29', help='Select arm controller')
    parser.add_argument('--hand', type=str, choices=['dex3', 'gripper', 'inspire1'], help='Select hand controller')

    args = parser.parse_args()
    print(f"args:{args}\n")

    # image client: img_config should be the same as the configuration in image_server.py (of Robot's development computing unit)
    img_config = {
        'fps':30,                                                          # frame per second
        'head_camera_type': 'realsense',                                  # opencv or realsense
        'head_camera_image_shape': [480, 640],                            # Head camera resolution  [height, width]
        'head_camera_id_numbers': ["335522072526"],                       # realsense camera's serial number
        # 'wrist_camera_type': 'opencv', 
        # 'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
        # 'wrist_camera_id_numbers': [0,1],                                 # '/dev/video0' and '/dev/video1' (opencv)
    }
    # img_config = {
    #     'fps': 30,
    #     'head_camera_type': 'opencv',
    #     'head_camera_image_shape': [480, 1280],  # Head camera resolution
    #     'head_camera_id_numbers': [0],
    #     'wrist_camera_type': 'opencv',
    #     'wrist_camera_image_shape': [480, 640],  # Wrist camera resolution
    #     'wrist_camera_id_numbers': [2, 4],
    # }


    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:5556")  # 绑定到端口

    # topic = "avp_arm_data"


    ASPECT_RATIO_THRESHOLD = 2.0 # If the aspect ratio exceeds this value, it is considered binocular
    if len(img_config['head_camera_id_numbers']) > 1 or (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
        BINOCULAR = True
    else:
        BINOCULAR = False
    if 'wrist_camera_type' in img_config:
        WRIST = True
    else:
        WRIST = False
    
    if BINOCULAR and not (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
        tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1] * 2, 3)
    else:
        tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1], 3)

    tv_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(tv_img_shape) * np.uint8().itemsize)
    tv_img_array = np.ndarray(tv_img_shape, dtype = np.uint8, buffer = tv_img_shm.buf)

    if WRIST:
        wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
        wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
        wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
        img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                 wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name, server_address='localhost')
    else:
        img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, server_address='localhost')

    image_receive_thread = threading.Thread(target = img_client.receive_process, daemon = True)
    image_receive_thread.daemon = True
    image_receive_thread.start()

    # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
    tv_wrapper = TeleVisionWrapper(BINOCULAR, tv_img_shape, tv_img_shm.name)
    
    realman_left_hand_array = np.zeros((75,), dtype=np.float64)
    realman_right_hand_array = np.zeros((75,), dtype=np.float64)
    # hand
    # if args.hand == "dex3":
    #     left_hand_array = Array('d', 75, lock = True)         # [input]
    #     right_hand_array = Array('d', 75, lock = True)        # [input]
    #     dual_hand_data_lock = Lock()
    #     dual_hand_state_array = Array('d', 14, lock = False)  # [output] current left, right hand state(14) data.
    #     dual_hand_action_array = Array('d', 14, lock = False) # [output] current left, right hand action(14) data.
    #     hand_ctrl = Dex3_1_Controller(left_hand_array, right_hand_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array)
    # elif args.hand == "gripper":
    #     left_hand_array = Array('d', 75, lock=True)
    #     right_hand_array = Array('d', 75, lock=True)
    #     dual_gripper_data_lock = Lock()
    #     dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
    #     dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
    #     gripper_ctrl = Gripper_Controller(left_hand_array, right_hand_array, dual_gripper_data_lock, dual_gripper_state_array, dual_gripper_action_array)
    # elif args.hand == "inspire1":
    #     left_hand_array = Array('d', 75, lock = True)          # [input]
    #     right_hand_array = Array('d', 75, lock = True)         # [input]
    #     dual_hand_data_lock = Lock()
    #     dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
    #     dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
    #     hand_ctrl = Inspire_Controller(left_hand_array, right_hand_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array)
    # else:
    #     pass


    # Inspire Hand
    left_hand_pos_array = Array('d', 75, lock = True)      # [input]
    right_hand_pos_array = Array('d', 75, lock = True)     # [input]
    dual_hand_data_lock = Lock()
    dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
    dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
    hand_ctrl = Inspire_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
    
    if args.record:
        recorder = EpisodeWriter(task_dir = args.task_dir, frequency = args.frequency, rerun_log = True)
        recording = False

    
    try:
        user_input = input("Please enter the start signal (enter 't' to start the IK program):\n")
        if user_input.lower() == 't':
            # arm_ctrl.speed_gradual_max()

            running = True
            while running:
                start_time = time.time()
                head_rmat, left_wrist, right_wrist, left_hand, right_hand = tv_wrapper.get_data()

                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = left_hand.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = right_hand.flatten()

                left_roll, left_pitch, left_yaw = matrix_to_rpy(left_wrist)
                right_roll, right_pitch, right_yaw = matrix_to_rpy(right_wrist)
                print(f"left wrist RPY: roll={left_roll}, pitch={left_pitch}, yaw={left_yaw}")
                print(f"right wrist RPY: roll={right_roll}, pitch={right_pitch}, yaw={right_yaw}")
                left_x, left_y, left_z = left_wrist[0, 3], left_wrist[1, 3], left_wrist[2, 3]
                right_x, right_y, right_z = right_wrist[0, 3], right_wrist[1, 3], right_wrist[2, 3]
                print(f"left wrist position: x={left_x}, y={left_y}, z={left_z}")
                print(f"right wrist position: x={right_x}, y={right_y}, z={right_z}")

                # send hand skeleton data to hand_ctrl.control_process
                # if args.hand:
                # realman_left_hand_array[:] = left_hand.flatten()
                # realman_right_hand_array[:] = right_hand.flatten()

                # left_hand_mat  = np.array(realman_left_hand_array[:]).reshape(25, 3).copy()
                # right_hand_mat = np.array(realman_right_hand_array[:]).reshape(25, 3).copy()

                # if not np.all(right_hand_mat == 0.0) and not np.all(left_hand_mat[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                #     left_euclidean_distance  = np.linalg.norm(left_hand_mat[9] - left_hand_mat[4])
                #     right_euclidean_distance = np.linalg.norm(right_hand_mat[9] - right_hand_mat[4])
                #     # Linear mapping from [0, THUMB_INDEX_DISTANCE_MAX] to gripper action range
                #     left_target_action  = np.interp(left_euclidean_distance, [THUMB_INDEX_DISTANCE_MIN, THUMB_INDEX_DISTANCE_MAX], [LEFT_MAPPED_MIN, LEFT_MAPPED_MAX])
                #     right_target_action = np.interp(right_euclidean_distance, [THUMB_INDEX_DISTANCE_MIN, THUMB_INDEX_DISTANCE_MAX], [RIGHT_MAPPED_MIN, RIGHT_MAPPED_MAX])
                # else:
                #     left_target_action  = (LEFT_MAPPED_MIN + LEFT_MAPPED_MAX) / 2
                #     right_target_action = (RIGHT_MAPPED_MIN + RIGHT_MAPPED_MAX) / 2

                # left_target_action =np.zeros(1)
                # right_target_action = np.zeros(1)

                
                # np concat left wrist position, left wrist orientation, right wrist position, right wrist orientation, left_target_action and right_target_action
                # 左手腕位置 [x, y, z]
                left_pos = [left_x, left_y, left_z]
                # 左手腕姿态 [roll, pitch, yaw]
                left_rpy = [left_roll, left_pitch, left_yaw]

                # 右手腕位置 [x, y, z]
                right_pos = [right_x, right_y, right_z]
                # 右手腕姿态 [roll, pitch, yaw]
                right_rpy = [right_roll, right_pitch, right_yaw]

                # 左右手抓取动作
                # gripper_actions = [left_target_action, right_target_action]
                with dual_hand_data_lock:
                    gripper_actions = dual_hand_action_array

                # 拼接成一个 numpy array
                current_command = np.concatenate([left_pos, left_rpy, right_pos, right_rpy, gripper_actions])

                upper_q_json = json.dumps(current_command.tolist())
                zmq_msg = f"avp_upper_command {upper_q_json}"
                socket.send_string(zmq_msg)
                print(f"[ZMQ Published] {zmq_msg}")


                # # get current state data.
                # current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
                # current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

                # # solve ik using motor data and wrist pose, then use ik results to control arms.
                # time_ik_start = time.time()
                # sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist, right_wrist, current_lr_arm_q, current_lr_arm_dq)
                # time_ik_end = time.time()
                # # print(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")

                # if sol_q is not None:
                #     sol_q_np = np.array(sol_q)  # 确保它是可序列化的结构
                #     sol_q_json = json.dumps(sol_q_np.tolist())  # 转换成 list，再转成 json 字符串
                #     zmq_msg = f"{topic} {sol_q_json}"
                #     socket.send_string(zmq_msg)
                #     print(f"[ZMQ Published] {zmq_msg}")

                tv_resized_image = cv2.resize(tv_img_array, (tv_img_shape[1] // 2, tv_img_shape[0] // 2))
                cv2.imshow("record image", tv_resized_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    running = False
                elif key == ord('s') and args.record:
                    recording = not recording # state flipping
                    if recording:
                        if not recorder.create_episode():
                            recording = False
                    else:
                        recorder.save_episode()

                # # record data
                # if args.record:
                #     # dex hand or gripper
                #     if args.hand == "dex3":
                #         with dual_hand_data_lock:
                #             left_hand_state = dual_hand_state_array[:7]
                #             right_hand_state = dual_hand_state_array[-7:]
                #             left_hand_action = dual_hand_action_array[:7]
                #             right_hand_action = dual_hand_action_array[-7:]
                #     elif args.hand == "gripper":
                #         with dual_gripper_data_lock:
                #             left_hand_state = [dual_gripper_state_array[1]]
                #             right_hand_state = [dual_gripper_state_array[0]]
                #             left_hand_action = [dual_gripper_action_array[1]]
                #             right_hand_action = [dual_gripper_action_array[0]]
                #     elif args.hand == "inspire1":
                #         with dual_hand_data_lock:
                #             left_hand_state = dual_hand_state_array[:6]
                #             right_hand_state = dual_hand_state_array[-6:]
                #             left_hand_action = dual_hand_action_array[:6]
                #             right_hand_action = dual_hand_action_array[-6:]
                #     else:
                #         print("No dexterous hand set.")
                #         pass
                #     # head image
                #     current_tv_image = tv_img_array.copy()
                #     # wrist image
                #     if WRIST:
                #         current_wrist_image = wrist_img_array.copy()
                #     # # arm state and action
                #     # left_arm_state  = current_lr_arm_q[:7]
                #     # right_arm_state = current_lr_arm_q[-7:]
                #     # left_arm_action = sol_q[:7]
                #     # right_arm_action = sol_q[-7:]

                #     # if recording:
                #     #     colors = {}
                #     #     depths = {}
                #     #     if BINOCULAR:
                #     #         colors[f"color_{0}"] = current_tv_image[:, :tv_img_shape[1]//2]
                #     #         colors[f"color_{1}"] = current_tv_image[:, tv_img_shape[1]//2:]
                #     #         if WRIST:
                #     #             colors[f"color_{2}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                #     #             colors[f"color_{3}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                #     #     else:
                #     #         colors[f"color_{0}"] = current_tv_image
                #     #         if WRIST:
                #     #             colors[f"color_{1}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                #     #             colors[f"color_{2}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                #     #     states = {
                #     #         "left_arm": {                                                                    
                #     #             "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                #     #             "qvel":   [],                          
                #     #             "torque": [],                        
                #     #         }, 
                #     #         "right_arm": {                                                                    
                #     #             "qpos":   right_arm_state.tolist(),       
                #     #             "qvel":   [],                          
                #     #             "torque": [],                         
                #     #         },                        
                #     #         "left_hand": {                                                                    
                #     #             "qpos":   left_hand_state,           
                #     #             "qvel":   [],                           
                #     #             "torque": [],                          
                #     #         }, 
                #     #         "right_hand": {                                                                    
                #     #             "qpos":   right_hand_state,       
                #     #             "qvel":   [],                           
                #     #             "torque": [],  
                #     #         }, 
                #     #         "body": None, 
                #     #     }
                #     #     actions = {
                #     #         "left_arm": {                                   
                #     #             "qpos":   left_arm_action.tolist(),       
                #     #             "qvel":   [],       
                #     #             "torque": [],      
                #     #         }, 
                #     #         "right_arm": {                                   
                #     #             "qpos":   right_arm_action.tolist(),       
                #     #             "qvel":   [],       
                #     #             "torque": [],       
                #     #         },                         
                #     #         "left_hand": {                                   
                #     #             "qpos":   left_hand_action,       
                #     #             "qvel":   [],       
                #     #             "torque": [],       
                #     #         }, 
                #     #         "right_hand": {                                   
                #     #             "qpos":   right_hand_action,       
                #     #             "qvel":   [],       
                #     #             "torque": [], 
                #     #         }, 
                #     #         "body": None, 
                #     #     }
                #     #     recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / float(args.frequency)) - time_elapsed)
                time.sleep(sleep_time)
                # print(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        print("KeyboardInterrupt, exiting program...")
    finally:     
        # arm_ctrl.ctrl_dual_arm_go_home()
        tv_img_shm.unlink()
        tv_img_shm.close()
        if WRIST:
            wrist_img_shm.unlink()
            wrist_img_shm.close()
        if args.record:
            recorder.close()
        print("Finally, exiting program...")
        exit(0)