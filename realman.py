import zmq
import json
import socket
import time
import pyrealsense2 as rs
import cv2
import numpy as np
import threading

LEFT_TCP_IP = "169.254.128.18"   # 替换为左臂机器人IP
RIGHT_TCP_IP = "169.254.128.19" # 替换为右臂机器人IP
ROBOT_TCP_PORT = 8080             # 端口假定一致

def parse_command(command):
    """
    解析zmq消息，返回左右手腕的目标位姿和夹爪动作
    command: [lx, ly, lz, lroll, lpitch, lyaw, rx, ry, rz, rroll, rpitch, ryaw, lgrip, rgrip]
    """
    left = {
        "position": command[0:3],
        "rpy": command[3:6],
        "gripper": command[12]
    }
    right = {
        "position": command[6:9],
        "rpy": command[9:12],
        "gripper": command[13]
    }
    return left, right

def build_movej_payload(pos, rpy):
    """
    构造movej-p接口的payload，格式如下：
    {"command":"movej_p","pose":[x_mm,y_mm,z_mm,rx,ry,rz],"v":50,"r":0,"trajectory_connect":0}
    """
    payload = {
        "command": "movej_p",
        "pose": [
            int(pos[0] * 1000000),  # x 米转毫米 * 1000
            int(pos[1] * 1000000),  # y 米转毫米 * 1000
            int(pos[2] * 1000000),  # z 米转毫米 * 1000
            rpy[0] * 1000,              # rx 弧度 * 1000
            rpy[1] * 1000,              # ry 弧度 * 1000 
            rpy[2] * 1000               # rz 弧度 * 1000
        ],
        "v": 50,                 # 速度系数
        "r": 0,                  # 交融半径
        "trajectory_connect": 0  # 不交融
    }
    return payload

def send_robot_command(left, right, s_left, s_right):
    left_payload = build_movej_payload(left["position"], left["rpy"])
    right_payload = build_movej_payload(right["position"], right["rpy"])
    try:
        s_left.sendall((json.dumps(left_payload) + "\r\n").encode())
        s_right.sendall((json.dumps(right_payload) + "\r\n").encode())
    except Exception as e:
        print(f"Robot control error: {e}")

    # 夹爪控制（如有接口，可补充）

def image_send_process(zmq_pub_addr="tcp://*:5555"):
    """
    采集realsense图像并通过ZMQ PUB发送
    """
    # 初始化realsense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # 初始化ZMQ PUB
    context = zmq.Context()
    socket_pub = context.socket(zmq.PUB)
    socket_pub.bind(zmq_pub_addr)

    print("Image sender started.")
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            # JPEG编码
            _, jpg_bytes = cv2.imencode('.jpg', color_image)
            # 发送
            socket_pub.send(jpg_bytes.tobytes())
            time.sleep(0.03)  # 约30fps
    except KeyboardInterrupt:
        print("Image sender stopped.")
    finally:
        pipeline.stop()
        socket_pub.close()
        context.term()

def main():
    # 启动图像发送线程
    img_thread = threading.Thread(target=image_send_process, daemon=True)
    img_thread.start()

    context = zmq.Context()
    socket_zmq = context.socket(zmq.SUB)
    socket_zmq.connect("tcp://localhost:5556")  # 与teleop_hand_and_arm.py一致
    socket_zmq.setsockopt_string(zmq.SUBSCRIBE, "avp_upper_command")

    # 建立左右臂TCP连接
    try:
        s_left = socket.create_connection((LEFT_TCP_IP, ROBOT_TCP_PORT), timeout=2)
        s_right = socket.create_connection((RIGHT_TCP_IP, ROBOT_TCP_PORT), timeout=2)
    except Exception as e:
        print(f"Failed to connect to robot arms: {e}")
        return

    print("Robot control started. Waiting for commands...")
    try:
        while True:
            msg = socket_zmq.recv_string()
            topic, data_json = msg.split(' ', 1)
            command = json.loads(data_json)
            left, right = parse_command(command)
            send_robot_command(left, right, s_left, s_right)
            time.sleep(0.01)  # 可调整
    except KeyboardInterrupt:
        print("Exiting robot control...")
    finally:
        socket_zmq.close()
        context.term()
        s_left.close()
        s_right.close()

if __name__ == "__main__":
    main()