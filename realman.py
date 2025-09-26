import zmq
import json
import socket
import time
import pyrealsense2 as rs
import cv2
import numpy as np
import threading
from hand_controller import ArmModbusGateway  # 新增：导入手部Modbus网关

LEFT_TCP_IP = "169.254.128.18"   # 替换为左臂机器人IP
RIGHT_TCP_IP = "169.254.128.19" # 替换为右臂机器人IP
ROBOT_TCP_PORT = 8080             # 端口假定一致

SPEED = 10  # 速度系数



def parse_command(command):
    """
    解析zmq消息，返回左右手腕的目标位姿和夹爪动作
    command: [lx, ly, lz, lroll, lpitch, lyaw, rx, ry, rz, rroll, rpitch, ryaw, lhand, rhand]
    """
    hand = command[12:]
    left = {
        "position": command[0:3],
        "rpy": command[3:6],
        "hand": hand[:7]
    }
    right = {
        "position": command[6:9],
        "rpy": command[9:12],
        "hand": hand[-7:]
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
        "v": min(SPEED, 100),                 # 速度系数
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

def normalized_hand_to_registers(hand_norm, scale=1000, expected_len=None):
    """
    将归一化手数据转换为控制器可写入的整数寄存器值。
    支持两种归一化域：
      - 若值在 [-1, 1]：按 (v+1)/2 映射到 [0, scale]
      - 否则假定在 [0, 1]：按 v 映射到 [0, scale]
    返回整数列表（每项为 0..scale 的整数）。
    """
    regs = []
    for v in hand_norm:
        try:
            fv = float(v)
        except Exception:
            fv = 0.0
        # 判断范围并映射
        if -1.0 <= fv <= 1.0:
            mapped = (fv + 1.0) / 2.0
        else:
            mapped = max(0.0, min(1.0, fv))
        val = int(round(mapped * scale))
        regs.append(val)
    if expected_len is not None and len(regs) < expected_len:
        regs.extend([0] * (expected_len - len(regs)))
    return regs

def main():
    # 启动图像发送线程
    img_thread = threading.Thread(target=image_send_process, daemon=True)
    img_thread.start()

    context = zmq.Context()
    socket_zmq = context.socket(zmq.SUB)
    socket_zmq.connect("tcp://localhost:5556")  # 与teleop_hand_and_arm.py一致
    socket_zmq.setsockopt_string(zmq.SUBSCRIBE, "avp_upper_command")

    # 建立左右臂TCP连接（运动控制）
    try:
        s_left = socket.create_connection((LEFT_TCP_IP, ROBOT_TCP_PORT), timeout=2)
        s_right = socket.create_connection((RIGHT_TCP_IP, ROBOT_TCP_PORT), timeout=2)
    except Exception as e:
        print(f"Failed to connect to robot arms: {e}")
        return

    # 新增：建立左右手的 Modbus 网关（用于灵巧手控制）
    left_gateway = ArmModbusGateway(LEFT_TCP_IP, arm_port=ROBOT_TCP_PORT)
    right_gateway = ArmModbusGateway(RIGHT_TCP_IP, arm_port=ROBOT_TCP_PORT)

    if not left_gateway.connect():
        print("Warning: 无法连接左侧机械臂控制器的Modbus接口，左手将不可控。")
        left_gateway = None
    else:
        if not left_gateway.setup_tool_modbus(timeout_ms=500):
            print("Warning: 左侧Modbus模式配置失败，左手可能不可控。")

    if not right_gateway.connect():
        print("Warning: 无法连接右侧机械臂控制器的Modbus接口，右手将不可控。")
        right_gateway = None
    else:
        if not right_gateway.setup_tool_modbus(timeout_ms=500):
            print("Warning: 右侧Modbus模式配置失败，右手可能不可控。")

    print("Robot control started. Waiting for commands...")
    try:
        while True:
            msg = socket_zmq.recv_string()
            topic, data_json = msg.split(' ', 1)
            command = json.loads(data_json)
            left, right = parse_command(command)

            # 发送臂运动指令（保留原有行为）
            # send_robot_command(left, right, s_left, s_right)

            # 新增：处理灵巧手控制，hand已经归一化
            # left['hand'] 和 right['hand'] 预期为数字列表
            try:
                # 将归一化数据转换为寄存器值（映射到0-1000）
                left_regs = normalized_hand_to_registers(left.get("hand", []), scale=2000)
                right_regs = normalized_hand_to_registers(right.get("hand", []), scale=2000)

                # 异步写入以避免阻塞主循环
                if left_gateway:
                    threading.Thread(target=left_gateway.write_registers, args=('angleSet', left_regs), daemon=True).start()
                if right_gateway:
                    threading.Thread(target=right_gateway.write_registers, args=('angleSet', right_regs), daemon=True).start()
            except Exception as e:
                print(f"Hand control error: {e}")

            time.sleep(0.01)  # 可调整
    except KeyboardInterrupt:
        print("Exiting robot control...")
    finally:
        socket_zmq.close()
        context.term()
        s_left.close()
        s_right.close()

        # 退出前关闭Modbus并断开连接
        try:
            if left_gateway:
                left_gateway.close_tool_modbus()
                left_gateway.disconnect()
        except Exception:
            pass
        try:
            if right_gateway:
                right_gateway.close_tool_modbus()
                right_gateway.disconnect()
        except Exception:
            pass

if __name__ == "__main__":
    main()