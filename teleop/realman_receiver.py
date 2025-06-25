import zmq
import json
import numpy as np

# 创建 ZMQ 上下文和 SUB socket
context = zmq.Context()
socket = context.socket(zmq.SUB)

# 连接到发布端（注意：用实际 IP 或 localhost）
socket.connect("tcp://192.168.131.57:5556")

# 订阅主题（只接收以 "avp_upper_command" 开头的消息）
socket.setsockopt_string(zmq.SUBSCRIBE, "avp_upper_command")

print("Waiting for avp_upper_command messages...")

while True:
    try:
        message = socket.recv_string()
        topic, data_json = message.split(" ", 1)  # 拆分主题与内容
        data_list = json.loads(data_json)
        command_array = np.array(data_list)

        print(f"Received command: {command_array}")
        # 可以进一步处理 command_array（长度14：位姿 + 动作）
        parsed_command = {
            "left_pos": command_array[0:3],
            "left_rpy": command_array[3:6],
            "right_pos": command_array[6:9],
            "right_rpy": command_array[9:12],
            "left_action": command_array[12],
            "right_action": command_array[13],
        }
        print("Left Wrist Position:", parsed_command["left_pos"])
        print("Left Wrist Orientation (RPY):", parsed_command["left_rpy"])
        print("Left Gripper Target Action:", parsed_command["left_action"])


    except KeyboardInterrupt:
        print("Interrupted, closing socket.")
        break
    except Exception as e:
        print(f"Error receiving message: {e}")