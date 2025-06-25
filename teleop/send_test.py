import zmq
import time
import json
import numpy as np

def generate_dummy_command():
    # 模拟左/右手位置与RPY，以及夹爪动作
    left_pos = [-0.23071, 0.271331, 0.02564]
    left_rpy = [1.2798433968504108, -0.06450288096899, 2.5688815793]
    right_pos = [0.21549, 0.333, 0.01093]
    right_rpy = [-1.3759377691, -0.2797421051056, 0.887657994]
    gripper_actions = [500, 500]

    command = np.concatenate([left_pos, left_rpy, right_pos, right_rpy, gripper_actions])
    return command.tolist()

def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")  # 与主程序保持一致

    print("ZMQ 测试发送器启动，向端口 5556 发送消息...\n")

    try:
        while True:
            command = generate_dummy_command()
            json_command = json.dumps(command)
            zmq_msg = f"avp_upper_command {json_command}"
            socket.send_string(zmq_msg)
            print(f"[ZMQ 测试已发送] {zmq_msg}")
            time.sleep(1)  # 每秒发送一次
    except KeyboardInterrupt:
        print("\n发送终止。")
    finally:
        socket.close()
        context.term()

if __name__ == '__main__':
    main()
