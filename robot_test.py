import json
import socket

LEFT_TCP_IP = "169.254.128.18"   # 左臂机器人IP
ROBOT_TCP_PORT = 8080            # 端口

def build_movej_payload(pos, rpy):
    """
    构造movej-p接口的payload，格式如下：
    {"command":"movej_p","pose":[x_mm,y_mm,z_mm,rx,ry,rz],"v":50,"r":0,"trajectory_connect":0}
    """
    payload = {
        "command": "movej_p",
        "pose": [
            int(pos[0] * 1000),   # x 米转毫米
            int(pos[1] * 1000),   # y 米转毫米
            int(pos[2] * 1000),   # z 米转毫米
            rpy[0],               # rx 弧度
            rpy[1],               # ry 弧度
            rpy[2]                # rz 弧度
        ],
        "v": 50,
        "r": 0,
        "trajectory_connect": 0
    }
    return payload

if __name__ == "__main__":
    # 示例左臂目标
    left_pos = [0.1, 0.2, 0.03]      # 单位: 米
    left_rpy = [0.4, 0.5, 0.6]       # 单位: 弧度

    payload = build_movej_payload(left_pos, left_rpy)
    print(json.dumps(payload))

    # 实际发送到机械臂
    try:
        s_left = socket.create_connection((LEFT_TCP_IP, ROBOT_TCP_PORT), timeout=2)
        s_left.sendall((json.dumps(payload) + "\r\n").encode())
        s_left.close()
        print("Payload sent to robot.")
    except Exception as e:
        print(f"Send error: {e}")
