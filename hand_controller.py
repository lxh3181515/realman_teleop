import socket
import json
import time
import sys

class ArmModbusGateway:
    """
    通过机械臂控制器的Modbus网关功能，直接控制末端灵巧手。
    V3: 修正了写入/读取指令的data字段格式，以匹配控制器期望的字节流。
    """
    def __init__(self, arm_ip, arm_port=8080):
        self.arm_ip = arm_ip
        self.arm_port = arm_port
        self.sock = None
        self.TOOL_PORT_ID = 1
        self.HAND_MODBUS_ID = 1
        self.HAND_BAUDRATE = 115200
        self.REG_ADDR = {
            'angleSet': 1486,
            'forceSet': 1498,
            'speedSet': 1522,
            'angleAct': 1546
        }

    def connect(self):
        print(f"--> 正在连接机械臂控制器: {self.arm_ip}:{self.arm_port}")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)
            self.sock.connect((self.arm_ip, self.arm_port))
            print("--> 连接成功！")
            return True
        except socket.error as e:
            print(f"!! 错误：连接失败 - {e}")
            self.sock = None
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            print("--> 已断开与机械臂的连接。")

    def _send_and_receive(self, command_dict):
        if not self.sock:
            print("!! 错误：未连接。")
            return None
        
        try:
            json_command = json.dumps(command_dict) + "\r\n"
            print(f"   [发送JSON]: {json_command.strip()}")
            self.sock.sendall(json_command.encode('utf-8'))
            
            response_raw = self.sock.recv(1024)
            response_str = response_raw.decode('utf-8').strip()
            
            if not response_str:
                print("!! 警告: 未收到回复。")
                return None

            print(f"   [接收JSON]: {response_str}")
            return json.loads(response_str)

        except json.JSONDecodeError:
            print(f"!! 错误: 无法解析返回的JSON: {response_str}")
            return None
        except socket.timeout:
            print("!! 错误: 等待回复超时。")
            return None
        except socket.error as e:
            print(f"!! 错误：网络通信失败 - {e}")
            return None

    def setup_tool_modbus(self, timeout_ms=500): # 稍微增加超时时间
        print("\n[步骤 1]: 正在配置末端接口为Modbus模式...")
        command = {
            "command": "set_modbus_mode",
            "port": self.TOOL_PORT_ID,
            "baudrate": self.HAND_BAUDRATE,
            "timeout": timeout_ms // 100
        }
        response = self._send_and_receive(command)
        if response and response.get("set_state") is True:
            print(">>> Modbus模式配置成功！")
            return True
        else:
            print("!! 错误: Modbus模式配置失败。")
            return False

    def close_tool_modbus(self):
        print("\n[步骤 3]: 正在关闭Modbus模式...")
        command = {
            "command": "close_modbus_mode",
            "port": self.TOOL_PORT_ID
        }
        self._send_and_receive(command) # 发送即可，不强求确认
        print(">>> 已发送关闭Modbus模式指令。")

    def write_registers(self, reg_name, int16_values):
        """
        【核心修正】将16位整数列表转换为字节列表后发送。
        """
        print(f"\n--- 正在写入 {reg_name} ---")
        if reg_name not in self.REG_ADDR:
            print(f"!! 错误: 未知的寄存器名称 '{reg_name}'")
            return False
        
        # 将16位整数列表转换为字节流列表
        byte_data = []
        for value in int16_values:
            # 确保值在有效范围内
            value = max(0, min(65535, value))
            high_byte = (value >> 8) & 0xFF  # 高8位
            low_byte = value & 0xFF         # 低8位
            byte_data.append(high_byte)
            byte_data.append(low_byte)
            
        command = {
            "command": "write_registers",
            "port": self.TOOL_PORT_ID,
            "device": self.HAND_MODBUS_ID,
            "address": self.REG_ADDR[reg_name],
            "num": len(int16_values), # 寄存器数量
            "data": byte_data         # 【修正】发送字节流
        }
        response = self._send_and_receive(command)
        if response and response.get("write_state") is True:
            print(f">>> {reg_name} 写入指令发送成功。")
            return True
        else:
            print(f"!! 错误: {reg_name} 写入指令发送失败。可能是因为控制器未返回确认。")
            return False

    def read_registers(self, reg_name, num_regs=6):
        """
        【核心修正】读取字节流并将其解析为16位整数列表。
        """
        print(f"\n--- 正在读取 {reg_name} ---")
        if reg_name not in self.REG_ADDR:
            print(f"!! 错误: 未知的寄存器名称 '{reg_name}'")
            return None
            
        command = {
            "command": "read_multiple_holding_registers",
            "port": self.TOOL_PORT_ID,
            "device": self.HAND_MODBUS_ID,
            "address": self.REG_ADDR[reg_name],
            "num": num_regs
        }
        response = self._send_and_receive(command)
        if response and "data" in response:
            byte_data = response['data']
            # 【修正】将返回的字节流解析为16位整数
            int16_values = []
            if len(byte_data) % 2 == 0:
                for i in range(0, len(byte_data), 2):
                    high_byte = byte_data[i]
                    low_byte = byte_data[i+1]
                    value = (high_byte << 8) + low_byte
                    int16_values.append(value)
                print(f">>> 读取成功，当前 {reg_name} 值为: {int16_values}")
                return int16_values
            else:
                print(f"!! 错误: 返回的字节数不是偶数: {byte_data}")
                return None
        else:
            print(f"!! 错误: 读取 {reg_name} 失败。")
            return None

# --- 主程序入口 ---
if __name__ == '__main__':
    ARM_IP_ADDRESS = "169.254.128.19"  # 使用您日志中成功的IP地址

    gateway = ArmModbusGateway(ARM_IP_ADDRESS)

    if not gateway.connect():
        sys.exit(1)

    try:
        if not gateway.setup_tool_modbus():
            gateway.disconnect()
            sys.exit(1)

        time.sleep(0.5)

        print("\n[初始化]: 正在设置默认速度和力度...")
        gateway.write_registers('speedSet', [300, 300, 300, 300, 300, 300])
        time.sleep(0.2)
        gateway.write_registers('forceSet', [200, 200, 400, 600, 800, 500])
        time.sleep(0.2)
        
        print("\n==============================================")
        print("✅ 已进入Modbus直接控制模式 (V3 - 字节流修正版)")
        print("   现在写入指令格式已修正，请重试。")
        print("----------------------------------------------")
        print(" 请输入6个关节的角度值 (0-1000), 用空格或逗号分隔。")
        print(" 特殊指令: 'read' (读取角度), 'exit' (退出)")
        print("==============================================")
        
        while True:
            command_input = input("\n请输入指令 > ").strip()
            
            if command_input.lower() == 'exit':
                break
            
            if command_input.lower() == 'read':
                gateway.read_registers('angleAct')
                continue

            try:
                parts = command_input.replace(',', ' ').split()
                angles = [int(p) for p in parts]
                
                if len(angles) != 6:
                    print("!! 错误: 必须输入6个角度值。")
                    continue
                
                if gateway.write_registers('angleSet', angles):
                    time.sleep(1.5)
                    print("   (自动回读以验证...)")
                    gateway.read_registers('angleAct')

            except ValueError:
                print("!! 错误: 请输入有效的整数。")
            except Exception as e:
                print(f"!! 执行指令时发生未知错误: {e}")

    except KeyboardInterrupt:
        print("\n程序被用户中断。")
    finally:
        gateway.close_tool_modbus()
        gateway.disconnect()
        print("\n程序已安全退出。")