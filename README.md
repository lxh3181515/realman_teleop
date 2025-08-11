<div align="center">
  <h1 align="center"> realman_teleop </h1>
  <h3 align="center"> 基于 Unitree avp_teleoperate 的 VR 手部姿态远程控制 </h3>
  <p align="center">
    <a> 中文 </a> | <a href="README_en.md">English</a>
  </p>
</div>

# 📖 项目介绍

**realman_teleop** 是在 [Unitree Robotics avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate) 项目的基础上开发的远程操作程序，旨在使用 **XR 设备**（如 Apple Vision Pro、PICO 4 Ultra Enterprise、Meta Quest 3）获取操作员手部姿态，并通过网络传输到 REALMAN 机器人进行执行。

与原项目不同的是，本项目将 **VR 手部姿态解析** 和 **机械臂逆解（IK）计算** 分离：
- VR 手部姿态解析仍由 `teleop_hand_and_arm.py` 在 **用户主机（Host）** 上运行。
- IK 计算和机械臂控制部分放在 **REALMAN 本体的计算机** 上进行。

> ⚠️ 当前版本 IK 使用 **MOVEJ** 方式实现，可能在某些姿态下无法解算，导致机械臂停机。这是已知限制，欢迎大家 PR 实现 **基于 QP 求解的 IK** 以提升稳定性和成功率。

---

# 🛠 支持功能

| 功能                     | 状态   |
| ------------------------ | ------ |
| XR 设备手部姿态解析       | ✅ 完成 |
| 姿态网络传输              | ✅ 完成 |
| REALMAN 端 MOVEJ 控制     | ✅ 完成（存在失败可能） |
| QP IK 控制                | ⏳ 待实现，欢迎 PR |
| 数据录制与可视化          | ✅ 完成 |

---

# 📦 安装与环境

本项目在 Ubuntu 20.04 / 22.04 测试，其他系统可能需调整配置。

## 1. 创建 conda 环境
```bash
conda create -n tv python=3.8
conda activate tv
```
## 2. 安装依赖

安装 pinocchio 版本
```bash
conda install pinocchio=3.1.0 -c conda-forge
pip install meshcat casadi
```

## 3. 下载本项目
```bash
git clone https://github.com/yourusername/realman_teleop.git
cd realman_teleop
pip install -r requirements.txt
```

⚙️ 使用方法
1. VR 手部姿态解析（Host 端）

在 用户主机（Host） 上运行 teleop_hand_and_arm.py，此进程负责：

    与 XR 设备通信，解析手部姿态

    通过网络将姿态数据发送到 REALMAN 本体

示例命令：

python teleop_hand_and_arm.py 

    --arm 选项此处仅用于指定姿态数据结构，不在 Host 上执行 IK

运行后，会在终端显示：

Please enter the start signal (enter 'r' to start the subsequent program):

此时按下 r 即可开始数据发送。
1. REALMAN 端控制

REALMAN 本体电脑上运行 MOVEJ 控制程序：

    接收来自 Host 的手部姿态数据

    使用 MOVEJ 方法进行 IK 并控制机械臂运动

⚠️ 注意：

    由于 MOVEJ 对 IK 不进行优化求解，在某些目标姿态下可能解算失败。

    一旦失败，机械臂会停机，需要人工重新启动。

    建议后续替换为 QP 求解器（例如使用 cvxpy 或 osqp）来实现更鲁棒的 IK。

3. Apple Vision Pro / XR 设备连接

如果使用 Apple Vision Pro，需要配置 HTTPS 本地连接（参考原项目的 mkcert 配置方法）：

    使用 mkcert 生成本地证书

    将证书安装到 XR 设备

    在 XR 设备浏览器中访问：

https://<Host_IP>:8012?ws=wss://<Host_IP>:8012

    点击 Enter VR 并允许权限

🧩 项目结构

realman_teleop/
│
├── teleop/
│   ├── image_server/          # 图像采集与传输（可选）
│   ├── open_television/       # XR 设备数据采集
│   ├── robot_control/         # 原有控制逻辑（本项目不在此执行 IK）
│   ├── utils/                  # 工具函数与数据录制
│   └── teleop_hand_and_arm.py  # VR 手部姿态解析与网络发送主程序
│
└── README.md

🔮 后续计划

增加 QP IK 求解器，提升姿态跟随稳定性

增加 自动恢复机制，防止停机

    支持更多 XR 设备手部跟踪接口

🙏 致谢

本项目基于以下开源项目开发：

    Unitree avp_teleoperate

    OpenTeleVision

    pinocchio

    以及其他相关依赖项目