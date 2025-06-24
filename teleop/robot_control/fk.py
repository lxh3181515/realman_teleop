import pinocchio as pin
import numpy as np
import os
from pinocchio import casadi as cpin    


model_path = "/home/ljc/realman_teleop/assets/robot_description"
urdf_path = os.path.join(model_path, "urdf/robots/embodied_dual_arms.urdf")

# 创建临时文件用于替换后的 URDF
with open(urdf_path, 'r') as f:
    urdf_content = f.read()

# 替换 package://robot_description 为实际路径
urdf_content = urdf_content.replace("package://robot_description", model_path)

# 写入临时文件
fixed_urdf_path = "/tmp/embodied_dual_arms_fixed.urdf"
with open(fixed_urdf_path, 'w') as f:
    f.write(urdf_content)

# 构建模型
robot = pin.RobotWrapper.BuildFromURDF(fixed_urdf_path, [model_path])



mixed_jointsToLockIDs = [
                                "universe" ,
                                "platform_joint" ,
                                "head_joint1" ,
                                "head_joint2" ,

                                "l_Joint_finger1" ,
                                "l_Joint_finger2" ,
                                "r_Joint_finger1" ,
                                "r_Joint_finger2" ,

                                "joint_left_wheel" ,
                                "joint_right_wheel" ,

                                "joint_swivel_wheel_1_1" ,
                                "joint_swivel_wheel_1_2" ,
                                "joint_swivel_wheel_2_1" ,
                                "joint_swivel_wheel_2_2" ,
                                "joint_swivel_wheel_3_1" ,
                                "joint_swivel_wheel_3_2" ,
                                "joint_swivel_wheel_4_1" ,
                                "joint_swivel_wheel_4_2" ,
                            ]
mixed_jointsToLockIDs.remove("universe")
print("模型中所有关节名：")
print(robot.model.names)

reduced_robot = robot.buildReducedRobot(
    list_of_joints_to_lock=mixed_jointsToLockIDs,
    reference_configuration=np.array([0.0] * robot.model.nq),
)

reduced_robot.model.addFrame(
    pin.Frame('L_ee',
                reduced_robot.model.getJointId('l_joint7'),
                pin.SE3(np.eye(3),
                        np.array([0.05,0,0]).T),
                pin.FrameType.OP_FRAME)
)

reduced_robot.model.addFrame(
    pin.Frame('R_ee',
                reduced_robot.model.getJointId('r_joint7'),
                pin.SE3(np.eye(3),
                        np.array([0.05,0,0]).T),
                pin.FrameType.OP_FRAME)
)
for idx, name in enumerate(reduced_robot.model.names):
    print(f"{idx}: {name}")
# Creating Casadi models and data for symbolic computing
# cmodel = cpin.Model(reduced_robot.model)
data = reduced_robot.model.createData()

# data = robot.createData()  # <-- 关键！

# 设置关节角度（角度单位：度 → 弧度）
q = np.zeros(robot.nq)
q[1] = np.deg2rad(-10)
q[2] = np.deg2rad(-60)
q[3] = np.deg2rad(-60)
q[4] = np.deg2rad(-90)
q[5] = np.deg2rad(20)
q[6] = np.deg2rad(-60)
q[7] = np.deg2rad(110)

q[8] = np.deg2rad(10)
q[9] = np.deg2rad(60)
q[10] = np.deg2rad(60)
q[11] = np.deg2rad(90)
q[12] = np.deg2rad(-20)
q[13] = np.deg2rad(60)
q[14] = np.deg2rad(-110)

# 前向运动学计算
pin.forwardKinematics(robot, data, q)
pin.updateFramePlacements(robot, data)

# 获取末端位姿
L_ee_id = robot.getFrameId("L_ee")
R_ee_id = robot.getFrameId("R_ee")

L_ee_pose = data.oMf[L_ee_id]
R_ee_pose = data.oMf[R_ee_id]

print("L_ee pose:\n", L_ee_pose)
print("R_ee pose:\n", R_ee_pose)
