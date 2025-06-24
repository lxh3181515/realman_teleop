import pinocchio as pin
import numpy as np

# 初始化模型
model_path = "/home/ljc/realman_teleop/assets/robot_description"
urdf_path = model_path + "/urdf/robots/embodied_dual_arms.urdf"

with open(urdf_path, 'r') as f:
    urdf_content = f.read()
urdf_content = urdf_content.replace("package://robot_description", model_path)
with open("/tmp/embodied_dual_arms_fixed.urdf", 'w') as f:
    f.write(urdf_content)

robot = pin.buildModelFromUrdf("/tmp/embodied_dual_arms_fixed.urdf")
data = robot.createData()

# 添加末端 frame（添加完之后要重新生成 data！）
robot.addFrame(pin.Frame('L_ee', robot.getJointId('l_joint7'), pin.SE3(np.eye(3), np.array([0.05,0,0])), pin.FrameType.OP_FRAME))
robot.addFrame(pin.Frame('R_ee', robot.getJointId('r_joint7'), pin.SE3(np.eye(3), np.array([0.05,0,0])), pin.FrameType.OP_FRAME))
data = robot.createData()  # <-- 关键！


# 设置关节角度（角度单位：度 → 弧度）
q = np.zeros(robot.nq)
# 设置左臂关节角度
q[3]  = np.deg2rad(-60)   # l_joint1
q[4]  = np.deg2rad(-90)   # l_joint2
q[5]  = np.deg2rad(20)    # l_joint3
q[6]  = np.deg2rad(-60)   # l_joint4
q[7]  = np.deg2rad(110)   # l_joint5
q[8]  = np.deg2rad(10)    # l_joint6
q[9]  = np.deg2rad(60)    # l_joint7
q[10] = np.deg2rad(0)    # l_Joint_finger1
q[11] = np.deg2rad(0)    # l_Joint_finger2

# 设置右臂关节角度
q[12] = np.deg2rad(60)   # r_joint1
q[13] = np.deg2rad(90)    # r_joint2
q[14] = np.deg2rad(-20)  # r_joint3
q[15] = np.deg2rad(60)     # r_joint4
q[16] = np.deg2rad(-110)     # r_joint5
q[17] = np.deg2rad(-10)     # r_joint6
q[18] = np.deg2rad(-60)     # r_joint7
q[19] = np.deg2rad(0)     # r_Joint_finger1
q[20] = np.deg2rad(0)     # r_Joint_finger2


for i, joint in enumerate(robot.joints):
    idx = joint.idx_q  # q 中的起始索引（例如对于 1 DoF joint 就是单个值）
    dof = joint.nq     # nq 表示这个 joint 占用几个 DoF
    if dof > 0:
        print(f"q[{idx}:{idx+dof}] -> joint name: {robot.names[i]}")

for i, joint in enumerate(robot.joints):
    idx = joint.idx_q
    dof = joint.nq
    if dof > 0:
        joint_name = robot.names[i]
        joint_q = q[idx:idx+dof]
        print(f"{joint_name}: q[{idx}:{idx+dof}] = {np.rad2deg(joint_q)} deg")


# 前向运动学计算
pin.forwardKinematics(robot, data, q)
pin.updateFramePlacements(robot, data)

# # 获取末端位姿
# L_ee_id = robot.getFrameId("L_ee")
# R_ee_id = robot.getFrameId("R_ee")

# L_ee_pose = data.oMf[L_ee_id]
# R_ee_pose = data.oMf[R_ee_id]

# print("L_ee pose:\n", L_ee_pose)
# print("R_ee pose:\n", R_ee_pose)


# 获取末端位姿
L_ee_id = robot.getFrameId("L_ee")
R_ee_id = robot.getFrameId("R_ee")

L_ee_pose = data.oMf[L_ee_id]
R_ee_pose = data.oMf[R_ee_id]

# 将旋转矩阵转成四元数（w, x, y, z）
L_quat = pin.Quaternion(L_ee_pose.rotation)
R_quat = pin.Quaternion(R_ee_pose.rotation)

print("L_ee position:\n", L_ee_pose.translation.T)
print("L_ee quaternion [w, x, y, z]:\n", L_quat.coeffs()[::-1])  # coeffs() 返回 [x, y, z, w]，[::-1] 变成 [w, x, y, z]

print("R_ee position:\n", R_ee_pose.translation.T)
print("R_ee quaternion [w, x, y, z]:\n", R_quat.coeffs()[::-1])