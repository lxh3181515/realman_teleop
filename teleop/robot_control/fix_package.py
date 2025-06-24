import pinocchio as pin
import os

# 你的 package 路径
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

# 输出模型信息
print("模型关节数量:", robot.model.njoints)
for i, name in enumerate(robot.model.names):
    print(f"Joint {i}: {name}")

# print("关节名称:", [j.name for j in robot.model.joints])
