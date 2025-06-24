#  ros1具身双臂机器人robot_description功能包详解



## **一. **robot_description功能包说明

robot_description功能包为具身双臂机器人提供动态化、可配置的URDF模型描述，用于在 ROS1 环境中进行机器人仿真、可视化和运动控制。

## **二. **robot_description功能包架构说明

```
robot_description
    ├── CMakeLists.txt
    ├── launch 
    │   └── load_robot.launch  #具身双臂机器人模型加载与可视化启动文件
    ├── meshes  具身双臂机器人模型放置位置
    │   ├── aoyi_hand
    │   ├── common
    │   │   ├── base_link_underpan.STL
    │   │   ├── bl_Link.STL
    │   │   ├── body_base_link.STL
    │   │   ├── br_Link.STL
    │   │   ├── camera_link.STL
    │   │   ├── fl_Link.STL
    │   │   ├── fr_Link.STL
    │   │   ├── head_link1.STL
    │   │   ├── head_link2.STL
    │   │   ├── l_base_link1.STL
    │   │   ├── link_left_wheel.STL
    │   │   ├── link_right_wheel.STL
    │   │   ├── link_swivel_wheel_1_1.STL
    │   │   ├── link_swivel_wheel_1_2.STL
    │   │   ├── link_swivel_wheel_2_1.STL
    │   │   ├── link_swivel_wheel_2_2.STL
    │   │   ├── link_swivel_wheel_3_1.STL
    │   │   ├── link_swivel_wheel_3_2.STL
    │   │   ├── link_swivel_wheel_4_1.STL
    │   │   ├── link_swivel_wheel_4_2.STL
    │   │   ├── platform_base_link.STL
    │   │   └── r_base_link1.STL
    │   ├── rm65
    │   │   ├── l_link1.STL
    │   │   ├── l_link2.STL
    │   │   ├── l_link3.STL
    │   │   ├── l_link4.STL
    │   │   ├── l_link5.STL
    │   │   ├── l_link6.STL
    │   │   ├── r_link1.STL
    │   │   ├── r_link2.STL
    │   │   ├── r_link3.STL
    │   │   ├── r_link4.STL
    │   │   ├── r_link5.STL
    │   │   └── r_link6.STL
    │   ├── rm75
    │   │   ├── l_link1.STL
    │   │   ├── l_link2.STL
    │   │   ├── l_link3.STL
    │   │   ├── l_link4.STL
    │   │   ├── l_link5.STL
    │   │   ├── l_link6.STL
    │   │   ├── l_link7.STL
    │   │   ├── r_link1.STL
    │   │   ├── r_link2.STL
    │   │   ├── r_link3.STL
    │   │   ├── r_link4.STL
    │   │   ├── r_link5.STL
    │   │   ├── r_link6.STL
    │   │   └── r_link7.STL
    │   ├── rmg24
    │   │   ├── base_link2.STL
    │   │   ├── Link_finger1.STL
    │   │   └── Link_finger2.STL
    │   └── yinshi
    │       ├── l_base_link.STL
    │       ├── l_hand_base_link.STL
    │       ├── l_hand_link.STL
    │       ├── r_hand_base_link.STL
    │       └── r_hand.STL
    ├── package.xml
    ├── ros1具身双臂机器人robot_description功能包详解.md
    └── urdf 具身双臂机器人xacro模型描述文件放置位置
        ├── common
        │   ├── body_head_platform_transmission.xacro
        │   ├── body_head_platform.urdf.xacro
        │   ├── common_gazebo.xacro
        │   └── woosh_agv.urdf.xacro
        ├── end_effectors
        │   ├── grippers
        │   │   ├── joint_rmg24.urdf.xacro
        │   │   ├── left_hand_rmg24_transmission.xacro
        │   │   ├── left_hand_rmg24.urdf.xacro
        │   │   ├── right_hand_rmg24_transmission.xacro
        │   │   └── right_hand_rmg24.urdf.xacro
        │   └── hands
        │       ├── joint.urdf.xacro
        │       ├── left_hand.urdf.xacro
        │       └── right_hand.urdf.xacro
        ├── rm65
        │   ├── rm65_b_v_left_transmission.xacro
        │   ├── rm65_b_v_left.urdf.xacro
        │   ├── rm65_b_v_right_transmission.xacro
        │   └── rm65_b_v_right.urdf.xacro
        ├── rm75
        │   ├── rm75_b_v_left_transmission.xacro
        │   ├── rm75_b_v_left.urdf.xacro
        │   ├── rm75_b_v_right_transmission.xacro
        │   └── rm75_b_v_right.urdf.xacro
        └── robots
            └── embodied_dual_arms.urdf.xacro

    16 directories, 82 files

```



## **三.**robot_description功能包使用

完成环境配置和ROS包编译后，通过以下命令启动机器人模型加载与可视化：

```bash
cd ~/embodied_robot
source devel/setup.bash
roslaunch robot_description load_robot.launch
```

上述命令使用默认参数：

```bash
arm_type:=rm65 end_effector:=rmg24
```

这将加载 RM65 机械臂搭配 rmg24 两指夹爪的 URDF 模型，如下图所示：

![image-20250511152213464](/home/i/robot_realman/rmc_aida_l_ros1/src/robot_description/images/rm65_rmg24.png)

### **RViz 配置**

如果 RViz 中未显示机器人模型，请按以下步骤配置：

1. 将左侧面板的 **Fixed Frame** 设置为 body_base_link。
2. 点击左侧下方的 **Add** 按钮，在弹出的窗口中选择 **RobotModel** 并添加。

### **参数配置**

支持的配置参数如下表所示：

| 参数名         | 可选值  | 说明                      |
| -------------- | ------- | ------------------------- |
| `arm_type`     | `rm65`  | 配置RM65机械臂（6自由度） |
|                | `rm75`  | 配置RM75机械臂（7自由度） |
| `end_effector` | `rmg24` | 末端为RMG24夹爪           |
|                | `aoyi`  | 末端为AOYI灵巧手          |

### **修改参数示例**

如当前机械臂配置的是RM75 机械臂和 aoyi 灵巧手，可运行：

```
roslaunch robot_description load_robot.launch arm_type:=rm75 end_effector:=aoyi
```

这将加载 RM75 机械臂搭配 傲意灵巧手的 URDF 模型，如下图所示：

![image-20250511153222595](/home/i/robot_realman/rmc_aida_l_ros1/src/robot_description/images/rm75_aoyi.png)

### 节点图



![image-20250511153222595](/home/i/robot_realman/rmc_aida_l_ros1/src/robot_description/images/description_graph.png)

1. joint_state_publisher_gui生成一个GUI界面，用户通过滑动条调整角度。

2. joint_state_publisher_gui发布调整过的关节状态到/joint_states话题。
3. robot_state_publisher订阅/joint_states，结合URDF计算链接之间的变换，并发布到/tf话题。
4. rviz使用/tf现实机器人的实时3D模型。
