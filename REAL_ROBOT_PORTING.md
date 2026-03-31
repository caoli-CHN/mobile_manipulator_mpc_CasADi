

# Mobile-Manipulator_MPC_CasADi 真机版 ROS 接口对照表

本文档用于说明本项目从 Gazebo 仿真迁移到真机时，需要替换的 ROS 接口、控制器、参数与状态反馈链路

相关入口文件：

- [catkin_ws/src/open_door_mpc/launch/rob_demo.launch](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/launch/rob_demo.launch)
- [catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py)
- [catkin_ws/src/open_door_mpc/script/utils/omnirob_ur5_ros.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/utils/omnirob_ur5_ros.py)
- [catkin_ws/src/asset/omni_rob/scripts/base_controller.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/asset/omni_rob/scripts/base_controller.py)
- [catkin_ws/src/asset/omni_rob/scripts/fake_odom.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/asset/omni_rob/scripts/fake_odom.py)

## 1. 当前系统结构

当前仓库里的控制闭环是仿真专用的，链路如下：

1. RViz 交互点发布目标位姿到 `/mobile_manipulator_mpc_target`
2. `mobile_manipulator_mpc_demo.py` 求解 MPC
3. MPC 内部通过运动学模型积分出下一时刻状态
4. 底盘通过 Gazebo service 直接设置模型位姿
5. 机械臂通过 `joint_position_controller/command` 下发关节位置
6. `fake_odom.py` 从 Gazebo 读取模型状态和 `/odom`

这意味着当前实现不是严格的真机闭环控制，而是：

- 底盘：仿真里“设状态”
- 机械臂：仿真里“发位置”
- 状态反馈：主要来自 Gazebo，而不是实际硬件控制器

真机迁移时，必须改成：

- 底盘：发速度命令给底盘控制器
- 机械臂：发速度命令或位置命令给真机控制器
- 状态反馈：从真实 `/odom`、`/joint_states`、TF、力传感器读取

## 2. 核心状态与控制量

本项目的 MPC 变量定义如下：

- 状态 `x = [x, y, theta, q1, q2, q3, q4, q5, q6]`
- 控制 `u = [v, omega, dq1, dq2, dq3, dq4, dq5, dq6]`

对应代码位置：

- 状态维度与控制维度定义见 [mobile_manipulator_mpc_demo.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py#L42)
- 运动学方程见 [mobile_manipulator_mpc_demo.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py#L70)

真机迁移时，必须保证每个控制周期都能获得这 9 维状态中的真实测量值或可靠估计值

## 3. 真机版 ROS 接口对照表

### 3.1 目标输入接口

| 功能 | 仿真接口 | 当前用途 | 真机建议接口 | 是否保留 |
| --- | --- | --- | --- | --- |
| 末端目标位姿输入 | `/mobile_manipulator_mpc_target` `geometry_msgs/Pose` | RViz 交互目标输入 | 保持不变，或替换为任务规划节点输出的目标位姿话题 | 建议保留 |

说明：

- 当前由交互 marker 节点发布，代码见 [catkin_ws/src/interactive_marker/src/interactive_marker.cpp](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/interactive_marker/src/interactive_marker.cpp#L5)
- 真机上如果不是手动拖 RViz，可以改为上层任务规划器发布同类型消息

### 3.2 底盘状态反馈接口

| 功能 | 仿真接口 | 当前用途 | 真机建议接口 | 备注 |
| --- | --- | --- | --- | --- |
| 底盘位姿反馈 | `/odom` `nav_msgs/Odometry` | `OmniRobUr5Ros` 读取机器人位置 | `/odom` 或 `/wheel_odom` 或定位融合输出 `/localization/odom` | 建议统一映射到 `/odom` |
| 底盘世界位姿来源 | `/gazebo/get_model_state` service | `fake_odom.py` 读取 Gazebo 模型状态 | 不使用 | 真机必须删除 |
| TF | `odom -> dummy` | `fake_odom.py` 广播 | `odom -> base_link` 由真实底盘驱动或定位系统提供 | 必须替换 |

说明：

- 当前 `/odom` 是虚拟仿真的，代码见 [fake_odom.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/asset/omni_rob/scripts/fake_odom.py#L47)
- 真机必须由真实底盘驱动、轮速里程计、SLAM、EKF 或融合定位提供 `/odom`
- 如果真机发布的坐标系不是 `odom -> base_link`，需要在 MPC 侧做 TF 适配

### 3.3 机械臂状态反馈接口

| 功能 | 仿真接口 | 当前用途 | 真机建议接口 | 备注 |
| --- | --- | --- | --- | --- |
| 关节角、关节速度、关节力矩反馈 | `/joint_states` `sensor_msgs/JointState` | 代码里预留但目前未启用 | 保持 `/joint_states` | 真机必接 |
| 关节顺序重映射 | `idx_remap = [4,3,0,5,6,7]` | 适配当前仿真 joint order | 根据真机关节命名重新实现 | 必须检查 |

说明：

- 当前订阅被注释，见 [omnirob_ur5_ros.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/utils/omnirob_ur5_ros.py#L31)
- 真机迁移时应恢复订阅 `/joint_states`
- 不建议继续依赖固定下标映射，建议改为按 `JointState.name` 查找对应关节名

推荐的真机关节名映射方式：

```python
joint_index = {name: i for i, name in enumerate(js.name)}
q1 = js.position[joint_index["shoulder_pan_joint"]]
```

### 3.4 末端位姿与力传感反馈接口

| 功能 | 仿真接口 | 当前用途 | 真机建议接口 | 备注 |
| --- | --- | --- | --- | --- |
| 末端力传感器 | `/ft_sensor_topic_ee_fixed_joint` `geometry_msgs/WrenchStamped` | 读取末端受力 | 替换为真机 F/T 话题，例如 `/wrench`、`/ft_sensor/raw` | 如果真机没有 F/T，可暂时不用 |
| 末端位姿 TF | `odom <-> ee_link` | `get_tip_pos()` 中查 TF | `base_link/odom/map -> ee_link/tool0` | 必须与实际 TF 树一致 |

说明：

- 当前 `get_tip_pos()` 依赖 TF，见 [omnirob_ur5_ros.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/utils/omnirob_ur5_ros.py#L90)
- 如果真机 TF 树里末端坐标系不是 `ee_link`，需要统一改名或在代码里替换 frame id

### 3.5 底盘控制输出接口

| 功能 | 仿真接口 | 当前用途 | 真机建议接口 | 备注 |
| --- | --- | --- | --- | --- |
| 底盘速度控制 | `/cmd_vel` `geometry_msgs/Twist` | 发送到底盘仿真控制器 | `/cmd_vel` 或真机底盘驱动指定的速度话题 | 建议保留统一接口名 |
| Gazebo 模型速度/位姿设置 | `/gazebo/set_model_state` service | 仿真里设置底盘位置或速度 | 不使用 | 真机必须删除 |

说明：

- 当前类里同时有 `base_ctrl()` 和 `world_frame_set()` 两种方式
- 真机只能保留 `base_ctrl()` 这类速度控制接口
- `world_frame_set()` 是直接改模型位姿，真机上没有等价物

### 3.6 机械臂控制输出接口

| 功能 | 仿真接口 | 当前用途 | 真机建议接口 | 备注 |
| --- | --- | --- | --- | --- |
| 关节位置控制 | `/joint1_position_controller/command` ... `/joint6_position_controller/command` | 当前主流程实际使用 | 真机若提供单关节位置接口可映射；更推荐轨迹或速度接口 | 可保留作为过渡方案 |
| 关节力矩控制 | `/joint1_effort_controller/command` ... `/joint6_effort_controller/command` | 代码中有 publisher，但主流程未使用 | 不建议直接用于首版真机迁移 | 暂不推荐 |
| 关节速度控制 | 当前无 | 无 | 推荐接入真机速度控制器，例如 `/joint_group_vel_controller/command` | 推荐新增 |
| 关节轨迹控制 | 当前无 | 无 | 推荐接入 `control_msgs/FollowJointTrajectory` 或轨迹话题 | 工业机械臂常见方案 |

建议优先级：

1. 若机械臂有稳定的速度接口，直接将 MPC 输出 `dq` 下发
2. 若只有位置接口，用 `q_cmd = q_now + dq * dt` 转成位置命令
3. 若只有轨迹接口，将单步命令封装为短时轨迹

### 3.7 夹爪控制接口

| 功能 | 仿真接口 | 当前用途 | 真机建议接口 | 备注 |
| --- | --- | --- | --- | --- |
| 左夹爪 | `/left_finger_controller/command` | 仿真夹爪控制 | 真机夹爪驱动话题或服务 | 需按真机替换 |
| 右夹爪 | `/right_finger_controller/command` | 仿真夹爪控制 | 真机夹爪驱动话题或服务 | 需按真机替换 |

### 3.8 可视化输出接口

| 功能 | 当前接口 | 当前用途 | 真机建议 |
| --- | --- | --- | --- |
| 末端规划轨迹 | `ee_planning_trajectory` | RViz 可视化 | 保留 |
| 底盘预测轨迹 | `base_predict_trajectory` | RViz 可视化 | 保留 |
| 末端实际轨迹 | `ee_real_trajectory` | RViz 可视化 | 保留 |
| 机器人球模型 | `rob_sphere_pub` | RViz 可视化 | 保留 |

这些接口不参与控制闭环，可直接保留

## 4. 服务接口

### 4.1 当前项目中的服务

当前项目主流程实际依赖的 service 只有 Gazebo 的：

- `/gazebo/get_model_state`
- `/gazebo/set_model_state`

这两个都属于仿真专用接口，真机上应完全移除

## 5. 需要读取的机器人状态参数和下发给控制器的控制参数

### 5.1 来自机器人的状态参数

这些量应由机器人实时反馈，不应写死在主脚本中：

| 参数 | 来源 | 建议接口 |
| --- | --- | --- |
| `x, y, theta` | 底盘里程计或定位系统 | `/odom` |
| `q1...q6` | 机械臂编码器 | `/joint_states` |
| `dq1...dq6` | 机械臂编码器或驱动反馈 | `/joint_states` |
| 末端力和力矩 | 六维力传感器 | `/wrench` |
| 末端位姿 | TF 或正运动学 | `tf` 或由 `/joint_states` 正解得到 |
| 目标位姿 | 上层任务规划器或 RViz | `/mobile_manipulator_mpc_target` |

### 5.2 下发给控制器的控制参数

这些量应由 MPC 求解后发送给控制器：

| 参数 | 含义 | 建议输出接口 |
| --- | --- | --- |
| `v` | 底盘线速度 | `/cmd_vel.linear.x` |
| `omega` | 底盘角速度 | `/cmd_vel.angular.z` |
| `dq1...dq6` | 机械臂关节速度 | 机械臂速度控制器 |
| `q_cmd1...q_cmd6` | 若没有速度接口，则由 `q_now + dq*dt` 生成的位置命令 | 机械臂位置控制器 |
| gripper state | 夹爪开闭命令 | 夹爪控制接口 |

### 5.3 当前代码里写死、真机时应改为配置或实时读取的参数

| 参数 | 当前位置 | 说明 |
| --- | --- | --- |
| 初始关节角 `q0` | [mobile_manipulator_mpc_demo.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py#L48) | 真机应从 `/joint_states` 读取 |
| 初始底盘位姿 | [mobile_manipulator_mpc_demo.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py#L47) | 真机应从 `/odom` 读取 |
| 关节限位与零位偏置 | [hmqr5_agl_map.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/utils/hmqr5_agl_map.py#L1) | 必须按真机重标定 |
| DH 参数与工具长度 | [ur5_dh.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/utils/ur5_dh.py#L8) | 必须按真机重标定 |
| 底盘到机械臂安装高度 `z_b2a` | [ur5_dh.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/utils/ur5_dh.py#L87) | 必须按真机标定 |
| MPC 权重与速度上限 | [mobile_manipulator_mpc_demo.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py#L30) | 需按真机动态性能调整 |

## 6. 推荐的真机接口命名方案

尽量少改 MPC 主体代码，统一做一个硬件适配层，把真机接口适配成下面这组稳定接口：

### 6.1 建议保留给 MPC 的标准输入

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/mobile_manipulator_mpc_target` | `geometry_msgs/Pose` | 目标末端位姿 |
| `/odom` | `nav_msgs/Odometry` | 底盘位姿 |
| `/joint_states` | `sensor_msgs/JointState` | 机械臂关节反馈 |
| `/wrench` | `geometry_msgs/WrenchStamped` | 末端力反馈，可选 |
| `tf: odom -> base_link -> ee_link` | TF | 底盘与末端位姿关系 |

### 6.2 建议保留给执行层的标准输出

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度控制 |
| `/arm_joint_velocity_cmd` | 自定义或标准数组消息 | 机械臂关节速度命令 |
| `/arm_joint_position_cmd` | 自定义或标准数组消息 | 若走位置控制时使用 |
| `/gripper_cmd` | 真机驱动定义 | 夹爪命令 |

推荐做法：

- `open_door_mpc` 只依赖上述统一接口
- 具体底盘驱动、机械臂驱动、夹爪驱动由单独 bridge 节点负责转换

这样后续更换真机型号时，不需要大改 MPC 本体

## 7. 控制器替换建议

### 7.1 底盘控制器

仿真当前做法：

- MPC 发 `/cmd_vel`
- `base_controller.py` 再转成 Gazebo `set_model_state`

真机建议：

- 直接由底盘驱动订阅 `/cmd_vel`
- 不再保留 [base_controller.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/asset/omni_rob/scripts/base_controller.py)

### 7.2 机械臂控制器

仿真当前做法：

- `joint1_position_controller/command` 到 `joint6_position_controller/command`

真机建议优先级：

1. 速度控制器
2. 位置控制器
3. 轨迹控制器

原因：

- MPC 求解结果本来就是关节速度 `dq`
- 直接发速度最自然，模型也最一致
- 如果改发位置，需要在接口层增加一层离散积分

### 7.3 夹爪控制器

仿真做法是两个 `Float64` 位置命令

真机可能出现以下情况：

- 单个服务控制开闭
- action 控制夹爪行程
- 厂商自定义消息

建议单独写 `gripper_bridge` 适配，不要把厂商接口直接写进 MPC 主体

## 8. 真机迁移时建议删除或禁用的仿真节点

以下节点或接口属于仿真专用，真机上应禁用：

| 文件/节点 | 原因 |
| --- | --- |
| [catkin_ws/src/asset/omni_rob/scripts/fake_odom.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/asset/omni_rob/scripts/fake_odom.py) | 只用于仿真 `/odom` |
| [catkin_ws/src/asset/omni_rob/scripts/base_controller.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/asset/omni_rob/scripts/base_controller.py) | 只用于 Gazebo 底盘控制 |
| `world_frame_set()` | 直接设模型绝对位姿，真机不存在 |
| `/gazebo/get_model_state` | Gazebo 专用 |
| `/gazebo/set_model_state` | Gazebo 专用 |
| [catkin_ws/src/asset/omni_rob/launch/omni_ur5_gazebo.launch](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/asset/omni_rob/launch/omni_ur5_gazebo.launch) | 真机不启动 Gazebo |

## 9. 真机迁移时建议新增的节点

建议新增以下 bridge 或适配节点：

| 节点名 | 作用 |
| --- | --- |
| `base_state_bridge` | 将真机底盘里程计、IMU、融合定位统一输出到 `/odom` |
| `arm_state_bridge` | 将真机机械臂反馈统一输出到 `/joint_states` |
| `arm_cmd_bridge` | 将 MPC 输出的关节速度或位置命令转成厂商控制接口 |
| `gripper_bridge` | 将夹爪标准命令转成厂商接口 |
| `tf_bridge` | 统一发布 `odom/base_link/ee_link` 所需 TF |

## 10. 推荐的真机 launch 结构

建议将当前仿真 launch 拆成真机版：

```xml
<launch>
  <include file="$(find your_robot_bringup)/launch/base_bringup.launch"/>
  <include file="$(find your_arm_bringup)/launch/arm_bringup.launch"/>
  <include file="$(find your_ft_bringup)/launch/ft_sensor.launch"/>

  <node pkg="your_bridge_pkg" type="base_state_bridge.py" name="base_state_bridge"/>
  <node pkg="your_bridge_pkg" type="arm_state_bridge.py" name="arm_state_bridge"/>
  <node pkg="your_bridge_pkg" type="arm_cmd_bridge.py" name="arm_cmd_bridge"/>
  <node pkg="your_bridge_pkg" type="gripper_bridge.py" name="gripper_bridge"/>

  <node pkg="interactive_marker" type="interactive_marker_node" name="interactive_marker_node"/>
  <node pkg="open_door_mpc" type="mobile_manipulator_mpc_demo.py" name="mpc_demo"/>
</launch>
```

## 11. 最小可用真机改造清单

如果目标是先“能在真机上跑起来”，最小改造集建议如下：

1. 恢复并正确实现 `/joint_states` 订阅
2. 用真实 `/odom` 替换 `fake_odom.py`
3. 删除 `world_frame_set()` 调用，底盘只发 `/cmd_vel`
4. 机械臂增加速度控制或位置控制适配层
5. 将 `state_now` 每周期用真实反馈刷新，而不是只靠 `shift_timestep()`
6. 重标定 DH、零位偏置、安装高度、关节限位
7. 重新整定 `v_max`、`v_arm_max`、`step_horizon`、`N`

## 12. 接口迁移总结

仿真版依赖 Gazebo “改模型状态”，真机版必须改成“读真实反馈 + 发真实控制命令”

最关键的替换关系如下：

| 仿真侧 | 真机侧 |
| --- | --- |
| `/gazebo/get_model_state` | `/odom` |
| `/gazebo/set_model_state` | `/cmd_vel` 或底盘驱动接口 |
| fake `/odom` | 真实里程计/定位 `/odom` |
| 单关节 position controller | 真机速度控制器或位置控制器 |
| 注释掉的 `/joint_states` | 必须启用的 `/joint_states` |
| Gazebo TF | 真机 TF 树 |

如果后续要进一步改代码，优先改 [catkin_ws/src/open_door_mpc/script/utils/omnirob_ur5_ros.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/utils/omnirob_ur5_ros.py) 和 [catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py](/home/caoli/Mobile-Manipulator_MPC_CasADi/catkin_ws/src/open_door_mpc/script/mobile_manipulator_mpc_demo.py)。
