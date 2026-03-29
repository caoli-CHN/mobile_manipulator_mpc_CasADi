

# Mobile-Manipulator_MPC_CasADi

基于非线性模型预测控制 (Nonlinear Model Predictive Control, NMPC) 的移动机械臂 (Mobile Manipulator) 全身运动规划与控制

- 控制对象（状态向量）：移动底盘 + 6 自由度机械臂 (x, y, theta, q1, q2, q3, q4, q5, q6)
- 控制输入（控制向量）：底盘线速度 + 底盘角速度 + 机械臂各关节角速度 (v, omega, q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot)
- 系统模型：运动学模型
- 非线性来源：底盘三角函数运动学 + 机械臂末端正运动学
- 离散化算法：四阶龙格-库塔法 (Runge-Kutta Method, RK4)
- 优化形式：离散时间非线性规划 (Nonlinear Programming, NLP)
- 符号建模与自动求导工具：CasADi (Computer Algebra for Differentiation)
- 非线性最优化求解器：内点优化器 (Interior Point OPTimizer, IPOPT )
- 控制执行方式：滚动时域优化 (Receding Horizon Optimization, RHO)

控制目标不是单纯让底盘到达某个二维位置，而是让“整机系统”协同运动，使机械臂末端执行器跟踪给定的目标位姿，同时控制输入尽量平滑、不过大，并满足动力学和状态边界约束

## 移动机械臂末端执行器移动至目标点

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch open_door_mpc rob_demo.launch
```

随后拖动 Rviz 中的交互对象，右键发送目标，移动机械臂即可实现跟随

<img src="./catkin_ws/pictures/001.gif"  />
