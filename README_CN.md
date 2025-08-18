# Python 机器人

1. 无人飞行器
2. 机械臂（ArmNavigation）
3. 两足机器人（Bipedal）
4. 倒立摆（invertedPendulum）
5. 



# Mapping 建图模块
## circle_fitting
用激光/深度传感器扫面一个圆形障碍物，得到若干边界离散点；在用最小二乘法把圆心和半径估计出来  
难点在于如何构建最小二乘法



# PathPlanning 路径规划模块

## BSplinePath B样条路径
给定一组稀疏的航点（way-points），生成
- 一条光滑且曲率连续的二维路径（x-y轨迹）
- 同时计算该路径上每一点的朝向角和曲率，以便后续做轨迹跟踪、速度规划和约束检查



