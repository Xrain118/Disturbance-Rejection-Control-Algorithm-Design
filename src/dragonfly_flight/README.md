# Dragonfly 飞行与键盘动臂

本文件继承自 Dragonfly，历史日期及成绩保留；本次复用和测试状态以[工程 README](../../README.md)为准。源码已迁移到本仓库。固定演示与自由飞行共用原 Xacro 和尺寸参数。飞行模式移除 world 固定关节，使用六个 Gazebo 电机、六个倾转力矩伺服和两个 effort 轨迹控制器。机械臂反作用来自多刚体动力学，没有额外注入“机械臂扰动力”。

2026-10-05 已完成 Dragonfly 原生 PX4 矢量控制适配。默认 `px4_native` 使用独立机型 40160，位置/速度、姿态/角速度及六电机六伺服分配均由 PX4 完成；ROS 发布位置轨迹与机械臂轨迹。2026-10-09 按实物图纸更新几何、单臂质量和 12 V 逐关节峰值后，已用新 5.415 kg 模型重新通过三轮完整验收，结果见 [工程 README](../../README.md)。参数、通道、补丁及历史结果见 [原生 PX4 适配说明](config/px4_vector/README.md)。组长原五文件作为本机参考保留在该目录 `original/`，不纳入本次仓库提交。当前机械臂固件位于仓库外的 `C:\Users\LENOVO\Desktop\arm`，来源、姿态与实机接入状态见工程 README；历史版本当时没有修改固件；本项目此次固件修改见[固件说明](../../docs/firmware.md)。

## 构建与环境

本机版本：Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic 8.15、PX4 v1.16.2、Micro XRCE-DDS Agent v2.4.3，以及 `/home/ubuntu/px4_ros2_ws` 的兼容消息。仿真使用 `build-sim` / `install-sim`；机载构建使用独立的 `build-onboard` / `install-onboard`，不包含本包。

```bash
cd "/home/ubuntu/Disturbance Rejection Control Algorithm Design"
bash scripts/build_sim.sh
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source scripts/env.sh
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
source install-sim/setup.bash
```

构建脚本设置 `PYTHONNOUSERSITE=1`，PX4 使用的 `jsonschema` 必须可由系统 Python 导入；本机于 2026-10-12 补齐系统包 `python3-jsonschema`（Ubuntu 可执行 `sudo apt-get install python3-jsonschema`）。构建不会自动安装系统依赖。

每个新终端执行 export/source。`PYTHONNOUSERSITE=1` 避免本机用户目录 NumPy 2 与系统 SciPy 冲突。

构建脚本应用 `config/px4_vector/px4_v1.16.2_vector.patch` 与 `config/px4_mag_enu.patch` 并编译 PX4。补丁仅在 `PX4_GZ_MAG_ENU=1` 时将 Gazebo 机体 FLU 磁场转换为 PX4 FRD；原生 x500 默认路径保留。Gazebo 使用 ENU 地磁场和 gauss，磁偏角来自 Harmonic 在 Zurich 原点的磁场表。这些 SITL 参数不用于实机。

## 三种启动方式

已完成构建时，可直接在 WSL 终端运行 `bash src/dragonfly_flight/scripts/sim.sh --gui --record`，另外两个终端分别运行同一脚本并传入 `keyboard`、`monitor`。该脚本自动加载下面所需环境。

一次只运行一个入口，固定演示不能与飞行共享控制器。

```bash
# 原固定机体演示（位置接口）
ros2 launch dragonfly_sim gazebo.launch.py

# 理想合力/合力矩调试，不启动 PX4 或旋翼模型
ros2 run dragonfly_flight flight --mode ideal_wrench --scenario hover --gui

# 原生 PX4 从地面解锁、平滑起飞，保持悬停并开放键盘（默认模式）
ros2 run dragonfly_flight flight --scenario manual --gui --record
```

目标高度 1.6 m，仿真约半速运行。PX4 地面出生时两臂保持折叠 `[0,0,π/2,0,0]`；悬停连续稳定 2 s 后，两臂以 0.10 rad/s、0.20 rad/s² 同步展开到竖直 `[0,0,0,0,0]`，约需 30 仿真秒。展开到位且机体再次连续稳定 2 s 后才显示 `ready=True`。主运行终端 Ctrl+C 结束整次仿真。省略 `--gui` 可无界面运行。

另一 WSL 终端加载环境后：

```bash
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=0
ros2 run dragonfly_flight keyboard

# 独立的实时 PlotJuggler 面板（300 s 缓冲）
bash src/dragonfly_flight/scripts/sim.sh monitor

# 只在终端查看机体中心世界坐标
ros2 topic echo /dragonfly/truth --field pose.pose.position
```

监控启动后，在 `Start Streaming?` 中选 `Yes`，接着保留预选的 8 个 `/dragonfly/...` 话题并点 `OK`。主窗口左侧应显示 `Stop`（表示正在接收）和 `Buffer: 300 sec`，时间轴持续增长；上方标签切换机体中心 XYZ、位置误差、姿态误差与机械臂状态。窗口可以最大化以显示全部标签。PlotJuggler 面板统一使用消息接收时间，以便没有 `header` 的 Float64/Bool 模长和状态话题与位姿曲线对齐；ROS 话题本身的 Pose/Vector header 仍保存仿真时间。

本机 WSLg 的 OpenGL 绘图路径曾在订阅后出现 `corrupted double-linked list` / `malloc(): unaligned tcache chunk detected`。`monitor` 入口使用 `--disable_opengl` 规避该崩溃，保留 ROS2 订阅与曲线功能。不要再附加 `--start_streamer`：PlotJuggler 3.17.2 中它会在布局已启动订阅后再次切换启停，造成曲线停在第一小段。目录不存在和 `connectSlotsByName` 警告本身不代表监控失败。

| 按键 | 动作 |
|---|---|
| L / R | 选择左 / 右臂，不产生运动 |
| 1～5 | 底座、肩、肘、腕俯仰、末端滚转 |
| + 或 = / - | 所选关节点动 ±5° |
| Space | 两臂减速停止并保持，飞行继续 |
| H | 所选臂返回竖直工作姿态 `[0,0,0,0,0]` |
| P | 竖直 → 肩 +5° → 停留 2 s → 竖直 |
| Esc / Ctrl+C | 停止机械臂并退出键盘，飞行继续 |

按键由获得焦点的 WSL 终端接收。正方向遵循 URDF 关节轴。显示实际角、目标角和状态。一次只执行一只臂的一个动作，动作中运动键丢弃，停止键优先。速度 0.10 rad/s，加速度 0.20 rad/s²。心跳丢失 0.5 墙钟秒、关节状态过期或机体越出 0.25 m / 8° 动臂安全范围时，手动动作停止。

活动范围为竖直工作姿态附近 ±10°，并与原 URDF 限位取交集。肘关节竖直姿态正好位于下限，因此在零位继续按负向会明确报告限位；先按正向到 +5° 后可用负向回到零位。管理器用加 3 mm 余量的碰撞几何包围盒检查路径，拒绝接近起落架的动作，不绕障规划；这不是全工作空间避碰保证。

左右 `follow_joint_trajectory` action 及关节顺序保留。管理器通过同一轨迹控制器的 `joint_trajectory` topic 下发平滑轨迹。飞行入口不启动旧 demo 和 `tilt_controller`；不要另开客户端直接发送轨迹绕过管理器。

## 自动验收与物理诊断

```bash
# 每轮：起飞并展开、30秒固定臂悬停、四段平移、左右臂标准回放各一次
ros2 run dragonfly_flight flight --scenario acceptance --record --output log/my_acceptance_1
ros2 run dragonfly_flight verify log/my_acceptance_1
# 使用新目录重复三轮；verify 未通过返回非零退出码。
# 或一次执行三轮并生成曲线：
bash src/dragonfly_flight/test/run_acceptance.sh

# 先稳定悬停，再短时自由落体 / 单轴已知合力
ros2 run dragonfly_flight flight --mode ideal_wrench --scenario freefall --output log/my_freefall
ros2 run dragonfly_flight flight --mode ideal_wrench --scenario force_x --output log/my_force_x
# force_y、force_z 同理，结果为 diagnostic.json。

# 无重力、无旋翼、无理想外力，检查内部动臂和机体反向响应
ros2 run dragonfly_flight flight --mode physics --scenario physics --step .002 --output log/my_momentum
# .001、.0005 步长使用新目录重复。

colcon test --build-base build-sim --install-base install-sim --packages-select dragonfly_arm dragonfly_control dragonfly_flight dragonfly_sim
colcon test-result --test-result-base build-sim --verbose
```

Gazebo 真值验收：30 s 位置 RMS ≤0.10 m、姿态 RMS ≤3°、饱和不持续超过 1 s；平移完成 5 s 后误差 ≤0.10 m；动臂峰值 ≤0.25 m / 8°、动作结束 5 s 后 ≤0.10 m / 3°。实际肩关节行程必须达到 4.5°，避免把未执行动作算作通过。理想力成绩不能用于旋翼悬停验收。

输出目录保存模型、控制器、PX4 参数快照、`flight.csv`、`events.json` 和进程日志；`--record` 保存 ROS bag，PX4 模式保存 ULog。`verify` 写 `metrics.json`，其中每只臂分别包含三轴/合成峰值、RMS、动作前 2 s 基线增量、5 s 恢复误差、进入验收带并保持 1 s 的恢复时间、关节行程和饱和时间。物理诊断保存 `momentum.csv`：force 列为线动量 kg·m/s，torque 列为绕世界原点角动量 kg·m²/s。

## 模型与控制边界

默认：Gazebo 传感器 → PX4 估计 → PX4 位置/速度控制 → PX4 姿态/角速度控制 → PX4 非线性有界分配 → Gazebo。ROS 仅发送 NED 位置轨迹，未创建飞行执行器发布者。显式 `--mode px4_actuators` 保留旧 ROS 控制/分配调试路径。ROS 轨迹与监测循环 100 Hz、仿真时间；起飞前用真值对齐世界原点，起飞后控制反馈来自 PX4。真值用于评价、机体 TF 和动臂稳定条件。

总质量 5.415 kg。折叠构型质心约 `(0.03533,0,-0.11626)` m，竖直构型质心约 `(0.01077,0,-0.14165)` m，全机惯量由更新后的 URDF 合成。每臂质量为 0.9735 kg，不含桌边固定夹；各实体质量分配、质心和惯量仍为简化估算，详见工程根 README。原生控制始终使用竖直构型标称质心与全惯量，折叠起飞、展开及后续动臂时不随关节实时更新，从而保留真实扰动。旧 direct_actuator 调试模式沿用折叠起飞、展开时切到竖直标称参数的两段策略。力矩统一以机体原点为参考。分配包含共同倾转轴、变化的旋翼位置和反扭矩，每周期最多三次有界最小二乘，均衡推力项抑制冗余解漂移。

仿真假设：推重比 2、电机时间常数 0.05 s、反扭矩/推力比 0.02 m、倾转 ±π/2 / 0.5 rad/s / 8 N·m。两臂由 500 Hz effort 轨迹控制器输出力矩，每臂按底座、肩、肘、腕、滚转顺序限幅为 `±[2.941995,5.883990,2.941995,2.941995,2.941995] N·m`。该数值来自 12 V ST3215 峰值力矩，肩部按两只舵机理想共同输出计算，不代表持续可用力矩。硬件参数仅保存在 `dragonfly_arm/config/model_dimensions.yaml`；生成的 `model.json.arm_effort_limits` 按完整关节名称保存，URDF、控制接口、Gazebo 实际限幅、饱和检测和验收共用同一来源。只有读取历史记录时兼容旧的标量限幅。厂家空载速度是资料值，运行速度和加速度仍沿用 `config/flight.yaml` 的保守值，不推测热模型或力矩—速度曲线。初始化一次 home，正常运行不重设关节位置。

飞行中只下发关节力矩。Gazebo 物理系统在存在 `JointForceCmd` 时优先执行力矩，未启用位置或速度伺服作为辅助。

`/dragonfly/truth` 是 `uav_base_link` 原点的 world ENU 真值。监控另发布 `/dragonfly/reference_pose`、`/dragonfly/position_error`、`/dragonfly/position_error_norm`、`/dragonfly/attitude_error`、`/dragonfly/attitude_error_norm`、`/dragonfly/arm_active` 和 `/dragonfly/ready`；位置误差定义为实际减目标，姿态分量为 roll/pitch/yaw，单位遵循 SI。`ready` 仅在部署完成、机体稳定且机械臂空闲时为真。`/dragonfly/efforts`、`/dragonfly/rotor_speeds` 用 CSV 字符串兼容当前 bridge。`/dragonfly/target` 仅在 manual 接受 world 位置目标，姿态始终保持 ENU 零航向水平。

PX4 估计过期 0.2 仿真秒、原生执行器/分配遥测过期 0.25 仿真秒、非法状态或离开已解锁 Offboard 将中止。分配器对非法求解连续三次输出清零，旧 ROS 分配模式则中止。监控先停止物理进程，再关闭本次子进程并保存日志，也监控控制进程卡住。Offboard 丢失设为终止，不依赖 x500 自动降落。直接执行器模式绕过原生着陆检测器的推力参考，因此关闭 SITL 自动着陆/起飞超时解锁计时，退出由本入口负责。

2026年11月3日已接入第一版模型前馈和飞行基座六维净残差 NDOB。NDOB 使用实际关节状态及实际旋翼转速／倾转角对应的模型执行器 wrench，默认关闭，可独立启用影子记录或闭环补偿；配置、日志、数学定义和未验证边界见 [NDOB 说明](../../docs/ndob.md)。本轮仍没有完成风场、实机标定、末端控制、抓取或 NDOB 动态飞行验收。同模型内部接触沿用原模型配置；几何间隙检查不能证明未建模接触的实物安全性。仿真结果不代表实物抗扰性能。
