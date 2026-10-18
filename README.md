# Disturbance Rejection Control Algorithm Design

整理日期：2026年10月18日。

研究双臂运动与外扰下的无人机稳定控制。当前交付包括 Dragonfly 仿真源码复用、双 ESP32 主动遥测接入、厂家参数电流力矩粗估、记录及回放。动力学前馈和残余扰动观测器仍待实现。原生 PX4 仿真作为独立基线，不连接真实机械臂。

[产品需求](产品需求.md) · [当前进度](当前进度.md) · [软件架构](软件架构.md) · [遥测协议](docs/telemetry-protocol.md) · [固件部署与修改](docs/firmware.md) · [验证记录](docs/verification-20261018.md)

## 构建与采集

环境为 Ubuntu 22.04 / ROS 2 Humble。仅采集需 ROS 消息依赖和 `python3-serial`，不需要 Gazebo、PX4 或旧 Dragonfly 工作区。先按本机软件源安装依赖，再运行：

```bash
cd "/home/ubuntu/Disturbance Rejection Control Algorithm Design"
bash scripts/build_hardware.sh
source install-hardware/setup.bash
ros2 launch dragonfly_serial dual_arm_serial.launch.py
```

启动前编辑 [双串口配置](src/dragonfly_serial/config/dual_arm_serial.yaml)，将两个占位路径替换为明确的 `/dev/serial/by-id/...`，填写实机设备 ID；不自动扫描、不猜左右。可复制 YAML 并通过 `config:=/absolute/path.yaml` 指定，也可直接覆盖节点参数：

```bash
ros2 run dragonfly_serial dual_arm_serial --ros-args \
  -p left.port:=/dev/serial/by-id/LEFT_DEVICE \
  -p right.port:=/dev/serial/by-id/RIGHT_DEVICE \
  -p left.device_id:=LEFT_ID -p right.device_id:=RIGHT_ID \
  -p record_path:=/tmp/arms-session-001.jsonl
```

记录文件必须不存在，避免覆盖已有试验。节点默认只接收，不逐帧查询、不发送动作。实际设备需已烧录支持主动流的固件；本次没有烧录。首次连接可能受 USB 转串口复位电路影响，不能承诺 DTR/RTS 置低消除所有复位。

仅在需要更改遥测配置时调用服务；`accepted` 表示本地排队，设备最终确认见 `config_ack`：

```bash
ros2 service call /hardware/left_arm/configure_telemetry \
  dragonfly_interfaces/srv/ConfigureTelemetry '{operation: 2, rate_hz: 50}'
# operation: 0 查询，1 关闭新版流，2 启用；rate_hz 1～50。
```

主要接口：每侧 `/hardware/{left,right}_arm/telemetry`、`joint_torque_estimate`，以及 `/hardware/joint_states` 和 `/hardware/diagnostics`。`JointState.effort` 留空，速度单位未核实时 `velocity` 留空；每条 JointState 只含该侧本轮有效关节，不拼接另一侧旧数据。

## 离线回放与可选运动学

```bash
ros2 run dragonfly_serial telemetry_replay /tmp/arms-session-001.jsonl --speed 1
```

回放不打开串口、不能发送配置，复用实时解析、关节换算和质量检查，并恢复记录中的换算配置。保留录制时主机墙钟时间；回放节奏使用记录的主机单调时钟间隔，诊断新鲜度使用回放运行时钟。ESP32 时钟既不与主机直接相减，也不在两臂之间视为同步。

已构建 `dragonfly_arm` 后，可启动 `ros2 launch dragonfly_serial hardware_kinematics.launch.py`，输出在 `/hardware` 下。它只针对收到完整五关节的位置消息计算 FK，不发布无人机世界位姿；几何差异尚待实物核对。硬件采集不自动启动此节点或仿真。

## 继承仿真与依赖隔离

源码来自 Dragonfly 提交 `8a272e61b05e95f82e57468e46fb77cfe6c01432`，原文件列表见 [迁移清单](docs/migration-source.json)，保留 [Apache 2.0 许可证](LICENSE)。仅迁移受 Git 管理的两个包、模型、配置和测试，未复制构建物、日志、历史 Git 或完整 PX4。

外部依赖集中在 [scripts/env.sh](scripts/env.sh)，支持 `ROS_SETUP`、`GZ_CONTROL_WS`、`PX4_ROS_WS`、`PX4_ROOT`、`XRCE_AGENT_ROOT`。可在未跟踪的 `config/local.env` 中覆盖。普通构建不修改共享 PX4；只有显式执行 `bash scripts/prepare_px4.sh` 才检查版本、应用补丁并构建 PX4。该步骤本次没有执行，沿用本机已准备的外部依赖。

```bash
bash src/dragonfly_flight/scripts/build.sh
bash scripts/test.sh
bash scripts/smoke.sh
# 飞行基线按继承说明显式运行，默认不启动硬件采集：
bash src/dragonfly_flight/scripts/sim.sh --help
```

[继承飞行说明](src/dragonfly_flight/README.md)与历史成绩不等于本次重新完成飞行验收。本次重新验证的范围和限制见[验证记录](docs/verification-20261018.md)。

## 使用边界

- **事实**：软件支持双路接收、类型化发布、错误记录与回放；已有软件检查结果可复核。
- **设计决策**：固件目标 50 Hz，主机串口 921600 8N1，舵机总线 1 Mbps；两路独立监测新鲜度。
- **研究假设**：厂家电流线性系数可提供有用的力矩粗估；改善抗扰性能仍需实验。
- **待确认**：实物型号、机械符号、速度单位、实机持续频率、时延、力矩精度及飞行失效策略。

力矩输出标记为 `factory_linear / uncalibrated / factory-v1`。它不是标定传感器测量、接触力矩或底座 wrench，不可仅因字段单位是 N·m 就直接用于飞行补偿。
