# Dragonfly 原生 PX4 矢量控制适配（2026-10-05）

当前默认 `px4_native` 已贯通 Gazebo 传感器 → PX4 EKF → PX4 位置/速度环 → PX4 姿态/角速度环 → PX4 非线性控制分配 → 六电机与六倾转伺服 → Gazebo 多刚体动力学。ROS 仅发布 Offboard 位置轨迹和机械臂轨迹，不发布飞行执行器指令。验收使用当前 Dragonfly 整机模型的 Gazebo 真值。

实际编译树是 `/home/ubuntu/PX4-Autopilot`，版本 v1.16.2；模型及入口是 `/home/ubuntu/Dragonfly`。桌面 PX4 目录不完整，不是本机编译树。WSL 仓库是运行源码入口，桌面镜像可能滞后，不作为当前状态依据。

## 原代码与本次适配

本机 `original/` 保留组长提供的五个参考文件，未修改，且不纳入本次仓库提交；构建只依赖仓库中的适配补丁。`manifest.json` 是首次接入的历史哈希，`native_manifest.json` 是当前适配源码的哈希。HTE 两个原文件与 v1.16.2 相同，矢量模式使用固定加速度尺度并关闭 HTE。

保留组长的水平姿态下直接产生三维推力的控制目标。将写死 8 kg、0.81 m、舵机偏置与 Motor 7–12 输出的分配实现替换为匹配当前三组双轴倾转机构的非线性有界分配器 `ControlAllocationDragonfly`。原生顺序去饱和分配器恢复为 PX4 上游实现；`DF_EN=0` 时位置控制也走上游路径，避免把 Dragonfly 参数应用到普通多旋翼。

| 问题 | 当前实现 |
|---|---|
| 启动机型及输出数量 | 独立 `40160_gz_dragonfly`；`DF_EN=1`、`CA_AIRFRAME=16`、`CA_METHOD=3`；6 Motor + 6 Servo |
| 坐标与推力语义 | Gazebo/world 使用 ENU，URDF/body 使用 FLU；ROS 轨迹转为 NED；位置环将 NED 推力按当前姿态转为 FRD，消息 `thrust_body` 保持 FRD 语义 |
| 固定姿态 | 飞行期望 roll/pitch=0；NED yaw=π/2 对应 ENU yaw=0；移除原 AUX 横滚映射，输入四元数检查并归一化 |
| 推力尺度及抗积分饱和 | `DF_ACC=19.62 m/s²`，水平抗积分饱和采用相同尺度，HTE 在矢量模式不改变尺度或积分器 |
| 机体参数 | 从同一 URDF 和尺寸配置生成质量、质心、全惯量、安装半径、轴距、桨毂偏移、推力与反扭矩参数 |
| 分配与约束 | 按 `Rz(方位角) Rx(outer) Ry(inner)` 计算旋翼轴和移动位置；包含旋翼反扭矩；每次至多 3 次线性化、有界最小二乘，限制推力、倾角与倾转速度 |
| 电机与伺服量纲 | `THR_MDL_FAC=0`，电机输出为 `sqrt(F/Fmax)`，伺服输出为 `angle/(π/2)`；不把伺服编码为电机 |
| 分配反馈 | 从实际限幅后的输出重建非线性合力/力矩，并还原为 PX4 控制量，供分配状态与角速度积分限幅使用 |

## 当前物理参数与通道

2026-10-09 图纸模型：质量 5.415 kg；安装半径 0.692820323 m；同组电机轴距 0.36 m；单元中心高度 0.06 m；桨毂沿倾转支架高度 0.038 m。推重比 2，单旋翼最大推力 17.70705 N，反扭矩/推力比 0.02 m。倾转范围 ±π/2，指令速度上限 0.5 rad/s；Gazebo 倾转以限幅力矩驱动，包含真实跟随误差。

标称参数取竖直机械臂构型：FLU 质心 `(0.01077138,0,-0.14164883)` m，惯量对角约 `(0.62136976,0.59689095,0.62865996)` kg·m²，`I_xz≈0.01044967` kg·m²。全惯量进入力矩尺度换算，质心偏移进入绕机体原点的力矩换算。两臂各 0.9735 kg；实体质量分配和简化几何惯量为估算。机械臂逐关节峰值限幅由共同尺寸配置经 URDF 写入 `model.json.arm_effort_limits`，不再统一使用 5 N·m。`DF_RATE=26.6666667` 与当前角速度环增益共同定义归一化角加速度尺度。

原生控制使用固定竖直构型标称质心/惯量；从折叠构型起飞、展开及后续动臂都由物理动力学产生扰动，不按实时关节位置补偿。旧 `px4_actuators` 调试模式仍使用其原有折叠/竖直两段标称参数，不参与本次原生验收。

| PX4 下标 | 对应对象 | 正方向/范围 |
|---|---|---|
| Motor 0,1 | A 单元两旋翼 | 0…1；交替反扭矩方向 |
| Motor 2,3 | B 单元两旋翼 | 同上 |
| Motor 4,5 | C 单元两旋翼 | 同上 |
| Servo 0,1 | A outer, inner | -1…1 对应 -90°…90°，零位水平 |
| Servo 2,3 | B outer, inner | 同上 |
| Servo 4,5 | C outer, inner | 同上 |

## 构建、启动和重验

在 WSL Ubuntu 执行：

```bash
cd "/home/ubuntu/Disturbance Rejection Control Algorithm Design"
bash src/dragonfly_flight/scripts/build.sh
# 默认是 px4_native：地面解锁、起飞、展开双臂后开放键盘
bash src/dragonfly_flight/scripts/sim.sh --gui --record
# 独立终端
bash src/dragonfly_flight/scripts/sim.sh keyboard
bash src/dragonfly_flight/scripts/sim.sh monitor
# 三轮整机验收（包含悬停、平移和左右臂各一次肩 +5° 回放）
bash src/dragonfly_flight/test/run_acceptance.sh log/my_native_acceptance
```

`--mode px4_actuators` 显式进入旧 ROS 分配调试模式；`ideal_wrench` 和 `physics` 继续用于诊断。一次只启动一套飞行实例。

构建脚本检查 PX4 HEAD 和补丁状态；重复构建不会重复修改源码。`px4_v1.16.2_vector.patch` 包含当前完整适配，包括机型、参数、分配、位置环和 DDS 输出。原有 Gazebo ENU 磁场修正仍由单独补丁应用。部分应用、版本不符或冲突会报错，避免覆盖现有工作。

```bash
python3 src/dragonfly_flight/test/px4_vector/run_px4_vector_tests.py --output /tmp/native_vector_tests
# 同一测试入口也可运行上游位置环回归
python3 src/dragonfly_flight/test/px4_vector/run_px4_vector_tests.py \
  --source /home/ubuntu/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PositionControlTest.cpp \
  --output /tmp/stock_position_tests
```

9 项原生 C++ 测试覆盖坐标变换、固定尺度、非法姿态、普通多旋翼路径、机构几何、分配收敛、角速度约束、不可达目标限幅和非线性残差；上游位置控制 16 项测试全部通过。ROS 工程测试覆盖参数生成、轨迹坐标和遥测过期等，构建及现有测试通过。单元测试不替代飞行验收。

## 验收依据与结果

每轮使用真实六旋翼推力、六个力矩倾转伺服与双臂多刚体动力学，从地面解锁起飞，展开双臂，再测量 30 s 悬停、四段平移、左右臂回放。位置真值是 `uav_base_link` 原点，控制反馈来自 PX4 EKF；真值仅用于起飞前原点对齐、评价及动臂许可。

验收阈值：悬停位置 RMS ≤0.10 m、姿态 RMS ≤3°、连续饱和 ≤1 s；平移完成 5 s 后误差 ≤0.10 m；动臂峰值 ≤0.25 m/8°，结束 5 s 后 ≤0.10 m/3°，实际肩关节行程 ≥4.5°。`verify` 对原生模式额外要求 `native_chain.json` 证明 PX4 的位置、速度、姿态、角速度和分配控制全部启用，ROS 不发布飞行执行器。

以下为 2026-10-05、旧 4.628 kg 模型的历史结果，不作为 2026-10-09 图纸模型的验收成绩。旧模型已通过独立悬停及三轮完整重复验收；第三轮确认独立机型 40160 启动。前两轮由 4001 启动后加载全套 Dragonfly 参数，位置环与分配均为原生 Dragonfly；第三轮直接使用 40160 配置。未沿用 x500 或旧 direct_actuator 成绩。

| 轮次 | 启动机型 | 悬停位置 RMS | 悬停姿态 RMS | 平移 5 s 最大误差 | 动臂后 5 s 最大误差 | 结论 |
|---|---:|---:|---:|---:|---:|---|
| 1 | 4001 | 3.94 cm | 0.79° | 6.27 cm | 5.94 cm | 通过 |
| 2 | 4001 | 5.75 cm | 0.70° | 9.01 cm | 9.43 cm | 通过 |
| 3 | 40160 | 4.64 cm | 1.45° | 5.97 cm | 3.82 cm | 通过 |


Windows 工作区 `outputs/flight/px4_native/` 保存替换前备份、当前补丁、构建/测试日志和验收记录。完整飞行记录位于 `/home/ubuntu/Dragonfly/log/native_acceptance_1`、`native_acceptance_2`、`native_acceptance_3`，包含模型、参数、CSV、事件、控制链证明、曲线、ROS bag 和 ULog。

本次结论适用于无风、当前估计质量/惯量与执行器参数的 SITL 场景。没有实机标定、硬件烧录或整机实飞验收；普通 x500 仅做上游位置控制单元回归，未做其整机飞行验收。

## 2026-10-09 图纸模型复验

采用当前 5.415 kg 图纸模型、每臂 0.9735 kg 及 12 V 逐关节峰值限制，独立机型 40160 连续三轮完整验收均通过。悬停位置 RMS 分别为 5.42/4.22/4.83 cm，姿态 RMS 为 1.04/0.52/0.57°；三轮连续饱和均为 0 s，十个机械臂关节的实际力矩均未超限。保持原伺服增益、运行限速和验收阈值，PX4 桥接与原生矢量适配代码未作本轮修改。ROS 两包构建、回归测试及固定基座 Gazebo 冒烟测试通过。

完整记录在 `/home/ubuntu/Dragonfly/log/drawing_native_acceptance_20261009_1`、`_2`、`_3`，详细恢复误差、肩关节行程及模型边界见 [工程 README](../../../../README.md)。这些新结果替代旧模型成绩作为当前参数的 SITL 验收依据，不代表实机或风扰验收。

恢复适配前源码应先检查并反向应用当前补丁，再处理构建脚本；保留单独磁场补丁。不要 reset 整个 PX4 仓库。原五文件及首次接入备份另存于 `outputs/flight/px4_vector_integration/`。
