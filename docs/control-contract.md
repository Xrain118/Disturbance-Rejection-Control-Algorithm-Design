# 第一步控制接口与基线契约

更新：2026-11-05。适用于本仓库 `px4_native` / `px4_wrench` SITL，不是实机标定或烧录证明。

## 唯一工程与版本证据

实施工程为 `E:/subjects/Disturbance Rejection Control Algorithm Design`，WSL 对应 `/mnt/e/subjects/Disturbance Rejection Control Algorithm Design`。`/home/ubuntu` 下同名检出不是同步镜像。仿真包必须来自本工程 `install-sim`；外部 PX4 默认 `/home/ubuntu/PX4-Autopilot`。

每次运行保存 `provenance.json`：工程/PX4 提交、工作区状态、文件 SHA-256、实际包路径、CMake 来源和 PX4 二进制哈希。`source.diff` / `px4_source.diff` 包含相对 HEAD 的暂存及未暂存差异；新增源码另存 `*_untracked/`。运行前核对 `native_manifest.json` 的适配文件，不自动修改外部 PX4。

`flight_config.yaml` 是本轮实际 YAML 原文，`model.json`、`robot.urdf`、`world.sdf`、`run.json`、`px4_parameters.json` 分别记录模型、物理环境、启动选项和 PX4 参数。桥和运行时使用同一 `model.json` SHA-256 作为模型版本。源码快照取自启动前；后续文档更新不改变既有实验快照。

物理插件从 `dragonfly_sim/lib` 加载，缺失时启动前报错。PX4 的 rcS 含不支持带空格工作目录的表达式，运行器为其创建独立 `/tmp/dragonfly_px4_*` 工作目录，退出后归档至本轮 `px4/` 并清理临时目录；路径关系写入 `px4_workspace.json`。这是启动兼容处理，不改变控制参数。

## 坐标、参考点与尺度

世界系为 ENU；PX4 世界系为 NED。机体系为 FLU，PX4 消息机体系为 FRD。定义 `S=diag(1,-1,-1)`。

`BodyWrenchCommand` 不改消息字段：`header.frame_id=uav_base_link`，力为 N，力矩为 N·m，`torque_reference_point=uav_base_link`。O 为机体原点，c 为 O 指向标称质心的 FLU 向量，J 为关于质心、以 FLU 表达的完整惯量矩阵，不能用绕 O 惯量替代。

```text
a_scale = 9.81 × thrust_to_weight
alpha_scale = 4 / 0.15
thrust_FRD = S F_FLU / (m a_scale)
torque_FRD = S J⁻¹ (tau_O - c × F_FLU) / alpha_scale

F_FLU = S thrust_FRD × m a_scale
tau_O = J (S torque_FRD) × alpha_scale + c × F_FLU
```

唯一换算实现为 `dragonfly_fcu_bridge.wrench_adapter`；桥、运行日志及 PX4 参数生成共用此契约。质量、质心和惯量取生成模型的固定竖直臂构型，不能在飞行展开时随意换成折叠构型尺度。实际多刚体动力学仍随机械臂运动变化。

PX4 `control_allocator_status.unallocated_*` 为“请求 − 限幅输出重建值”；其物理残差也须按上述完整线性映射转换，包含 `c × 残差力`。这些输出重建值不是实际执行器作用测量。旧 `px4_actuators` 调试分配器的残差为“实现 − 请求”，不与本基线混用。

桥继续检查消息有效性、坐标、参考点、有限数值、模型版本、序列递增及消息时效；默认最大年龄 0.2 s、未来容差 0.05 s。第一步不调整控制增益、分配约束或超时阈值。

## 六电机和六倾转通道

下标为零基，PX4 参数编号从 1 开始。三单元方位角依次为 0、120°、240°，姿态按 `Rz(方位角) Rx(outer) Ry(inner)`。正角遵循关节局部轴右手定则；每组左右旋翼共用两个倾角，推力独立。

| 单元 | 电机下标与关节 | 伺服下标与关节 |
|---|---|---|
| A | 0 `unit_a_rotor_left_joint`；1 `unit_a_rotor_right_joint` | 0 `unit_a_outer_joint`；1 `unit_a_inner_joint` |
| B | 2 `unit_b_rotor_left_joint`；3 `unit_b_rotor_right_joint` | 2 `unit_b_outer_joint`；3 `unit_b_inner_joint` |
| C | 4 `unit_c_rotor_left_joint`；5 `unit_c_rotor_right_joint` | 4 `unit_c_outer_joint`；5 `unit_c_inner_joint` |

- 电机指令 `u=sqrt(F/Fmax)`，范围 0…1；Gazebo 目标转速 `u × max_rotor_speed`，力 `kf × omega²`。`THR_MDL_FAC=0`。偶数电机 `ccw`，奇数 `cw`；机体反扭矩沿推力轴分别为负、正。
- 伺服指令 `v=angle/tilt_limit`，范围 −1…1。当前 `tilt_limit=π/2` 与 Gazebo Servo ±90° 映射一致；零位推力沿局部 +Z。改动角度范围必须同时检查分配与 Gazebo 映射，不能只改一处配置。
- `SIM_GZ_EC_FUNC1…6=101…106`；`SIM_GZ_SV_FUNC1…6=201…206`。不采用桌面旧版 Motor 7–12 舵机编码。

## 控制所有权

| 模式 | PX4 位置/速度/姿态/角速度反馈 | Offboard、分配 | ROS 飞行输出 |
|---|---|---|---|
| `px4_native` | 四项均开 | 均开 | 位置轨迹 |
| `px4_wrench` | 四项均关 | 均开 | 物理 wrench，经桥转换为推力/力矩设定 |

运行时每个控制周期检查模式标志，每个仿真秒刷新 `/fmu/in/actuator_motors`、`/fmu/in/actuator_servos` 的发布者列表；两模式均要求列表为空。链路证据定期写入 `native_chain.json` / `wrench_chain.json`；违例写入证据并中止。离线验收调用相同规则，缺失证据失败。该检查不能替代后续真实部署的命令授权与链路治理。

## 无补偿对照与复现

```bash
cd '/mnt/e/subjects/Disturbance Rejection Control Algorithm Design'
bash scripts/build_sim.sh
bash scripts/run_baselines.sh --output log/step1_baselines_NEW
# 单轮可通过 --config 指定完整 YAML；省略时仍使用包内 flight.yaml。
bash src/dragonfly_flight/scripts/sim.sh --mode px4_wrench \
  --scenario acceptance --record --config log/step1_baselines_NEW/baseline_config.yaml \
  --output log/individual_NEW
```

批处理交替运行三轮原生、三轮研究无补偿验收；两者使用相同配置和 0.002 s 物理步长，机械臂差量前馈不开启，NDOB `enabled=false`、`compensation_enabled=false`。研究模式保留标称逆动力学反馈；当前前馈模块仍计算差量供日志诊断，但施加量为零，不能将其描述为全部动力学计算被关闭。

每轮独立保存 ROS bag、PX4 ULog、CSV、链路、模型、参数与失败信息。`summary.json` 汇总所有尝试；无悬停等不完整记录生成失败指标，不跳过失败样本。单轮默认墙钟上限 1200 s，超时仅结束该轮拥有的进程，不终止其他任务。

验收保持原阈值：悬停至少 29.9 s，位置 RMS ≤0.1 m，姿态 RMS ≤3°，连续饱和 ≤1 s；四段平移完成 5 s 后误差 ≤0.1 m；左右臂峰值 ≤0.25 m/8°，动作结束 5 s 后 ≤0.1 m/3°，肩行程 ≥4.5°，关节力矩和起落架间隙检查通过。片段缺失或不足以覆盖恢复时刻均失败。

只有接口检查和六轮基线都通过，第一步才整体完成。失败时保留原因，不放宽阈值、不提前加入 ESO 补偿。当前结果见项目根目录《当前进度》。
