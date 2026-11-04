# 飞行基座六维净残差 NDOB

更新日期：2026年11月3日。代码位于 `src/dragonfly_flight/dragonfly_flight/ndob.py`，运行接入位于 `runtime.py`。当前完成的是纯计算模块、单元测试和 SITL 运行链接入；尚未完成 ROS／Gazebo／PX4 动态验收或实机验证。

## 1. 观测对象与坐标

NDOB 输出无人机**飞行基座**的六维净残差，不是左右机械臂安装底座各自的扰动，也不能从该总残差中分离左臂、右臂、风扰或执行器误差。

\[
\widehat d_O=
\begin{bmatrix}\widehat d_{F,O}\\\widehat d_{\tau,O}\end{bmatrix}\in\mathbb R^6.
\]

- 顺序：三轴力在前，三轴力矩在后；单位分别为 N、N·m。
- 坐标：机体 FLU；世界模型沿用 ENU，不直接混用 PX4 NED／FRD。
- 力矩参考点：`uav_base_link` 原点 \(O\)。
- 正号：残差对飞行基座的正向作用。补偿量为正时，从期望整机 wrench 中减去。

残差可能包含左右臂模型误差、关节跟踪误差引起的附加作用、质心／惯量误差、执行器响应误差、风扰和其他未建模动态。模块不重新估计质量、质心或惯量。

## 2. 动量型 NDOB

完整耦合模型的基座方程为：

\[
M_{bb}\dot\nu_b+M_{ba}\ddot q+h_b=W_{\rm act}+d_O.
\]

定义基座六维广义动量：

\[
p=M_{bb}\nu_b+M_{ba}\dot q=
\begin{bmatrix}P\\L_O\end{bmatrix}.
\]

已知动量运输与重力偏置为：

\[
b=\begin{bmatrix}
\omega\times P\\
\omega\times L_O+v\times P
\end{bmatrix}+g_b,
\qquad \dot p=W_{\rm act}+d_O-b.
\]

辅助变量 \(\xi\) 与正对角增益 \(L\) 构成：

\[
\widehat d_O=L(p-\xi),
\qquad
\dot\xi=W_{\rm act}+\widehat d_O-b.
\]

代码不直接微分 IMU 加速度或状态估计值。辅助方程采用前向离散更新，并检查 `max(L·dt) < 2`；异常步长、时间跳变或非法输入会使观测失效并重新初始化。

`coupled_momentum_and_bias()` 从现有 URDF 耦合动力学计算 \(p\) 和 \(b\)。完整模型已经包含左右臂惯性、重力和速度耦合，因此**不再叠加左右臂底座 wrench**，避免重复计数。

## 3. 实际输入来源

| 输入 | 当前来源 | 性质与限制 |
|---|---|---|
| 基座姿态、线速度、角速度 | PX4 `VehicleLocalPosition`、`VehicleOdometry`，转换到 ENU／FLU | 状态估计，不是 Gazebo 真值控制输入 |
| 双臂实际 \(q,\dot q\) | `/joint_states` 中十个机械臂关节 | 仿真实际反馈；缺少或非有限值会使 NDOB 失效 |
| 六个实际倾转角 | `/joint_states` 中六个倾转关节 | 与机械臂共用关节状态时间 |
| 六个旋翼实际转速 | `/dragonfly/rotor_speeds` | 当前为仿真执行器状态，不是实机已验证测量链路 |
| 执行器 wrench | 实际转速和倾转角，经现有 `Platform.wrench()` 计算 | **基于执行器状态的模型估计**，不是六维力传感器测量 |
| 重力、惯性及速度耦合 | 当前 URDF 和 `CoupledDynamics` | 参数仍有估算，误差进入净残差 |

执行器 wrench 使用实际／估计执行器状态，而不是控制器期望 wrench。它自然包含反馈、模型前馈和已施加 NDOB 补偿对执行器的作用，防止将已有补偿再次当作外扰。

当前 `input_t` 是 PX4 状态、关节状态和旋翼状态中最旧的**接收时刻**；字符串桥接的旋翼话题没有独立采样时间戳。输入年龄已记录，但这不等于完成严格的多传感器硬件时间同步。

## 4. 补偿接口

运行链路为：

\[
W_{\rm cmd}=W_{\rm nominal}+\Delta W_{\rm model}-W_{\rm NDOB,comp}.
\]

补偿只作用于机载电脑输出的期望整机六维 wrench，随后仍经原 `BodyWrenchCommand`、PX4 归一化适配和 Dragonfly 控制分配器。没有直接修改单个电机、倾转舵机、机械臂轨迹、原 PD 增益或验收阈值。

默认配置保持原系统行为：

```yaml
ndob:
  enabled: false
  shadow_mode: true
  compensation_enabled: false
```

影子运行只记录、不补偿：

```yaml
ndob:
  enabled: true
  shadow_mode: true
  compensation_enabled: false
```

只有在影子运行、已知扰动真值和补偿符号测试通过后，才可显式设置：

```yaml
ndob:
  enabled: true
  shadow_mode: false
  compensation_enabled: true
```

`flight.yaml` 还提供平移／转动增益、输出力／力矩限幅、补偿变化率、状态超时、异常步长和持续饱和保护参数。现有数值仅为未验证的仿真起始值，不是实机安全参数。

## 5. 保护与日志

- 状态过期、未来时间、NaN、异常 `dt` 和大时间跳变：观测失效，补偿按变化率平滑撤除，下一份有效状态重新初始化。
- 输出限幅：分别限制三轴残差力和三轴残差力矩。
- 补偿变化率：限制启用、撤除和恢复时的突变。
- 持续执行器饱和：达到 `saturation_hold_time` 后停止增加补偿并撤除，记录 `allocator_saturation_block`。
- `shadow_mode=true` 或 `compensation_enabled=false` 时，补偿目标始终为零。

启用后输出：

- `ndob.csv`：估计值、原始估计、补偿量、动量、已知偏置、执行器 wrench、输入时刻／年龄、有效性、限幅、重置和饱和事件。
- `/research/ndob/estimated_wrench`：FLU、关于 \(O\) 的净残差。
- `/research/ndob/compensation_wrench`：同坐标与参考点，正号表示应从期望 wrench 中减去的量。
- `/research/ndob/status`：开关、有效性、输入年龄、执行器输入来源和事件计数。
- `ndob_config.json`：本次实际配置快照；`--record` 时上述 ROS 话题加入 rosbag。

`compensation_active=false` 表示当前不允许新补偿；失效后的变化率撤除过程中，日志中的补偿量仍可非零，这是预期行为。

## 6. 测试与运行

纯计算测试：

```bash
cd src/dragonfly_flight/test
python -m unittest test_ndob test_coupled_dynamics test_feedforward -v
```

2026年11月4日复验：上述 22 项通过；飞控 wrench 适配另外 5 项通过；新增和修改 Python 文件通过 `py_compile`。测试覆盖零扰动、六通道恒定扰动、时变扰动、已知偏置变化、补偿符号、动量恒等式、旋转与参考点、输出／变化率限幅、失效恢复、持续饱和保护，以及复用动力学求值与直接计算的一致性。

完整环境中构建与启动：

```bash
bash src/dragonfly_flight/scripts/build.sh
ros2 run dragonfly_flight flight --mode px4_wrench --scenario hover --record
```

NDOB 启用时与前馈复用同一次完整动力学求值。重力 wrench 由质量矩阵中的总质量和一阶质量矩恢复，不再为 NDOB 额外执行两次完整模型计算；该优化经过了静重力和非零速度／动臂动量恒等式测试。

正式验收仍需依次完成：固定臂悬停、左臂标准动作、右臂标准动作、双臂展开的影子运行；Gazebo 已知作用点力／力矩真值测试；最后才是保持原 PD、分配器和验收条件不变的闭环 A/B。5° 标准动作与 90° 展开必须分开统计。

## 7. SITL 尝试与未验证边界

2026年11月3日至4日已实际构建并运行 ROS 2／Gazebo／PX4 SITL：

- `log/ndob_shadow_hover_20261104_141643`：已有前馈开启、NDOB 补偿关闭，完成 30.012 秒固定臂悬停。工程 `verify` 结果为 `passed=true`，位置 RMS 0.04448 m、姿态 RMS 1.0413°、最长持续饱和 0 s。该轮证明当前前馈控制链能够完成悬停，但不能证明 NDOB 改善性能。
- 同一轮 NDOB 在悬停窗口内的观测有效率约 90.36%，发生 131 次 `abnormal_dt`、120 次重新初始化；输出限幅为 0 次，补偿量严格为 0。末 30 秒残差力 RMS 约为 `[0.0480, 0.0334, 0.0378]` N，残差力矩 RMS 约为 `[0.00512, 0.01234, 0.00499]` N·m。观测重置仍过多，不能将该有效率视为已通过观测质量验收。
- `log/ndob_shadow_hover_20261103_1742`：补偿和前馈关闭，运行至 `predeploy`，未进入 30 秒正式悬停窗口。末 30 秒位置 RMS 约 0.122 m、姿态 RMS 约 20.46°，因此不是合格悬停数据。观测有效率约 76.05%，`abnormal_dt` 事件 704 次；wrench 计算中位耗时约 77.73 ms。该轮主要暴露了基线稳定裕度和重复动力学计算开销。
- `log/ndob_shadow_hover_optimized_ff`：共享动力学求值并启用已有前馈，实验在起飞阶段被中断，不能用于稳态精度或抗扰结论。
- 各轮 `ndob_summary.json` 由 `scripts/analyze_ndob_shadow.py` 生成；完整轮次另有工程验收文件 `metrics.json`。本轮用户配置保留 `enabled=true`、`shadow_mode=true`、`compensation_enabled=false`。
- 为运行 SITL，修复了含空格路径下的 PX4 启动方式、飞控桥接 ROS 2 可执行入口安装配置，以及低频 `vehicle_control_mode` 与快速执行器遥测混用同一超时造成的误中止。快速执行器／分配话题仍保持 0.25 秒超时，低频模式状态单独使用 1 秒超时。
- `input_age` 使用运行时接收时刻，不是硬件采样时间戳；接近零不能证明多传感器同步或真实感知延迟很小。

当前已取得飞行验收通过的固定臂悬停数据，但 NDOB 观测连续性和周期抖动仍需改进。尚没有已知扰动真值、收敛时间、真实估计延迟、噪声、闭环 A/B 或实机验证，不能宣称 NDOB 改善飞行性能或实机有效。前馈关闭与原 B 组严格等价性仍是正式对照实验前的必要检查。
