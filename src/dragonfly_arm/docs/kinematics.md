# RoArm-M3 五轴运动学

本模型用于同构主从遥操作的几何与计算基础。两台机载机械臂共用参数和算法；左右安装变换由整机 URDF 给出。本次提供正解、解析逆解、雅可比及只求解的 ROS 接口。

## 参数来源和建模约定

参考文件是用户提供的 `C:\Users\LENOVO\Desktop\arduino\RoArm-M3_example\RoArm-M3_config.h` 与 `RoArm-M3_module.h`，并以 2026-10-09 用户实物图纸更新几何。采用 `EEMode=1`、`L4A=171.67 mm`、`EoAT_A=EoAT_B=0`；末端偏移以图纸 `L4B=13.29 mm` 为准，覆盖原源码的 `13.69 mm`。没有调用 `configEoAT()` 的 67.85/64.16/59.07/51.07 mm 安装档位。Arduino 工程保持原样，未来硬件对接必须统一 0.40 mm 参数差异。

2026-10-12 用户确认，当前机械臂固件控制源码位于 `C:\Users\LENOVO\Desktop\arm\RoArm-M3_example`，后续固件分析与对接以该目录为准。上文旧目录仅记录建模时的参数来源。

图纸尺寸与源码运动学约定构成模型依据，不表示舵机零点已标定。单臂总质量和舵机峰值采用厂家规格；连杆质量分配、质心、惯量、机体安装外参和未标注外形仍为近似。图纸水平总长 316.15 mm 与两段长度之和 316.16 mm 的 0.01 mm 差异作为标注舍入处理。

唯一活动参数源为 `config/model_dimensions.yaml` 中的 `arm_kinematics`。Python 库与 Xacro 均读取它。长度为米、角度为弧度；以下所有位姿默认在对应机械臂安装基座 `left_base_link` 或 `right_base_link` 中表达。

| 名称 | 符号 | 当前值 |
|---|---|---:|
| 基座至肩高度 | h | 0.12606 m |
| 肩至肘局部偏移 | a = (ax,0,az) | (0.03000,0,0.23682) m |
| 肘至腕局部偏移 | b = (bx,0,bz) | (0,0,0.14449) m |
| 腕至工具点局部偏移 | c = (cx,0,cz) | (0.01329,0,0.17167) m |
| 模型折叠角 | q_home | (0,0,π/2,0,0) rad |

五个关节角 `q=(B,S,E,W,R)` 分别为底座、肩、肘、腕俯仰、末端滚转，遵循右手规则。肩部两个反向驱动舵机只贡献一个自由度。夹爪 ID17 单独开合，不计入五轴向量，也没有第六个运动关节。

根据约定，把第五轴沿末端延伸向量 c 放置：定义 `γ=atan2(cx,cz)`，`L=||c||`，第五轴原点与腕俯仰原点重合，其局部 +Z 为 `Ry(γ)·ez`。这是使工具位置与源码“不随 roll 改变”的计算一致的理想化轴线；源码没有给出腕部转轴之间的独立轴心距离。若实测发现滚转轴不共线或抓取点有偏心，必须更新运动链，不能只把偏心点当作轴向长度。当前 `tool0` 是该固定参考点，其方向随 roll 变化。

机体到基座变换为 `Trans(0, ±0.15, -0.05) Rx(π)`，左臂取正号、右臂取负号。由此：

`world_T_tool = world_T_uav · uav_T_base · base_T_tool(q)`。

本次仿真的 `world_T_uav` 固定在 z=1.60 m；运动学库没有固定机体的假设，未来可由飞行器位姿提供此矩阵。

## 正运动学

用 `Trans(v)` 表示齐次平移，`Rx/Ry/Rz` 表示右手旋转，则：

```text
base_T_tool = Rz(B) Trans(0,0,h)
              Ry(S) Trans(a)
              Ry(E) Trans(b)
              Ry(W+γ) Rz(R) Trans(0,0,L)
```

令 `φ=S+E+W`，则旋转部分为 `Rz(B) Ry(φ+γ) Rz(R)`。局部径向距离和高度可以独立写成：

```text
r = ax cos(S) + az sin(S)
  + bx cos(S+E) + bz sin(S+E)
  + cx cos(φ) + cz sin(φ)

z = h - ax sin(S) + az cos(S)
      - bx sin(S+E) + bz cos(S+E)
      - cx sin(φ) + cz cos(φ)
x = r cos(B)
y = r sin(B)
```

这与原源码的平面公式形式等价，但末端偏移改用新图纸值，且这里 z 包含基座高度 h。源码的 `lastZ` 没有加 h，其计算原点高度在肩部；不要直接把该 z 当作本模型的基座 z。初始姿态得到 `(x,y,z)=(0.34616,0,0.34959) m`。

五维 `TaskPose(x,y,z,pitch,roll)` 采用固件角度约定：`pitch=φ−π/2`，`roll=R`。`pitch` 不等同于任意世界系欧拉角，也没有包含 γ。完整工具旋转可通过正解或 `pose_from_task()` 得到。后者默认基座朝向位置的正径向；若位置位于基座反向平面，应显式传入 `base_angle`。`inverse_task()` 会检查正负两个径向平面并按 seed 选解。

## 解析逆运动学和选解

输入完整的 `4×4` 工具目标矩阵和可选五关节 seed。五轴不能满足任意六自由度目标；求解器检查目标旋转是否与位置要求的径向平面兼容，不会静默投影姿态。

1. 对非零 xy 距离，枚举 `B=atan2(y,x)` 和 B+π 对应的正/负径向平面。xy 距离接近零时保持 seed 的 B；若该 B 无法表达目标姿态，返回姿态不兼容。
2. 将目标旋转转入候选基座平面，分解为 `Ry(β) Rz(R)`，检查重构误差。`φ=β−γ`。
3. 从工具位置减去 `L·(sinβ, cosβ)`，得到肩系平面中的腕点 `(rw,zw)`，其中 zw 还减去 h。
4. 定义 `l2=||a||`、`l3=||b||`、`t2=atan2(ax,az)`、`t3=atan2(bx,bz)`，计算：

```text
D = (rw² + zw² - l2² - l3²) / (2 l2 l3)
δ = ±acos(D)
θ2 = atan2(rw,zw) - atan2(l3 sinδ, l2+l3 cosδ)
S = θ2 - t2
E = δ + t2 - t3
W = φ - S - E
```

超出余弦定义域的目标返回不可达；浮点舍入范围内的值才截取到 [-1,1]。枚举全部分支和每个关节的限位内 `2π` 等价表示，去重后返回所有有效解。

因为关节有硬限位，选择距离使用五关节真实坐标差的欧氏范数，不能把跨越 ±π 硬限位当作可直接走的短路。用上一帧关节角作为 seed 可以减少构型跳变。未提供 seed 使用 YAML 中的 `home`；节点不会暗中把最新 `/joint_states` 代入 seed。存在几何解但全部超限时单独报告 `joint_limit`。

参数可替换，但解析结构要求三段偏移位于局部 XZ 平面、三根俯仰轴平行、末端滚转轴按上述约定放置。基座轴线处以及两连杆共线处属于奇异情况；本次提供位置逆解和雅可比，不提供奇异区的速度控制器。返回成功仅代表运动学和关节限位可行，不包含碰撞检查或轨迹动态可行性。

## 雅可比

`jacobian(q)` 返回基座坐标系下的 `6×5` 几何雅可比：前三行为工具点线速度，后三行为角速度，满足 `[v;ω]=J(q) q_dot`。对第 i 个转动关节，用基座系轴向 `ui`、轴心 `oi` 和工具点 p 构造：

```text
Jv_i = ui × (p - oi)
Jw_i = ui
```

末端参考点位于第五轴上，因此第五列的线速度为零；滚转只改变工具方向。测试用中心有限差分验证位置导数和 `R_dot Rᵀ` 的角速度。

## Python 和 ROS 用法

先在 WSL 工程中构建并加载环境：

```bash
cd "/home/ubuntu/Disturbance Rejection Control Algorithm Design"
source /opt/ros/humble/setup.bash
source scripts/env.sh
source "$GZ_CONTROL_WS/install/setup.bash"
colcon build --symlink-install --packages-select dragonfly_arm
source install/setup.bash
```

纯 Python 库安装名为 `dragonfly_arm_kinematics`，不依赖 ROS 节点运行。ROS 服务生成包保留为 `dragonfly_arm`，以避免 Humble 中接口生成和纯 Python 安装目标重名。

```python
from pathlib import Path
from dragonfly_arm_kinematics import ArmParameters, RoArmM3Kinematics

parameters = ArmParameters.from_yaml(Path(
    "src/dragonfly_arm/config/model_dimensions.yaml"))
arm = RoArmM3Kinematics(parameters)
target = arm.forward(parameters.home)
task = arm.task_pose(parameters.home)
jacobian = arm.jacobian(parameters.home)
result = arm.inverse(target, seed=parameters.home)
if result.success:
    print(result.selected.positions, result.selected.position_error)
    print([candidate.positions for candidate in result.solutions])
```

`forward()` 和 `jacobian()` 接受五个有限关节角，可计算限位外位置用于分析；需要执行前限制时调用 `within_limits()` 或使用受限逆解。`inverse()`、`inverse_task()` 返回 `IKResult`，包含 `status`、`message`、`solutions`、`selected`；失败时 `selected=None`。

`ros2 launch dragonfly_arm gazebo.launch.py` 会自动启动运动学节点；单独启动计算接口使用：

```bash
ros2 run dragonfly_arm kinematics_node
```

| 接口 | 类型与约定 |
|---|---|
| `/joint_states` | 订阅 `sensor_msgs/JointState`，按名称提取各侧五关节，不依赖消息内的顺序 |
| `/left_arm/tool_pose` | `geometry_msgs/PoseStamped`，`frame_id=left_base_link` |
| `/right_arm/tool_pose` | 同上，`frame_id=right_base_link` |
| `/left_arm/solve_ik` | `dragonfly_arm/srv/SolveIK` |
| `/right_arm/solve_ik` | 同上 |

位姿时间戳沿用输入关节状态时间戳。缺少某侧完整五关节或含非有限值时，不发布该侧位姿；不会把缺失关节默认为零。服务只返回解，不发送 action、trajectory、串口或执行器命令。夹爪状态由未来遥操作接口独立携带。

请求字段为 `target`（PoseStamped）、`seed`（固定 5 个角）、`use_seed`（bool）。目标 `frame_id` 必须严格匹配对应基座，不自动转换 world 或另一侧坐标系。姿态四元数顺序为 xyzw，接受有限非零四元数并归一化；全零四元数无效。

返回 `success`、`status`、`message`、`joint_positions[5]`、`candidate_count`、`position_error`（m）、`orientation_error`（rad）。仅在 success 为真时使用角度和误差字段。ROS 返回选中解及候选数量，全部候选可从 Python API 读取。

| 状态值 | 名称 | 含义 |
|---:|---|---|
| 0 | STATUS_OK | 成功 |
| 1 | STATUS_INVALID_INPUT | 非有限位置/seed、无效四元数等 |
| 2 | STATUS_INVALID_FRAME | 目标参考系错误 |
| 3 | STATUS_UNREACHABLE | 当前姿态要求下的位置不可达 |
| 4 | STATUS_ORIENTATION_MISMATCH | 目标方向不符合五轴约束 |
| 5 | STATUS_JOINT_LIMIT | 几何解全部违反软件关节限位 |

下面示例对模型折叠位姿求解，工具方向已包括末端安装角 γ：

```python
import rclpy
from rclpy.node import Node
from dragonfly_arm.srv import SolveIK
from dragonfly_arm_kinematics import quaternion_from_matrix

rclpy.init()
node = Node("ik_example")
client = node.create_client(SolveIK, "/left_arm/solve_ik")
if not client.wait_for_service(timeout_sec=5.0):
    node.destroy_node()
    rclpy.shutdown()
    raise RuntimeError("Start the kinematics node first")
request = SolveIK.Request()
request.target.header.frame_id = "left_base_link"
request.target.pose.position.x, request.target.pose.position.y, request.target.pose.position.z = target[:3, 3]
quat = request.target.pose.orientation
quat.x, quat.y, quat.z, quat.w = quaternion_from_matrix(target)
request.seed = list(parameters.home)
request.use_seed = True
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
print(future.result() if future.done() else "IK service timed out")
node.destroy_node()
rclpy.shutdown()
```

此例接续上一个 Python 片段中的 `target`、`parameters`。双臂使用相同模型，右臂只需更换服务名和 frame_id。

## 舵机角度边界与后续标定

本模型的 q 是固件逻辑关节角，不是舵机原始 ticks。`RoArm-M3_module.h` 中底座与滚转反馈方向相反；肩主动/从动舵机方向相反。反馈近似换算如下，`u` 为对应舵机位置：

| 逻辑关节 | ID | 源码反馈换算 |
|---|---|---|
| B | 11 | `π − u·2π/4096` |
| S | 12（主动） | `u·2π/4096 − π` |
| E | 14 | `u·2π/4096 − π/2` |
| W | 15 | `u·2π/4096 − π` |
| R | 16 | `π − u·2π/4096` |
| 夹爪 | 17 | 独立开合值，不进入 q |

源码指令中位为 2047 ticks，反馈公式中位为 2048 ticks，存在一编码器步的约定差；肘部指令限制 1024～3071 ticks 对应 [0,π−2π/4096]。本次按计划使用理想弧度限位 [0,π]，不把它宣称为实物校准范围。接入实物时应统一零位、符号、编码器端点和主从安装变换后再下发。地面机械臂同构同尺寸时，可优先以校准后的 q 对应控制；本库提供末端几何与逆解，并不替代通信、限速、滤波、碰撞处理及急停层。

## 验证命令

```bash
colcon test --packages-select dragonfly_arm --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=72 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=dragonfly_roarm_validation \
  bash src/dragonfly_arm/test/smoke_test.sh log/roarm_validation
```

测试覆盖源参数与已知初始点、随机全范围 FK/IK 回算和全部候选复核、±π 两端与累计角跨界、基座轴线和肘部共线、不可达/限位/姿态错误、雅可比有限差分、独立 URDF 链对照，以及 ROS 消息校验。冒烟测试最后可保存三张实际场景图；不会连接 Arduino 或实物。
