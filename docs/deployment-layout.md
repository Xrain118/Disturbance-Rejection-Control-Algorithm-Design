# 仿真与机载部署边界

更新日期：2026年11月5日。

本仓库使用同一条 Git 历史维护共享算法、机载程序和 Gazebo 仿真，避免仿真与实机分支中的模型、接口和控制算法长期漂移。构建产物彼此隔离；机载构建不查找、编译或安装 Gazebo 包。

## 包与运行位置

| ROS 包 | 内容 | 机载电脑 | WSL 仿真 |
|---|---|---:|---:|
| `dragonfly_control` | 浮动基座耦合动力学、模型前馈、六维 NDOB | 是 | 是 |
| `dragonfly_arm` | 共同几何、惯性参数、URDF、FK/IK | 是 | 是 |
| `dragonfly_interfaces` | 共同消息与服务 | 是 | 是 |
| `dragonfly_serial` | 双 ESP32 串口遥测、记录与回放 | 是 | 可选 |
| `dragonfly_fcu_bridge` | 物理 wrench 到 PX4 归一化目标的适配 | 是 | 是 |
| `dragonfly_onboard` | 不含仿真时间和 Gazebo 的机载启动入口 | 是 | 否 |
| `dragonfly_sim` | Gazebo 世界、启动文件、演示和 C++ 物理插件 | 否 | 是 |
| `dragonfly_flight` | 当前 SITL 任务流程、Gazebo/PX4 运行时和仿真验收 | 否 | 是 |

`dragonfly_control` 是唯一的前馈和 NDOB 算法实现。仿真运行时导入该包，后续机载飞行控制节点也必须导入同一包，不复制算法文件。

## 独立构建

机载电脑构建：

```bash
bash scripts/build_onboard.sh
source install-onboard/setup.bash
ros2 launch dragonfly_onboard onboard.launch.py \
  arm_config:=/absolute/path/to/dual_arm_serial.yaml
```

机载启动默认只启动双臂遥测。`start_fcu_bridge` 默认关闭；只有提供与当前机型一致的 `model_metadata`、确认模式所有权并完成台架验证后才应显式启用。当前仿真 `dragonfly_flight/runtime.py` 仍依赖 Gazebo 真值、仿真时钟和仿真执行器反馈，不能部署到机载电脑。

WSL 仿真构建：

```bash
bash scripts/build_sim.sh
source install-sim/setup.bash
bash src/dragonfly_flight/scripts/sim.sh --help
```

固定机体 Gazebo 演示入口为：

```bash
ros2 launch dragonfly_sim gazebo.launch.py
ros2 launch dragonfly_sim demo.launch.py
```

`build-hardware/install-hardware` 继续保留为仅双臂遥测的最小构建；完整机载部署使用 `build-onboard/install-onboard`。

## Git 使用方式

`main` 同时保存共享、机载和仿真包。结构调整在短期功能分支中评审后合并；实验和部署通过提交哈希或标签锁定。不要建立长期 `hardware` 和 `gazebo` 分支来分别保存同一算法的两个副本。

机载电脑若不希望保留仿真源码，可从同一提交生成 `install-onboard` 安装产物进行部署；也可以使用 sparse-checkout，但这只是检出优化，不改变包之间的版本关系。
