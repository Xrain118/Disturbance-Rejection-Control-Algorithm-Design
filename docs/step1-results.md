# 第一步基线复现结果（2026-11-05）

**原生 3/3 通过，研究无补偿 0/3 通过。接口实现和软件测试完成，第一步整体验收未通过。**

工程：`E:/subjects/Disturbance Rejection Control Algorithm Design`；WSL 从对应 `/mnt/e/subjects/…` 构建和运行。未修改外部 PX4、桌面参考源码或实机固件。完整复现契约见[控制接口与基线](control-contract.md)。

## 六轮正式记录

| 模式/轮次 | 完整验收 | 悬停位置 RMS（m） | 悬停姿态 RMS（°） | 连续饱和（s） |
|---|---|---:|---:|---:|
| px4_native / 1 | 通过 | 0.07231 | 0.62042 | 0.00000 |
| px4_wrench / 1 | 失败 | 未取得有效片段 | 未取得有效片段 | 未取得有效片段 |
| px4_native / 2 | 通过 | 0.03003 | 0.62771 | 0.00000 |
| px4_wrench / 2 | 失败 | 未取得有效片段 | 未取得有效片段 | 未取得有效片段 |
| px4_native / 3 | 通过 | 0.03480 | 0.57384 | 0.00000 |
| px4_wrench / 3 | 失败 | 未取得有效片段 | 未取得有效片段 | 未取得有效片段 |

批次采用相同模型（仅模式字段不同）、实验 YAML 和 PX4 二进制；六轮控制源码一致，PX4 适配清单哈希全部匹配。每轮独立保存参数、模型、源码差异、构建路径、ROS bag 和 ULog。第一轮后的差异限于文档、测试及版本记录模块，详细清单保存在诊断结果中。实际加载模块均来自 E 盘工程。

已从每轮 ULog 读取初始参数及参数更新，核对62项关键参数（六轮共372项），包括 DF 质量/质心/完整惯量/缩放、分配模式、六电机六伺服映射及 HTE 开关，均与本轮请求配置一致。详见[ULog参数核对](../log/step1_baselines_20261105_v3/parameter_audit.json)。

全程未启用机械臂差量前馈或 NDOB。研究日志的前馈施加量为零，没有 NDOB 日志。现有标称动力学反馈保持不变，未修改增益、控制分配或验收阈值。名义100 Hz计时器并不等于实际执行100 Hz。

## 研究无补偿的失败证据

- px4_wrench 第1轮：Acceptance sequence did not finish in 220 seconds；Controller exited (1); see controller.log
- px4_wrench 第2轮：Acceptance sequence did not finish in 220 seconds；Controller exited (1); see controller.log
- px4_wrench 第3轮：Acceptance sequence did not finish in 220 seconds；Controller exited (1); see controller.log

以下为试验末尾30秒统计，**不是有效悬停指标**。原始研究记录只有 takeoff/predeploy 阶段，没有满足展开条件，因而没有后续悬停、平移和动臂验收片段。

| 研究轮次 | 最后30秒位置 RMS（m） | 姿态 RMS（°） | 日志控制频率（Hz） | wrench计算中位数（ms） |
|---|---:|---:|---:|---:|
| 1 | 0.20116 | 20.38286 | 47.38 | 36.39 |
| 2 | 0.19566 | 20.35193 | 44.81 | 37.48 |
| 3 | 0.20954 | 20.22983 | 43.68 | 38.43 |

控制链证明四个 PX4 原生反馈环均已退出，Offboard 与分配启用，ROS 执行器输入发布者清单为空；接口检查通过。末段分配残差很小，不能据此把姿态偏差解释为分配饱和。日志中的模型预测差量俯仰力矩约 −1.4 N·m，但实际施加补偿为零。

数据支持优先检查“折叠起飞构型相对固定竖直臂标称模型的配平偏差”及“研究控制循环计算开销”两个问题；这仍是排查方向，不是已经完成的因果证明。前馈模块关闭输出后仍计算完整模型差量用于日志，标称控制本身也调用多刚体逆动力学，存在额外开销。此轮没有通过加入积分、前馈或 ESO 来改变对照控制器。

下一步先恢复研究无补偿基线的起飞/展开能力：核对静态配平与姿态反馈对恒定力矩偏差的处理；同时分析和削减不必要计算，验证优化前后标称控制输出等价、周期满足要求。随后按原阈值重跑三轮研究基线，再推进双 ESO。不能把六轮尝试等同于六轮验收通过。

## 软件验证与运行修复

- 8 个 ROS 包构建通过；48 项 Python 测试和 10 项 PX4 C++ 测试通过。跨语言测试使用实际 Python 桥生成非零质心/完整惯量夹具，再由外部编译树中的实际分配器验证。
- 修复分配残差回到机体原点时缺失的交叉力矩；统一参数缩放来源；运行和离线验收共用模式规则，缺失关键片段不能通过。
- 修复 PX4 启动脚本不兼容含空格工作目录的问题：临时工作目录无空格，退出后归档到 E 盘；物理插件从迁移后的 `dragonfly_sim/lib` 加载。
- `log/step1_baselines_20261105` 和 `_v2` 为此前启动/插件排障记录，保留但不计入正式飞行成绩。

## 原始结果入口

远端提交包含下列汇总、诊断、测试报告和各轮模型/参数/模式检查记录。完整 CSV、ROS bag、ULog 与源码快照保留在本机 E 盘同名实验目录，大型原始日志未上传。

- [六轮汇总](../log/step1_baselines_20261105_v3/summary.json)、[诊断与版本一致性](../log/step1_baselines_20261105_v3/diagnostics.json)、[实际模块来源](../log/step1_baselines_20261105_v3/loaded_modules.json)。
- 同目录每轮子目录保存 `flight.csv`、`metrics.json`、链路证明和 `provenance.json`；研究轮次另含 `research.csv` 与失败原因。
- [初始Python测试](../log/step1_tests.xml)、[离线验收反例测试](../log/step1_verify_tests.xml)、[模型及运行入口复验](../log/step1_final_model_run_tests.xml)、[PX4 C++测试](../log/step1_px4_tests/tests.xml)。Python合计48项为去重计数，不把重复运行累加成新测试。
- [最终构建日志](../log/step1_build.log)。
