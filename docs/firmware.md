# ESP32 固件修改与部署记录

整理日期：2026年10月18日。

固件工程位于 `C:/Users/LENOVO/Desktop/arm/RoArm-M3_example`，不属于本仓库；本次没有连接设备、烧录或修改 Dragonfly 原工程。

修改前完整备份：`C:/Users/LENOVO/Desktop/arm-backups/before-telemetry-20261018-150542/RoArm-M3_example`。
编译与变更记录目录：`C:/Users/LENOVO/Desktop/arm-telemetry-build`。

## 修改内容

- `telemetry_math.h`：逐舵机系数、电气零偏、带符号原始值解码、跳过错失周期的调度计算。
- `telemetry_types.h` / `telemetry_wire.h`：独立快照及共享可测试的 JSON 序列化，七舵机原始测量与力矩粗估。
- `telemetry_platform.h`：绝对截止时间 UART 读写、事务互斥、回复 ID/长度/校验和检查、遥测与日志有界队列、唯一发送任务。
- `telemetry.h`：周期采集、快照、时间/序号/丢帧统计、T606 配置与确认；目标 50 Hz，失败项 null。
- 原模块：所有舵机控制调用经过事务锁；旧反馈读快照，补全肩从动；ESP-NOW 回调只提交命令，主上下文核对模式/来源并执行；主循环保留旧按需反馈，旧自动流限制 10 Hz；应用串口日志统一按完整行输出。

原库 SCServo 1.0.2 的 Read 检查校验和但不核对回复 ID/长度，因此新增反馈读取函数显式核对，避免迟到包归错舵机。未修改用户安装的 SCServo 库。

`getFeedback` 中模式与力矩使能不在本轮 bulk 寄存器块内，标为未知占位；旧1051仍不具备逐舵机有效性。旧消费者缓存可能用于其原有逻辑，但新版遥测每轮新建快照，绝不复用上轮值充当成功测量。

保留旧运动指令和轨迹语义，上位机新驱动不发送运动。主循环中的长动作不占用整个运动过程的总线锁；但配置命令需要主上下文处理，不是实时急停通道。ESP-NOW 命令队列容量 8，满时丢弃新命令并计数；没有承诺无限缓存或即时执行。

## 编译环境

基线成功编译使用 `esp32:esp32:esp32:PartitionScheme=huge_app`、ESP32 Arduino **3.0.7**。先尝试的 PlatformIO 默认 Arduino 2.x 与原固件 ESP-NOW 回调签名不兼容，失败记录保留，没有据此改变原运动代码或降低核心版本。

| 依赖 | 本次基线实际版本 |
|---|---|
| ArduinoJson | 7.4.2 |
| SCServo | 1.0.2 |
| INA219_WE | 1.4.1 |
| Adafruit SSD1306 | 2.5.17 |
| Adafruit GFX | 1.12.6 |
| Adafruit BusIO | 1.17.4 |
| Preferences / LittleFS / FS / WiFi / Network / Wire / WebServer / SPI | 随 ESP32 3.0.7 |

可复现编译命令（PowerShell；仅 compile，不 upload）：

```powershell
& 'C:/Users/LENOVO/Desktop/arm-telemetry-build/cli/arduino-cli.exe' compile `
  --jobs 8 `
  --config-file 'C:/Users/LENOVO/.arduinoIDE/arduino-cli.yaml' `
  --fqbn 'esp32:esp32:esp32:PartitionScheme=huge_app' `
  --build-path 'C:/Users/LENOVO/Desktop/arm-telemetry-build/modified' `
  'C:/Users/LENOVO/Desktop/arm/RoArm-M3_example'
```

修改版编译结果见 [验证记录](verification-20261018.md)与外部 `modified-arduino.log`。编译目标是本机可复现的 ESP32 Dev Module 配置，是否完全匹配每块实物板仍需核对。

## 本机原生测试

仓库只保存测试和协议向量，通过 include 路径测试外部固件的实际头文件，不上传固件本身：

```bash
FIRMWARE_ROOT=/mnt/c/Users/LENOVO/Desktop/arm/RoArm-M3_example \
ARDUINOJSON_ROOT=/mnt/c/Users/LENOVO/Documents/Arduino/libraries/ArduinoJson \
bash scripts/test_firmware.sh
```

测试包括电流/力矩符号、合法负1、调度跳周期、无效项、共用测试向量和帧长，以及实际 DeadlineServo/BusGuard/队列代码在模拟 UART/RTOS 边界下的超时、并发事务和积压。模拟了长期动作写入/等待、回放式写入、并发命令与采集；这不是在 ESP32 上执行完整轨迹或 Wi-Fi 栈的硬件测试，不证明 FreeRTOS 实机调度、实际舵机响应或动作轨迹正确性。

## 部署前仍需核实

**保留的上电动作**：初始化后仍调用 `moveToRecordedInitialPose()`，发送六轴目标 `[0.124252444, -1.526310884, 2.997398460, -0.150330117, 0.006135923, 3.129320807] rad`，速度参数100、加速度参数8，然后启动自动遥测。它发送目标但不等精确到位；上电会运动，遥测不是唯一行为。

确认板卡/串口电路、左右设备身份、型号、接线、机械方向、位置零位和速度配置后，再安排离桨台架测试。测量七舵机读取成功率、50 Hz 持续频率、扫描耗时、超期/丢帧与温升；分别检查静止、长轨迹、任务回放、ESP-NOW、失联和超时。2 ms / 15 ms 为可调初值，不够时应显式调整预算或降低目标频率，不能伪造有效数据维持频率。

厂家线性粗估未标定，不自动扣“同姿态无负载电流”，不把关节电流直接当作机体外扰。精度、带宽及飞行闭环用途仍需单独论证。
