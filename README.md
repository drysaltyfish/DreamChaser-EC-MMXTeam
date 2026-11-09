# Fusai

STM32F407 机器人主控固件：FreeRTOS + STM32 HAL + CMake/Ninja，实现与树莓派上位机之间的
**v1.3 互斥运动协议**（云台与底盘不能同时运动），并预留云台、底盘与遥测接口。

- MCU：STM32F407VETx，168 MHz，512 KB Flash / 128 KB SRAM
- 链路：默认 UART5，115200 8N1，DMA + 空闲中断接收
- 控制周期：云台、底盘各 50 Hz
- 协议：帧格式、命令、状态机与对接说明见 [docs/protocol.md](docs/protocol.md)

## 构建

依赖 arm-none-eabi-gcc、CMake ≥ 3.22、Ninja。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/Fusai.elf`。Release 构建把 preset 换成 `Release`。
当前构建资源占用约为 RAM 32 KB、Flash 47 KB（具体数值以构建输出为准）。

## 目录结构

```text
Core/
  Inc/
    app/         应用层：任务、命令路由、遥测、硬件端口、配置
    motion/      运动层：仲裁、云台、底盘、互斥控制
    protocol/    协议层：常量、编解码、解析状态机、消息结构
    utils/       通用：CRC、单生产者/单消费者环形缓冲
    uart_connection.h
  Src/
    app/         对应 app 层实现
    motion/      对应 motion 层实现
    protocol/    对应 protocol 层实现
    utils/       对应 utils 层实现
    driver/      UART 传输层、电机驱动
    tasks/       FreeRTOS 启动任务
docs/
  protocol.md    协议与上位机对接说明
```

分层依赖方向：`app → motion → protocol/utils`，`app → driver/uart_connection`。
`motion_arbiter` 不依赖 HAL/FreeRTOS，便于单独验证。

## 串口配置

所有链路参数集中在 `Core/Inc/app/app_config.h`。换 UART 时只需要：

1. 在 CubeMX 中配置目标 UART 与对应 DMA（外设→内存、Circular、字节对齐）。
2. 修改 `APP_LINK_UART`：
   ```c
   extern UART_HandleTypeDef huart5;
   #define APP_LINK_UART (&huart5)
   ```
3. 确认 `APP_LINK_DMA_RX_SIZE` 与 DMA 缓冲区大小一致；接收环形缓冲
   `APP_LINK_RX_RING_SIZE` 必须是 2 的幂。

DMA 尺寸、任务优先级/栈、队列深度、控制周期、超时、云台限位、底盘加速度
都在同一个头文件中。

## 运行概览

上电后 `StartDefaultTask` 调用 `App_Init()` 并自删除，`App_Init()` 创建：

| 任务 | 优先级 | 栈（words） | 职责 |
| --- | ---: | ---: | --- |
| LinkTask | 5 | 320 | UART 接收、协议解析、急停帧优先处理 |
| CommandTask | 4 | 384 | 命令路由、心跳超时检查 |
| GimbalTask | 3 | 256 | 云台 50 Hz 步进 |
| ChassisTask | 3 | 256 | 底盘 50 Hz 步进 |
| TxTask | 2 | 256 | 发送队列编码与串口发送 |
| TelemetryTask | 1 | 256 | STATUS 周期/变化上报 |

上电发送一次 3 字节的 `READY`；`STATUS` 每 1 s 或状态变化时发送。心跳从启动
时刻开始计时，500 ms 内没有收到 `HEARTBEAT` 会进入故障（`ERROR 0x04`），
必须用 `SET_MODE(0)` 清除。

## 安全行为

- `SET_MODE(3)` 在解析层立即急停，不进入命令队列：云台失能、目标位置置为当前位置、
  底盘按最大减速度停车，owner 进入 `ESTOP_FAULT`，只接受 `SET_MODE(0)` 退出。
- 300 ms 没有新的非零速度命令，底盘按正常减速度停车。
- 连续 5 帧 CRC 错误上报一次 `ERROR 0x01`；帧格式错误上报 `ERROR 0x02`。
- 云台运动命令与底盘速度命令共享互斥仲裁：云台运动期间底盘速度命令被拒绝，
  底盘运动/未完全停止期间云台运动命令被拒绝。

## 已知限制

- `GimbalPort` 目前是空实现，需要接入实际伺服/步进驱动。
- `ChassisPort` 默认关闭（`APP_CHASSIS_DRIVE_ENABLED 0`），因为仓库里只定义了
  MotorA；实现 MotorB/C/D 后可打开宏接入 `Vehicle_SetVelocity`。
- 急停中的“激光关闭”未实现：仓库没有激光驱动。
- 云台没有编码器反馈，`actual_pan/actual_tilt` 是模型积分值；到达目标不会额外发送
  异步 ACK，上位机通过 `STATUS` 中的实际角度判断。
- 1.3 不包含标定、零点和 Flash 持久化；云台使用 `APP_GIMBAL_*` 固定限位。
  后续工作见 [docs/protocol.md](docs/protocol.md) 的 TODO 一节。
- 未启用独立看门狗（IWDG）。
- 没有宿主机单元测试环境（开发机无 host gcc/clang），当前以目标板编译验证为主。

## 版本

协议版本 1.3。版本号在 `Core/Inc/protocol/protocol_defs.h` 的 `PROTO_VERSION_MAJOR` /
`PROTO_VERSION_MINOR` 中维护，并通过 `READY` 上报。
