# Fusai

STM32F407 机器人主控固件：FreeRTOS + STM32 HAL + CMake/Ninja，实现 **v2.0 双链路
互斥运动协议**。树莓派通过 USART3 只控制云台，HC-05 蓝牙模块通过 UART5 只控制底盘；
两条链路共用同一个运动仲裁器，云台和底盘不能同时运动。仓库同时提供 Python SDK
（[sdk/](sdk/README.md)）、Windows 蓝牙底盘测试台（[sdk/tests/](sdk/tests/README.md)）
和完整的协议说明（[docs/protocol.md](docs/protocol.md)）。

- MCU：STM32F407VETx，168 MHz，512 KB Flash / 128 KB SRAM
- 云台链路：USART3（PB10 TX / PB11 RX），115200 8N1，DMA Circular + IDLE
- 底盘链路：UART5（PC12 TX / PD2 RX），115200 8N1，DMA Circular + IDLE
- 控制周期：云台、底盘各 50 Hz
- 协议：帧格式、命令、状态机与对接说明见 [docs/protocol.md](docs/protocol.md)

## 功能概览

- **帧层**：`AA 55 | LEN | CMD | DATA | CRC16 | 0D`，CRC-16/MODBUS，0xAA 转义，
  支持 `AA 55` 重同步。
- **双链路**：云台命令只接受来自 PI 链路，底盘命令只接受来自 BT 链路；走错链路回
  `ERROR(WRONG_LINK)`，不会执行。
- **互斥仲裁**：owner 取值为 NONE / GIMBAL / CHASSIS / ESTOP_FAULT；云台运动期间
  底盘非零速度被拒绝，底盘运动或未完全停止期间云台命令被拒绝。
- **安全**：`SET_MODE(3)` 在解析层立即急停，不进入命令队列；速度 300 ms 超时；
  每条链路独立心跳（500 ms），PI 掉线只释放云台，BT 掉线按最大减速度停车。
- **遥测**：PI 收 `GIMBAL_STATUS`（1 Hz/变化），BT 收 `CHASSIS_STATUS`
  （10 Hz/变化）和 `MOTOR_STATUS`（10 Hz）。
- **可配置**：串口绑定、DMA/环形缓冲、任务优先级/栈、队列深度、控制周期、超时、
  云台限位和底盘加速度集中在 `Core/Inc/app/app_config.h`。

## 构建

依赖 arm-none-eabi-gcc、CMake ≥ 3.22、Ninja。

~~~bash
cmake --preset Debug
cmake --build --preset Debug
~~~

产物：`build/Debug/Fusai.elf`。Release 构建把 preset 换成 `Release`。
最近一次 Debug 构建占用 RAM 37360 B、Flash 53348 B（具体数值以构建输出为准）。

## 树莓派 Python SDK

`sdk/` 是用 uv 管理的 Python 包 `fusai-sdk`，实现同一套 2.0 帧、命令、序号与
错误语义。客户端按链路角色工作：

~~~python
from fusai_sdk import FusaiClient, Link

gimbal = FusaiClient(port="/dev/ttyAMA0", link=Link.PI)   # 云台
chassis = FusaiClient(port="COM5", link=Link.BT)          # 蓝牙底盘
~~~

在错误链路上调用云台/底盘方法会抛出 `WrongLinkError`，不会发出帧。两条链路各有
独立的 `GIMBAL_STATUS` / `CHASSIS_STATUS` 回调与等待方法。SDK 还保留十字路点、
最小二乘雅可比估计等标定辅助工具，但固件当前不提供标定命令，也不写 Flash。

~~~bash
cd sdk
uv sync
uv run pytest
~~~

用法、心跳/序号恢复规则与示例见 [sdk/README.md](sdk/README.md)。

## Windows 测试图形界面

`sdk/tests/gui.py` 是 Tkinter 蓝牙底盘测试台，固定使用 `Link.BT`，提供
`vx` / `vy` / `wz` 滑块、电机实时转速面板、模式/安全按钮，以及逐帧收发日志
（可暂停、过滤心跳、保存 TSV）。连接栏是“蓝牙设备”下拉框，只列出蓝牙串口，
例如 `HC-05 (COM5)`，不会混入普通 USB 串口。

~~~bash
cd sdk
uv sync
uv run python tests/gui.py --port COM5 --baud 115200
~~~

不打开蓝牙串口、只检查界面：`uv run python tests/gui.py --self-test`。安全说明和界面
细节见 [sdk/tests/README.md](sdk/tests/README.md)。

## 目录结构

~~~text
Core/
  Inc/
    app/         应用层：任务、命令路由、遥测、硬件端口、双链路表、配置
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
sdk/             上位机 Python SDK（uv 管理）
  tests/gui.py   Windows Tkinter 蓝牙底盘测试台
~~~

分层依赖方向：`app → motion → protocol/utils`，`app → driver/uart_connection`。
`motion_arbiter` 不依赖 HAL/FreeRTOS，便于单独验证。

## 串口配置

串口绑定只在 `Core/Inc/app/app_config.h`：

~~~c
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

#define APP_PI_UART (&huart3)
#define APP_BT_UART (&huart5)
~~~

换 UART 时：

1. 在 CubeMX（`Fusai.ioc`）中配置目标 UART 和对应 DMA（外设→内存、Circular、
   字节对齐），重新生成代码。
2. 修改 `APP_PI_UART` 或 `APP_BT_UART` 以及对应的 `extern`。
3. 确认 `APP_LINK_DMA_RX_SIZE` 与 DMA 缓冲一致；`APP_LINK_RX_RING_SIZE` 必须是
   2 的幂。

DMA 尺寸、任务优先级/栈、队列深度、控制周期、超时、云台限位、底盘加速度都在同一个
头文件中。两条链路的 UART 传输层是两个独立实例，HAL 回调按 `huart` 指针分发。

## 运行概览

上电后 `StartDefaultTask` 调用 `App_Init()` 并自删除。`App_Init()` 初始化运动
控制、两个 `UartConnection`、发送队列和命令队列，然后创建：

| 任务 | 优先级 | 栈（words） | 职责 |
| --- | ---: | ---: | --- |
| link_pi / link_bt | 5 | 320 | 各自的 UART 接收、协议解析、心跳直通、急停优先处理 |
| command | 4 | 384 | 共享命令队列分发、心跳超时检查 |
| gimbal | 3 | 256 | 云台 50 Hz 步进 |
| chassis | 3 | 256 | 底盘 50 Hz 步进、速度超时、减速 |
| tx_pi / tx_bt | 2 | 256 | 各自的发送队列编码与串口发送 |
| telemetry | 1 | 256 | 两链路 READY/状态、10 Hz 电机遥测 |
| motor（既有） | 4 | 256 | 10 ms 电机 PID 与转速遥测，由 `Motor_Init()` 创建 |

上电后两条链路各发送一次 3 字节 `READY`：PI 的 flags 为 `HAS_GIMBAL`，BT 的
flags 为 `HAS_CHASSIS | HAS_MOTOR_TELEMETRY`。客户端晚连接时，固件在该链路收到
首个 `HEARTBEAT` 时补发一次 `READY`。两条链路的心跳分别计时；从未收到过心跳的
链路不会超时，因此只接树莓派或只接蓝牙都能独立工作。

## 安全行为

- `SET_MODE(3)` 在解析层立即急停，不进入命令队列：云台失能、目标位置置为当前
  位置、底盘按最大减速度停车，owner 进入 `ESTOP_FAULT`，只接受 `SET_MODE(0)` 退出。
- `SET_MODE(0)` 按链路释放：PI 只释放云台，BT 只释放底盘；急停锁存时任一链路都可
  全局解除。
- 300 ms 没有新的非零速度命令，底盘按正常减速度停车。
- PI 心跳超时只释放 `GIMBAL`、禁用云台，不影响底盘；BT 心跳超时让底盘按最大减速度
  停车，停稳后释放 `CHASSIS`，不影响云台。
- 连续 5 帧 CRC 错误上报一次 `ERROR 0x01`；帧格式错误上报 `ERROR 0x02`。
- 云台运动命令与底盘速度命令共享互斥仲裁：云台运动期间底盘速度命令被拒绝，
  底盘运动/未完全停止期间云台运动命令被拒绝。

## 已知限制

- `GimbalPort` 目前是空实现，需要接入实际伺服/步进驱动；`actual_pan/actual_tilt`
  是模型积分值，不是编码器反馈。
- `ChassisPort` 目前只驱动 MotorA（`target_rpm = vx * 110`，10 ms PID 闭环）；
  `vy` 和 `wz` 会被协议接受但还没有执行器，MotorB/C/D 和
  `Vehicle_SetVelocity` 尚未实现。
- 急停中的“激光关闭”未实现：仓库没有激光驱动。
- 云台到达目标不会额外发送异步 ACK，上位机通过 `GIMBAL_STATUS` 中的实际角度判断。
  MotorA 有编码器反馈和 PID 闭环；B/C/D 没有反馈。
- `MOTOR_STATUS` 中只有 A 的 mask 位置位，B/C/D 报 0。
- 2.0 不包含标定、零点和 Flash 持久化；云台使用 `APP_GIMBAL_*` 固定限位。
  后续工作见 [docs/protocol.md](docs/protocol.md) 的 TODO 一节。
- 未启用独立看门狗（IWDG）。
- 没有宿主机单元测试环境（开发机无 host gcc/clang），当前以目标板编译和 SDK 单测
  验证为主。

## 版本

协议版本 2.0。版本号在 `Core/Inc/protocol/protocol_defs.h` 的
`PROTO_VERSION_MAJOR` / `PROTO_VERSION_MINOR` 中维护，并通过 `READY` 上报。
