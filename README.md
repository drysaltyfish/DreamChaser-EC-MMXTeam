# Fusai

STM32F407 机器人主控固件：FreeRTOS + STM32 HAL + CMake/Ninja，实现 **v2.0 双链路
互斥运动协议**。树莓派通过 USART3 只控制云台，BLE 4.0 蓝牙模块（`hc05V2.3_le`）
通过 UART5 只控制四轮全向底盘；
两条链路共用同一个运动仲裁器，云台和底盘不能同时运动。仓库同时提供 Python SDK
（[sdk/](sdk/README.md)）、Windows 蓝牙底盘测试台（[sdk/tests/](sdk/tests/README.md)）
和完整的协议说明（[docs/protocol.md](docs/protocol.md)）。

- MCU：STM32F407VETx，167.936 MHz（12.288 MHz HSE），512 KB Flash / 128 KB SRAM
- 云台链路：USART3（PB10 TX / PB11 RX），115200 8N1，DMA Circular
- 底盘链路：UART5（PC12 TX / PD2 RX），115200 8N1，DMA Circular
- 控制周期：云台、底盘各 50 Hz；四轮电机编码器闭环 100 Hz（10 ms）
- 底盘执行：TIM8 四路 PWM（约 20 kHz）+ IN2 方向，TIM1–TIM4 正交编码器
- 协议：帧格式、命令、状态机与对接说明见 [docs/protocol.md](docs/protocol.md)

## 功能概览

- **帧层**：`AA 55 | LEN | CMD | DATA | CRC16 | 0D`，CRC-16/MODBUS，0xAA 转义，
  支持 `AA 55` 重同步。
- **双链路**：云台命令只接受来自 PI 链路，底盘命令只接受来自 BT 链路；走错链路回
  `ERROR(WRONG_LINK)`，不会执行。
- **互斥仲裁**：owner 取值为 NONE / GIMBAL / CHASSIS / ESTOP_FAULT；云台运动期间
  底盘非零速度被拒绝，底盘运动或未完全停止期间云台命令被拒绝。
- **四轮底盘**：`SET_VELOCITY` 的 vx/vy/omega 经斜坡与准入后交给 `ChassisPort`，
  `Vehicle_SetVelocity` 完成全向轮运动学解算和四轮统一限幅，输出 A/B/C/D 的
  RPM 目标；每轮 10 ms 编码器测速 + PID 闭环，经 TIM8 PWM 与 IN2 方向脚驱动。
- **安全**：`SET_MODE(3)` 在解析层立即急停，不进入命令队列；速度 300 ms 超时；
  每条链路独立心跳（500 ms），PI 掉线只释放云台，BT 掉线按最大减速度停车。
- **遥测**：PI 收 `GIMBAL_STATUS`（1 Hz/变化），BT 收 `CHASSIS_STATUS`
  （10 Hz/变化）和 `MOTOR_STATUS`（10 Hz，四轮 `present_mask = 0x0F`）。
- **可配置**：串口绑定、DMA/环形缓冲、任务优先级/栈、队列深度、控制周期、超时、
  云台限位和底盘加速度集中在 `Core/Inc/app/app_config.h`；轮距/轮径、编码器方向、
  PID 增益和电机上限在 `vehicle.h` / `motor.c`。

## 构建

依赖 arm-none-eabi-gcc、CMake ≥ 3.22、Ninja。

~~~bash
cmake --preset Debug
cmake --build --preset Debug
~~~

产物：`build/Debug/Fusai.elf`。Release 构建把 preset 换成 `Release`。
最近一次 Debug 构建（2026-11-14，arm-none-eabi-gcc 13.3.1）占用 RAM 37840 B、
Flash 55752 B（分别约 28.9% / 10.6%，具体数值以构建输出为准）。

## 树莓派 Python SDK

`sdk/` 是用 uv 管理的 Python 包 `fusai-sdk`，实现同一套 2.0 帧、命令、序号与
错误语义。SDK 是独立仓库（[DreamChaser-EC-SDK-MMXTeam](https://github.com/drysaltyfish/DreamChaser-EC-SDK-MMXTeam)），
在本仓库中以 git submodule 固定版本；克隆父仓库后先执行：

~~~bash
git submodule update --init sdk
~~~

客户端按链路角色工作：

~~~python
from fusai_sdk import BleTransport, FusaiClient, Link

gimbal = FusaiClient(port="/dev/ttyAMA0", link=Link.PI)   # 云台（树莓派串口）
chassis = FusaiClient(
    transport=BleTransport("F0:72:D1:80:FB:EB"), link=Link.BT
)                                                         # BLE 底盘
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
（可暂停、过滤心跳、保存 TSV）。连接栏是“BLE 设备”下拉框，显示附近及 Windows
已记录的 BLE 设备，名称后标注“已连接 / 串口”（例如
`原神？启动！（已连接 / 串口） [F0:72:D1:80:FB:EB]`），不会混入普通串口。已连接且
带 FFE0/Nordic 串口服务的设备会排在最前并自动选中。点“扫描”会搜索约 10 秒；也可以
直接输入地址。模块是 `hc05V2.3_le`（BLE 4.0 透传，没有 SPP），Windows 不需要先配对，
选中后直接“连接”。

~~~bash
cd sdk
uv sync
uv run python tests/gui.py --port F0:72:D1:80:FB:EB
~~~

不打开 BLE、只检查界面：`uv run python tests/gui.py --self-test`。安全说明和界面
细节见 [sdk/tests/README.md](sdk/tests/README.md)。

## 目录结构

~~~text
Core/
  Inc/
    app/         应用层：任务、命令路由、遥测、硬件端口、双链路表、配置
    motion/      运动层：仲裁、云台、底盘、互斥控制
    protocol/    协议层：常量、编解码、解析状态机、消息结构
    utils/       通用：CRC、单生产者/单消费者环形缓冲
    motor.h      四轮电机、编码器、PID 接口
    vehicle.h    全向轮几何常量与运动学接口
    pid.h        PID 接口
    uart_connection.h  串口传输层接口
  Src/
    app/         对应 app 层实现
    control/     PID 实现、四轮全向轮运动学
    motion/      对应 motion 层实现
    protocol/    对应 protocol 层实现
    utils/       对应 utils 层实现
    driver/      UART 传输层、电机驱动（PWM/方向/编码器/PID 任务）
    tasks/       FreeRTOS 启动任务
    main.c       CubeMX 外设初始化、时钟、TIM10 10 ms 中断
docs/
  protocol.md    协议与上位机对接说明
sdk/             上位机 Python SDK（独立仓库，git submodule）
  tests/gui.py   Windows Tkinter 蓝牙底盘测试台
~~~

分层依赖方向：`app → motion → protocol/utils`，`app → control`（运动学与 PID），
`app → driver/uart_connection`；`motion_arbiter` 不依赖 HAL/FreeRTOS，便于单独验证。

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

## 电机与底盘驱动

`ChassisPort` 把运动层输出的归一化 `vx/vy/omega` 接到四轮底盘：
`MotionControl_TickChassis` 每 20 ms 给出斜坡后的三轴输出，
`ChassisPort_Apply` 调用 `Vehicle_SetVelocity` 做全向轮运动学解算和四轮统一
限幅，写入 A/B/C/D 的 RPM 目标；`Motor_Init` 创建的 `MotorCtrlTask` 每 10 ms
采样编码器、滤波并跑 PID，最后写 TIM8 PWM。

### 硬件映射

| 轮位 | 位置 | PWM（IN1） | 方向（IN2） | 编码器 | 计数取反 |
| --- | --- | --- | --- | --- | --- |
| A | 右后（RR） | TIM8_CH4 / PC9 | PA9 | TIM1（PE9/PE11） | 是 |
| B | 右前（RF） | TIM8_CH3 / PC8 | PA8 | TIM2（PA5/PB3） | 是 |
| C | 左前（LF） | TIM8_CH2 / PC7 | PD15 | TIM3（PA6/PA7） | 否 |
| D | 左后（LR） | TIM8_CH1 / PC6 | PD14 | TIM4（PD12/PD13） | 是 |

- 电机为 11 线编码器 + 90:1 减速箱，TI12 模式对两相上下沿都计数，因此
  `CPR = 11 × 90 × 4 = 3960`；每个 10 ms 周期的计数差换算成 RPM。
- TIM8 计数时钟 167.936 MHz、ARR 8400，PWM 约 20 kHz；IN2 与 PWM 组合决定
  正反转，非零输出至少保留 100/8400 的比较值。
- TIM10 每 10 ms 触发中断，`Motor_10ms_Tick()` 在中断中读走四路计数并用任务
  通知唤醒 `MotorCtrlTask`（优先级 4、栈 512 words）；任务中完成 RPM 换算、
  一阶低通、PID（Kp 0.75 / Ki 8.5 / Kd 0）并写回 PWM，输出限幅 ±100 RPM。
- 运动学常量在 `Core/Inc/vehicle.h`：轮半径 45 mm，`LX = 135.71 mm`、
  `LY = 155.0 mm`；归一化三轴映射到约 ±0.47 m/s 平移和 ±1.62 rad/s 旋转，
  某轮超限时四轮等比例缩放。
- 电机/编码器参数（`MotorX_Handle` 的 PWM 通道、IN2 引脚、编码器定时器、
  计数取反标志、PID 增益）集中在 `Core/Src/driver/motor.c`，轮位映射与左右轮
  符号在 `Core/Src/control/vehicle.c`；换车或改接线后先核对这两处以及实车转向。

## 运行概览

上电后 `StartDefaultTask` 调用 `App_Init()` 并自删除。`App_Init()` 初始化运动
控制、`GimbalPort`/`ChassisPort`（后者经 `Motor_Init()` 创建电机任务）、两个
`UartConnection`、发送队列和命令队列，然后创建：

| 任务 | 优先级 | 栈（words） | 职责 |
| --- | ---: | ---: | --- |
| link_pi / link_bt | 5 | 320 | 各自的 UART 接收、协议解析、心跳直通、急停优先处理 |
| command | 4 | 384 | 共享命令队列分发、心跳超时检查 |
| gimbal | 3 | 256 | 云台 50 Hz 步进 |
| chassis | 3 | 256 | 底盘 50 Hz 步进、速度超时、减速 |
| tx_pi / tx_bt | 2 | 256 | 各自的发送队列编码与串口发送 |
| telemetry | 1 | 256 | 两链路 READY/状态、10 Hz 电机遥测 |
| motor | 4 | 512 | 10 ms 四轮编码器采样、滤波与 PID，由 `Motor_Init()` 创建，TIM10 中断唤醒 |

上电后两条链路各发送一次 3 字节 `READY`：PI 的 flags 为 `HAS_GIMBAL`，BT 的
flags 为 `HAS_CHASSIS | HAS_MOTOR_TELEMETRY`。客户端晚连接时，固件在该链路收到
首个 `HEARTBEAT` 时补发一次 `READY`。两条链路的心跳分别计时；从未收到过心跳的
链路不会超时，因此只接树莓派或只接蓝牙都能独立工作。

## 安全行为

- `SET_MODE(3)` 在解析层立即急停，不进入命令队列：云台失能、目标位置置为当前
  位置、底盘四轮按最大减速度停车，owner 进入 `ESTOP_FAULT`，只接受
  `SET_MODE(0)` 退出。
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
- 底盘的轮径、轮距、轮位映射、编码器方向和左右轮符号都是实车标定常量
  （`vehicle.h`、`motor.c` 与 `vehicle.c`），换车或改接线后必须重新核对；
  底盘只有四轮编码器测速，没有里程计、IMU 或电流反馈。
- 电机没有电流采样和堵转/过热保护；速度上限由 `MOTOR_MAX_RPM`（100 RPM，约
  0.47 m/s）限制。
- 急停中的“激光关闭”未实现：仓库没有激光驱动。
- 云台到达目标不会额外发送异步 ACK，上位机通过 `GIMBAL_STATUS` 中的实际角度判断。
- 2.0 不包含标定、零点和 Flash 持久化；云台使用 `APP_GIMBAL_*` 固定限位。
  后续工作见 [docs/protocol.md](docs/protocol.md) 的 TODO 一节。
- 未启用独立看门狗（IWDG）。
- 没有宿主机单元测试环境（开发机无 host gcc/clang），当前以目标板编译和 SDK 单测
  验证为主。

## 版本

协议版本 2.0。版本号在 `Core/Inc/protocol/protocol_defs.h` 的
`PROTO_VERSION_MAJOR` / `PROTO_VERSION_MINOR` 中维护，并通过 `READY` 上报。
