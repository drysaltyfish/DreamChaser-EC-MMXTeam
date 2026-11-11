# Fusai v2.0 双链路互斥运动协议

本文描述 STM32 主控与两类上位机之间的串口协议，以及固件中的互斥运动仲裁、
云台/底盘模型和对接建议。协议版本为 **2.0**，版本号由 READY 帧上报。

2.0 把原来的单条串口拆成两条物理链路，但仍共用同一套帧格式和一个中央运动仲裁器：

| 链路 | 物理接口 | 对端 | 拥有的功能 |
| --- | --- | --- | --- |
| PI | USART3，PB10 TX / PB11 RX | 树莓派 | 云台运动、云台状态、心跳 |
| BT | UART5，PC12 TX / PD2 RX | BLE 4.0 透传（`hc05V2.3_le`，GATT FFE0/FFE1） | 底盘速度、底盘/电机遥测、心跳 |

两条链路都能发送 `SET_MODE` 和 `HEARTBEAT`，但运动命令集互不重叠：
云台命令只接受来自 PI，底盘命令只接受来自 BT，走错链路回 `ERROR(WRONG_LINK)`。
互斥关系（同一时刻只有一个执行器拥有运动权）仍然由 STM32 统一维护。

- 主控：STM32F407VETx，FreeRTOS，控制周期 20 ms
- 两条链路均为 115200 8N1，无硬件流控
- 字节序：所有多字节整数均为小端（little-endian）
- 角度单位：0.01 度（LSB = 0.01°）
- 树莓派侧 Python 实现见 [sdk/README.md](../sdk/README.md)
- Windows 蓝牙底盘测试台见 [sdk/tests/README.md](../sdk/tests/README.md)

## 1. 物理层

| 项目 | PI 链路 | BT 链路 |
| --- | --- | --- |
| 外设 | USART3 | UART5 |
| TX / RX | PB10 / PB11 | PC12 / PD2 |
| 波特率 | 115200 | 115200 |
| 数据位 / 校验 / 停止位 | 8 / 无 / 1 | 8 / 无 / 1 |
| 流控 | 无 | 无 |
| RX DMA | DMA1 Stream1 Channel4，Circular | DMA1 Stream0 Channel4，Circular |
| DMA 优先级 | LOW | MEDIUM |
| UART / DMA 中断优先级 | 5 / 5 | 5 / 5 |
| 接收方式 | DMA Circular + UART IDLE | DMA Circular + UART IDLE |

两条链路各自维护 DMA 缓冲和软件环形缓冲，参数集中在
`Core/Inc/app/app_config.h`：

- `APP_PI_UART` / `APP_BT_UART`：唯二的串口绑定点。
- `APP_LINK_DMA_RX_SIZE`（256）：DMA 接收缓冲。
- `APP_LINK_RX_RING_SIZE`（1024）：软件环形缓冲，必须是 2 的幂。
- `APP_LINK_TX_TIMEOUT_MS`（50）：阻塞发送超时。

换串口时只改 `APP_*_UART` 和对应的 `extern UART_HandleTypeDef`，再在 CubeMX
（`Fusai.ioc`）里配置引脚、DMA 和中断；应用层不直接引用任何 UART 句柄。
BT 链路的模块对外是 BLE 4.0 GATT 透传（`AT+VERSION` 返回 `hc05V2.3_le`），
不是经典蓝牙 SPP，也不会生成 Windows COM 口。主机侧连接 vendor 服务
`0000ffe0-...` 下的 `0000ffe1-...`（该特征同时支持 write 和 notify），
把帧原样写入；BLE 的分包/重组由模块完成，对 STM32 仍然只是一个 UART，不改写、
不分包、不加校验，因此上面的帧可以原样通过。蓝牙链路必须使用 115200；9600 的带宽
不足以承载 10 Hz 遥测和 20–50 Hz 命令。模块被其他主机连接或处于 AT 模式时不会广播，
主机可以先按地址直连，或直接输入已知地址。

## 2. 帧格式

两条链路使用完全相同的帧格式。

### 2.1 字段

| 偏移 | 字段 | 长度 | 说明 |
| ---: | --- | ---: | --- |
| 0 | SOF1 | 1 | 固定 0xAA |
| 1 | SOF2 | 1 | 固定 0x55 |
| 2 | LEN | 1 | CMD + DATA 的字节数，范围 1..64 |
| 3 | CMD | 1 | 命令码 |
| 4 | DATA | LEN-1 | 命令载荷，最多 63 字节 |
| 3+LEN | CRC_L | 1 | CRC16 低字节 |
| 4+LEN | CRC_H | 1 | CRC16 高字节 |
| 5+LEN | EOF | 1 | 固定 0x0D |

未转义帧总长为 LEN + 6。LEN=1 表示只有 CMD、没有 DATA（例如 HEARTBEAT）。
LEN 最大 64，因此 DATA 最大 63。转义后的最坏线长为 137 字节。

### 2.2 CRC

CRC 使用 CRC-16/MODBUS：

- 多项式 0xA001（反向表示），初值 0xFFFF
- 结果不异或
- 覆盖范围是未转义的 LEN、CMD、DATA，不包括 SOF、CRC 本身和 EOF
- 线上按低字节在前、高字节在后发送

参考实现位于 `Core/Src/utils/crc.c` 的 `Crc16Modbus()`。

### 2.3 转义与重同步

SOF（0xAA 0x55）和 EOF（0x0D）不转义。LEN、CMD、DATA、CRC 中的每个 0xAA
都替换为两个字节：

~~~text
0xAA  ->  0xAA 0x00
~~~

接收端规则：

- 数据区中的 0xAA 0x00 还原为 0xAA。
- 数据区中出现 0xAA 0x55 表示新的 SOF：立即丢弃当前帧，从新帧的 LEN 状态重新开始。
- 数据区中出现 0xAA 后跟其它字节：视为非法转义，当前帧丢弃，回到等待 SOF。
- 0x0D 只有出现在帧尾位置才作为 EOF；数据中的 0x0D 不转义，由 LEN 决定边界。
- 一帧 CRC 错、LEN 非法（0 或大于 64）、EOF 非法，都会丢弃当前帧并计一次格式错误。

### 2.4 示例

HEARTBEAT（无数据）：

~~~text
AA 55 01 10 01 EC 0D
~~~

- LEN=0x01，CMD=0x10
- CRC 覆盖 01 10，结果为 0xEC01，低字节 0x01 在前

SET_MODE(1)（云台跟踪，只能从 PI 链路发送）：

~~~text
AA 55 02 04 01 13 00 0D
~~~

SET_MODE(3)（急停，两条链路都可发送）：

~~~text
AA 55 02 04 03 92 C1 0D
~~~

SET_DELTA(seq=1, dpan=16, dtilt=32)：

~~~text
AA 55 06 02 01 10 00 20 00 5C 22 0D
~~~

SET_TARGET(seq=2, coord=2, pan=16, tilt=32)：

~~~text
AA 55 07 01 02 02 10 00 20 00 74 0F 0D
~~~

包含 0xAA 的 DATA 会在线路上展开，例如 DATA 中的 AA 00 AA 00 变成
AA 00 00 AA 00 00。接收端按上面的规则还原。

## 3. 链路与命令集

### 3.1 链路白名单

固件先区分未定义命令与链路归属：未定义或保留的 CMD 回
`ERROR(UNKNOWN_CMD, cmd)`；已知命令不属于当前链路时回
`ERROR(WRONG_LINK, cmd)`。两种情况都不会执行命令，也不会更新序号。

| CMD | 名称 | PI 链路 | BT 链路 | 说明 |
| ---: | --- | :---: | :---: | --- |
| 0x01 | SET_TARGET | 允许 | 拒绝 | 绝对云台目标 |
| 0x02 | SET_DELTA | 允许 | 拒绝 | 增量云台目标 |
| 0x03 | SET_VELOCITY | 拒绝 | 允许 | 底盘速度 |
| 0x04 | SET_MODE | 允许 | 允许 | mode 0/3 两链路；mode 1 仅 PI |
| 0x05 | SET_TARGET_STREAM | 允许 | 拒绝 | 云台路点序列 |
| 0x10 | HEARTBEAT | 允许 | 允许 | 链路心跳 |
| 0x11 | 保留 | 拒绝 | 拒绝 | 1.2 的 CALIBRATE，收到回 UNKNOWN_CMD |

### 3.2 出站命令

| CMD | 名称 | PI 链路 | BT 链路 | DATA 长度 | 说明 |
| ---: | --- | :---: | :---: | ---: | --- |
| 0x80 | READY | 发送 | 发送 | 3 | 上电/首次心跳后的能力与版本 |
| 0x81 | GIMBAL_STATUS | 发送 | — | 9 | 云台与链路健康状态 |
| 0x82 | ACK | 发送 | 发送 | 3 | 云台运动与模式命令应答 |
| 0x83 | ACK_MOTION | — | 发送 | 3 | 底盘速度应答 |
| 0x84 | NACK_BUSY | 发送 | 发送 | 3 | 运动资源被占用 |
| 0x85 | NACK_SEQ_LOST | 发送 | 发送 | 2 | 序号不连续 |
| 0x86 | ERROR | 发送 | 发送 | 2 | 错误上报 |
| 0x87 | MOTOR_STATUS | — | 发送 | 9 | 四轮实时转速 |
| 0x88 | CHASSIS_STATUS | — | 发送 | 11 | 底盘输出状态 |

应答总是回到发起命令的那条链路，不会跨链路回复。GIMBAL_STATUS 只从 PI 发出，
CHASSIS_STATUS 和 MOTOR_STATUS 只从 BT 发出；但两个状态帧携带同一组链路健康标志，
所以 PI 也能看到蓝牙链路是否丢失，反之亦然。

### 3.3 READY 能力位

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | HAS_GIMBAL | 固件包含云台 |
| 1 | HAS_CHASSIS | 固件包含底盘执行链（当前为 MotorA） |
| 2 | HAS_MOTOR_TELEMETRY | 固件包含四轮转速遥测 |

当前固件：

- PI 的 READY.flags = 0x01（HAS_GIMBAL）。
- BT 的 READY.flags = 0x06（HAS_CHASSIS | HAS_MOTOR_TELEMETRY）。

上电后两条链路各发送一次 READY；客户端在设备启动之后才连上时，固件在该链路
收到首个 HEARTBEAT 时再补发一次 READY，便于重新握手。READY 不周期发送。

## 4. 命令载荷

偏移均相对 DATA 的第一个字节。云台命令只能来自 PI，底盘命令只能来自 BT；
下面不再重复标注链路。

### 4.1 SET_TARGET (0x01)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | seq | 云台运动序号 |
| 1 | u8 | coord_type | 必须为 2（0.01°） |
| 2..3 | i16 | pan | 目标 pan，0.01° |
| 4..5 | i16 | tilt | 目标 tilt，0.01° |

DATA 长度必须为 6，coord_type 必须为 2，否则回 `ERROR(0x02)`。

### 4.2 SET_DELTA (0x02)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | seq | 云台运动序号 |
| 1..2 | i16 | delta_pan | 增量，0.01° |
| 3..4 | i16 | delta_tilt | 增量，0.01° |

DATA 长度必须为 5。每轴增量先被限制到 ±80（±0.80°），再累加到当前目标，
最后按机械限位截断。发生任一截断时 ACK 置 CLAMPED。

### 4.3 SET_VELOCITY (0x03)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | seq | 底盘速度序号（与云台序号独立） |
| 1..2 | i16 | vx | 前后，-1000..1000 |
| 3..4 | i16 | vy | 左右，-1000..1000 |
| 5..6 | i16 | omega | 旋转，-1000..1000 |

DATA 长度必须为 7。三轴输入会被归一化到 -1.0..1.0，并受底盘加速度限制。
三轴中任一轴超出 -1000..1000 时仍接受，但 ACK_MOTION 置 CLAMPED。
三轴全零是正常的“停车”命令，不改变运动 owner。

### 4.4 SET_MODE (0x04)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | mode | 0/1/3 |

| mode | 含义 | PI | BT |
| ---: | --- | :---: | :---: |
| 0 | 释放：按链路释放云台或底盘；急停锁存时全局解除 | 允许 | 允许 |
| 1 | 云台跟踪模式，owner=GIMBAL | 允许 | 回 ERROR(WRONG_LINK) |
| 2 | 保留（1.2 的标定模式） | ERROR(0x02) | ERROR(0x02) |
| 3 | 急停，立即生效，不进入命令队列 | 允许 | 允许 |

DATA 长度必须为 1，mode 只接受 0/1/3。SET_MODE(3) 没有 ACK；其余成功回
`ACK(0x04, seq=0, flags=0x04)`。

### 4.5 SET_TARGET_STREAM (0x05)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | seq | 云台运动序号 |
| 1 | u8 | coord_type | 必须为 2 |
| 2 | u8 | count | 路点数，1..10 |
| 3+4i | i16 | pan[i] | 第 i 个路点 pan |
| 5+4i | i16 | tilt[i] | 第 i 个路点 tilt |

DATA 长度必须为 3+4*count。固件复制并钳位所有路点，然后从第一个路点开始
按 0.1° 容差依次逼近。路径执行期间到达判定按当前路点；执行完毕后 path_active
清零。新 SET_TARGET/SET_DELTA/SET_MODE(0) 会取消当前路径。

### 4.6 HEARTBEAT (0x10)

DATA 长度必须为 0。收到后只刷新该链路自己的心跳时间，不回 ACK。
心跳超时为 500 ms，但只在该链路曾经收到过心跳后开始计时；从未连接过的链路
不会因为“没有心跳”而进入故障。

启动时固件在两条链路各发送一次 READY；客户端在设备启动之后才连上时，固件在
该链路收到首个 HEARTBEAT 时补发 READY。心跳策略见第 7 节。

### 4.7 保留命令 (0x11)

1.2 的 CALIBRATE 从 1.3 起已移除，0x11 当前为保留命令码。收到 0x11 时固件按
未知命令处理，回 `ERROR(0x07, 0x11)`。

### 4.8 MOTOR_STATUS (0x87)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | present_mask | bit0..bit3 对应 A/B/C/D，1 表示该轮有有效转速反馈 |
| 1..2 | i16 | rpm_A | A 轮实时转速，RPM |
| 3..4 | i16 | rpm_B | B 轮实时转速，RPM |
| 5..6 | i16 | rpm_C | C 轮实时转速，RPM |
| 7..8 | i16 | rpm_D | D 轮实时转速，RPM |

DATA 长度固定为 9，只从 BT 链路发送。mask 的 bit4..bit7 保留为 0；没有对应电机的
位为 0，转速字段填 0。当前硬件只实现 MotorA，因此 mask 为 0x01，B/C/D 为占位。
固件每 100 ms 发送一次 MOTOR_STATUS（10 Hz）。它是单向周期上报，不需要 ACK，
也不占序号。

## 5. 应答与错误

### 5.1 ACK (0x82)

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | 被应答的命令码 |
| 1 | u8 | 序号（SET_MODE 为 0） |
| 2 | u8 | flags |

flags 位：

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | AT_TARGET | 命令接受时云台已经在目标容差内 |
| 1 | CLAMPED | 目标/增量/路点被限幅截断 |
| 2 | ACCEPTED | 命令已接受 |
| 3 | REJECTED | 保留 |

云台运动命令被接受时至少置 ACCEPTED。重复序号会重发缓存的 ACK，不重复执行。
SET_TARGET_STREAM 的 AT_TARGET 在设置路径时通常为 0，因为路径立即激活；
到达路点不会额外发送异步 ACK。

### 5.2 ACK_MOTION (0x83)

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | 速度命令序号 |
| 1 | u8 | motion_state |
| 2 | u8 | flags |

motion_state 取值为下面的底盘状态。flags 位：

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | ACCEPTED | 速度命令已接受 |
| 1 | CLAMPED | 输入被限幅 |
| 2 | STOPPED | 当前为全零停车 |

全零 SET_VELOCITY 回 `ACK_MOTION(seq, 0, 0x04)`，不改 owner。
非零命令因云台占用被拒绝时回 `ACK_MOTION(seq, 5, 0x00)`，其中 5 是
BUSY_REJECT；这不是 NACK_BUSY，也不更新序号。

### 5.3 NACK_BUSY (0x84)

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | 被拒绝的命令码 |
| 1 | u8 | 序号 |
| 2 | u8 | 当前 owner |

用于云台运动命令在底盘占用时到达，以及 PI 的 SET_MODE(1) 在底盘未停时到达。
owner 取值见第 6 节。

### 5.4 NACK_SEQ_LOST (0x85)

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | 被拒绝的命令码 |
| 1 | u8 | 期望序号 expected = last_seq + 1（mod 256） |

被拒绝的命令不会更新 last_seq。

### 5.5 ERROR (0x86)

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | 错误码 |
| 1 | u8 | 细节，通常是被拒绝/出错的命令码，CRC/帧错为 0 |

| 错误码 | 名称 | 触发条件 |
| ---: | --- | --- |
| 0x01 | CRC | 连续 5 帧 CRC 错误，只上报一次 |
| 0x02 | FRAME | LEN/EOF/转义/字段长度/字段值非法；心跳帧带数据；格式错误 |
| 0x03 | STATE | 急停状态下发运动命令，或模式状态不匹配 |
| 0x04 | HEARTBEAT | 该链路 500 ms 没有有效 HEARTBEAT |
| 0x05 | 保留 | 1.2 的 CALIBRATION_MISSING |
| 0x06 | 保留 | 1.2 的 FLASH |
| 0x07 | UNKNOWN_CMD | 未定义的 CMD（含 0x11） |
| 0x08 | WRONG_LINK | 命令不属于当前链路 |

每次发送 ERROR 都会让状态帧的 err_count 饱和自增。命令队列溢出只增加 err_count，
不会逐帧发 ERROR，避免错误风暴。连续 CRC 计数在收到合法帧后清零。
错误的回复只发回出错的那条链路。

### 5.6 GIMBAL_STATUS (0x81，PI 链路)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | gimbal_state | 0 IDLE / 1 TRACKING / 2 保留 / 3 ERROR / 4 ESTOP |
| 1 | u8 | owner | 0 NONE / 1 GIMBAL / 2 CHASSIS / 3 ESTOP_FAULT |
| 2..3 | i16 | pan | 当前实际 pan，0.01°（模型值） |
| 4..5 | i16 | tilt | 当前实际 tilt，0.01°（模型值） |
| 6..7 | u16 | err_count | 错误计数，饱和到 65535 |
| 8 | u8 | flags | 见 5.8 |

### 5.7 CHASSIS_STATUS (0x88，BT 链路)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | motion_state | 0 STOPPED / 1 MOVING / 2 TIMEOUT_DECEL / 3 FAULT / 4 ESTOP / 5 BUSY_REJECT |
| 1 | u8 | owner | 同 GIMBAL_STATUS |
| 2..3 | i16 | velocity_x | 当前 vx 输出，-1000..1000 |
| 4..5 | i16 | velocity_y | 当前 vy 输出，-1000..1000 |
| 6..7 | i16 | omega | 当前旋转输出，-1000..1000 |
| 8..9 | u16 | err_count | 错误计数，饱和到 65535 |
| 10 | u8 | flags | 见 5.8 |

velocity_x/y/omega 是底盘斜坡后的实际输出（归一化值 ×1000），不是收到的命令值。
收到全零命令后它也会随着减速逐渐降到 0。

### 5.8 状态标志

GIMBAL_STATUS.flags 与 CHASSIS_STATUS.flags 使用同一组位，内容来自同一份全局快照：

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | 保留 | 1.2 的 CAL_VALID |
| 1 | EMERGENCY | 全局急停锁存中（只由 SET_MODE(3) 产生） |
| 2 | PI_HEARTBEAT_OK | PI 链路已收到心跳且距上次 ≤500 ms |
| 3 | CHASSIS_STOPPED | 底盘三轴输出已停止 |
| 4 | BT_HEARTBEAT_OK | BT 链路已收到心跳且距上次 ≤500 ms |
| 5 | PI_LINK_LOST | PI 链路心跳曾超时 |
| 6 | BT_LINK_LOST | BT 链路心跳曾超时 |
| 7 | 保留 | 0 |

2.0 不再把心跳丢失等同于全局急停：心跳超时只置 LINK_LOST，并执行该链路的
局部安全动作（见第 7 节），不会置 EMERGENCY。EMERGENCY 只在 SET_MODE(3) 后置位。

状态发送时机：

- PI 的 GIMBAL_STATUS：状态字段变化时立即发送，另外每 1 s 发送一次。
- BT 的 CHASSIS_STATUS：状态字段变化时立即发送，另外每 100 ms 发送一次。
- “状态字段变化”只比较 state/owner/flags；pan/tilt 和速度输出的细微变化不会
  触发额外帧，按各自的周期更新。
- 上电初始化完成后两条链路各发送一帧状态。

## 6. 互斥仲裁

### 6.1 owner

| 值 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | NONE | 没有运动占用 |
| 1 | GIMBAL | 云台占用，底盘非零速度命令被拒绝 |
| 2 | CHASSIS | 底盘占用，云台运动命令被拒绝 |
| 3 | ESTOP_FAULT | 急停锁存，所有运动命令被拒绝 |

云台 owner 不会自动释放，必须由 PI 发 SET_MODE(0)，或 PI 心跳超时后由固件释放。
底盘 owner 在底盘完全停止后由 50 Hz 循环自动释放。急停只接受 SET_MODE(0) 退出。

### 6.2 云台运动准入顺序

PI 链路的 SET_TARGET / SET_DELTA / SET_TARGET_STREAM 按以下顺序检查：

1. 全局急停锁存：回 `ERROR(0x03)`，detail=命令码。
2. owner==CHASSIS 或底盘未完全停止：回 `NACK_BUSY`，owner 为当前 owner。
3. 序号检查：
   - 首次命令（序号基线无效）接受任意 seq。
   - seq==last_seq：重复，重发缓存 ACK，不执行。
   - seq==last_seq+1（mod 256）：执行。
   - 其它：回 `NACK_SEQ_LOST`，expected=last_seq+1。
4. 接受后 owner=GIMBAL，gimbal_state=TRACKING，更新目标。

“底盘未完全停止”由状态标志 CHASSIS_STOPPED 为 0 判断，即使 owner 已经释放，
只要底盘还在减速，云台命令也会被拒绝。

### 6.3 底盘速度准入顺序

BT 链路的 SET_VELOCITY 按以下顺序检查：

1. 全局急停锁存：回 `ERROR(0x03)`。
2. 三轴全零：接受，回 `ACK_MOTION(seq, 0, 0x04)`，owner 不变，清速度超时，
   底盘按正常减速度停车。重复的零速度序号只重发 ACK，不重复执行。
3. 非零且（owner==GIMBAL 或 gimbal_state!=IDLE 或云台未到目标）：
   回 `ACK_MOTION(seq, 5, 0x00)`，不更新序号，不刷新速度超时。
4. 序号检查，规则同上；速度命令使用独立的 seq 基线。
5. 接受后 owner=CHASSIS，motion_state=MOVING，刷新 300 ms 速度超时，
   回 `ACK_MOTION(seq, 1, flags)`。

注意：非零速度命令的“忙”应答是 ACK_MOTION + BUSY_REJECT，不是 NACK_BUSY，
这是为了与“速度命令总是用 ACK_MOTION 应答”的约定一致。

### 6.4 SET_MODE

SET_MODE(0) 按链路释放：

PI 链路：

- 清空云台序号基线。
- 云台目标置为当前位置，gimbal_state=IDLE，伺服释放。
- 如果 owner==GIMBAL：owner=NONE。
- 不影响底盘 owner、速度看门狗或底盘输出。

BT 链路：

- 清空底盘速度序号基线，关闭速度看门狗。
- 如果 owner==CHASSIS 且底盘已停：owner=NONE，motion_state=STOPPED。
- 如果 owner==CHASSIS 但底盘还在动：owner 保持到完全停止，motion_state 进入
  TIMEOUT_DECEL。
- 不影响云台 owner 或云台状态。

急停锁存期间：任一链路的 SET_MODE(0) 都解除全局急停，同时清空两个序号基线。
解除后底盘若还在减速，motion_state 进入 TIMEOUT_DECEL，停稳后回到 STOPPED。
以上情况成功都回 `ACK(0x04, 0, 0x04)`。

SET_MODE(1) 云台跟踪：

- 只允许 PI 链路；BT 链路回 `ERROR(0x08)`。
- 急停锁存：回 `ERROR(0x03)`。
- owner==CHASSIS 或底盘未停：回 `NACK_BUSY`。
- 否则 owner=GIMBAL，gimbal_state=TRACKING，清云台序号基线。
- 成功回 `ACK(0x04, 0, 0x04)`。

SET_MODE(2)：

- 1.2 的云台标定模式，1.3 起已移除。
- 固件在解析层拒绝该值，回 `ERROR(0x02, 0x04)`，不改变 owner 或序号。

SET_MODE(3) 急停：

- 两条链路都允许。在协议解析层立即执行，不进入命令队列。
- 只要 SET_MODE 帧的第一个 DATA 字节为 3，即使长度不是 1，也会触发急停。
- 云台失能、目标=当前位置、底盘按最大减速度停车，owner=ESTOP_FAULT，
  gimbal_state=ESTOP，motion_state=ESTOP。
- 不回 ACK；立即向两条链路各发一帧状态。
- 只接受 SET_MODE(0) 退出。
- 仓库没有激光驱动，急停不会操作激光。

### 6.5 状态转换

~~~text
NONE --PI 云台运动命令 / SET_MODE(1)--> GIMBAL
NONE --BT 非零 SET_VELOCITY--> CHASSIS
GIMBAL --PI SET_MODE(0)--> NONE
GIMBAL --PI 心跳超时--> NONE
GIMBAL --SET_MODE(3)--> ESTOP_FAULT
CHASSIS --完全停止--> NONE
CHASSIS --BT SET_MODE(0)--> CHASSIS（保持到停止）--> NONE
任意 --SET_MODE(3)--> ESTOP_FAULT
ESTOP_FAULT --任一链路 SET_MODE(0)--> NONE / TIMEOUT_DECEL
~~~

### 6.6 序号规则

- 云台命令共享一个 last_seq；速度命令使用独立的 last_seq。
- 第一次命令接受任意 seq，之后从该值建立基线。
- 重复 seq：只重发缓存应答，不重复执行，不推进 last_seq。
- 下一个合法 seq 是 last_seq+1（uint8 回绕，255 后面是 0）。
- BUSY、DENIED、NACK、WRONG_LINK 都不更新 last_seq。
- PI 的 SET_MODE(0) 只清云台基线；BT 的 SET_MODE(0) 只清底盘基线。
- SET_MODE(1) 只清云台基线。
- 急停期间序号保持；退出急停（任一链路 SET_MODE(0)）后清空两个基线。

## 7. 时序与安全

| 项目 | 值 | 说明 |
| --- | ---: | --- |
| 控制周期 | 20 ms | 云台/底盘各 50 Hz |
| 速度超时 | 300 ms | 无新的非零速度命令后按正常减速度停车 |
| 心跳超时 | 500 ms | 每条链路独立计时 |
| GIMBAL_STATUS 周期 | 1000 ms | 状态变化时额外发送 |
| CHASSIS_STATUS 周期 | 100 ms | 状态变化时额外发送 |
| MOTOR_STATUS 周期 | 100 ms | 四轮转速，10 Hz，只发 BT |
| CRC 报警阈值 | 5 帧 | 连续 5 帧 CRC 错，ERROR(0x01) 一次 |
| 链路轮询 | 10 ms | LinkTask 通知等待上限 |
| 发送超时 | 50 ms | 单帧阻塞发送上限 |
| 建议重传 | 50 ms × 3 | 客户端等待应答，超时用相同 seq 重发 |

### 7.1 速度超时

- motion_state 从 MOVING 变为 TIMEOUT_DECEL，底盘使用正常减速度（3.5/s）。
- 300 ms 超时不是急停；急停使用最大减速度（8.0/s）。
- 超时后 owner 仍是 CHASSIS，直到底盘完全停止才释放。

### 7.2 心跳分链路处理

两条链路各自有“是否曾经收到心跳”和“最后心跳时刻”。从未收到过心跳的链路
不会超时，因此只接树莓派或只接蓝牙都能独立工作。

PI 心跳超时：

- 释放 GIMBAL owner（如果当前是云台占用），gimbal_state=IDLE，云台失能，
  GimbalPort 被禁用。
- 不改变底盘 owner、速度看门狗和底盘输出。
- 立即向 PI 回 `ERROR(0x04)`，并向两条链路更新状态。
- 状态 flags 置 PI_LINK_LOST；不会置 EMERGENCY。

BT 心跳超时：

- 清掉速度看门狗，底盘按最大减速度（8.0/s）停车，motion_state 进入
  TIMEOUT_DECEL；停稳后释放 CHASSIS owner 并回到 STOPPED。
- 不改变云台 owner 或云台状态。
- 立即向 BT 回 `ERROR(0x04)`，并向两条链路更新状态。
- 状态 flags 置 BT_LINK_LOST；不会置 EMERGENCY。

急停锁存期间不检测心跳超时。BLE 连接只能由主机发起，STM32 不能主动重连；
重新连接后客户端应重新发心跳，并检查状态与序号基线再恢复控制。

## 8. 云台模型

| 参数 | 值 | 说明 |
| --- | ---: | --- |
| 角度单位 | 0.01° | 18000 = 180.00° |
| pan 限位 | `APP_GIMBAL_PAN_MIN/MAX` = -9000..9000 | ±90° |
| tilt 限位 | `APP_GIMBAL_TILT_MIN/MAX` = 0..9000 | 0..90° |
| 最大速度 | 9000 LSB/s | 90°/s |
| 到达容差 | 10 LSB | 0.1° |
| 单帧增量限幅 | ±80 LSB | ±0.8° |
| 最大路点数 | 10 | SET_TARGET_STREAM |

- actual_pan/actual_tilt 是模型积分值，不是编码器反馈；GimbalPort 需要把
  这两个值接到实际执行器。
- 每个 20 ms 控制周期最多移动 180 LSB（9000 × 0.02）。
- 路点序列按顺序执行；到达当前路点后，下一个控制周期切换到下一个路点。
  切换的那个周期不移动，因此每个路点会多一个 20 ms 的停顿。
- Gimbal_SetTarget 和 Gimbal_AddDelta 会取消正在执行的路点序列。
- SET_MODE(0) 会取消路点序列并把目标置为当前位置。

## 9. 底盘模型

SET_VELOCITY 的三轴输入范围是 -1000..1000，固件归一化到 -1.0..1.0。

| 参数 | 值 |
| --- | ---: |
| 归一化 | 输入 / 1000 |
| 死区 | 0.02 |
| 加速度 | 2.5 /s |
| 正常减速度 | 3.5 /s |
| 急停减速度 | 8.0 /s |
| 停止阈值 | 0.0005 |

- 输出通过 ChassisPort_Apply(vx, vy, omega) 交给底盘执行层。
- 当前 ChassisPort_Apply 只把 vx 映射到 MotorA.target_rpm（vx×110），MotorA 由
  10 ms PID 闭环；MotorB/C/D 和 Vehicle_SetVelocity 尚未实现。
- BT 的 READY 始终声明 HAS_CHASSIS；没有独立的底盘使能开关。vy/omega 会被
  协议接受并参与斜坡与状态输出，但在接入完整底盘前不驱动任何电机。
- 三轴分别按加速/减速/反向/急停选择加速度，不再共用 vx 的加速度。
- 全零命令立即把目标设为 0，输出按正常减速度下降；完全停止后 owner 释放。

## 10. 标定、零点与持久化（当前不包含）

1.3 删除了 1.2 的 CALIBRATE 命令、SET_MODE(2) 标定模式、标定校验以及 Flash
双区记录和 CRC32 校验；2.0 沿用这一决定。原因：云台没有编码器/回零，上电时实际
角度未知；没有可信的绝对参考时，持久化零点不能改善控制，反而引入 Flash 擦除阻塞
和失败模式。

当前行为：

- 云台使用 `APP_GIMBAL_PAN_MIN/MAX`、`APP_GIMBAL_TILT_MIN/MAX` 固定机械限位。
- READY 只有版本和能力位，不再有 `calibration_valid/pan_zero/tilt_zero`。
- 状态 flags 的 bit0 保留，不再表示 CAL_VALID。
- 0x11 是保留命令码，收到回 ERROR(0x07)；SET_MODE(2) 回 ERROR(0x02)。
- 固件不执行任何 Flash 擦除/编程，也不再使用 CRC32。

未来接入编码器/回零后，建议按以下顺序恢复标定能力：

1. 增加回零或绝对位置来源，确定上电时 pan/tilt 的可信坐标。
2. 定义逻辑零点与软限位，并真正应用到 Gimbal 的目标/状态坐标。
3. 重新引入标定命令（建议新 CMD，或复用 0x11 并定义新子命令），协议再升一个
   次版本。
4. 再选择持久化方案（Flash 双区或外部 EEPROM），并定义掉电/损坏恢复策略。

## 11. 启动与对接

### 11.1 上电顺序

1. STM32 初始化 USART3/UART5、DMA、运动控制、任务。
2. TelemetryTask 向两条链路各发送一帧 READY，并立即发送首帧 GIMBAL_STATUS、
   CHASSIS_STATUS 和 MOTOR_STATUS。
3. 两条 LinkTask 开始接收；各自在收到首个 HEARTBEAT 时补发一次 READY。
4. 树莓派收到 PI 的 READY(2.0)，检查 HAS_GIMBAL；蓝牙客户端收到 BT 的
   READY(2.0)，检查 HAS_CHASSIS / HAS_MOTOR_TELEMETRY。
5. 树莓派发 SET_MODE(1) 进入云台跟踪；蓝牙客户端发 SET_VELOCITY 驱动底盘。
6. 之后两条链路各自按 100–200 ms 周期发送 HEARTBEAT。

### 11.2 树莓派状态机

| 状态 | 行为 |
| --- | --- |
| WAIT_READY | 打开串口，立即发 HEARTBEAT，等待 READY |
| IDLE | SET_MODE(0)，定期心跳，处理 GIMBAL_STATUS |
| GIMBAL_TRACKING | SET_MODE(1)，发 SET_TARGET/SET_DELTA/STREAM，维护 seq |
| ESTOP | 收到 EMERGENCY 或主动急停；发 SET_MODE(3)，之后只发 SET_MODE(0) 解除 |

树莓派不再负责底盘，因此 2.0 没有 `CHASSIS_REMOTE` 状态。

### 11.3 蓝牙客户端

- 通过 BLE GATT 连接模块地址（例如 `F0:72:D1:80:FB:EB`），使用 vendor 服务
  `0000ffe0-...` 和特征 `0000ffe1-...`；该特征同时支持 write 和 notify，
  写入帧、从 notify 读回帧。不需要 PIN 配对，也没有波特率设置。
- 发送 HEARTBEAT 维持链路；收到 READY 后按 300 ms 速度超时持续发送 SET_VELOCITY，
  或主动发全零停车。
- 断开连接时无法发全零命令，固件会在 500 ms 心跳超时后按最大减速度停车。
- 重新连接后先读 CHASSIS_STATUS，确认 owner 和 motion_state，再重新建立速度序号。

### 11.4 重传与序号

- 每条需要应答的命令启动 50 ms 定时器，最多重发 3 次。
- 重发使用相同 seq；固件会重发缓存应答，不会重复执行。
- 收到 NACK_SEQ_LOST 时，用 expected 作为新 seq 重发。
- 收到 NACK_BUSY 时不要忙等，先读状态 owner 决定是否释放或等底盘停。
- 收到 ERROR(WRONG_LINK) 说明客户端把命令发到了错误链路，应检查端口绑定，
  不要原样重试。
- 心跳不需要应答；即使命令重传，也不要停止心跳。

### 11.5 客户端解析建议

- 先按 SOF/转义规则还原，再校验 LEN 和 CRC；不要把转义后的字节直接当 DATA。
- 处理 0xAA 0x55 重同步；串口噪声可能制造假 SOF。
- 维护接收超时；一帧内超过约 10 ms 没有新字节可以丢弃半帧。
- 对 GIMBAL_STATUS 的 pan/tilt 做低通或直接使用；它们是模型值，不是编码器值。
- MOTOR_STATUS 每 100 ms 独立发送；按 present_mask 过滤占位轮，不要假设四轮都有反馈。
- 不要依赖“到达目标”的异步 ACK；用 GIMBAL_STATUS 的 pan/tilt 与目标比较，
  或等待 1 s 周期状态。

## 12. 固件实现说明

### 12.1 任务

| 任务 | 优先级 | 栈（words） | 职责 |
| --- | ---: | ---: | --- |
| link_pi / link_bt | 5 | 320 | 各自的 UART 接收、解码、心跳直通、急停优先处理 |
| command | 4 | 384 | 共享命令队列分发、心跳超时检查 |
| gimbal | 3 | 256 | 20 ms 云台步进 |
| chassis | 3 | 256 | 20 ms 底盘步进、速度超时、减速 |
| tx_pi / tx_bt | 2 | 256 | 各自的发送队列、编码、阻塞发送 |
| telemetry | 1 | 256 | 两条链路的状态与 READY、10 Hz 电机遥测 |
| motor（既有） | 4 | 256 | 10 ms 电机 PID 与转速遥测 |

### 12.2 数据流

- 两个 LinkTask 各自持有独立的 `ProtoDecoder`，互不共享解析状态。
- HEARTBEAT 绕过共享命令队列，直接更新对应链路的心跳；首个心跳补发 READY。
- 任一链路收到 SET_MODE(3) 时，LinkTask 立即调用 `MotionControl_EmergencyStop`，
  清空命令队列，并向两条链路发送状态，不等待 CommandTask。
- 其他帧封装为 `{link, frame}` 进入共享命令队列，由 CommandTask 统一路由。
- 回复由 `CommandRouter` 发回来源链路的独立 TxQueue；两条链路不会互相阻塞。
- `MotionControl` 用 FreeRTOS 互斥锁保护仲裁器、云台和底盘状态；上层任务不会
  直接并发修改运动状态。
- 两条 UART 的 `UartConnection` 通过 HAL 回调中的 huart 指针找到对应实例，
  circular DMA 下的接收位置由 `__HAL_DMA_GET_COUNTER` 自行计算，不依赖 HAL 的
  IDLE 长度。

### 12.3 配置

链路、缓冲、任务优先级/栈、队列深度、超时和模型参数都在
`Core/Inc/app/app_config.h`。修改 `APP_PI_UART/APP_BT_UART` 时，同步修改
`Fusai.ioc` 中的引脚、DMA 和中断配置，并重新生成代码。

## 13. 已知差距

- GimbalPort 是空实现，尚未接入实际伺服/步进驱动；actual 是模型积分值。
- ChassisPort 目前只驱动 MotorA；MotorB/C/D 和 Vehicle_SetVelocity 尚未实现。
- MotorB/C/D 尚未实现：MOTOR_STATUS 只置位 A 的 mask 位，B/C/D 报 0。
- MotorA 有编码器反馈和 10 ms PID 闭环；B/C/D 无反馈。
- 急停不操作激光：仓库没有激光驱动。
- 不包含标定、零点和持久化；云台只有固定机械限位。
- 云台到达目标没有异步 ACK；只能通过 GIMBAL_STATUS 观察。
- 未启用 IWDG。
- 没有宿主机单元测试环境；当前以目标板编译和 SDK 单测验证为主。
- 命令队列溢出会计入 err_count，但不会逐帧发 ERROR。
- TxQueue 发送失败只内部计数，未计入 err_count。
- BLE 连接只能由主机发起；STM32 不能主动重连蓝牙。

## 14. TODO

- 接入云台编码器或回零开关，建立可信的绝对角度来源。
- 设计并实现逻辑零点、软限位和标定流程；标定数据再考虑持久化。
- 接入实际 GimbalPort（伺服/步进）并验证方向、限位和失能行为。
- 实现 MotorB/C/D，接入 Vehicle_SetVelocity 并验证底盘闭环。
- 增加激光驱动，并把急停扩展到激光关闭。
- 评估 IWDG 与任务心跳的配合。
- 增加宿主机单元测试（motion_arbiter、codec、decoder 可脱离 HAL 测试）。
- 修复 TxQueue 发送失败/丢弃计数未进入 err_count 的问题。
