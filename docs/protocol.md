# Fusai v1.2 互斥运动协议

本文描述 STM32 主控与树莓派上位机之间的串口协议，以及固件中的互斥运动仲裁、
云台/底盘模型、标定流程和对接建议。协议版本为 1.2，版本号由 READY 帧上报。

- 主控：STM32F407VETx，FreeRTOS，控制周期 20 ms
- 默认链路：UART5，115200 8N1，无硬件流控
- 字节序：所有多字节整数均为小端（little-endian）
- 角度单位：0.01 度（LSB = 0.01°）

## 1. 物理层

| 项目 | 值 |
| --- | --- |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | 无 |
| 停止位 | 1 |
| 流控 | 无 |
| 默认 UART | UART5（PC12 TX / PD2 RX） |
| 接收方式 | DMA Circular + UART IDLE 事件 |

换 UART 时修改 Core/Inc/app/app_config.h 中的 APP_LINK_UART 即可；DMA 通道、
引脚和中断需要在 CubeMX 中配置。接收环形缓冲大小 APP_LINK_RX_RING_SIZE 必须是
2 的幂，DMA 缓冲大小 APP_LINK_DMA_RX_SIZE 必须与 HAL 调用一致。

## 2. 帧格式

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

参考实现位于 Core/Src/utils/crc.c 的 Crc16Modbus()。

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

SET_MODE(1)（云台跟踪）：

~~~text
AA 55 02 04 01 13 00 0D
~~~

SET_MODE(3)（急停）：

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

## 3. 命令总表

| CMD | 名称 | 方向 | DATA 长度 | 说明 |
| ---: | --- | --- | ---: | --- |
| 0x01 | SET_TARGET | Pi → STM32 | 6 | 绝对云台目标 |
| 0x02 | SET_DELTA | Pi → STM32 | 5 | 增量云台目标 |
| 0x03 | SET_VELOCITY | Pi → STM32 | 7 | 底盘速度 |
| 0x04 | SET_MODE | Pi → STM32 | 1 | 模式/释放/标定/急停 |
| 0x05 | SET_TARGET_STREAM | Pi → STM32 | 3+4n | 云台路点序列，n=1..10 |
| 0x10 | HEARTBEAT | Pi → STM32 | 0 | 链路心跳 |
| 0x11 | CALIBRATE | Pi → STM32 | 1 或 3 | 标定查询/设零/提交 |
| 0x80 | READY | STM32 → Pi | 8 | 启动/查询/提交后的能力与版本 |
| 0x81 | STATUS | STM32 → Pi | 10 | 周期/变化状态 |
| 0x82 | ACK | STM32 → Pi | 3 | 云台运动与模式命令应答 |
| 0x83 | ACK_MOTION | STM32 → Pi | 3 | 底盘速度应答 |
| 0x84 | NACK_BUSY | STM32 → Pi | 3 | 运动资源被占用 |
| 0x85 | NACK_SEQ_LOST | STM32 → Pi | 2 | 序号不连续 |
| 0x86 | ERROR | STM32 → Pi | 2 | 错误上报 |

## 4. 命令载荷

偏移均相对 DATA 的第一个字节。

### 4.1 SET_TARGET (0x01)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | seq | 云台运动序号 |
| 1 | u8 | coord_type | 必须为 2（0.01°） |
| 2..3 | i16 | pan | 目标 pan，0.01° |
| 4..5 | i16 | tilt | 目标 tilt，0.01° |

DATA 长度必须为 6，coord_type 必须为 2，否则回 ERROR(0x02)。

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

DATA 长度必须为 7。三轴输入会被归一化到 -1.0..1.0 并受底盘加速度限制。
三轴中任一轴超出 -1000..1000 时仍接受，但 ACK_MOTION 置 CLAMPED。
三轴全零是正常的“停车”命令，不改变运动 owner。

### 4.4 SET_MODE (0x04)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | mode | 0/1/2/3 |

| mode | 含义 |
| ---: | --- |
| 0 | 释放：清序号；云台释放；底盘保持到完全停止；急停/故障解除 |
| 1 | 云台跟踪模式，owner=GIMBAL |
| 2 | 云台标定模式，owner=GIMBAL，云台伺服释放 |
| 3 | 急停，立即生效，不进入命令队列 |

DATA 长度必须为 1，mode 必须 ≤3。SET_MODE(3) 没有 ACK；其余模式成功回
ACK(0x04, seq=0, flags=0x04)。

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
清零。新 SET_TARGET/SET_DELTA/SET_MODE(0/2) 会取消当前路径。

### 4.6 HEARTBEAT (0x10)

DATA 长度必须为 0。收到后刷新心跳时间，不回 ACK。心跳超时是 500 ms。
上电后 500 ms 内没有收到心跳会进入故障，必须用 SET_MODE(0) 清除。

### 4.7 CALIBRATE (0x11)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | subcmd | 0x01/0x02/0x03/0x04 |
| 1..2 | i16 | value | 仅 PAN/TILT 使用 |

| subcmd | 名称 | DATA 长度 | 说明 |
| ---: | --- | ---: | --- |
| 0x01 | PAN | 3 | 设置 pan 零点参考值 |
| 0x02 | QUERY | 1 | 回 READY |
| 0x03 | COMMIT | 1 | 提交到 Flash |
| 0x04 | TILT | 3 | 设置 tilt 零点参考值 |

PAN/TILT/COMMIT 只在 SET_MODE(2) 标定状态下接受，否则回 ERROR(0x03)。
QUERY 在任何状态都可以使用。PAN/TILT 成功回 ACK(0x11, seq=0, flags=0x04)；
COMMIT 先回 ACK，Flash 操作完成后成功回 READY，失败回 ERROR(0x06, 0x03)。

## 5. 应答与错误

### 5.1 ACK (0x82)

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | 被应答的命令码 |
| 1 | u8 | 序号（SET_MODE/CALIBRATE 为 0） |
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

全零 SET_VELOCITY 回 ACK_MOTION(seq, 0, 0x01)，不改 owner。
非零命令因云台占用被拒绝时回 ACK_MOTION(seq, 5, 0x00)，其中 5 是
BUSY_REJECT；这不是 NACK_BUSY，也不更新序号。

### 5.3 NACK_BUSY (0x84)

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u8 | 被拒绝的命令码 |
| 1 | u8 | 序号 |
| 2 | u8 | 当前 owner |

用于云台运动命令在底盘占用或标定中到达时。owner 取值见第 6 节。

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
| 0x03 | STATE | 急停/故障状态下发运动命令，或标定状态不匹配 |
| 0x04 | HEARTBEAT | 500 ms 没有有效 HEARTBEAT |
| 0x05 | CALIBRATION_MISSING | 没有有效标定记录时发云台运动命令 |
| 0x06 | FLASH | 标定提交失败，细节 0x03 表示 COMMIT |
| 0x07 | UNKNOWN_CMD | 未定义的 CMD |

每次发送 ERROR 都会让 STATUS.err_count 饱和自增。命令队列溢出只增加 err_count，
不会逐帧发 ERROR，避免错误风暴。连续 CRC 计数在收到合法帧后清零。

### 5.6 STATUS (0x81)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | gimbal_state | 0 IDLE / 1 TRACKING / 2 CALIBRATING / 3 ERROR / 4 ESTOP |
| 1 | u8 | owner | 0 NONE / 1 GIMBAL / 2 CHASSIS / 3 ESTOP_FAULT |
| 2 | u8 | motion_state | 0 STOPPED / 1 MOVING / 2 TIMEOUT_DECEL / 3 FAULT / 4 ESTOP / 5 BUSY_REJECT |
| 3..4 | i16 | pan | 当前实际 pan，0.01° |
| 5..6 | i16 | tilt | 当前实际 tilt，0.01° |
| 7..8 | u16 | err_count | 错误计数，饱和到 65535 |
| 9 | u8 | flags | 见下表 |

STATUS.flags：

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | CAL_VALID | Flash 中有有效标定记录 |
| 1 | EMERGENCY | 急停或心跳故障锁存中 |
| 2 | HEARTBEAT_OK | 已收到心跳且距上次 ≤500 ms |
| 3 | CHASSIS_STOPPED | 底盘三轴输出已停止 |

STATUS 在以下时机发送：上电后第一次状态、状态字段变化、每 1 s 周期。
pan/tilt 的变化本身不会触发额外 STATUS，只在状态变化或 1 Hz 周期时更新。

### 5.7 READY (0x80)

| 偏移 | 类型 | 字段 | 说明 |
| ---: | --- | --- | --- |
| 0 | u8 | version_major | 1 |
| 1 | u8 | version_minor | 2 |
| 2 | u8 | flags | 能力位 |
| 3 | u8 | calibration_valid | 0/1 |
| 4..5 | i16 | pan_zero | 标定 pan 零点 |
| 6..7 | i16 | tilt_zero | 标定 tilt 零点 |

READY.flags：

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | HAS_GIMBAL | 固件包含云台 |
| 1 | HAS_CHASSIS | 底盘驱动已编译启用 |
| 2 | CAL_VALID | 有有效标定记录 |

READY 在上电、CALIBRATE QUERY、标定提交成功后发送，不周期发送。

## 6. 互斥仲裁

### 6.1 owner

| 值 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | NONE | 没有运动占用 |
| 1 | GIMBAL | 云台占用，底盘速度命令被拒绝 |
| 2 | CHASSIS | 底盘占用，云台运动命令被拒绝 |
| 3 | ESTOP_FAULT | 急停或心跳故障，所有运动命令被拒绝 |

云台 owner 不会自动释放，必须 SET_MODE(0)。底盘 owner 在底盘完全停止后由
50 Hz 循环自动释放。急停/故障只接受 SET_MODE(0) 退出。

### 6.2 云台运动准入顺序

SET_TARGET / SET_DELTA / SET_TARGET_STREAM 按以下顺序检查：

1. STATUS.EMERGENCY 置位：回 ERROR(0x03)，detail=命令码。
2. owner==CHASSIS 或底盘未完全停止：回 NACK_BUSY，owner 为当前 owner。
3. gimbal_state==CALIBRATING：回 NACK_BUSY，owner=GIMBAL。
4. 没有有效标定：回 ERROR(0x05)，detail=命令码。
5. 序号检查：
   - 首次命令（序号基线无效）接受任意 seq。
   - seq==last_seq：重复，重发缓存 ACK，不执行。
   - seq==last_seq+1（mod 256）：执行。
   - 其它：回 NACK_SEQ_LOST，expected=last_seq+1。
6. 接受后 owner=GIMBAL，gimbal_state=TRACKING，更新目标。

“底盘未完全停止”由 STATUS.CHASSIS_STOPPED 为 0 判断，即使 owner 已经释放，
只要底盘还在减速，云台命令也会被拒绝。

### 6.3 底盘速度准入顺序

SET_VELOCITY 按以下顺序检查：

1. STATUS.EMERGENCY 置位：回 ERROR(0x03)。
2. 三轴全零：接受，回 ACK_MOTION(seq, 0, 0x01)，owner 不变，清速度超时，
   底盘按正常减速度停车。
3. 非零且（owner==GIMBAL 或 gimbal_state!=IDLE 或云台未到目标）：
   回 ACK_MOTION(seq, 5, 0x00)，不更新序号，不刷新速度超时。
4. 序号检查，规则同上；速度命令使用独立的 seq 基线。
5. 接受后 owner=CHASSIS，motion_state=MOVING，刷新 300 ms 速度超时，
   回 ACK_MOTION(seq, 1, flags)。

注意：非零速度命令的“忙”应答是 ACK_MOTION + BUSY_REJECT，不是 NACK_BUSY，
这是为了与协议中“速度命令总是用 ACK_MOTION 应答”的约定一致。

### 6.4 SET_MODE

SET_MODE(0) 释放：

- 清空云台和速度序号基线。
- owner==GIMBAL：owner=NONE，云台目标置为当前位置，伺服释放。
- owner==CHASSIS：owner 保持到小车完全停止；速度超时/减速继续生效。
- owner==ESTOP_FAULT：清除急停/故障锁存；如果底盘还在运动，motion_state
  进入 TIMEOUT_DECEL，完全停止后回到 STOPPED。
- 成功回 ACK(0x04, 0, 0x04)。

SET_MODE(1) 云台跟踪：

- 急停/故障：ERROR(0x03)。
- owner==CHASSIS 或底盘未停：NACK_BUSY。
- 否则 owner=GIMBAL，gimbal_state=TRACKING，清云台序号基线。
- 成功回 ACK(0x04, 0, 0x04)。

SET_MODE(2) 云台标定：

- 准入条件与 SET_MODE(1) 相同。
- 否则 owner=GIMBAL，gimbal_state=CALIBRATING，云台伺服释放，清云台序号基线。
- 成功回 ACK(0x04, 0, 0x04)。
- 标定模式可以在云台运动过程中进入，会立即取消当前云台运动/路径。

SET_MODE(3) 急停：

- 在协议解析层立即执行，不进入命令队列。
- 只要 SET_MODE 帧的第一个 DATA 字节为 3，即使长度不是 1，也会触发急停。
- 云台失能、目标=当前位置、底盘按最大减速度停车，owner=ESTOP_FAULT，
  gimbal_state=ESTOP，motion_state=ESTOP。
- 不回 ACK；只接受 SET_MODE(0) 退出。
- 仓库没有激光驱动，急停不会操作激光。

### 6.5 状态转换

~~~text
NONE --云台运动命令--> GIMBAL
NONE --非零 SET_VELOCITY--> CHASSIS
GIMBAL --SET_MODE(0)--> NONE
GIMBAL --SET_MODE(2)--> GIMBAL(CALIBRATING)
GIMBAL --SET_MODE(3)--> ESTOP_FAULT
CHASSIS --完全停止--> NONE
CHASSIS --SET_MODE(0)--> CHASSIS（保持到停止）--> NONE
任意 --SET_MODE(3)--> ESTOP_FAULT
ESTOP_FAULT --SET_MODE(0)--> NONE / TIMEOUT_DECEL
~~~

### 6.6 序号规则

- 云台命令共享一个 last_seq；速度命令使用独立的 last_seq。
- 第一次命令接受任意 seq，之后从该值建立基线。
- 重复 seq：只重发缓存应答，不重复执行，不推进 last_seq。
- 下一个合法 seq 是 last_seq+1（uint8 回绕，255 后面是 0）。
- BUSY、DENIED、NACK 都不更新 last_seq。
- SET_MODE(0) 清空两个基线；SET_MODE(1/2) 只清空云台基线。
- 急停期间序号保持；退出急停（SET_MODE(0)）后清空。

## 7. 时序与安全

| 项目 | 值 | 说明 |
| --- | ---: | --- |
| 控制周期 | 20 ms | 云台/底盘各 50 Hz |
| 速度超时 | 300 ms | 无新的非零速度命令后按正常减速度停车 |
| 心跳超时 | 500 ms | 无心跳进入故障，ERROR(0x04) |
| STATUS 周期 | 1000 ms | 状态变化时额外发送 |
| CRC 报警阈值 | 5 帧 | 连续 5 帧 CRC 错，ERROR(0x01) 一次 |
| 链路轮询 | 10 ms | LinkTask 通知等待上限 |
| 建议重传 | 50 ms × 3 | Pi 侧等待应答，超时用相同 seq 重发 |

速度超时行为：

- motion_state 从 MOVING 变为 TIMEOUT_DECEL，底盘使用正常减速度（3.5/s）。
- 300 ms 超时不是急停；急停使用最大减速度（8.0/s）。
- 超时后 owner 仍是 CHASSIS，直到底盘完全停止才释放。

心跳故障行为：

- 首次心跳前以上电时刻为基准；因此 Pi 必须在上电后 500 ms 内发第一帧心跳。
- 故障后 owner=ESTOP_FAULT，gimbal_state=ERROR，motion_state=FAULT，
  底盘按最大减速度停车。
- 必须 SET_MODE(0) 清除；清除后如果底盘还在动，先进入 TIMEOUT_DECEL。

## 8. 云台模型

| 参数 | 值 | 说明 |
| --- | ---: | --- |
| 角度单位 | 0.01° | 18000 = 180.00° |
| pan 限位 | -18000..18000 | ±180° |
| tilt 限位 | -9000..9000 | ±90° |
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
- SET_MODE(0/2) 会取消路点序列并把目标置为当前位置。

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
- 默认 APP_CHASSIS_DRIVE_ENABLED=0，ChassisPort 是空实现；打开宏后调用
  Vehicle_SetVelocity。仓库目前只定义了 MotorA，MotorB/C/D 尚未实现。
- 三轴分别按加速/减速/反向/急停选择加速度，不再共用 vx 的加速度。
- 全零命令立即把目标设为 0，输出按正常减速度下降；完全停止后 owner 释放。

## 10. 标定与 Flash

### 10.1 流程

1. 确保底盘完全停止。
2. 发送 SET_MODE(2)，收到 ACK。
3. 可选：CALIBRATE QUERY(0x02) 读取当前 READY。
4. 对每个轴发送 CALIBRATE PAN(0x01) 或 TILT(0x04)，value 为当前物理参考角
   （i16，0.01°）。固件只记录到 RAM，不移动云台，回 ACK。
5. 发送 CALIBRATE COMMIT(0x03)，先回 ACK。
6. 低优先级 TelemetryTask 擦除/编程 Flash：
   - 成功：回 READY，calibration_valid=1，STATUS.CAL_VALID=1。
   - 失败：回 ERROR(0x06, 0x03)。
7. 发送 SET_MODE(1) 进入跟踪，或 SET_MODE(0) 释放。

没有有效标定时，云台运动命令回 ERROR(0x05)，必须先完成标定。

### 10.2 Flash 布局

| 项目 | 值 |
| --- | --- |
| Bank A | Sector 6，0x08040000，128 KB |
| Bank B | Sector 7，0x08060000，128 KB |
| 记录大小 | 32 字节 |
| 写入方式 | 整扇区擦除 + 按字编程 |
| 校验 | magic + version + length + CRC32 |

记录字段（小端）：

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | u32 | magic = 0x43414C31（"CAL1"） |
| 4 | u16 | version = 1 |
| 6 | u16 | length = 32 |
| 8 | u32 | sequence |
| 12 | i16 | pan_zero |
| 14 | i16 | tilt_zero |
| 16 | i16 | pan_min |
| 18 | i16 | pan_max |
| 20 | i16 | tilt_min |
| 22 | i16 | tilt_max |
| 24 | u32 | reserved |
| 28 | u32 | CRC32（覆盖前 28 字节） |

上电时读取两个 Bank，选择 magic/version/length/CRC 有效且 sequence 最大的记录。
没有有效记录时使用 APP_GIMBAL_* 默认值，valid=false。提交时写到当前有效区之外的
另一个 Bank；如果两个都无效，写 Bank A。这样掉电不会破坏旧记录。

注意：当前固件只持久化并上报 pan_zero/tilt_zero 和限位，云台运动模型仍使用
APP_GIMBAL_* 机械限位，没有把零点偏移应用到目标/状态坐标。若上位机需要逻辑
零点，请在 Pi 侧补偿；或者后续在 Gimbal 层接入偏移。

## 11. 启动与 Pi 对接

### 11.1 上电顺序

1. STM32 初始化 UART/DMA、Flash 标定、任务。
2. TelemetryTask 发送 READY。
3. LinkTask 开始接收；500 ms 内等待第一帧 HEARTBEAT。
4. Pi 收到 READY，检查 version 和 CAL_VALID。
5. 如果 CAL_VALID=0，进入标定流程；否则 SET_MODE(1) 或 SET_MODE(0)。
6. 之后按 100–200 ms 周期发送 HEARTBEAT。

### 11.2 建议的 Pi 状态机

| 状态 | 行为 |
| --- | --- |
| WAIT_READY | 打开串口，立即发 HEARTBEAT，等待 READY |
| CALIBRATING | 发 SET_MODE(2)，按轴标定，COMMIT，等 READY/ERROR |
| IDLE | SET_MODE(0)，定期心跳，处理 STATUS |
| GIMBAL_TRACKING | SET_MODE(1)，发 SET_TARGET/SET_DELTA/STREAM，维护 seq |
| CHASSIS_REMOTE | 发非零 SET_VELOCITY，维护独立 seq，持续心跳 |
| ESTOP | 收到 EMERGENCY 或主动急停；发 SET_MODE(3)，之后只发 SET_MODE(0) 解除 |

推荐的重传策略：

- 每条需要应答的命令启动 50 ms 定时器，最多重发 3 次。
- 重发使用相同 seq；固件会重发缓存 ACK，不会重复执行。
- 收到 NACK_SEQ_LOST 时，用 expected 作为新 seq 重发。
- 收到 NACK_BUSY 时不要忙等，先读 STATUS.owner 决定是否 SET_MODE(0) 或等底盘停。
- 心跳不需要应答；即使命令重传，也不要停止心跳。

### 11.3 Pi 侧解析建议

- 先按 SOF/转义规则还原，再校验 LEN 和 CRC；不要把转义后的字节直接当 DATA。
- 处理 0xAA 0x55 重同步；串口噪声可能制造假 SOF。
- 维护接收超时；一帧内超过约 10 ms 没有新字节可以丢弃半帧。
- 对 STATUS 的 pan/tilt 做低通或直接使用；它们是模型值，不是编码器值。
- 不要依赖“到达目标”的异步 ACK；用 STATUS.actual 与目标比较，或轮询 STATUS。

## 12. 固件实现说明

- LinkTask：UART 接收、解码、HEARTBEAT 直通、SET_MODE(3) 优先处理、CRC/格式
  错误上报。
- CommandTask：命令路由、心跳超时检查；每 20 ms 至少处理一次队列。
- GimbalTask/ChassisTask：20 ms 固定周期步进。
- TxTask：从发送队列取帧、编码、阻塞发送（50 ms 超时）。
- TelemetryTask：READY、STATUS、Flash 提交。
- MotionControl 用 FreeRTOS 互斥锁保护仲裁器、云台和底盘状态；上层任务不会
  直接并发修改运动状态。
- 链路参数集中在 Core/Inc/app/app_config.h。

## 13. 已知差距

- GimbalPort 是空实现，尚未接入实际伺服/步进驱动；actual 是模型积分值。
- ChassisPort 默认关闭，因为 MotorB/C/D 尚未实现。
- 急停不操作激光：仓库没有激光驱动。
- pan_zero/tilt_zero 与 Flash 限位只存储/上报，未应用到云台运动模型。
- 云台到达目标没有异步 ACK；只能通过 STATUS 观察。
- Flash 擦除/编程会阻塞 CPU 约秒级；标定提交时底盘必须已停。
- 未启用 IWDG。
- 没有宿主机单元测试环境；当前以目标板编译验证为主。
- 命令队列溢出会计入 STATUS.err_count，但不会逐帧发 ERROR。
- TxQueue 发送失败只内部计数，未计入 STATUS.err_count。
