# Fusai Python SDK

Fusai STM32 运动协议的宿主侧 Python 客户端，对应协议 v1.4。它面向树莓派或任何带串口的
Linux/Windows 主机，不依赖 STM32 构建，也不会烧录固件或产生标定副作用。

线格式、命令集、仲裁规则和安全行为见 [../docs/protocol.md](../docs/protocol.md)。
本包只镜像这些字节，不发明私有命令。

## 环境要求

- Python 3.9 或更高版本
- [uv](https://docs.astral.sh/uv/)
- pyserial（由 uv 安装）

## 安装与测试

~~~bash
cd sdk
uv sync
uv run pytest
~~~

uv 会创建 `.venv` 和 `uv.lock`。协议编解码、客户端和标定辅助代码不需要串口设备即可导入；
测试使用内存传输。

## 快速开始

~~~python
from fusai_sdk import FusaiClient, Mode

with FusaiClient("/dev/ttyAMA0", baudrate=115200) as robot:
    ready = robot.wait_ready(timeout=5.0)
    print("firmware", ready.version)

    robot.release()               # 仅台架：清除启动时的心跳故障
    robot.track_gimbal()          # SET_MODE(1)，运动权交给云台
    robot.set_target(100, 200)    # 1.00 度 pan，2.00 度 tilt
    status = robot.wait_status(timeout=2.0)
    print(status.pan, status.tilt, status.owner)

    robot.release()               # SET_MODE(0)，清空序号基线
~~~

打开客户端时会立即发送一帧 HEARTBEAT，并默认每 100 ms 启动一次心跳线程。固件在 500 ms
没有心跳后进入故障，所以串口一可用就应创建客户端。只有应用自己发送 `heartbeat()` 时才把
`auto_heartbeat` 设为 `False`。重复打开已打开的客户端是空操作；`close()` 之后再次
`open()` 会重置缓存状态和两个序号基线。

心跳看门狗在上电时就开始计时。如果本进程在 500 ms 之后才打开链路，STM32 可能已经锁存在
`ESTOP_FAULT`；后续心跳只能维持链路，不会自动清除故障。没有真实急停的台架上，应在跟踪前
调用 `release()`（SET_MODE 0）。`release()` 会清除真实急停，因此绝不能在意外停车后自动
调用。

固件在上电后发送一次 READY；如果 Pi 比 STM32 晚启动，首个 HEARTBEAT 会让固件补发一次
READY。若仍然超时，协议本身没有查询命令，应通过读取 STATUS 确认链路是否存活。

## 示例

~~~bash
uv run python examples/basic_gimbal.py /dev/ttyAMA0 --pan 100 --tilt 200
uv run python examples/chassis_velocity.py /dev/ttyAMA0 --vx 300 --duration 3
~~~

底盘示例会经 `SET_VELOCITY` 驱动 MotorA（`vx × 110` RPM 目标，10 ms PID 闭环）。
MotorB/C/D 尚未实现；`APP_CHASSIS_DRIVE_ENABLED=0` 当前只影响 READY 的
`HAS_CHASSIS` 能力位。

## API 概览

~~~python
FusaiClient(
    port=None, *,
    transport=None,
    baudrate=115200,
    read_timeout=0.05,
    heartbeat_interval=0.1,
    auto_heartbeat=True,
    command_timeout=0.3,
    retries=2,
    resync_on_seq_lost=True,
    on_status=None, on_motor_status=None, on_ready=None, on_error=None,
)
~~~

`port` 和 `transport` 必须恰好提供一个。测试可以传入任何带 `open()`、`read(size)`、
`write(data)`、`close()` 的对象。

命令：

| 方法 | 线上命令 | 返回 |
| --- | --- | --- |
| set_target(pan, tilt) | SET_TARGET | Ack |
| set_delta(dpan, dtilt) | SET_DELTA | Ack |
| set_target_stream(points) | SET_TARGET_STREAM | Ack |
| set_velocity(vx, vy, omega) | SET_VELOCITY | AckMotion |
| stop_chassis() | SET_VELOCITY(0,0,0) | AckMotion |
| set_mode(0 或 1) | SET_MODE | Ack |
| release() | SET_MODE(0) | Ack |
| track_gimbal() | SET_MODE(1) | Ack |
| emergency_stop() | SET_MODE(3) | None，无 ACK |
| heartbeat() | HEARTBEAT | None |

`emergency_stop()` 绕过命令锁，因此不会排在正在等待回复的命令后面。它会让挂起命令抛出
`EmergencyStop`，并把两个序号基线都标记为未知；在之后调用 `release()`（SET_MODE 0）之前，
固件会一直停留在 `ESTOP_FAULT`。绕过锁意味着急停帧可能在另一事务等待回复时写入；那个事务
随后会抛出 `EmergencyStop` 或超时。

pan/tilt 是有符号 int16，单位 0.01 度。底盘速度分量是 int16，范围 -1000..1000，由固件
归一化。SET_DELTA 由固件按轴限幅到 ±80（0.80 度）。

状态访问：

- `status`：最新的 Status 或 None
- `motor_status`：最新的 MotorStatus 或 None
- `ready`：最新的 Ready 或 None
- `link_error`：传输错误（如果有）
- `is_link_alive`：最近 2 秒内收到过帧时为真（STATUS 本身只有 1 Hz）
- `wait_ready(timeout)`
- `wait_status(predicate=None, timeout)`
- `wait_motor_status(timeout=5.0)`

回调在读取线程上执行。回调应保持简短，或把工作交给应用队列。回调抛出的异常会被记录，
不会向上传播。

## 序号规则与失败处理

云台命令和底盘速度命令使用独立序号。命令使用下一个序号发送；只有收到被接受的 ACK 或
ACK_MOTION 后 SDK 才推进计数器。忙和拒绝命令保留候选序号，与固件行为一致。

ACK 丢失是有歧义的：固件可能已经执行了命令。SDK 不会静默复用旧序号，以免把新的运动变成
重复帧。命令超时后：

- 后续云台运动抛出 `SequenceUnknownError`。用 `track_gimbal()` 或 `release()` 恢复，
  它们会清空固件序号基线。`track_gimbal()` 只重新同步序号，不会取消已经在动的目标或路径；
  取消云台运动要用 `release()`。
- 后续非零底盘运动抛出 `SequenceUnknownError`。用 `stop_chassis()` 恢复：它用新序号发送
  全零速度，不会被判定为重复帧；也可以用 `release()`。
- 全零速度仍然允许发送，并推进到新序号。

可选的 `resync_on_seq_lost` 行为是安全的，因为 `NACK_SEQ_LOST` 表示固件没有执行该命令；
SDK 采用固件期望的序号并有限重试（2 次）。如果应用想自己处理这个决策，可设
`resync_on_seq_lost=False`。

类型化异常：

- `BusyError`：云台命令被底盘占用或运动状态阻塞
- `SequenceLostError`：固件期望另一个序号
- `SequenceUnknownError`：超时导致序号基线不确定
- `EmergencyStop`：急停中断了当前挂起命令
- `CommandTimeout`：配置的重试次数内没有匹配回复
- `MotionRejected`：速度被 ACK 但未被接受，包括 motion_state=5 BUSY_REJECT
- `DeviceError`：固件 ERROR 帧，含 code 和 detail
- `ProtocolError`：出站载荷格式错误或收到意外回复
- `ReadyTimeout` / `StatusTimeout`：状态等待超时
- `TransportError`：串口打开/读/写失败

请求/回复匹配规则：ACK、ACK_MOTION、NACK_BUSY 用命令和序号匹配；NACK_SEQ_LOST 和 ERROR
只用命令匹配。与当前挂起事务不匹配的迟到重复回复会被忽略。NACK_SEQ_LOST 和 ERROR 在线上
没有序号，因此同一命令的前一事务极晚到达的回复无法与当前事务区分。客户端串行化命令，且
序号恢复会在超时歧义后阻塞运动，从而缩小这个窗口；在高一层重试的应用仍应把超时命令的结果
视为未知。

## 标定与估计

协议 1.4 没有标定命令，也没有 STM32 Flash 持久化。Pi 负责观测、估计、验证和持久化；
STM32 负责实时执行器环路、限位和安全。

SDK 只提供可复用的宿主侧组件：

- `cross_targets(center_pan, center_tilt, pan_amplitude, tilt_amplitude)`：生成九个安全的
  十字激励路点。调用者在当前机构限位内选择幅值。
- `CalibrationSample`：成对的关节向量和观测向量。
- `JacobianEstimator`：用中心化最小二乘拟合“观测增量对关节增量”，给出残差 RMS 和
  Frobenius 条件数估计。

~~~python
from fusai_sdk import JacobianEstimator, cross_targets

points = cross_targets(0, 3000, 800, 800)
robot.set_target_stream(points)

estimator = JacobianEstimator()
# 对每个路点，在云台稳定后观测视觉特征：
# estimator.add((pan, tilt), observed_feature)
result = estimator.solve()
if result.residual_rms < 0.5 and result.condition_number < 1e6:
    publish_new_jacobian(result.matrix)
~~~

矩阵每行对应一个观测分量，每列对应一个关节分量。采样、特征提取、阈值选择、版本管理和
回滚到 last-known-good 雅可比仍属于应用职责。没有编码器的云台，actual 位置是固件模型值；
有意义的估计应使用外部视觉或其他独立传感器。

## 图形测试台

`sdk/tests/gui.py` 是 Windows 下使用的 Tkinter 台架工具，提供底盘 `vx` / `vy` / `wz`
滑块、电机实时转速面板、模式/安全按钮和逐帧收发日志。它复用本 SDK，不需要单独的
虚拟环境：

~~~bash
cd sdk
uv run python tests/gui.py --port COM5 --baud 115200
~~~

安全说明和界面细节见 [tests/README.md](tests/README.md)。

## 协议备注

- SOF 是 AA 55；EOF 是 0D；帧体中的 0xAA 转义为 AA 00。
- CRC 是 CRC-16/MODBUS，覆盖 LEN、CMD、DATA，低字节先发。
- LEN 计入 CMD 和 DATA，范围 1..64。
- SET_MODE(2) 和命令 0x11 保留，固件会拒绝。
- STATUS 以 1 Hz 和状态变化时发送；MOTOR_STATUS 以 10 Hz 发送，二者独立。
- MOTOR_STATUS 的 `present_mask` 只有对应电机存在时才置位；当前固件为 0x01，
  B/C/D 的转速字段为 0。
- 云台限位在 STM32 的 app_config.h 中，不在本 SDK 里。不要在宿主代码里硬编码另一份
  构建的限位。

## 开发

~~~bash
uv run pytest -q
uv run python -m compileall -q src tests
~~~

测试覆盖公开参考帧、转义与重同步、CRC 报警行为、消息布局、序号恢复、类型化 NACK 处理、
fire-and-forget 急停、状态/电机遥测回调以及最小二乘辅助函数。
