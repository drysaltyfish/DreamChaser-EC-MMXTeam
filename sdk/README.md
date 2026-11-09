# Fusai Python SDK

Host-side Python client for the Fusai STM32 motion protocol, version 1.3.
It is intended for a Raspberry Pi or any Linux/Windows host with a serial
port.  The package has no dependency on the STM32 build and no firmware
flashing or calibration side effects.

The wire format, command set, arbitration rules, and safety behavior are
documented in ../docs/protocol.md.  This package mirrors those bytes; it does
not invent private commands.

## Requirements

- Python 3.9 or newer
- uv (https://docs.astral.sh/uv/)
- pyserial, installed by uv

## Install and test

~~~bash
cd sdk
uv sync
uv run pytest
~~~

uv creates .venv and uv.lock.  The protocol codec, client, and calibration
helpers are importable without a serial device; tests use an in-memory
transport.

## Quick start

~~~python
from fusai_sdk import FusaiClient, Mode

with FusaiClient("/dev/ttyAMA0", baudrate=115200) as robot:
    ready = robot.wait_ready(timeout=5.0)
    print("firmware", ready.version)

    robot.release()               # bench only: clear a boot-time heartbeat fault
    robot.track_gimbal()          # SET_MODE(1), owner becomes gimbal
    robot.set_target(100, 200)    # 1.00 deg pan, 2.00 deg tilt
    status = robot.wait_status(timeout=2.0)
    print(status.pan, status.tilt, status.owner)

    robot.release()               # SET_MODE(0), clear sequence baselines
~~~

Opening the client sends one HEARTBEAT immediately and starts a heartbeat
thread every 100 ms by default.  The firmware faults after 500 ms without a
heartbeat, so create the client as soon as the serial link is available.
Set auto_heartbeat=False only if the application sends heartbeat() itself.
Opening an already opened client is a no-op; after close(), a new open()
resets cached state and both sequence baselines.

The heartbeat watchdog is armed at boot.  If this process opens the link after
that window, the STM32 is already latched in ESTOP_FAULT; incoming heartbeats
keep it alive but do not clear the fault.  On a bench with no real emergency,
call release() (SET_MODE 0) before tracking.  release() clears a real
emergency too, so never call it automatically after an unexpected stop.

The firmware sends READY exactly once after boot.  If the Pi starts after the
STM32, wait_ready() will time out; this is not a protocol failure.  Reading
STATUS is the normal way to confirm a live link at any time.

## Examples

~~~bash
uv run python examples/basic_gimbal.py /dev/ttyAMA0 --pan 100 --tilt 200
uv run python examples/chassis_velocity.py /dev/ttyAMA0 --vx 300 --duration 3
~~~

The chassis example exercises the protocol path; the default firmware build
has APP_CHASSIS_DRIVE_ENABLED=0, so it does not move motors until that port is
implemented and enabled.

## API overview

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
    on_status=None, on_ready=None, on_error=None,
)
~~~

Provide exactly one of port or transport.  Tests can pass any object with
open(), read(size), write(data), and close().

Commands:

| Method | Wire command | Result |
| --- | --- | --- |
| set_target(pan, tilt) | SET_TARGET | Ack |
| set_delta(dpan, dtilt) | SET_DELTA | Ack |
| set_target_stream(points) | SET_TARGET_STREAM | Ack |
| set_velocity(vx, vy, omega) | SET_VELOCITY | AckMotion |
| stop_chassis() | SET_VELOCITY(0,0,0) | AckMotion |
| set_mode(0 or 1) | SET_MODE | Ack |
| release() | SET_MODE(0) | Ack |
| track_gimbal() | SET_MODE(1) | Ack |
| emergency_stop() | SET_MODE(3) | None, no ACK |
| heartbeat() | HEARTBEAT | None |

emergency_stop() bypasses the command lock so it cannot wait behind an
in-flight command.  It fails a pending command with EmergencyStop and marks
both sequence baselines unknown; the firmware stays in ESTOP_FAULT until a
later release() (SET_MODE 0).  Bypassing the lock means the emergency frame
may be written while another transaction is waiting for its reply; that
transaction then raises EmergencyStop or times out.

Pan and tilt are signed int16 in 0.01 degree units.  Chassis velocity
components are int16 in -1000..1000 and are normalized by the firmware.
SET_DELTA is clamped by the firmware per axis to +/-80 (0.80 degree).

State access:

- status: latest Status or None
- ready: latest Ready or None
- link_error: transport failure, if any
- is_link_alive: true when a frame arrived within the last 2 seconds
  (STATUS itself is only sent at 1 Hz)
- wait_ready(timeout)
- wait_status(predicate=None, timeout)

Callbacks run on the reader thread.  Keep them short or hand work to an
application queue.  Exceptions raised by callbacks are logged, not propagated.

## Sequence rules and failure handling

Gimbal commands and chassis velocity commands have independent sequence
counters.  A command is sent with the next sequence; the SDK advances the
counter only after an accepted ACK or an ACK_MOTION.  Busy and rejected
commands keep the candidate sequence, matching the firmware.

A lost ACK is ambiguous: the firmware may have executed the command.  The SDK
does not silently reuse the old sequence in a way that could turn a new
motion into a duplicate.  After a command timeout:

- further gimbal motion raises SequenceUnknownError.  Recover with
  track_gimbal() or release(), which clear the firmware sequence baseline.
  track_gimbal() only resyncs the sequence; it does not cancel a target or
  path that is already moving.  Use release() to cancel gimbal motion.
- further nonzero chassis motion raises SequenceUnknownError.  Recover with
  stop_chassis(), which sends an all-zero velocity with a fresh sequence and
  is not classified as a duplicate, or with release().
- an all-zero velocity is still allowed and advances to a fresh sequence.

The optional resync_on_seq_lost behavior is safe because NACK_SEQ_LOST means
the firmware did not execute the command; the SDK adopts the expected
sequence and retries a bounded number of times (2).  Disable it with
resync_on_seq_lost=False if the application wants to handle that decision.

Typed exceptions:

- BusyError: gimbal command blocked by chassis ownership or motion
- SequenceLostError: firmware expected another sequence
- SequenceUnknownError: a timeout left the sequence baseline ambiguous
- EmergencyStop: emergency stop interrupted this pending command
- CommandTimeout: no matching reply after the configured retries
- MotionRejected: velocity was acknowledged but not accepted, including
  motion_state=5 BUSY_REJECT
- DeviceError: firmware ERROR frame, with code and detail
- ProtocolError: malformed outbound payload or unexpected reply
- ReadyTimeout / StatusTimeout: state-wait deadlines
- TransportError: serial open/read/write failure

Request/response matching uses command and sequence for ACK, ACK_MOTION, and
NACK_BUSY, and command for NACK_SEQ_LOST and ERROR.  A late duplicate reply
that does not match the current pending transaction is ignored.  NACK_SEQ_LOST
and ERROR carry no sequence on the wire, so a very late reply from a previous
transaction of the same command cannot be distinguished from the current one.
The client serializes commands, and sequence recovery blocks motion after the
ambiguous timeout, which keeps this window narrow; applications that retry at
a higher level should still treat the result of a timed-out command as unknown.

## Calibration and estimation

Protocol 1.3 has no calibration command and no STM32 Flash persistence.  The
Pi owns observation, estimation, validation, and persistence; the STM32 owns
the real-time actuator loop, limits, and safety.

The SDK provides only reusable host-side pieces:

- cross_targets(center_pan, center_tilt, pan_amplitude, tilt_amplitude):
  nine safe waypoints for an initial cross excitation.  The caller chooses
  amplitudes within the current mechanism limits.
- CalibrationSample: paired joint and observation vectors.
- JacobianEstimator: centered least-squares fit of observation delta against
  joint delta, with residual RMS and a Frobenius condition estimate.

~~~python
from fusai_sdk import JacobianEstimator, cross_targets

points = cross_targets(0, 3000, 800, 800)
robot.set_target_stream(points)

estimator = JacobianEstimator()
# for each point, observe a vision feature after the gimbal settles:
# estimator.add((pan, tilt), observed_feature)
result = estimator.solve()
if result.residual_rms < 0.5 and result.condition_number < 1e6:
    publish_new_jacobian(result.matrix)
~~~

The matrix has one row per observation component and one column per joint
component.  Sampling, feature extraction, threshold choice, versioning, and
atomic rollback to a last-known-good Jacobian remain application concerns.
For a gimbal without encoders, actual position is the firmware model value;
use external vision or another independent sensor for meaningful estimation.

## Protocol notes

- SOF is AA 55; EOF is 0D; body 0xAA is escaped as AA 00.
- CRC is CRC-16/MODBUS over LEN, CMD, and DATA, sent low byte first.
- LEN counts CMD plus DATA and is 1..64.
- SET_MODE(2) and command 0x11 are reserved in 1.3 and rejected.
- STATUS is sent at 1 Hz and on state changes, not on every pan/tilt update.
- The gimbal limits live in the STM32 app_config.h, not in this SDK.  Do not
  hardcode a different build's limits in host code.

## Development

~~~bash
uv run pytest -q
uv run python -m compileall -q src tests
~~~

The tests cover the published reference frames, escaping and resynchronizing,
CRC alarm behavior, message layouts, sequence recovery, typed NACK handling,
fire-and-forget emergency stop, callbacks, and the least-squares helper.
