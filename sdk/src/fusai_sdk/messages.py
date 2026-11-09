"""Payload encoders, decoders, and small value objects.

The frame layer is in fusai_sdk.framing; this module owns the bytes inside a
frame.  All multi-byte integers are little-endian, matching the STM32
implementation.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any, Sequence, Tuple, Union

from .constants import (
    AckFlags,
    AckMotionFlags,
    Command,
    CoordType,
    GimbalState,
    MAX_DATA_LEN,
    MAX_STREAM_POINTS,
    Mode,
    MotionState,
    Owner,
    ReadyFlags,
    StatusFlags,
)
from .exceptions import ProtocolError
from .framing import Frame


@dataclass(frozen=True)
class Ack:
    command: int
    seq: int
    flags: int

    @property
    def accepted(self) -> bool:
        return bool(self.flags & AckFlags.ACCEPTED)

    @property
    def rejected(self) -> bool:
        return bool(self.flags & AckFlags.REJECTED)

    @property
    def at_target(self) -> bool:
        return bool(self.flags & AckFlags.AT_TARGET)

    @property
    def clamped(self) -> bool:
        return bool(self.flags & AckFlags.CLAMPED)


@dataclass(frozen=True)
class AckMotion:
    seq: int
    motion_state: int
    flags: int

    @property
    def accepted(self) -> bool:
        return bool(self.flags & AckMotionFlags.ACCEPTED)

    @property
    def clamped(self) -> bool:
        return bool(self.flags & AckMotionFlags.CLAMPED)

    @property
    def stopped(self) -> bool:
        return bool(self.flags & AckMotionFlags.STOPPED)

    @property
    def busy_rejected(self) -> bool:
        return self.motion_state == int(MotionState.BUSY_REJECT)


@dataclass(frozen=True)
class NackBusy:
    command: int
    seq: int
    owner: int


@dataclass(frozen=True)
class NackSeqLost:
    command: int
    expected_seq: int


@dataclass(frozen=True)
class ErrorEvent:
    code: int
    detail: int


@dataclass(frozen=True)
class Status:
    gimbal_state: int
    owner: int
    motion_state: int
    pan: int
    tilt: int
    error_count: int
    flags: int

    @property
    def emergency(self) -> bool:
        return bool(self.flags & StatusFlags.EMERGENCY)

    @property
    def heartbeat_ok(self) -> bool:
        return bool(self.flags & StatusFlags.HEARTBEAT_OK)

    @property
    def chassis_stopped(self) -> bool:
        return bool(self.flags & StatusFlags.CHASSIS_STOPPED)

    @property
    def owner_name(self) -> str:
        return _enum_label(Owner, self.owner)

    @property
    def gimbal_state_name(self) -> str:
        return _enum_label(GimbalState, self.gimbal_state)

    @property
    def motion_state_name(self) -> str:
        return _enum_label(MotionState, self.motion_state)


@dataclass(frozen=True)
class Ready:
    version_major: int
    version_minor: int
    flags: int

    @property
    def version(self) -> Tuple[int, int]:
        return (self.version_major, self.version_minor)

    @property
    def has_gimbal(self) -> bool:
        return bool(self.flags & ReadyFlags.HAS_GIMBAL)

    @property
    def has_chassis(self) -> bool:
        return bool(self.flags & ReadyFlags.HAS_CHASSIS)


Inbound = Union[Ack, AckMotion, NackBusy, NackSeqLost, ErrorEvent, Status, Ready, Frame]


def _enum_label(enum_type: Any, value: int) -> str:
    try:
        return enum_type(value).name
    except ValueError:
        return str(value)


def _u8(value: int, name: str) -> int:
    if not isinstance(value, int) or not 0 <= value <= 0xFF:
        raise ValueError("%s must be an integer in 0..255" % name)
    return value


def _i16(value: int, name: str) -> int:
    if not isinstance(value, int) or not -0x8000 <= value <= 0x7FFF:
        raise ValueError("%s must be an int16" % name)
    return value


def _require_len(data: bytes, expected: int, command: str) -> None:
    if len(data) != expected:
        raise ProtocolError(
            "%s payload must be %d bytes, got %d" % (command, expected, len(data))
        )


def encode_set_target(seq: int, pan: int, tilt: int) -> bytes:
    return struct.pack(
        "<BBhh",
        _u8(seq, "seq"),
        int(CoordType.GIMBAL_ANGLE),
        _i16(pan, "pan"),
        _i16(tilt, "tilt"),
    )


def encode_set_delta(seq: int, dpan: int, dtilt: int) -> bytes:
    return struct.pack("<Bhh", _u8(seq, "seq"), _i16(dpan, "dpan"), _i16(dtilt, "dtilt"))


def encode_set_velocity(seq: int, vx: int, vy: int, omega: int) -> bytes:
    return struct.pack(
        "<Bhhh",
        _u8(seq, "seq"),
        _i16(vx, "vx"),
        _i16(vy, "vy"),
        _i16(omega, "omega"),
    )


def encode_set_mode(mode: int) -> bytes:
    value = int(mode)
    if value not in (int(Mode.RELEASE), int(Mode.TRACK_GIMBAL), int(Mode.EMERGENCY_STOP)):
        raise ValueError("mode must be 0, 1, or 3 in protocol v1.3")
    return struct.pack("<B", value)


def encode_set_target_stream(seq: int, points: Sequence[Tuple[int, int]]) -> bytes:
    point_list = list(points)
    if not 1 <= len(point_list) <= MAX_STREAM_POINTS:
        raise ValueError("stream must contain 1..%d points" % MAX_STREAM_POINTS)

    data = bytearray(
        struct.pack(
            "<BBB",
            _u8(seq, "seq"),
            int(CoordType.GIMBAL_ANGLE),
            len(point_list),
        )
    )
    for index, point in enumerate(point_list):
        if len(point) != 2:
            raise ValueError("point %d must be (pan, tilt)" % index)
        data.extend(struct.pack("<hh", _i16(point[0], "pan"), _i16(point[1], "tilt")))
    if len(data) > MAX_DATA_LEN:
        raise ValueError("stream payload is too long")
    return bytes(data)


def encode_heartbeat() -> bytes:
    return b""


def decode_ack(data: bytes) -> Ack:
    _require_len(data, 3, "ACK")
    command, seq, flags = struct.unpack("<BBB", data)
    return Ack(command=command, seq=seq, flags=flags)


def decode_ack_motion(data: bytes) -> AckMotion:
    _require_len(data, 3, "ACK_MOTION")
    seq, motion_state, flags = struct.unpack("<BBB", data)
    return AckMotion(seq=seq, motion_state=motion_state, flags=flags)


def decode_nack_busy(data: bytes) -> NackBusy:
    _require_len(data, 3, "NACK_BUSY")
    command, seq, owner = struct.unpack("<BBB", data)
    return NackBusy(command=command, seq=seq, owner=owner)


def decode_nack_seq_lost(data: bytes) -> NackSeqLost:
    _require_len(data, 2, "NACK_SEQ_LOST")
    command, expected_seq = struct.unpack("<BB", data)
    return NackSeqLost(command=command, expected_seq=expected_seq)


def decode_error(data: bytes) -> ErrorEvent:
    _require_len(data, 2, "ERROR")
    code, detail = struct.unpack("<BB", data)
    return ErrorEvent(code=code, detail=detail)


def decode_status(data: bytes) -> Status:
    _require_len(data, 10, "STATUS")
    gimbal_state, owner, motion_state, pan, tilt, error_count, flags = struct.unpack(
        "<BBBhhHB", data
    )
    return Status(
        gimbal_state=gimbal_state,
        owner=owner,
        motion_state=motion_state,
        pan=pan,
        tilt=tilt,
        error_count=error_count,
        flags=flags,
    )


def decode_ready(data: bytes) -> Ready:
    _require_len(data, 3, "READY")
    major, minor, flags = struct.unpack("<BBB", data)
    return Ready(version_major=major, version_minor=minor, flags=flags)


def parse_inbound(frame: Frame) -> Inbound:
    """Decode a known inbound frame; return the frame for unknown commands."""

    command = frame.cmd
    if command == int(Command.ACK):
        return decode_ack(frame.data)
    if command == int(Command.ACK_MOTION):
        return decode_ack_motion(frame.data)
    if command == int(Command.NACK_BUSY):
        return decode_nack_busy(frame.data)
    if command == int(Command.NACK_SEQ_LOST):
        return decode_nack_seq_lost(frame.data)
    if command == int(Command.ERROR):
        return decode_error(frame.data)
    if command == int(Command.STATUS):
        return decode_status(frame.data)
    if command == int(Command.READY):
        return decode_ready(frame.data)
    return frame
