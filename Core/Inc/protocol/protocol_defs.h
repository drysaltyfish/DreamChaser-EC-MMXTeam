#ifndef PROTOCOL_DEFS_H
#define PROTOCOL_DEFS_H

#include <stdint.h>

/** Wire protocol version implemented by this firmware. */
#define PROTO_VERSION_MAJOR 1u
#define PROTO_VERSION_MINOR 4u

/* Frame delimiters. */
#define PROTO_SOF 0xAAu
#define PROTO_SOF_SECOND 0x55u
#define PROTO_EOF 0x0Du
#define PROTO_ESCAPE_SECOND 0x00u

/* LEN counts CMD plus DATA, so a frame carries at most 63 data bytes.
 * The decoded body buffer also holds the LEN byte itself. */
#define PROTO_LEN_MAX 64u
#define PROTO_DATA_MAX 63u
#define PROTO_BODY_MAX (PROTO_LEN_MAX + 1u)

/* Worst case on the wire: SOF, escaped body (LEN + CMD + DATA), escaped CRC,
 * EOF. */
#define PROTO_WIRE_MAX (2u + 2u * (PROTO_BODY_MAX + 2u) + 1u)

/* Maximum waypoints accepted by one SET_TARGET_STREAM command. */
#define PROTO_STREAM_MAX 10u

/** Command identifiers. Values are part of the wire contract. */
typedef enum {
    PROTO_CMD_SET_TARGET = 0x01,
    PROTO_CMD_SET_DELTA = 0x02,
    PROTO_CMD_SET_VELOCITY = 0x03,
    PROTO_CMD_SET_MODE = 0x04,
    PROTO_CMD_SET_TARGET_STREAM = 0x05,
    PROTO_CMD_HEARTBEAT = 0x10,
    /* 0x11 was CALIBRATE in 1.2; reserved since 1.3. */
    PROTO_CMD_READY = 0x80,
    PROTO_CMD_STATUS = 0x81,
    PROTO_CMD_ACK = 0x82,
    PROTO_CMD_ACK_MOTION = 0x83,
    PROTO_CMD_NACK_BUSY = 0x84,
    PROTO_CMD_NACK_SEQ_LOST = 0x85,
    PROTO_CMD_ERROR = 0x86,
    PROTO_CMD_MOTOR_STATUS = 0x87
} proto_cmd_t;

/** Error codes carried by ERROR frames. */
typedef enum {
    PROTO_ERROR_CRC = 0x01,
    PROTO_ERROR_FRAME = 0x02,
    PROTO_ERROR_STATE = 0x03,
    PROTO_ERROR_HEARTBEAT = 0x04,
    /* 0x05 and 0x06 were CALIBRATION_MISSING and FLASH in 1.2; reserved. */
    PROTO_ERROR_UNKNOWN_CMD = 0x07
} proto_error_t;

/** Ownership of the mutually exclusive motion resource. */
typedef enum {
    PROTO_OWNER_NONE = 0,
    PROTO_OWNER_GIMBAL = 1,
    PROTO_OWNER_CHASSIS = 2,
    PROTO_OWNER_ESTOP_FAULT = 3
} proto_owner_t;

/** Gimbal state reported in STATUS. */
typedef enum {
    PROTO_GIMBAL_IDLE = 0,
    PROTO_GIMBAL_TRACKING = 1,
    PROTO_GIMBAL_RESERVED = 2, /* CALIBRATING in 1.2; never set since 1.3 */
    PROTO_GIMBAL_ERROR = 3,
    PROTO_GIMBAL_ESTOP = 4
} proto_gimbal_state_t;

/** Chassis state reported in STATUS and ACK_MOTION. */
typedef enum {
    PROTO_MOTION_STOPPED = 0,
    PROTO_MOTION_MOVING = 1,
    PROTO_MOTION_TIMEOUT_DECEL = 2,
    PROTO_MOTION_FAULT = 3,
    PROTO_MOTION_ESTOP = 4,
    PROTO_MOTION_BUSY_REJECT = 5
} proto_motion_state_t;

/** SET_MODE argument values. */
typedef enum {
    PROTO_MODE_RELEASE = 0,
    PROTO_MODE_GIMBAL_TRACKING = 1,
    PROTO_MODE_RESERVED = 2, /* calibration mode in 1.2; rejected since 1.3 */
    PROTO_MODE_EMERGENCY_STOP = 3
} proto_mode_t;

/** ACK flag bits. */
#define PROTO_ACK_AT_TARGET 0x01u
#define PROTO_ACK_CLAMPED 0x02u
#define PROTO_ACK_ACCEPTED 0x04u
#define PROTO_ACK_REJECTED 0x08u

/** ACK_MOTION flag bits. */
#define PROTO_ACK_MOTION_ACCEPTED 0x01u
#define PROTO_ACK_MOTION_CLAMPED 0x02u
#define PROTO_ACK_MOTION_STOPPED 0x04u

/** STATUS flag bits. Bit 0 was CAL_VALID in 1.2; reserved in 1.3. */
#define PROTO_STATUS_EMERGENCY 0x02u
#define PROTO_STATUS_HEARTBEAT_OK 0x04u
#define PROTO_STATUS_CHASSIS_STOPPED 0x08u

/** READY flag bits. Bit 2 was CAL_VALID in 1.2 and reserved in 1.3; in
 * 1.4 it announces motor RPM telemetry. */
#define PROTO_READY_HAS_GIMBAL 0x01u
#define PROTO_READY_HAS_CHASSIS 0x02u
#define PROTO_READY_HAS_MOTOR_TELEMETRY 0x04u

/** Number of wheels reported by MOTOR_STATUS, in A/B/C/D order. */
#define PROTO_MOTOR_COUNT 4u

/** Coordinate types accepted by SET_TARGET and SET_TARGET_STREAM. */
#define PROTO_COORD_ANGLE_001DEG 2u

/** SET_DELTA per-axis limit, in 0.01 degree units (0.80 degree). */
#define PROTO_DELTA_LIMIT 80

/** One decoded or to-be-encoded command frame. */
typedef struct {
    uint8_t cmd;
    uint8_t data[PROTO_DATA_MAX];
    uint8_t data_len;
} ProtoFrame;

#endif /* PROTOCOL_DEFS_H */
