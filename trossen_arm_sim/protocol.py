"""Wire protocol of the Trossen arm controller (libtrossen_arm v1.10).

The driver talks to the arm controller over two channels:

- TCP port 50001: configuration channel. Each message is a uint16
  little-endian length prefix followed by the payload. The payload starts
  with a command indicator byte.
- UDP port 50000: realtime channel. Each datagram is a raw payload starting
  with a command indicator byte.

All multi-byte values are little-endian. Angles are in rad, gripper
positions in m, efforts in Nm (arm) or N (gripper).
"""

import struct
from dataclasses import dataclass
from enum import IntEnum


TCP_PORT = 50001
UDP_PORT = 50000

DRIVER_VERSION = (1, 10, 0)

NUM_JOINTS = 7  # 6 arm joints + gripper (wxai_v0)


class TCPCommand(IntEnum):
    HANDSHAKE = 0
    SET_HOME = 1
    SET_CONFIGURATION = 2
    GET_CONFIGURATION = 3
    GET_LOG = 4
    UPDATE_DEFAULT_EEPROM = 5
    REBOOT = 6


class UDPCommand(IntEnum):
    SET_ROBOT_INPUT = 0
    GET_ROBOT_OUTPUT = 1


class Mode(IntEnum):
    IDLE = 0
    POSITION = 1
    VELOCITY = 2
    EXTERNAL_EFFORT = 3
    EFFORT = 4


class ConfigurationAddress(IntEnum):
    FACTORY_RESET_FLAG = 0
    IP_METHOD = 1
    MANUAL_IP = 2
    DNS = 3
    GATEWAY = 4
    SUBNET = 5
    JOINT_CHARACTERISTICS = 6
    ERROR_STATE = 7
    MODES = 8
    END_EFFECTOR = 9
    JOINT_LIMITS = 10
    MOTOR_PARAMETERS = 11


class ErrorState(IntEnum):
    """Error states of the arm controller, as reported to the driver."""
    NONE = 0
    ETHERNET_INIT_FAILED = 1
    CAN_INIT_FAILED = 2
    JOINT_COMMAND_FAILED = 3
    JOINT_FEEDBACK_FAILED = 4
    JOINT_CLEAR_ERROR_FAILED = 5
    JOINT_ENABLE_FAILED = 6
    JOINT_DISABLE_FAILED = 7
    JOINT_SET_HOME_FAILED = 8
    JOINT_DISABLED_UNEXPECTEDLY = 9
    JOINT_OVERHEATED = 10
    INVALID_MODE = 11
    INVALID_ROBOT_COMMAND = 12
    INVALID_CONFIGURATION_ADDRESS = 13
    ROBOT_INPUT_MODE_MISMATCH = 14
    JOINT_LIMIT_EXCEEDED = 15
    ROBOT_INPUT_INFINITE = 16


class Model(IntEnum):
    WXAI_V0 = 0
    VXAI_V0_RIGHT = 1
    VXAI_V0_LEFT = 2


# struct JointInputRaw { uint8 mode; <pad 3>; union { float a, b, c; }; }
JOINT_INPUT_STRUCT = struct.Struct("<B3x3f")
# struct JointOutputRaw { float position, velocity, effort, external_effort,
#                         rotor_temperature, driver_temperature; }
JOINT_OUTPUT_STRUCT = struct.Struct("<6f")
# struct HeaderRaw { uint32 id; <pad 4>; uint64 timestamp_us; }
OUTPUT_HEADER_STRUCT = struct.Struct("<I4xQ")
# struct JointLimitRaw { float position_min, position_max, position_tolerance,
#                        velocity_max, velocity_tolerance, effort_max,
#                        effort_tolerance; }
JOINT_LIMIT_STRUCT = struct.Struct("<7f")
# struct JointCharacteristicRaw { float effort_correction,
#     friction_transition_velocity, friction_constant_term,
#     friction_coulomb_coef, friction_viscous_coef, position_offset; }
JOINT_CHARACTERISTIC_STRUCT = struct.Struct("<6f")

# struct LinkRaw { float mass; float inertia[9]; float origin_xyz[3];
#                  float origin_rpy[3]; } -> 16 floats
# struct EndEffectorRaw { LinkRaw palm, finger_left, finger_right;
#                         float offset_finger_left, offset_finger_right,
#                         pitch_circle_radius; }
END_EFFECTOR_SIZE = 3 * 16 * 4 + 3 * 4


@dataclass
class JointInput:
    mode: Mode
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0

    @classmethod
    def parse(cls, data: bytes) -> "JointInput":
        mode, a, b, c = JOINT_INPUT_STRUCT.unpack(data)
        mode = Mode(mode)
        if mode == Mode.POSITION:
            return cls(mode, position=a, velocity=b)
        if mode == Mode.VELOCITY:
            return cls(mode, velocity=a)
        if mode in (Mode.EXTERNAL_EFFORT, Mode.EFFORT):
            return cls(mode, effort=a)
        return cls(mode)


@dataclass
class JointOutput:
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    external_effort: float = 0.0
    rotor_temperature: float = 40.0
    driver_temperature: float = 40.0

    def pack(self) -> bytes:
        return JOINT_OUTPUT_STRUCT.pack(
            self.position, self.velocity, self.effort,
            self.external_effort, self.rotor_temperature,
            self.driver_temperature,
        )


def parse_robot_input(payload: bytes, num_joints: int = NUM_JOINTS) -> list[JointInput]:
    """Parse the body of a SET_ROBOT_INPUT datagram (after the command byte)."""
    size = JOINT_INPUT_STRUCT.size
    assert len(payload) == num_joints * size, \
        f"expected {num_joints * size} bytes of joint inputs, got {len(payload)}"
    return [
        JointInput.parse(payload[i * size:(i + 1) * size])
        for i in range(num_joints)
    ]


def pack_robot_output(error_state: ErrorState, output_id: int, timestamp_us: int,
                      joint_outputs: list[JointOutput]) -> bytes:
    """Pack a robot output datagram: [error_state][header][joint outputs]."""
    return (
        bytes([error_state])
        + OUTPUT_HEADER_STRUCT.pack(output_id, timestamp_us)
        + b"".join(out.pack() for out in joint_outputs)
    )


def pack_handshake_response(error_state: ErrorState, model: Model) -> bytes:
    """[error_state][model][fw_major][fw_minor][fw_patch]

    The driver requires the firmware major and minor versions to match its
    own (1.10.x).
    """
    major, minor, patch = DRIVER_VERSION
    return bytes([error_state, model, major, minor, patch])
