"""TCP/UDP server emulating a Trossen arm controller backed by MuJoCo."""

import logging
import socket
import struct
import threading
import time

from .protocol import (
    END_EFFECTOR_SIZE,
    JOINT_CHARACTERISTIC_STRUCT,
    JOINT_LIMIT_STRUCT,
    TCP_PORT,
    UDP_PORT,
    ConfigurationAddress,
    ErrorState,
    Mode,
    Model,
    TCPCommand,
    UDPCommand,
    pack_handshake_response,
    pack_robot_output,
    parse_robot_input,
)
from .sim import WidowXSim

logger = logging.getLogger("trossen_arm_sim")


class TrossenArmSimServer:
    """Emulates the arm controller's network interface.

    TCP (port 50001) carries handshake and configuration commands, framed
    with a uint16 little-endian length prefix. UDP (port 50000) carries the
    realtime joint input/output cycle: every SET_ROBOT_INPUT datagram is
    answered with one robot output datagram sent back to its source.
    """

    # Minimum realtime cycle period. Keeps the driver's daemon loop from
    # spinning unboundedly while staying far below the driver's 1 ms UDP
    # receive timeout; a slower pace would back up stale responses.
    CYCLE_PERIOD_S = 0.0002

    def __init__(self, sim: WidowXSim, host: str = "0.0.0.0",
                 model: Model = Model.WXAI_V0):
        self.sim = sim
        self.host = host
        self.model = model
        self.num_joints = sim.num_joints
        self.modes = [Mode.IDLE] * self.num_joints
        self.end_effector = bytes(END_EFFECTOR_SIZE)
        self._start_time = time.monotonic()
        self._output_id = 0
        self._last_cycle = 0.0
        self._running = False
        self._threads: list[threading.Thread] = []
        self._tcp_socket: socket.socket | None = None
        self._udp_socket: socket.socket | None = None

    def start(self) -> None:
        self._running = True
        self._tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._tcp_socket.bind((self.host, TCP_PORT))
        self._tcp_socket.listen(1)

        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_socket.bind((self.host, UDP_PORT))

        for target, name in ((self._tcp_loop, "tcp-server"),
                             (self._udp_loop, "udp-server")):
            thread = threading.Thread(target=target, daemon=True, name=name)
            thread.start()
            self._threads.append(thread)
        logger.info("Listening on %s (TCP %d, UDP %d)",
                    self.host, TCP_PORT, UDP_PORT)

    def stop(self) -> None:
        self._running = False
        for sock in (self._tcp_socket, self._udp_socket):
            if sock is not None:
                sock.close()
        self._tcp_socket = None
        self._udp_socket = None

    # ------------------------------------------------------------------ TCP

    def _tcp_loop(self) -> None:
        while self._running:
            try:
                conn, addr = self._tcp_socket.accept()
            except OSError:
                return  # socket closed by stop()
            logger.info("TCP client connected: %s:%d", *addr)
            try:
                self._serve_tcp_client(conn)
            except ConnectionError as exc:
                logger.info("TCP client error: %s", exc)
            finally:
                conn.close()
                logger.info("TCP client disconnected")

    def _serve_tcp_client(self, conn: socket.socket) -> None:
        while self._running:
            header = self._recv_exact(conn, 2)
            if header is None:
                return
            (length,) = struct.unpack("<H", header)
            request = self._recv_exact(conn, length)
            if request is None:
                return
            response = self._handle_tcp_request(request)
            if response is not None:
                conn.sendall(struct.pack("<H", len(response)) + response)
            if request[0] == TCPCommand.REBOOT:
                return

    @staticmethod
    def _recv_exact(conn: socket.socket, size: int) -> bytes | None:
        data = b""
        while len(data) < size:
            chunk = conn.recv(size - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _handle_tcp_request(self, request: bytes) -> bytes:
        command = TCPCommand(request[0])
        logger.debug("TCP request: %s (%d bytes)", command.name, len(request))

        if command == TCPCommand.HANDSHAKE:
            ip = socket.inet_ntoa(request[1:5])
            (udp_port,) = struct.unpack("<H", request[5:7])
            logger.info("Handshake: driver UDP endpoint %s:%d", ip, udp_port)
            return pack_handshake_response(self.model)

        if command == TCPCommand.SET_CONFIGURATION:
            self._apply_configuration(ConfigurationAddress(request[1]), request[2:])
            return bytes([ErrorState.NONE])

        if command == TCPCommand.GET_CONFIGURATION:
            address = ConfigurationAddress(request[1])
            return bytes([ErrorState.NONE]) + self._read_configuration(address)

        if command == TCPCommand.GET_LOG:
            return bytes([ErrorState.NONE]) + b"simulation controller: no log entries"

        # SET_HOME, UPDATE_DEFAULT_EEPROM, REBOOT: acknowledge without effect.
        return bytes([ErrorState.NONE])

    def _apply_configuration(self, address: ConfigurationAddress,
                             payload: bytes) -> None:
        logger.info("set_configuration(%s, %d bytes)", address.name, len(payload))
        if address == ConfigurationAddress.MODES:
            self.modes = [Mode(b) for b in payload[:self.num_joints]]
            logger.info("Modes set to %s", [m.name for m in self.modes])
        elif address == ConfigurationAddress.END_EFFECTOR:
            self.end_effector = payload
        # Network settings, joint characteristics, limits, motor parameters
        # and error-state resets have no effect on the simulation.

    def _read_configuration(self, address: ConfigurationAddress) -> bytes:
        logger.info("get_configuration(%s)", address.name)
        if address == ConfigurationAddress.FACTORY_RESET_FLAG:
            return bytes([0])
        if address == ConfigurationAddress.IP_METHOD:
            return bytes([0])  # manual
        if address in (ConfigurationAddress.MANUAL_IP, ConfigurationAddress.DNS,
                       ConfigurationAddress.GATEWAY, ConfigurationAddress.SUBNET):
            return socket.inet_aton({
                ConfigurationAddress.MANUAL_IP: "192.168.1.2",
                ConfigurationAddress.DNS: "8.8.8.8",
                ConfigurationAddress.GATEWAY: "192.168.1.1",
                ConfigurationAddress.SUBNET: "255.255.255.0",
            }[address])
        if address == ConfigurationAddress.JOINT_CHARACTERISTICS:
            characteristic = JOINT_CHARACTERISTIC_STRUCT.pack(
                1.0, 0.1, 0.0, 0.0, 0.0, 0.0)
            return characteristic * self.num_joints
        if address == ConfigurationAddress.ERROR_STATE:
            return bytes([ErrorState.NONE])
        if address == ConfigurationAddress.MODES:
            return bytes(self.modes)
        if address == ConfigurationAddress.END_EFFECTOR:
            return self.end_effector
        if address == ConfigurationAddress.JOINT_LIMITS:
            limits = b""
            for low, high in self.sim.joint_limits:
                is_gripper = len(limits) // JOINT_LIMIT_STRUCT.size == self.num_joints - 1
                velocity_max = 0.5 if is_gripper else 3.14
                effort_max = 100.0 if is_gripper else 27.0
                limits += JOINT_LIMIT_STRUCT.pack(
                    low, high, 0.01, velocity_max, 0.1, effort_max, 1.0)
            return limits
        assert address == ConfigurationAddress.MOTOR_PARAMETERS
        raise NotImplementedError("motor parameters are not simulated")

    # ------------------------------------------------------------------ UDP

    def _udp_loop(self) -> None:
        while self._running:
            try:
                datagram, addr = self._udp_socket.recvfrom(4096)
                # Drain any backlog and answer only the newest request; the
                # driver has already timed out on the older ones and a queued
                # backlog would make it read stale outputs forever.
                self._udp_socket.setblocking(False)
                try:
                    while True:
                        datagram, addr = self._udp_socket.recvfrom(4096)
                except BlockingIOError:
                    pass
                finally:
                    self._udp_socket.setblocking(True)
            except OSError:
                return  # socket closed by stop()
            command = UDPCommand(datagram[0])
            if command == UDPCommand.SET_ROBOT_INPUT:
                inputs = parse_robot_input(datagram[1:], self.num_joints)
                self.sim.apply_joint_commands(
                    [joint.mode for joint in inputs],
                    [joint.position for joint in inputs],
                )
            self._udp_socket.sendto(self._pack_output(), addr)
            self._pace_cycle()

    def _pace_cycle(self) -> None:
        now = time.monotonic()
        delay = self._last_cycle + self.CYCLE_PERIOD_S - now
        if delay > 0:
            time.sleep(delay)
        self._last_cycle = time.monotonic()

    def _pack_output(self) -> bytes:
        self._output_id += 1
        timestamp_us = int((time.monotonic() - self._start_time) * 1e6)
        return pack_robot_output(self._output_id, timestamp_us,
                                 self.sim.read_joint_outputs())
