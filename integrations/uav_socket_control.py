from __future__ import annotations

import argparse
import socket
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional


DEFAULT_MAGIC = 0x55AA
HEADER_SIZE = 6
CHECKSUM_SIZE = 2
MAX_PAYLOAD_LEN = 0xFFFF


class ProtocolError(ValueError):
    """Raised when an incoming frame does not match the protocol."""


class MessageType(IntEnum):
    UNLOCK = 0x01
    TAKEOFF = 0x02
    SET_MODE = 0x03
    RETURN_HOME = 0x04

    UNLOCK_ACK = 0x11
    TAKEOFF_ACK = 0x12
    SET_MODE_ACK = 0x13
    RETURN_HOME_ACK = 0x14

    STATUS_REPORT = 0x21


class UAVMode(IntEnum):
    STABILIZE = 0
    AUTO = 3
    FOLLOW = 23


ChecksumFunc = Callable[[bytes], int]


def sum16_checksum(data: bytes) -> int:
    """
    Default checksum implementation.

    The docx defines a 2-byte checksum field but does not define the algorithm.
    A simple 16-bit additive checksum is used by default and can be replaced
    later by passing a different checksum function to UAVProtocolCodec.
    """

    return sum(data) & 0xFFFF


@dataclass(frozen=True)
class UAVFrame:
    msg_type: int
    uav_id: int
    payload: bytes = b""

    def __post_init__(self) -> None:
        if not 0 <= int(self.msg_type) <= 0xFF:
            raise ValueError(f"msg_type out of range: {self.msg_type!r}")
        if not 0 <= int(self.uav_id) <= 0xFF:
            raise ValueError(f"uav_id out of range: {self.uav_id!r}")

        payload = bytes(self.payload)
        if len(payload) > MAX_PAYLOAD_LEN:
            raise ValueError(f"payload too large: {len(payload)} bytes")

        object.__setattr__(self, "msg_type", int(self.msg_type))
        object.__setattr__(self, "uav_id", int(self.uav_id))
        object.__setattr__(self, "payload", payload)

    @property
    def payload_len(self) -> int:
        return len(self.payload)


@dataclass(frozen=True)
class UnlockResult:
    success: bool
    uav_id: int
    attempts: int
    ack_state: Optional[int]
    response_frame: Optional[UAVFrame]
    detail: str

    def __bool__(self) -> bool:
        return self.success


@dataclass(frozen=True)
class StatusReport:
    """
    Placeholder for later status parsing.

    The current document only says the payload contains status + position +
    velocity but does not define the concrete binary layout, so the raw payload
    is preserved for the next stage.
    """

    uav_id: int
    raw_payload: bytes

    @classmethod
    def from_frame(cls, frame: UAVFrame) -> "StatusReport":
        if frame.msg_type != MessageType.STATUS_REPORT:
            raise ProtocolError(
                f"expected status report frame, got msg_type=0x{frame.msg_type:02X}"
            )
        return cls(uav_id=frame.uav_id, raw_payload=frame.payload)


class UAVProtocolCodec:
    def __init__(
        self,
        magic: int = DEFAULT_MAGIC,
        byteorder: str = "big",
        checksum_func: ChecksumFunc = sum16_checksum,
    ) -> None:
        if byteorder not in ("big", "little"):
            raise ValueError("byteorder must be 'big' or 'little'")
        self.magic = int(magic) & 0xFFFF
        self.byteorder = byteorder
        self.checksum_func = checksum_func

    def encode(self, frame: UAVFrame) -> bytes:
        payload = frame.payload
        body = (
            self.magic.to_bytes(2, self.byteorder)
            + bytes((frame.msg_type, frame.uav_id))
            + len(payload).to_bytes(2, self.byteorder)
            + payload
        )
        checksum = self.checksum_func(body) & 0xFFFF
        return body + checksum.to_bytes(2, self.byteorder)

    def decode(self, packet: bytes) -> UAVFrame:
        if len(packet) < HEADER_SIZE + CHECKSUM_SIZE:
            raise ProtocolError(f"packet too short: {len(packet)} bytes")

        magic = int.from_bytes(packet[:2], self.byteorder)
        if magic != self.magic:
            raise ProtocolError(
                f"bad magic 0x{magic:04X}, expected 0x{self.magic:04X}"
            )

        payload_len = int.from_bytes(packet[4:6], self.byteorder)
        expected_len = HEADER_SIZE + payload_len + CHECKSUM_SIZE
        if len(packet) != expected_len:
            raise ProtocolError(
                f"packet length mismatch: got {len(packet)} expected {expected_len}"
            )

        body = packet[:-2]
        expected_checksum = self.checksum_func(body) & 0xFFFF
        checksum = int.from_bytes(packet[-2:], self.byteorder)
        if checksum != expected_checksum:
            raise ProtocolError(
                f"bad checksum 0x{checksum:04X}, expected 0x{expected_checksum:04X}"
            )

        return UAVFrame(
            msg_type=packet[2],
            uav_id=packet[3],
            payload=packet[6:-2],
        )

    def pack_float32(self, value: float) -> bytes:
        fmt = ">f" if self.byteorder == "big" else "<f"
        return struct.pack(fmt, float(value))


class GroundStationUAVSocketController:
    """
    Standalone ground-station controller for UAV command frames.

    Current stage:
    - unlock workflow is fully implemented
    - takeoff / mode / return-home / status parsing keep dedicated placeholders
    """

    def __init__(
        self,
        remote_host: str,
        remote_port: int,
        *,
        transport: str = "udp",
        local_host: str = "0.0.0.0",
        local_port: int = 0,
        timeout_sec: float = 1.0,
        max_retries: int = 3,
        retry_interval_sec: float = 0.2,
        recv_buffer_size: int = 65535,
        codec: Optional[UAVProtocolCodec] = None,
    ) -> None:
        transport_value = str(transport or "udp").strip().lower()
        if transport_value not in ("udp", "tcp"):
            raise ValueError("transport must be 'udp' or 'tcp'")

        self.remote_host = remote_host
        self.remote_port = int(remote_port)
        self.transport = transport_value
        self.local_host = local_host
        self.local_port = int(local_port)
        self.timeout_sec = max(0.01, float(timeout_sec))
        self.max_retries = max(1, int(max_retries))
        self.retry_interval_sec = max(0.0, float(retry_interval_sec))
        self.recv_buffer_size = max(1024, int(recv_buffer_size))
        self.codec = codec or UAVProtocolCodec()
        self.sock: Optional[socket.socket] = None

    def __enter__(self) -> "GroundStationUAVSocketController":
        return self.open()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> "GroundStationUAVSocketController":
        if self.sock is not None:
            return self

        sock_type = socket.SOCK_DGRAM if self.transport == "udp" else socket.SOCK_STREAM
        sock = socket.socket(socket.AF_INET, sock_type)
        sock.settimeout(self.timeout_sec)

        if self.transport == "udp":
            sock.bind((self.local_host, self.local_port))
            sock.connect((self.remote_host, self.remote_port))
        else:
            if self.local_port or self.local_host not in ("", "0.0.0.0"):
                sock.bind((self.local_host, self.local_port))
            sock.connect((self.remote_host, self.remote_port))

        self.sock = sock
        return self

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.close()
        finally:
            self.sock = None

    def send_frame(self, frame: UAVFrame) -> bytes:
        if self.sock is None:
            self.open()
        packet = self.codec.encode(frame)
        assert self.sock is not None
        if self.transport == "udp":
            self.sock.send(packet)
        else:
            self.sock.sendall(packet)
        return packet

    def receive_frame(self) -> UAVFrame:
        if self.sock is None:
            self.open()
        assert self.sock is not None

        if self.transport == "udp":
            packet = self.sock.recv(self.recv_buffer_size)
            return self.codec.decode(packet)

        header = self._recv_exact(HEADER_SIZE)
        payload_len = int.from_bytes(header[4:6], self.codec.byteorder)
        rest = self._recv_exact(payload_len + CHECKSUM_SIZE)
        return self.codec.decode(header + rest)

    def wait_for_frame(
        self,
        *,
        expected_msg_type: int,
        expected_uav_id: int,
        wait_timeout_sec: Optional[float] = None,
    ) -> UAVFrame:
        timeout_value = self.timeout_sec if wait_timeout_sec is None else max(0.01, float(wait_timeout_sec))
        deadline = time.monotonic() + timeout_value
        if self.sock is None:
            self.open()
        assert self.sock is not None
        original_timeout = self.sock.gettimeout()

        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"timed out waiting for msg_type=0x{expected_msg_type:02X} "
                        f"from uav_id={expected_uav_id}"
                    )
                self.sock.settimeout(remaining)
                try:
                    frame = self.receive_frame()
                except socket.timeout:
                    continue
                except ProtocolError:
                    continue
                if frame.msg_type != int(expected_msg_type):
                    continue
                if frame.uav_id != int(expected_uav_id):
                    continue
                return frame
        finally:
            self.sock.settimeout(original_timeout)

    def unlock_uav(
        self,
        uav_id: int,
        *,
        wait_timeout_sec: Optional[float] = None,
        max_retries: Optional[int] = None,
    ) -> UnlockResult:
        retries = self.max_retries if max_retries is None else max(1, int(max_retries))
        last_frame: Optional[UAVFrame] = None
        last_state: Optional[int] = None
        last_detail = "unlock feedback not received"

        for attempt in range(1, retries + 1):
            self.send_frame(self.build_unlock_frame(uav_id))

            try:
                response = self.wait_for_frame(
                    expected_msg_type=MessageType.UNLOCK_ACK,
                    expected_uav_id=uav_id,
                    wait_timeout_sec=wait_timeout_sec,
                )
            except TimeoutError:
                last_detail = "unlock feedback timeout"
            except ProtocolError as exc:
                last_detail = f"protocol error while waiting for unlock feedback: {exc}"
            else:
                last_frame = response
                if len(response.payload) != 1:
                    last_detail = (
                        "unlock feedback payload length must be 1 byte, "
                        f"got {len(response.payload)}"
                    )
                else:
                    last_state = response.payload[0]
                    if last_state == 1:
                        return UnlockResult(
                            success=True,
                            uav_id=int(uav_id),
                            attempts=attempt,
                            ack_state=last_state,
                            response_frame=response,
                            detail="unlock confirmed by UAV feedback",
                        )
                    last_detail = f"unlock rejected by UAV feedback: state={last_state}"

            if attempt < retries and self.retry_interval_sec > 0:
                time.sleep(self.retry_interval_sec)

        return UnlockResult(
            success=False,
            uav_id=int(uav_id),
            attempts=retries,
            ack_state=last_state,
            response_frame=last_frame,
            detail=last_detail,
        )

    def build_unlock_frame(self, uav_id: int) -> UAVFrame:
        return UAVFrame(msg_type=MessageType.UNLOCK, uav_id=uav_id, payload=b"")

    def build_takeoff_frame(self, uav_id: int, target_altitude: float) -> UAVFrame:
        return UAVFrame(
            msg_type=MessageType.TAKEOFF,
            uav_id=uav_id,
            payload=self.codec.pack_float32(target_altitude),
        )

    def build_mode_frame(self, uav_id: int, mode: int | UAVMode) -> UAVFrame:
        return UAVFrame(
            msg_type=MessageType.SET_MODE,
            uav_id=uav_id,
            payload=bytes((int(mode) & 0xFF,)),
        )

    def build_return_home_frame(self, uav_id: int) -> UAVFrame:
        return UAVFrame(msg_type=MessageType.RETURN_HOME, uav_id=uav_id, payload=b"")

    def takeoff_uav(self, uav_id: int, target_altitude: float) -> None:
        raise NotImplementedError(
            "Takeoff flow is reserved for the next stage. "
            "Use build_takeoff_frame() when wiring it in."
        )

    def set_uav_mode(self, uav_id: int, mode: int | UAVMode) -> None:
        raise NotImplementedError(
            "Mode-setting flow is reserved for the next stage. "
            "Use build_mode_frame() when wiring it in."
        )

    def return_home(self, uav_id: int) -> None:
        raise NotImplementedError(
            "Return-home flow is reserved for the next stage. "
            "Use build_return_home_frame() when wiring it in."
        )

    def parse_status_report(self, frame: UAVFrame) -> StatusReport:
        return StatusReport.from_frame(frame)

    def _recv_exact(self, size: int) -> bytes:
        assert self.sock is not None
        chunks = bytearray()
        while len(chunks) < size:
            chunk = self.sock.recv(size - len(chunks))
            if not chunk:
                raise ConnectionError("socket closed while receiving frame")
            chunks.extend(chunk)
        return bytes(chunks)


class MockUnlockUAVResponder:
    """
    Lightweight UDP responder for local verification.

    It only implements the unlock ack path so the ground-station workflow can be
    tested without touching the existing project files.
    """

    def __init__(
        self,
        bind_host: str = "127.0.0.1",
        bind_port: int = 19001,
        *,
        unlock_state: int = 1,
        response_delay_sec: float = 0.0,
        drop_first_n: int = 0,
        codec: Optional[UAVProtocolCodec] = None,
    ) -> None:
        self.bind_host = bind_host
        self.bind_port = int(bind_port)
        self.unlock_state = 1 if int(unlock_state) else 0
        self.response_delay_sec = max(0.0, float(response_delay_sec))
        self.drop_first_n = max(0, int(drop_first_n))
        self.codec = codec or UAVProtocolCodec()
        self.sock: Optional[socket.socket] = None
        self._unlock_requests_seen = 0

    def __enter__(self) -> "MockUnlockUAVResponder":
        return self.open()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> "MockUnlockUAVResponder":
        if self.sock is not None:
            return self
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_host, self.bind_port))
        self.sock = sock
        return self

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.close()
        finally:
            self.sock = None

    def serve_once(self, timeout_sec: float = 5.0) -> bool:
        if self.sock is None:
            self.open()
        assert self.sock is not None

        self.sock.settimeout(timeout_sec)
        packet, addr = self.sock.recvfrom(65535)
        frame = self.codec.decode(packet)
        if frame.msg_type != MessageType.UNLOCK:
            return False

        self._unlock_requests_seen += 1
        if self._unlock_requests_seen <= self.drop_first_n:
            return False

        if self.response_delay_sec > 0:
            time.sleep(self.response_delay_sec)

        ack = UAVFrame(
            msg_type=MessageType.UNLOCK_ACK,
            uav_id=frame.uav_id,
            payload=bytes((self.unlock_state,)),
        )
        self.sock.sendto(self.codec.encode(ack), addr)
        return True

    def serve_forever(self) -> None:
        self.open()
        while True:
            self.serve_once(timeout_sec=None)


def _run_unlock_cli(args: argparse.Namespace) -> int:
    controller = GroundStationUAVSocketController(
        remote_host=args.remote_host,
        remote_port=args.remote_port,
        transport=args.transport,
        local_host=args.local_host,
        local_port=args.local_port,
        timeout_sec=args.timeout_sec,
        max_retries=args.max_retries,
        retry_interval_sec=args.retry_interval_sec,
    )
    with controller:
        result = controller.unlock_uav(args.uav_id)
    print(
        f"unlock success={result.success} "
        f"uav_id={result.uav_id} "
        f"attempts={result.attempts} "
        f"ack_state={result.ack_state} "
        f"detail={result.detail}"
    )
    return 0 if result.success else 1


def _run_mock_cli(args: argparse.Namespace) -> int:
    responder = MockUnlockUAVResponder(
        bind_host=args.bind_host,
        bind_port=args.bind_port,
        unlock_state=args.unlock_state,
        response_delay_sec=args.response_delay_sec,
        drop_first_n=args.drop_first_n,
    )
    print(
        f"mock unlock responder listening on {args.bind_host}:{args.bind_port} "
        f"(unlock_state={args.unlock_state}, drop_first_n={args.drop_first_n})"
    )
    with responder:
        responder.serve_forever()
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Standalone UAV socket controller for ground-station unlock flow."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    unlock_parser = subparsers.add_parser("unlock", help="send unlock command and wait for feedback")
    unlock_parser.add_argument("--remote-host", default="127.0.0.1")
    unlock_parser.add_argument("--remote-port", type=int, required=True)
    unlock_parser.add_argument("--local-host", default="0.0.0.0")
    unlock_parser.add_argument("--local-port", type=int, default=0)
    unlock_parser.add_argument("--transport", choices=("udp", "tcp"), default="udp")
    unlock_parser.add_argument("--uav-id", type=int, required=True)
    unlock_parser.add_argument("--timeout-sec", type=float, default=1.0)
    unlock_parser.add_argument("--max-retries", type=int, default=3)
    unlock_parser.add_argument("--retry-interval-sec", type=float, default=0.2)
    unlock_parser.set_defaults(handler=_run_unlock_cli)

    mock_parser = subparsers.add_parser("mock-uav", help="start a local UDP unlock responder")
    mock_parser.add_argument("--bind-host", default="127.0.0.1")
    mock_parser.add_argument("--bind-port", type=int, default=19001)
    mock_parser.add_argument("--unlock-state", type=int, choices=(0, 1), default=1)
    mock_parser.add_argument("--response-delay-sec", type=float, default=0.0)
    mock_parser.add_argument("--drop-first-n", type=int, default=0)
    mock_parser.set_defaults(handler=_run_mock_cli)

    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
