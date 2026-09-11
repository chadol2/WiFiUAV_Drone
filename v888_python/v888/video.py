"""
Video stream handling for the V888 drone (WiFi-UAV protocol family, same as Z908 Pro Max).

The drone sends MJPEG frames as a custom fragmented UDP protocol (NOT
standard RTP/RTSP/MPEG-TS), with the JPEG container headers (SOI, DQT,
SOF0, SOS) stripped out -- only the compressed scan data is sent over the
wire, plus a 56-byte proprietary fragment header per UDP datagram.

This module reconstructs full JPEG images by:
  1. Sending a one-time START_STREAM trigger.
  2. Parsing each incoming UDP datagram's 56-byte header to find which
     frame and fragment it belongs to.
  3. Collecting fragments until a frame is complete.
  4. Re-attaching a synthesized standard JPEG header in front of the
     fragment data, since the drone never sends one.
  5. Periodically sending ACK/frame-request packets, matching what the
     stock app does to keep the drone streaming.

This logic (fragment header layout, ACK packet shape, missing-JPEG-header
behavior) was confirmed against the turbodrone project's reverse-engineered
WiFi-UAV implementation (https://github.com/marshallrichards/turbodrone,
backend/protocols/wifi_uav_video_protocol.py and backend/utils/
wifi_uav_jpeg.py / wifi_uav_packets.py / wifi_uav_ack_state.py), which in
turn comes from decompiling the WiFi UAV Android app
(com.lcfld.fldpublic).
"""

from __future__ import annotations

import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from . import protocol as proto

# --------------------------------------------------------------------------- #
# JPEG header synthesis (the drone never sends SOI/DQT/SOF0/SOS, only the
# compressed scan data -- these have to be generated on our side)
# --------------------------------------------------------------------------- #

_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"

_STD_LUMINANCE_QT = [
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99,
]
_STD_CHROMINANCE_QT = [
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
]


def _scale_qt(base_table: list[int], quality: int) -> list[int]:
    """
    Scale a base (quality=50) quantization table to a different JPEG
    quality level, using the standard IJG IJG/libjpeg scaling formula.
    quality: 1-100 (50 returns the base table unchanged).

    The drone's actual encoder quality is unknown -- if decoded frames
    look subtly wrong (e.g. "bits seem clipped/lost") even with the
    correct resolution and chroma subsampling, the quantization table
    mismatch is the most likely remaining cause, since it's the one
    header field this module cannot read from the drone's packets (the
    drone never sends DQT segments at all). Try varying this if so.
    """
    quality = max(1, min(100, quality))
    scale = 5000 / quality if quality < 50 else 200 - quality * 2
    scaled = []
    for v in base_table:
        nv = (v * scale + 50) // 100
        scaled.append(max(1, min(255, int(nv))))
    return scaled


def _dqt_segment(table_id: int, table: list[int]) -> bytes:
    payload = bytes([table_id]) + bytes(table)
    length = len(payload) + 2
    return b"\xff\xdb" + length.to_bytes(2, "big") + payload


def _sof0_segment(width: int, height: int, y_sampling: int = 0x11) -> bytes:
    """
    y_sampling controls the luma (Y) component's horizontal/vertical
    sampling factors (high nibble=H, low nibble=V); chroma (Cb/Cr) are
    always 0x11. Common values:
        0x11 -> 4:4:4 (no chroma subsampling)
        0x21 -> 4:2:2 (horizontal subsampling only)
        0x22 -> 4:2:0 (both horizontal and vertical subsampling)
    Different camera channels on this drone may encode with different
    subsampling; if colors look smeared/bled on a given channel, try a
    different value here (use jpeg_y_sampling on streamon()/VideoReceiver).
    """
    components = bytes([1, y_sampling, 0, 2, 0x11, 1, 3, 0x11, 1])
    length = 8 + 9
    return (
        b"\xff\xc0"
        + length.to_bytes(2, "big")
        + b"\x08"
        + height.to_bytes(2, "big")
        + width.to_bytes(2, "big")
        + b"\x03"
        + components
    )


def _sos_segment() -> bytes:
    return bytes([
        0xFF, 0xDA, 0x00, 0x0C, 0x03,
        0x01, 0x00, 0x02, 0x11, 0x03, 0x11,
        0x00, 0x3F, 0x00,
    ])


def _build_jpeg_header(width: int, height: int, y_sampling: int = 0x11, quality: int = 50) -> bytes:
    luma_qt = _scale_qt(_STD_LUMINANCE_QT, quality) if quality != 50 else _STD_LUMINANCE_QT
    chroma_qt = _scale_qt(_STD_CHROMINANCE_QT, quality) if quality != 50 else _STD_CHROMINANCE_QT
    return (
        _SOI
        + _dqt_segment(0, luma_qt)
        + _dqt_segment(1, chroma_qt)
        + _sof0_segment(width, height, y_sampling=y_sampling)
        + _sos_segment()
    )


# --------------------------------------------------------------------------- #
# Fragment header parsing
# --------------------------------------------------------------------------- #


def _parse_fragment_header(payload: bytes):
    """
    Returns (frame_id, frag_id, fragment_total, width, height, jpeg_payload)
    or None.

    Native packet layout (confirmed against turbodrone's
    wifi_uav_video_protocol.py and docs/research/wifi_uav.md):
        byte 0        : 0x93
        byte 1        : message type; 0x01 means JPEG fragment
        bytes 2..3    : total packet length (little-endian)
        bytes 8..15   : image sequence (little-endian u64)
        bytes 32..35  : fragment index (little-endian u32)
        bytes 36..39  : fragment count (little-endian u32)
        bytes 44..45  : frame width, little-endian u16
        bytes 46..47  : frame height, little-endian u16
        bytes 56+     : JPEG scan-data payload

    Width/height matter because different camera channels on this drone
    (e.g. switching to the secondary/downward camera via
    `V888.set_camera_index(1)`) use a different, lower resolution than the
    main camera -- a fixed JPEG header sized for the main camera produces
    a corrupted/incomplete image for the secondary one. Reading the real
    width/height from each packet and rebuilding the synthesized JPEG
    header to match avoids this.
    """
    if len(payload) < 56 or payload[0] != 0x93 or payload[1] != 0x01:
        return None

    declared_len = int.from_bytes(payload[2:4], "little")
    if declared_len == len(payload):
        frame_id = int.from_bytes(payload[8:16], "little")
        frag_id = int.from_bytes(payload[32:36], "little")
        fragment_total = int.from_bytes(payload[36:40], "little")
        width = int.from_bytes(payload[44:46], "little")
        height = int.from_bytes(payload[46:48], "little")
        if fragment_total > 0 and frag_id < fragment_total:
            return frame_id, frag_id, fragment_total, width, height, payload[56:]
        return None

    # Fallback for shorter/older-style packets.
    frame_id = int.from_bytes(payload[16:18], "little")
    frag_id = int.from_bytes(payload[32:34], "little")
    fragment_total = frag_id + 1 if payload[2] != 0x38 else 0
    width = int.from_bytes(payload[44:46], "little") if len(payload) >= 48 else 0
    height = int.from_bytes(payload[46:48], "little") if len(payload) >= 48 else 0
    return frame_id, frag_id, fragment_total, width, height, payload[56:]


# --------------------------------------------------------------------------- #
# ACK / frame-request packets -- NOT CURRENTLY USED.
#
# turbodrone's reverse-engineered implementation builds a native-shaped
# per-frame ACK/request packet and sends it continuously to keep the
# stream alive. However, packet capture of this drone's actual traffic
# showed NO such packets at all -- only a handful of 4-byte START_STREAM
# triggers near the start of a session (see `_warmup_loop` below), with
# the drone continuing to stream afterward seemingly kept alive by the
# ordinary 50Hz RC control packets already being sent on the same socket.
#
# Kept here (unused) in case a future capture or a different drone
# firmware variant turns out to need it -- wire `_request_loop`-style
# periodic sending back up using these builders if frames stop arriving
# after the initial burst.
# --------------------------------------------------------------------------- #

_NEUTRAL_COMMAND = bytes(6)  # 6 neutral control bytes, no flags
_DEFAULT_QUALITY_PARAMS = bytes([0x32, 0x4B, 0x14, 0x2D, 0x00])


def _build_ack_slot(seq: int, status: int, bitmap: bytes = b"") -> bytes:
    record_len = 16 + len(bitmap)
    return (
        (seq & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "little")
        + (status & 0xFFFFFFFF).to_bytes(4, "little")
        + record_len.to_bytes(4, "little")
        + bitmap
    )


def _build_frame_request_packet(command_seq: int, ack_slots: list[bytes]) -> bytes:
    packet = bytearray()
    packet += b"\xef\x02\x00\x00"
    packet += b"\x02\x02\x00\x01"
    packet += bytes([len(ack_slots) & 0xFF])
    packet += b"\x00\x00\x00"
    packet += (command_seq & 0xFFFFFFFF).to_bytes(4, "little")
    packet += len(_NEUTRAL_COMMAND).to_bytes(2, "little")
    packet += _NEUTRAL_COMMAND.ljust(64, b"\x00")
    packet += _DEFAULT_QUALITY_PARAMS
    for slot in ack_slots:
        packet += slot
    packet[2:4] = len(packet).to_bytes(2, "little")
    return bytes(packet)


# --------------------------------------------------------------------------- #
# Frame assembly state
# --------------------------------------------------------------------------- #


@dataclass
class _FrameSlot:
    frame_id: int = -1
    fragment_total: int = 0
    width: int = 0
    height: int = 0
    fragments: dict = field(default_factory=dict)

    def reset(self, frame_id: int, fragment_total: int, width: int, height: int) -> None:
        self.frame_id = frame_id
        self.fragment_total = fragment_total
        self.width = width
        self.height = height
        self.fragments = {}

    def is_complete(self) -> bool:
        return self.fragment_total > 0 and len(self.fragments) == self.fragment_total

    def ordered_payload(self) -> bytes:
        return b"".join(self.fragments[i] for i in range(self.fragment_total))


class VideoFrame:
    """A single decoded-container JPEG frame, djitellopy `BackgroundFrameRead`-
    style: read `.frame` for the latest available JPEG bytes."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg_bytes: Optional[bytes] = None
        self.frame_count = 0

    def _update(self, jpeg_bytes: bytes) -> None:
        with self._lock:
            self._jpeg_bytes = jpeg_bytes
            self.frame_count += 1

    @property
    def frame(self) -> Optional[bytes]:
        """Latest complete frame as encoded JPEG bytes, or None if no frame
        has arrived yet. Decode with e.g. OpenCV:
            import cv2, numpy as np
            arr = np.frombuffer(frame_read.frame, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        """
        with self._lock:
            return self._jpeg_bytes


class VideoReceiver:
    """
    Background receiver that reconstructs JPEG frames from the drone's
    fragmented UDP video stream and exposes the latest one through
    `.frame` (mirroring djitellopy's `get_frame_read()` pattern).

    IMPORTANT: this drone sends its video stream back to whichever local
    UDP port sent the START_STREAM trigger -- the same socket the control
    (RC) packets are sent from in capture. This class therefore does NOT
    open its own socket; it must be given the already-open control socket
    to share (see `shared_socket`).
    """

    REQUEST_INTERVAL = 0.05  # how often to (re-)ask for the current frame

    def __init__(
        self,
        shared_socket: socket.socket,
        drone_ip: str = proto.DEFAULT_DRONE_IP,
        video_port: int = proto.DEFAULT_VIDEO_PORT,
        control_ports: tuple[int, ...] = (8800, 8801),
        jpeg_width: int = 640,
        jpeg_height: int = 360,
        jpeg_y_sampling: int = 0x11,
        send_acks: bool = True,
    ) -> None:
        self.drone_ip = drone_ip
        self.video_port = video_port
        self.control_ports = control_ports
        # Used only as a fallback if a packet's own width/height fields
        # (read per-frame, see _handle_fragment) come back as 0/missing --
        # normally the real per-frame resolution from the packet itself is
        # used instead, since it can differ by camera channel.
        self._default_width = jpeg_width
        self._default_height = jpeg_height
        # Mutable so callers can experiment at runtime (e.g.
        # video.jpeg_y_sampling = 0x22) without restarting the stream --
        # useful if colors look smeared/bled on a particular camera
        # channel and a different chroma subsampling needs testing.
        self.jpeg_y_sampling = jpeg_y_sampling
        # Same idea as jpeg_y_sampling -- mutable at runtime for
        # experimentation. The drone never sends its real DQT (quantization
        # table), so this module always guesses; if colors look subtly
        # wrong even with the right resolution/sampling (e.g. "bits seem
        # clipped"), try adjusting this (1-100, 50 = the bundled base
        # table unscaled).
        self.jpeg_quality = 50

        # Shared with the V888 control connection -- NOT created here, and
        # NOT closed by stop(). The owner (V888.end()/streamoff()) is
        # responsible for the socket's lifetime.
        self._sock: socket.socket = shared_socket
        self._rx_thread: Optional[threading.Thread] = None
        self._warmup_thread: Optional[threading.Thread] = None
        self._running = threading.Event()

        # FOR TESTING ONLY: if False, _queue_ack() becomes a no-op, so no
        # 124-byte ACK-carrying control packets ever go out for this
        # stream. Used to test whether the drone actually requires the
        # ACK slots to keep streaming video, or whether that was only
        # ever a Z908-side requirement carried over by assumption. See
        # examples/v888_video_no_ack_test.py. Leave True for normal use.
        self.send_acks = send_acks

        self._slot = _FrameSlot()
        self._current_frame_id = 1
        self._recent_delivered: deque = deque(maxlen=32)
        self._ack_queue: deque = deque(maxlen=4)
        self._ack_queue_lock = threading.Lock()

        self.frame_read = VideoFrame()
        self.frames_ok = 0
        self.frames_dropped = 0
        self.raw_packets_received = 0
        self.unrecognized_packets = 0

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Packet capture of the stock app showed it only ever sends the
        # 4-byte START_STREAM trigger a handful of times near the start of
        # a session (observed: port 8801 once, then 8800 twice, then 8801
        # once more, all within ~0.2s) -- no separate per-frame ACK/request
        # packets were present in capture at all. The drone keeps streaming
        # on its own afterward, apparently kept alive by the ordinary 50Hz
        # control (RC) packets the V888 class is already sending on this
        # same socket. So: send the trigger a few times, then do nothing
        # else video-specific.
        self._warmup_thread = threading.Thread(target=self._warmup_loop, daemon=True)
        self._warmup_thread.start()

    def _warmup_loop(self) -> None:
        trigger_sequence = [
            (self.control_ports[-1], 0.0),
            (self.control_ports[0], 0.1),
            (self.control_ports[0], 0.1),
            (self.control_ports[-1], 0.0),
        ]
        for port, delay_after in trigger_sequence:
            if not self._running.is_set():
                return
            self._safe_sendto(proto.START_STREAM, port)
            time.sleep(delay_after)

        # If nothing arrived within a couple seconds, retry once more --
        # captures showed the trigger sequence is small and cheap, so a
        # repeat is harmless if the first attempt was missed.
        for _ in range(20):
            if not self._running.is_set() or self.raw_packets_received > 0:
                return
            time.sleep(0.1)
        if self._running.is_set() and self.raw_packets_received == 0:
            for port, delay_after in trigger_sequence:
                if not self._running.is_set():
                    return
                self._safe_sendto(proto.START_STREAM, port)
                time.sleep(delay_after)

    def stop(self) -> None:
        self._running.clear()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._warmup_thread is not None:
            self._warmup_thread.join(timeout=1.0)
            self._warmup_thread = None
        # NOTE: we do not close self._sock here -- it's shared with the
        # V888 control connection, and closing it here would also break
        # flight control. Whoever passed it in owns its lifetime.

    # ------------------------------------------------------------------ #
    def _safe_sendto(self, payload: bytes, port: int) -> None:
        try:
            self._sock.sendto(payload, (self.drone_ip, port))
        except OSError:
            pass

    def _rx_loop(self) -> None:
        while self._running.is_set():
            try:
                payload, _addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Windows-specific: an ICMP "port unreachable" came back
                # for an earlier send (e.g. the 8801 trigger, which this
                # drone doesn't listen on) and Windows surfaces that as a
                # recv() error on this UDP socket instead of silently
                # ignoring it like other OSes do. This is NOT a reason to
                # stop listening -- the drone may still answer normally on
                # 8800 right afterward.
                continue
            except OSError:
                break
            self.raw_packets_received += 1
            parsed = _parse_fragment_header(payload)
            if parsed is None:
                self.unrecognized_packets += 1
                continue
            frame_id, frag_id, fragment_total, width, height, jpeg_payload = parsed
            self._handle_fragment(frame_id, frag_id, fragment_total, width, height, jpeg_payload)

    def _handle_fragment(
        self,
        frame_id: int,
        frag_id: int,
        fragment_total: int,
        width: int,
        height: int,
        jpeg_payload: bytes,
    ) -> None:
        if frame_id in self._recent_delivered:
            # Already assembled this frame -- but the drone is still
            # sending its fragments, which suggests it hasn't seen our
            # earlier ACK yet (or wants to see it repeated) before moving
            # on to the next frame. Re-queue the ACK rather than silently
            # dropping the packet.
            self._queue_ack(frame_id)
            return
        if self._slot.frame_id != frame_id:
            if self._slot.frame_id != -1 and not self._slot.is_complete():
                self.frames_dropped += 1
            self._slot.reset(frame_id, fragment_total, width, height)
        self._slot.fragments[frag_id] = jpeg_payload

        if self._slot.is_complete():
            # Build the JPEG header using THIS frame's actual resolution --
            # switching cameras (V888.set_camera_index()) can change the
            # resolution mid-stream (e.g. the secondary/downward camera
            # uses a lower resolution than the main camera), and a header
            # sized for the wrong resolution produces a corrupted/
            # incomplete-looking decoded image.
            slot_w = self._slot.width or self._default_width
            slot_h = self._slot.height or self._default_height
            header = _build_jpeg_header(
                slot_w, slot_h, y_sampling=self.jpeg_y_sampling, quality=self.jpeg_quality
            )
            jpeg_bytes = header + self._slot.ordered_payload() + _EOI
            self.frame_read._update(jpeg_bytes)
            self.frames_ok += 1
            self._recent_delivered.append(frame_id)
            self._current_frame_id = frame_id + 1
            self._queue_ack(frame_id)

    def _queue_ack(self, frame_id: int) -> None:
        """
        Queue ACK slots to be embedded in the next outgoing RC control
        packet (see V888._send_current_state(), which calls
        `pending_ack_slots()`).

        Packet capture of the stock app showed it always sends TWO ACK
        slots together, of DIFFERENT sizes: one for the frame just
        completed (status=1 "complete", 20 bytes, WITH a 4-byte
        `0xffffffff` bitmap) and one for the NEXT frame_id pre-emptively
        (status=3 "delivered", 16 bytes, no bitmap). This was confirmed
        byte-for-byte identical between a real Z908 Pro Max capture and a
        real V888 capture (both show the same 36-byte 88..123 region for
        124-byte packets: 20-byte slot + 16-byte slot). Omitting the
        bitmap on the completed-frame slot (an earlier version of this
        module did) produces a 32-byte region instead of 36 -- 4 bytes
        short of what real captures show; the meaning of the bitmap
        value itself is unconfirmed (assumed to signal "all fragments
        received").

        No-ops entirely if self.send_acks is False (see __init__).

        CONFIRMED by real test (send_acks=False, handshake otherwise
        intact): exactly ONE frame (frame_id=1) is received successfully,
        then the stream stalls completely for the rest of the session (no
        more frames for the remaining ~15s tested) -- ACK slots ARE
        required to keep the stream going past the first frame, same as
        Z908. The first frame arrives regardless of ACKs (handshake is
        what gates that); every frame after it needs the previous one
        acknowledged.
        """
        if not self.send_acks:
            return
        with self._ack_queue_lock:
            self._ack_queue.append(
                proto.AckSlot(
                    seq=frame_id,
                    status=proto.ACK_STATUS_COMPLETE,
                    bitmap=b"\xff\xff\xff\xff",
                )
            )
            self._ack_queue.append(
                proto.AckSlot(seq=frame_id + 1, status=proto.ACK_STATUS_DELIVERED)
            )

    def pending_ack_slots(self) -> list:
        """
        Pop and return any queued video-frame ACK slots, for embedding in
        the next RC control packet. Called by V888's send loop; each ACK
        is only returned once.
        """
        with self._ack_queue_lock:
            slots = list(self._ack_queue)
            self._ack_queue.clear()
        return slots