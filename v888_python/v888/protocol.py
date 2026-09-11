"""
Low-level WiFi-UAV ("FLD-compatible") protocol for the V888 drone.

This file is adapted from the Z908 Pro Max implementation of this same
protocol family. Byte-for-byte cross-checking against V888 packet
captures (PCAPdroid, root capture mode) confirmed the envelope layout,
RC command sub-field, checksum, terminator, and camera-index field are
IDENTICAL between V888 and Z908 -- same "WiFi-UAV" protocol, same SDK/
vendor lineage. Differences found so far, specific to V888, are called
out inline below.

This module only builds/parses raw bytes. It has no socket or threading
code. See `drone.py` for the user-facing djitellopy-style API.

Envelope layout (variable length depending on `command` and `ack_slots`):

    offset  0..1    magic            ef 02
    offset  2..3    total packet length, little-endian u16 (self-referential)
    offset  4..7    fixed            02 02 00 01
    offset  8       num_ack_slots    u8, number of ACK slot records that follow
    offset  9..11   reserved         00 00 00
    offset 12..15   command_seq      little-endian u32, increments every packet sent
    offset 16..17   command_len      little-endian u16, length of the `command` field
    offset 18..18+command_len-1      command (RC bytes, see below, when non-empty)
    ... zero-padded out to a fixed 64-byte command slot ...
    offset 82..86   quality_params   32 4b 14 2d 00 (fixed, observed constant)
    offset 87       reserved         00
    offset 88..     ack_slots        num_ack_slots * ACK slot records (see below)

RC "command" sub-field layout (when present, 20 bytes in capture):

    offset 0..1     magic            66 14
    offset 2        roll             0x80 = neutral, + = right,  - = left
    offset 3        pitch            0x80 = neutral, + = forward,- = back
    offset 4        throttle         0x80 = neutral, + = up,     - = down
                                       (V888: full 0x00-0xff swing observed
                                        at every speed level; roll/pitch/yaw
                                        all reach the same full range too,
                                        but only at the app's max "speed"
                                        setting -- see percent_to_axis()
                                        below for the full confirmed detail)
    offset 5        yaw              0x80 = neutral, + = right turn
    offset 6        command flag     bit 0 = takeoff/land (one-key, shared)
                                       bit 1 = emergency stop (V888: confirmed
                                       1:1 against the app's dedicated "Stop"
                                       button, which cuts the motors when the
                                       takeoff/land toggle alone doesn't)
                                       bit 2 = gyro / calibration (V888: untested)
                                       bit 3 = flip / roll (V888: untested)
    offset 7        headless flag    0x02 = off, 0x03 = on (V888: only ever
                                       captured off; on is untested)
    offset 8..17    padding          10 x 0x00
    offset 18       checksum         XOR of bytes [2..7] (the 6 control bytes)
    offset 19       terminator       0x99

ACK slot layout — CONFIRMED byte-for-byte against both a real Z908 Pro
Max capture and a real V888 capture (identical in both):

    offset 0..7     seq              little-endian u64 (frame sequence being ACKed)
    offset 8..11    status           little-endian u32 (0=receiving,1=complete,
                                       2=dropped, 3=delivered)
    offset 12..15   record_len       little-endian u32, self-descriptive
                                       total length of this slot (16, or
                                       16+len(bitmap) when a bitmap is present)
    offset 16..      bitmap          present only on the "complete" slot in
                                       real captures (4 bytes, always seen as
                                       0xffffffff); absent (0 bytes) on the
                                       "delivered" slot

    In every 124-byte packet observed, exactly two slots are sent back
    to back: a 20-byte "complete" slot (record_len=20, WITH the 4-byte
    0xffffffff bitmap) for the frame just finished, followed by a
    16-byte "delivered" slot (record_len=16, no bitmap) pre-announcing
    the next frame_id. 20 + 16 = 36 = 124 - 88, exactly matching the
    124-byte envelope size with num_ack_slots=2. (An earlier version of
    this file's docstring said slots were always 16 bytes/"no bitmap
    observed" -- that was based on a test call that didn't pass a
    bitmap, not on packet capture; real captures always include it on
    the first slot.)

These offsets were confirmed by packet capture (PCAPdroid, root capture
mode) against a V888 drone (control/video endpoint 192.168.169.1, same
as Z908 Pro Max), cross-checked against a Z908-specific implementation of
this same protocol family, which was in turn cross-checked against the
decompiled-APK findings in the turbodrone project
(https://github.com/marshallrichards/turbodrone,
docs/research/wifi_uav.md, backend/utils/wifi_uav_packets.py, and
backend/utils/wifi_uav_ack_state.py).

V888-specific protocol differences confirmed so far (see `drone.py` /
project docs for detail):
  * Only UDP 8800 responds for control; UDP 8801 was tried by the stock
    app but always got back ICMP port-unreachable in capture (Z908 used
    both 8800 and 8801).
  * A second, independent arm/stop mechanism was also observed: a brief
    (~0.1s) throttle PULSE to near-max (0xb3->0xff) arms the motors, and
    a brief pulse to 0x00 stops them -- with the command flag byte
    staying 0x00 throughout (i.e. NOT going through the takeoff/land or
    stop flag bits at all). How this relates to the flag-based
    takeoff()/emergency_stop() below (prerequisite arming step vs. fully
    independent alternate control path) is not yet confirmed.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

# --------------------------------------------------------------------------- #
# Network defaults
# --------------------------------------------------------------------------- #

DEFAULT_DRONE_IP = "192.168.169.1"
DEFAULT_CONTROL_PORT = 8800
DEFAULT_VIDEO_PORT = 1234

# --------------------------------------------------------------------------- #
# Static fragments (taken 1:1 from packet captures / turbodrone)
# --------------------------------------------------------------------------- #

_ENVELOPE_MAGIC = bytes([0xEF, 0x02])
_ENVELOPE_FIXED = bytes([0x02, 0x02, 0x00, 0x01])
_QUALITY_PARAMS = bytes([0x32, 0x4B, 0x14, 0x2D])  # quality1, quality2, q_threshold1, q_threshold2
_COMMAND_SLOT_SIZE = 64  # the command field is zero-padded out to this size

_RC_MAGIC = bytes([0x66, 0x14])
_RC_PADDING = bytes(10)
_RC_TERMINATOR = bytes([0x99])

# Video-start trigger sent once (or repeated) to UDP 8800/8801 to ask the
# drone to begin streaming to the phone's UDP 1234.
START_STREAM = b"\xef\x00\x04\x00"

# Camera-channel select / SSID-info handshake packets. These are sent once
# near the start of a session, observed in capture and confirmed against
# turbodrone's `wifi_uav_packets.py` (SSID2/SSID3). Their exact effect is
# not fully confirmed (possibly camera-channel registration rather than
# a live switch command); kept here for completeness.
HANDSHAKE_UNK = b"\xef\x20\x06\x00\x01\x65"
HANDSHAKE_CAM_2 = (
    b"\xef\x20\x19\x00\x01\x67"
    b"\x3c\x69\x3d\x32\x5e\x62\x66\x5f\x73\x73\x69\x64\x3d\x63\x6d\x64"
    b"\x3d\x32\x3e"
)
HANDSHAKE_CAM_3 = (
    b"\xef\x20\x19\x00\x01\x67"
    b"\x3c\x69\x3d\x32\x5e\x62\x66\x5f\x73\x73\x69\x64\x3d\x63\x6d\x64"
    b"\x3d\x33\x3e"
)

# --------------------------------------------------------------------------- #
# Command flag bits (within the RC command sub-field)
# --------------------------------------------------------------------------- #

FLAG_TAKEOFF_OR_LAND = 0x01  # one-key shared takeoff/land action
FLAG_STOP = 0x02             # emergency stop (immediate motor cutoff)
FLAG_CALIBRATION = 0x04      # gyro calibration / "check"
FLAG_FLIP = 0x08             # flip / 360 roll
# NOTE: bits 6-7 were tried as a camera-tilt control based on turbodrone's
# decompiled findings, but real-world testing showed no effect -- this
# drone's camera tilt (confirmed to exist on the physical RF remote) is
# apparently not exposed over the WiFi-UAV protocol at all, so this is no
# longer used.

HEADLESS_OFF = 0x02
HEADLESS_ON = 0x03

# CONFIRMED by real V888 capture (Follow Me test): offset25 -- previously
# assumed to be a simple binary "headless" flag -- is actually a MODE
# BITFIELD. Observed values so far: 0x02 (base/off), 0x03 (headless on,
# = MODE_BASE | MODE_HEADLESS), 0x06 (Follow Me on, = MODE_BASE |
# MODE_FOLLOW_ME). Whether 0x07 (both headless AND follow-me at once) is
# a valid/reachable combination has not been observed.
MODE_BASE = 0x02        # always-on base bit, present in every value seen
MODE_HEADLESS = 0x01
MODE_FOLLOW_ME = 0x04

NEUTRAL_AXIS = 0x80

# ACK slot status values
ACK_STATUS_RECEIVING = 0
ACK_STATUS_COMPLETE = 1
ACK_STATUS_DROPPED = 2
ACK_STATUS_DELIVERED = 3

# --------------------------------------------------------------------------- #
# Control state
# --------------------------------------------------------------------------- #


@dataclass
class ControlState:
    """One snapshot of stick + flag state, in raw protocol units (0-255)."""

    roll: int = NEUTRAL_AXIS
    pitch: int = NEUTRAL_AXIS
    throttle: int = NEUTRAL_AXIS
    yaw: int = NEUTRAL_AXIS
    takeoff_or_land: bool = False
    stop: bool = False
    calibrate: bool = False
    flip: bool = False
    headless: bool = False
    follow_me: bool = False  # CONFIRMED: offset25 mode bitfield, see MODE_* above
    camera_index: int = 0  # 0 = main/front camera, 1 = secondary/downward camera

    def command_byte(self) -> int:
        value = 0
        if self.takeoff_or_land:
            value |= FLAG_TAKEOFF_OR_LAND
        if self.stop:
            value |= FLAG_STOP
        if self.calibrate:
            value |= FLAG_CALIBRATION
        if self.flip:
            value |= FLAG_FLIP
        return value & 0xFF

    def headless_byte(self) -> int:
        """Named for backwards compat, but actually builds the full
        offset25 mode bitfield (base + headless + follow-me)."""
        value = MODE_BASE
        if self.headless:
            value |= MODE_HEADLESS
        if self.follow_me:
            value |= MODE_FOLLOW_ME
        return value & 0xFF


def build_rc_command_field(state: ControlState) -> bytes:
    """
    Build the 20-byte RC "command" sub-field embedded inside the ACK/
    request envelope. This is what actually moves the drone -- it must be
    embedded in an envelope (see `build_packet()`) to be sent.
    """
    controls = bytes([
        state.roll & 0xFF,
        state.pitch & 0xFF,
        state.throttle & 0xFF,
        state.yaw & 0xFF,
        state.command_byte(),
        state.headless_byte(),
    ])
    checksum = 0
    for b in controls:
        checksum ^= b

    return (
        _RC_MAGIC
        + controls
        + _RC_PADDING
        + bytes([checksum])
        + _RC_TERMINATOR
    )


class PacketCounters:
    """The rolling u32 command-sequence counter embedded in every envelope.

    Thread-safe: arm_motors_test() sends packets directly from the
    caller's thread WHILE the background send loop may also be running
    concurrently, so two threads can call snapshot_and_advance() at
    close to the same time. A lock avoids handing out a duplicate
    command_seq value to two packets in that situation.
    """

    __slots__ = ("command_seq", "_lock")

    def __init__(self) -> None:
        self.command_seq = 0
        self._lock = threading.Lock()

    def snapshot_and_advance(self) -> int:
        with self._lock:
            value = self.command_seq
            self.command_seq = (self.command_seq + 1) & 0xFFFFFFFF
            return value


@dataclass
class AckSlot:
    """One native ACK slot record, mirroring turbodrone's `build_ack_slot`."""

    seq: int
    status: int = ACK_STATUS_COMPLETE
    bitmap: bytes = b""

    def to_bytes(self) -> bytes:
        record_len = 16 + len(self.bitmap)
        return (
            (self.seq & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "little")
            + (self.status & 0xFFFFFFFF).to_bytes(4, "little")
            + record_len.to_bytes(4, "little")
            + self.bitmap
        )


def build_packet(
    counters: PacketCounters,
    command: bytes = b"",
    ack_slots: list = None,
    active_camera_index: int = 0,
) -> bytes:
    """
    Build one native ACK/request envelope, optionally carrying an RC
    command sub-field and/or per-frame video ACK slots.

    Both RC control and video-frame ACKs travel inside this same envelope
    shape -- this matches what was observed in packet capture (the drone
    stopped responding with video when this module previously sent
    RC-only packets without any ACK slots, even though flight control by
    itself still worked).

    `active_camera_index` selects which camera feed the drone should send
    (0 = main/front, 1 = secondary/downward, per turbodrone's decompiled
    `nativeSetCameraIndex(z ? 1 : 0)`). This is carried at envelope offset
    86 in every packet -- not a one-off command -- so it must be held at
    the desired value continuously (the regular RC send loop already does
    this automatically via V888's `camera_index` state).
    """
    if ack_slots is None:
        ack_slots = []

    command_seq = counters.snapshot_and_advance()

    pkt = bytearray()
    pkt += _ENVELOPE_MAGIC
    pkt += b"\x00\x00"  # length placeholder, filled in at the end
    pkt += _ENVELOPE_FIXED
    pkt.append(len(ack_slots) & 0xFF)
    pkt += b"\x00\x00\x00"
    pkt += command_seq.to_bytes(4, "little")
    pkt += len(command).to_bytes(2, "little")
    pkt += command.ljust(_COMMAND_SLOT_SIZE, b"\x00")
    pkt += _QUALITY_PARAMS
    pkt.append(active_camera_index & 0xFF)
    pkt += b"\x00"
    for slot in ack_slots:
        pkt += slot.to_bytes()

    pkt[2:4] = len(pkt).to_bytes(2, "little")
    return bytes(pkt)


def build_control_packet(
    state: ControlState,
    counters: PacketCounters,
    ack_slots: list = None,
    active_camera_index: int = 0,
) -> bytes:
    """
    Build one RC control envelope (RC command embedded, plus any pending
    video ACK slots). This is the packet that should be sent at the
    regular control rate (e.g. 50Hz) to both fly the drone and keep its
    video stream alive.
    """
    command = build_rc_command_field(state)
    return build_packet(
        counters,
        command=command,
        ack_slots=ack_slots,
        active_camera_index=active_camera_index,
    )


def clamp_byte(value: int) -> int:
    return max(0, min(255, value))


def percent_to_axis(percent: float) -> int:
    """
    Map a -100..100 percent value (djitellopy-style RC input) onto the
    drone's raw 0..255 axis byte, centred on 0x80.

    IMPORTANT (supersedes an earlier note in this file): roll/pitch/yaw
    are NOT hardware-limited to a narrow range around centre. The stock
    app has a 3-level "speed" button (30% / 60% / 100%, client-side
    only -- confirmed by comparing captures at each level, where every
    byte of the outgoing envelope was identical except the roll/pitch/
    yaw values themselves; no separate "current speed level" field is
    ever sent to the drone):

      speed level 1 (30%, the app's default): max swing ~0x59..0xa6
        (-39/+38 around centre, i.e. this formula's +/-30 percent)
      speed level 2 (60%):                    max swing ~0x33..0xcc
        (roughly this formula's +/-60 percent) -- CONFIRMED identical
        for roll, pitch, AND yaw (yaw's captured range at this level,
        0x33..0xcc, matched roll/pitch's to the byte)
      speed level 3 (100%):                   max swing reaches the
        FULL 0x00..0xff range -- i.e. roll/pitch/yaw behave exactly
        like throttle once "speed" is maxed out. (yaw's minimum was
        directly confirmed reaching 0x00 at this level; its maximum
        was only captured up to 0xc3 in a short press, but given the
        symmetric confirmation on the low end this is assumed to also
        reach 0xff on a full press, consistent with roll/pitch.)

    In other words, throttle was never a special unrestricted axis --
    every axis accepts the full 0..255 range; the stock app's default
    UI setting (speed level 1) just happens to only ever command a
    fraction of that range for roll/pitch/yaw. Callers of this function
    can safely use the full -100..100 range on every axis; there is no
    protocol-level reason to hold back.
    """
    percent = max(-100.0, min(100.0, float(percent)))
    return clamp_byte(round(NEUTRAL_AXIS + percent * 1.27))