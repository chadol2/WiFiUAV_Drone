"""
V888 drone control, in the style of djitellopy's `Tello` class.

This is adapted from a Z908 Pro Max implementation of the same underlying
"WiFi-UAV" protocol family -- packet capture confirmed V888 and Z908 share
an identical envelope/RC/video-header format (see `protocol.py`/`video.py`
docstrings for exact cross-check notes and the handful of V888-specific
differences found, e.g. port usage and an extra throttle-pulse arm/stop
path).

Quick start
-----------

    from v888 import V888

    drone = V888()
    drone.connect()
    drone.takeoff()
    drone.send_rc_control(0, 50, 0, 0)   # left_right, forward_back, up_down, yaw
    time.sleep(2)
    drone.send_rc_control(0, 0, 0, 0)
    drone.land()
    drone.end()

Notes / differences from a real Tello
--------------------------------------
* This drone speaks a one-way, ACK-less UDP protocol: there is no telemetry
  coming back from the flight controller, only the video stream. Methods
  like `get_battery()` are therefore not available; this module does not
  fake them.
* Distances ("move_forward(30)") are not supported because the protocol has
  no closed-loop position feedback -- the app itself only exposes stick
  position + one-key takeoff/land. Use `send_rc_control()` for movement,
  the same way djitellopy users do when they want raw stick control.
* Takeoff and land share one physical button on the real app
  (`one-key takeoff/land`, command flag bit 0). Calling `land()` after
  `takeoff()` sends the same flag again; the drone decides whether that
  means "land" based on its own flight state. There is no independent
  landing command.
* A background thread re-sends the current control state continuously
  (default 50 Hz), because the drone requires a steady stream of packets to
  keep flying -- there is no "set once and it holds" command.
* V888 capture showed the one-key land toggle sometimes doesn't cut the
  motors (observed: 5 repeated land toggles with no effect). The app has a
  separate, dedicated "Stop" button for this -- confirmed 1:1 against
  command flag bit 1 (`emergency_stop()` below) -- use it as a fallback
  when `land()` doesn't seem to be working.

Video
-----
    drone.streamon()
    frame_read = drone.get_frame_read()
    jpeg_bytes = frame_read.frame   # latest complete JPEG frame, or None
    drone.streamoff()

The drone sends MJPEG frames as fragmented, headerless UDP packets; this
module reassembles them and synthesizes a standard JPEG header so the
result can be decoded normally (e.g. with OpenCV's `cv2.imdecode`). This
was verified against real V888 packet captures: reassembling by the
`frame_id` field (not fragment-index resets) and prepending a synthesized
SOI/DQT/SOF0/SOS header successfully decoded real frames (640x360).
"""

from __future__ import annotations

import platform
import socket
import subprocess
import threading
import time
from typing import Optional

from . import protocol as proto
from .video import VideoReceiver, VideoFrame


def get_connected_ssid() -> Optional[str]:
    """
    Best-effort read of the SSID of the Wi-Fi network this computer is
    CURRENTLY associated with, via OS-specific commands. Returns None if
    it can't be determined (not on Wi-Fi, unsupported OS, command not
    found, permissions issue, etc.) -- callers should treat None as
    "unknown", not as "not connected".

    This is a much stronger check than is_network_reachable()/IP-route
    probing: a computer can have a valid route to 192.168.169.1 (e.g.
    via a default gateway on a totally different Wi-Fi network) and
    still not actually be on the drone's AP at all. Checking the SSID
    catches that false-positive case; IP reachability alone does not.
    """
    system = platform.system()
    try:
        if system == "Windows":
            out = subprocess.check_output(
                ["netsh", "wlan", "show", "interfaces"],
                encoding="utf-8", errors="ignore", timeout=3,
            )
            for line in out.splitlines():
                line = line.strip()
                # Match "SSID" but not "BSSID" (netsh prints both).
                if line.startswith("SSID") and not line.startswith("BSSID"):
                    return line.split(":", 1)[1].strip() or None
        elif system == "Darwin":
            airport = (
                "/System/Library/PrivateFrameworks/Apple80211.framework"
                "/Versions/Current/Resources/airport"
            )
            try:
                out = subprocess.check_output(
                    [airport, "-I"], encoding="utf-8", errors="ignore", timeout=3,
                )
                for line in out.splitlines():
                    line = line.strip()
                    if line.startswith("SSID:"):
                        return line.split(":", 1)[1].strip() or None
            except (OSError, subprocess.SubprocessError):
                # `airport` was removed in newer macOS versions; fall back.
                out = subprocess.check_output(
                    ["networksetup", "-getairportnetwork", "en0"],
                    encoding="utf-8", errors="ignore", timeout=3,
                )
                if ":" in out:
                    return out.split(":", 1)[1].strip() or None
        elif system == "Linux":
            out = subprocess.check_output(
                ["iwgetid", "-r"], encoding="utf-8", errors="ignore", timeout=3,
            )
            ssid = out.strip()
            return ssid or None
    except (OSError, subprocess.SubprocessError):
        return None
    return None


class V888Error(RuntimeError):
    """Raised for V888-specific usage errors."""


class V888:
    """
    djitellopy-style controller for the V888 Pro Max ("WiFi UAV" / FLD-
    compatible family) drone.

    Parameters
    ----------
    drone_ip:
        IP address of the drone's control endpoint. Default matches the
        drone's own AP-assigned address (192.168.169.1).
    control_port:
        UDP port for control packets. Default 8800.
    send_rate_hz:
        How many control packets per second the background thread sends.
        The protocol needs a steady ~50 Hz stream; going much lower can
        cause the drone to consider the link lost.

    Physical remote control note
    -----------------------------
    CONFIRMED by real capture: the drone's physical RF remote does NOT
    use WiFi at all -- it talks to the drone over a separate RF channel
    entirely outside this protocol (no trace of it ever appears in
    WiFi traffic). This means this library and a physical remote can be
    connected to the drone at the same time without any network-level
    conflict (they're on completely different radios) -- but the
    DRONE'S OWN FIRMWARE arbitrates which one it actually listens to.

    CONFIRMED (tested both orderings): priority goes to WHICHEVER
    CHANNEL ESTABLISHED ACTIVE CONTROL FIRST, not to RF unconditionally:

      - Remote powered on and active FIRST, this library connects
        SECOND: the drone accepts the remote's commands and silently
        ignores this library's commands (confirmed by capture -- this
        library's packets go out fine over WiFi, the drone just doesn't
        act on them). Camera switching was an exception and kept
        working from the app even then.
      - This library connects and arms the motors FIRST (e.g. via
        arm_motors_test()), remote powered on SECOND: the OPPOSITE
        happens -- the remote's disarm command is ignored while this
        library's connection is still active. Only after this library
        disconnects (e.g. end()) does the remote regain control (an
        immediate throttle-up on the remote armed the motors right
        away once the WiFi side was gone).

    Practical implication: if you connect with this library first and
    keep the connection open, you hold priority over a physical remote
    turned on afterward. If a physical remote is already active when
    you connect, expect this library's control commands to be silently
    ignored until the remote is turned off (or you may still be able to
    "win" by connecting and establishing control before the remote is
    powered on -- not yet tested as a race/timing scenario, only as
    "which one was already fully active first").

    This applies to V888's specific remote, which has NO screen/
    monitor. Hypothesis (unverified on V888, but consistent with this
    project's experience on other monitor-equipped remotes): a remote
    WITH a screen would need the video feed to display it, and video in
    this protocol family only flows over WiFi -- so a monitor-equipped
    remote would likely need to join the drone's WiFi AP itself, which
    could then conflict with this library connecting (unlike V888's
    screen-less, WiFi-free remote).
    """

    def __init__(
        self,
        drone_ip: str = proto.DEFAULT_DRONE_IP,
        control_port: int = proto.DEFAULT_CONTROL_PORT,
        send_rate_hz: float = 50.0,
    ) -> None:
        self.drone_ip = drone_ip
        self.control_port = control_port
        self._send_interval = 1.0 / send_rate_hz

        self._sock: Optional[socket.socket] = None
        self._counters = proto.PacketCounters()
        self._state = proto.ControlState()
        self._state_lock = threading.Lock()

        self._send_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._is_connected = False
        self._send_failure_warned = False
        self._last_send_warning_time = 0.0
        # 0.0 sentinel means "never sent successfully yet" -- seconds_since_
        # last_successful_send() treats that as +inf, not "0 seconds ago".
        self._last_successful_send_time = 0.0
        self._sending_paused = False
        self._video: Optional[VideoReceiver] = None

    # ------------------------------------------------------------------ #
    # Connection lifecycle
    # ------------------------------------------------------------------ #
    def connect(
        self,
        check_network: bool = True,
        require_network: bool = False,
        require_ssid_prefix: Optional[str] = None,
        start_sending: bool = True,
        skip_handshake: bool = False,
    ) -> None:
        """
        Perform the connection handshake, open the control socket, and
        start the background send loop.

        This does not perform a Wi-Fi association -- connect your computer
        to the drone's AP (SSID usually `FLOW_xxxxxx`) the normal OS way
        before calling this.

        If `check_network` is True (default), this does a best-effort
        check that `drone_ip` is actually reachable from this computer's
        current network before opening the control socket.

          - If `require_network` is False (default), an unreachable
            result only PRINTS A WARNING and continues -- this is the
            original, lenient behavior, useful if this check is wrong
            for your setup (e.g. an unusual routing setup).
          - If `require_network` is True, an unreachable result instead
            RAISES V888Error and connect() does not proceed -- no socket
            is opened, no handshake is sent, and no control command can
            follow.

        `require_ssid_prefix` (e.g. "FLOW_") adds a SECOND, STRONGER
        check on top of the IP-route probe above: it reads the actual
        SSID this computer's Wi-Fi is currently associated with (via
        `get_connected_ssid()`) and requires it to start with this
        prefix, raising V888Error if it doesn't match (or can't be
        read). This catches a real false-positive the plain IP-route
        check misses: a computer can have a valid route to
        192.168.169.1 via a default gateway on a COMPLETELY DIFFERENT
        Wi-Fi network and still pass the IP check -- SSID checking
        catches that. Leave this None (default) to skip the SSID check
        (e.g. on a platform get_connected_ssid() doesn't support, or if
        your drone's SSID doesn't follow the usual FLOW_ naming).

        Neither check can guarantee the drone itself is listening -- the
        protocol has no handshake ACK -- these only catch "not on the
        drone's Wi-Fi at all", which is the common failure mode.

        If `start_sending` is False, the socket is opened, the handshake
        is still performed, and the background thread is started, but it
        will not actually transmit RC control packets until
        `_resume_sending()` is called (which `takeoff()` and
        `send_rc_control()` do automatically). This is useful if you
        want to call `streamon()` before any control packets go out --
        testing showed video stops responding once control packets start
        flowing, so getting the trigger out first, uncontested, matters.

        If `skip_handshake` is True, the HELLO/status-envelope/kickstart
        sequence is skipped entirely and the background loop starts
        immediately -- this is FOR TESTING ONLY (see
        `examples/v888_video_no_handshake_test.py` /
        `v888_video_with_handshake_test.py` for the paired experiment).

        CONFIRMED by a 3-way controlled comparison (skip_handshake=True
        x2, skip_handshake=False x1, all otherwise identical scripts):
        the handshake is NOT required for plain RC control (arming/
        stopping worked reliably either way), but IS required for a
        reliable video stream. Without it, streamon() got either 0
        frames or exactly 1 frame before stalling (inconsistent between
        runs); with it, streamon() delivered 169 clean frames over
        ~12s with no stalling. Leave skip_handshake=False (the default)
        for any real use, including control-only use -- there's no
        upside to skipping it, and it breaks video.

        BUGFIX: an earlier version of this method opened the socket and
        immediately started blasting RC packets, without ever sending
        the connection handshake (HELLO x2 -> a dozen status envelopes
        -> a short "kickstart" command pair). Those handshake constants
        existed in protocol.py but were never actually sent anywhere in
        this codebase -- dead code. Real capture confirmed the drone
        needs this handshake before it reliably starts listening to RC
        packets (this project's own protocol docs describe the exact
        sequence and note the kickstart pair gets RESENT if the drone
        doesn't respond promptly). Skipping it meant control commands
        sent shortly after connect() (e.g. arm_motors_test() called
        right away, with no streamon() first) could silently go
        nowhere -- a real user hit exactly this ("motor doesn't spin at
        all"). This version sends the full handshake synchronously
        before returning (unless skip_handshake=True).
        """
        if self._is_connected:
            return

        if check_network and not self._is_network_reachable():
            if require_network:
                raise V888Error(
                    f"Could not reach {self.drone_ip} on this computer's "
                    f"current network -- refusing to connect(). Check that "
                    f"you're connected to the drone's Wi-Fi access point "
                    f"(SSID usually starts with 'FLOW_'), then try again. "
                    f"(Pass require_network=False to downgrade this to a "
                    f"warning instead.)"
                )
            print(
                f"[v888] WARNING: could not reach {self.drone_ip} on this "
                f"computer's current network. Check that you're connected "
                f"to the drone's Wi-Fi access point (SSID usually starts "
                f"with 'FLOW_') before sending commands. Continuing anyway "
                f"in case this check is wrong for your setup."
            )

        if require_ssid_prefix is not None:
            current_ssid = get_connected_ssid()
            if current_ssid is None:
                raise V888Error(
                    f"require_ssid_prefix='{require_ssid_prefix}' was given, "
                    f"but this computer's current Wi-Fi SSID could not be "
                    f"determined (unsupported OS, no Wi-Fi adapter, missing "
                    f"command, or a permissions issue). Pass "
                    f"require_ssid_prefix=None to skip this check."
                )
            if not current_ssid.startswith(require_ssid_prefix):
                raise V888Error(
                    f"Connected to Wi-Fi '{current_ssid}', which does not "
                    f"start with '{require_ssid_prefix}' -- this doesn't "
                    f"look like the drone's access point. Connect to the "
                    f"drone's Wi-Fi (SSID usually starts with 'FLOW_') "
                    f"before calling connect()."
                )

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # A short receive timeout lets the video receiver thread (which
        # shares this socket, see streamon()) poll for incoming video
        # packets without blocking forever. Sending is unaffected by this.
        self._sock.settimeout(0.5)
        self._is_connected = True

        if not skip_handshake:
            self._send_handshake()

        self._sending_paused = not start_sending
        self._stop_event.clear()
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._send_thread.start()

    def _send_handshake(self) -> None:
        """
        Replay the connection handshake sequence confirmed by packet
        capture: HELLO x2 -> a dozen 88-byte status envelopes (counter
        0..11) -> a short unknown command -> two "camera/SSID" command
        strings. Real capture showed RC packets reliably start flowing
        only after this sequence; if delayed, the app was seen to
        RESEND the short-command + camera-string pair and RC started
        immediately after -- so this method mirrors that pacing rather
        than firing everything as fast as possible.
        """
        addr = (self.drone_ip, self.control_port)

        def send(data: bytes) -> None:
            try:
                self._sock.sendto(data, addr)
            except OSError:
                pass  # e.g. ICMP port-unreachable from the unused 8801 path; ignore

        send(proto.START_STREAM)
        send(proto.START_STREAM)

        for i in range(12):  # counter 0..11, matches captured handshake
            send(proto.build_packet(self._counters))
            time.sleep(0.025)

        send(proto.HANDSHAKE_UNK)
        send(proto.HANDSHAKE_CAM_2)
        send(proto.HANDSHAKE_CAM_3)
        time.sleep(1.0)  # let the drone settle into listening for RC before we start.
        # (bumped from an earlier 0.5s: a real test showed arm_motors_test()
        # called immediately after connect() with no extra caller-side delay
        # could fail to actually spin the motor, while the same call worked
        # reliably when the caller added an extra ~1s sleep first. Baking in
        # more margin here by default rather than relying on every caller to
        # remember their own delay.)

    def is_network_reachable(self) -> bool:
        """
        Public wrapper around the network-reachability probe used
        internally by connect(). Lets callers check "is this computer
        even on the drone's Wi-Fi?" BEFORE calling connect() or issuing
        any control command -- useful for a fail-fast guard at the top
        of a flight script, so a control command is never sent while
        not actually connected to the drone's network.

        Same caveat as the internal check: this only confirms a route
        to `drone_ip` exists on this computer's network. It cannot
        confirm the drone itself is listening (the protocol has no
        handshake ACK), so a True result is necessary but not
        sufficient for "the drone will respond".
        """
        return self._is_network_reachable()

    def _is_network_reachable(self) -> bool:
        """
        Best-effort check that `drone_ip` is on a network this computer can
        currently route to, without sending any drone protocol packets.

        This is NOT a guarantee the drone answers -- the WiFi-UAV control
        protocol has no handshake or ACK, so there is no reliable way to
        confirm the drone itself is listening. It only catches the more
        common failure of "this computer isn't even on the drone's Wi-Fi".
        """
        probe_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe_sock.settimeout(0.2)
            # connect() on a UDP socket doesn't send anything on the wire;
            # it just asks the OS to pick a local route/interface for this
            # destination. If the OS has no route at all (e.g. not on any
            # network in that address space), this raises OSError.
            probe_sock.connect((self.drone_ip, self.control_port))
            local_ip, _ = probe_sock.getsockname()
            return not local_ip.startswith("0.0.0.0")
        except OSError:
            return False
        finally:
            probe_sock.close()

    def end(self, land_first: bool = True, land_settle_time: float = 2.0) -> None:
        """
        Stop the background sender and close the socket.

        IMPORTANT: this protocol has no autonomous failsafe. If the socket
        just closes while the drone is airborne, the drone keeps flying
        with whatever the last received stick values were -- it does NOT
        land on its own. By default, `end()` therefore calls `land()`
        (held for its app-matched ~1.5s) and gives it `land_settle_time`
        extra seconds before actually closing the socket.

        Set `land_first=False` if you have already called `land()`
        yourself just before calling `end()`. In testing, sending a second
        land command (or any throttle command) immediately after a drone
        has already touched down and cut its motors has caused it to take
        off again instead of staying down -- so don't rely on this
        default double-checking behavior if you already know it landed.
        """
        if self._is_connected and land_first:
            try:
                self.land()
                time.sleep(land_settle_time)
            except Exception:
                pass

        self.streamoff()

        self._stop_event.set()
        if self._send_thread is not None:
            self._send_thread.join(timeout=1.0)
            self._send_thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self._is_connected = False

    def streamon(
        self,
        video_port: int = proto.DEFAULT_VIDEO_PORT,
        jpeg_width: int = 640,
        jpeg_height: int = 360,
        warmup_seconds: float = 1.0,
        send_acks: bool = True,
    ) -> None:
        """
        Start receiving and reassembling the drone's video stream.

        After calling this, use `get_frame_read().frame` to read the
        latest complete JPEG frame (as raw bytes), the same pattern as
        djitellopy's `streamon()` / `get_frame_read()`.

        `jpeg_width`/`jpeg_height` only affect the synthesized JPEG header
        used to make the drone's headerless fragments decodable -- if
        frames fail to decode, try adjusting these to match the drone's
        actual resolution.

        If `send_acks` is False, this stream's frame-completion ACK slots
        are never sent (see `VideoReceiver.send_acks`) -- FOR TESTING
        ONLY. CONFIRMED by real test: without ACKs, exactly one frame
        arrives and then the stream stalls completely (same behavior as
        Z908). Leave True (the default) for normal use -- there's no
        upside to disabling this outside of protocol experiments.

        IMPORTANT: testing showed the video-start trigger needs a brief,
        uncontested head start -- if RC control packets are already
        flowing when the trigger goes out, the drone may not respond with
        video at all. To get this right automatically, this method
        pauses the background RC sender (if it was already running),
        sends the trigger, waits `warmup_seconds` (default 1.0s), and
        only then resumes RC sending -- which by this point also carries
        the drone's video-frame ACKs (see `pending_ack_slots()`), keeping
        the stream alive going forward.
        """
        self._require_connected()
        if self._video is not None:
            return

        was_sending = not self._sending_paused
        self._pause_sending()

        self._video = VideoReceiver(
            shared_socket=self._sock,
            drone_ip=self.drone_ip,
            # V888 capture: only control_port (8800) ever responds; 8801
            # was tried by the stock app too but always got back ICMP
            # port-unreachable. Kept in the tuple anyway since sending to
            # it is harmless (VideoReceiver._safe_sendto swallows OSError)
            # and matches the app's own observed behavior.
            control_ports=(self.control_port, self.control_port + 1),
            video_port=video_port,
            jpeg_width=jpeg_width,
            jpeg_height=jpeg_height,
            send_acks=send_acks,
        )
        self._video.start()
        time.sleep(warmup_seconds)

        if was_sending:
            self._resume_sending()

    def streamoff(self) -> None:
        """Stop the video receiver, if running."""
        if self._video is not None:
            self._video.stop()
            self._video = None

    def get_frame_read(self) -> VideoFrame:
        """
        Returns the video frame reader. Call `streamon()` first.

        `.frame` on the returned object holds the latest complete JPEG
        frame as bytes (or None if nothing has arrived yet). Decode it
        with e.g.:

            import cv2
            import numpy as np
            frame_read = drone.get_frame_read()
            jpeg_bytes = frame_read.frame
            if jpeg_bytes:
                img = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        """
        if self._video is None:
            raise V888Error("Video stream not started. Call streamon() first.")
        return self._video.frame_read

    def __enter__(self) -> "V888":
        self.connect()
        return self

    def __exit__(self, *exc_info) -> None:
        self.end()

    # ------------------------------------------------------------------ #
    # Background send loop
    # ------------------------------------------------------------------ #
    def _send_loop(self) -> None:
        while not self._stop_event.is_set():
            self._send_current_state()
            time.sleep(self._send_interval)

    def _send_current_state(self) -> None:
        if self._sock is None or self._sending_paused:
            return
        ack_slots = self._video.pending_ack_slots() if self._video is not None else None
        with self._state_lock:
            packet = proto.build_control_packet(
                self._state,
                self._counters,
                ack_slots=ack_slots,
                active_camera_index=self._state.camera_index,
            )
        try:
            self._sock.sendto(packet, (self.drone_ip, self.control_port))
            self._send_failure_warned = False
            self._last_successful_send_time = time.time()
        except OSError as exc:
            # Don't spam the console at send_rate_hz -- warn immediately on
            # the first failure, then at most once every 2s while it's
            # still failing (not just once ever), so a long outage stays
            # visible instead of going quiet after the first line.
            now = time.time()
            if not self._send_failure_warned or (now - self._last_send_warning_time) >= 2.0:
                # Best-effort: report which SSID this computer is CURRENTLY
                # on when the failure happens. A real-world case this
                # caught: Windows silently roaming away from the drone's
                # AP mid-flight to a different known Wi-Fi network with
                # internet access (the drone's AP has none, which Windows'
                # own connectivity heuristics can deprioritize). Seeing
                # "now on: <other network>" in the log is a direct
                # confirmation of that, versus e.g. a real signal dropout
                # where the SSID would still read as the drone's AP.
                current_ssid = get_connected_ssid()
                ssid_note = (
                    f", currently associated with Wi-Fi: '{current_ssid}'"
                    if current_ssid is not None
                    else ""
                )
                print(
                    f"[v888] WARNING: failed to send control packet to "
                    f"{self.drone_ip}:{self.control_port} ({exc}). Check "
                    f"your Wi-Fi connection to the drone's access point. "
                    f"Will keep retrying silently. "
                    f"({self.seconds_since_last_successful_send():.1f}s since "
                    f"last successful send{ssid_note})"
                )
                self._send_failure_warned = True
                self._last_send_warning_time = now

    def seconds_since_last_successful_send(self) -> float:
        """
        How long it's been since a control packet was last actually
        handed off to the OS successfully (not since it was last
        *attempted* -- attempts happen at send_rate_hz regardless).

        Returns float('inf') if no packet has ever been sent
        successfully yet (e.g. called right after connect(), before the
        first send has had a chance to succeed).

        Use this from your own script's main thread to notice a link
        outage the background sender only logs to the console -- e.g.
        pause a scripted maneuver sequence, or bail out to land()/
        emergency_stop() attempts, if this grows too large mid-flight.
        Note: if the link is actually down, those recovery calls may
        also fail to send for the same reason: there is no way to
        command the drone at all while the OS can't route to it. This
        is meant for detecting and surfacing the outage promptly, not
        for guaranteeing a way out of it.
        """
        if self._last_successful_send_time == 0.0:
            return float("inf")
        return time.time() - self._last_successful_send_time

    def is_link_healthy(self, max_silence: float = 1.0) -> bool:
        """
        True if a control packet was successfully sent within the last
        `max_silence` seconds. At the default 50Hz send rate that's
        normally ~50 successful sends; a False result here means
        sending has actually been failing (OS-level send errors, e.g.
        Wi-Fi association lost), not just that the drone hasn't
        acknowledged anything (this protocol has no ACK for RC packets
        at all, success here only means the OS accepted the packet for
        transmission).
        """
        return self.seconds_since_last_successful_send() <= max_silence

    def _require_connected(self) -> None:
        if not self._is_connected:
            raise V888Error("Not connected. Call connect() first.")

    # ------------------------------------------------------------------ #
    # Flight control - djitellopy-style API
    # ------------------------------------------------------------------ #
    def takeoff(self, flag_hold_time: float = 1.5) -> None:
        """
        Hold the one-key takeoff/land flag on for `flag_hold_time` seconds
        (default 1.5s), stick otherwise neutral. Packet capture of the
        stock app pressing this button showed the flag held high for
        about 1.16-1.24s, so 1.5s gives a small margin over that while
        still being a deliberate "press and release", not a held-down
        button.

        CONFIRMED (user-verified real flight test): this flag toggle
        performs arm + takeoff together and is fully sufficient on its
        own -- no throttle-arming gesture is needed first. The separate
        `arm_motors_test()` gesture is NOT a prerequisite step for
        takeoff; it's an independent ground-test-only path (motors spin
        but the drone does NOT lift off), confirmed by direct observation
        of a real unit: throttle-up pulse alone spins the motors without
        flight, and throttle-down pulse alone stops them again.
        """
        self._require_connected()
        self._resume_sending()
        self._pulse_flag("takeoff_or_land", flag_hold_time)

    def land(self, flag_hold_time: float = 1.5) -> None:
        """
        Hold the one-key takeoff/land flag on for `flag_hold_time` seconds
        (default 1.5s) while airborne, with all other sticks left at
        whatever they currently are (normally neutral after hover()).

        This default was tuned from a packet capture of the stock app
        landing successfully from head height: the app held the flag for
        about 1.19s with throttle/pitch/roll/yaw all neutral the whole
        time -- it did not push throttle down. An earlier, shorter
        (0.5s) hold and a throttle-down variant (`land_descend()`) were
        both tried and left the drone hovering just above the ground
        instead of touching down; matching the app's actual timing is the
        current best-known-good approach.

        IMPORTANT: once the flag pulse finishes, this STOPS the background
        sender entirely (no more packets go out at all) instead of falling
        back to sending neutral packets forever. In testing, continuing to
        send neutral-stick packets after a successful landing made the
        drone lift off again. If you call takeoff() or send_rc_control()
        again afterward, sending automatically resumes.
        """
        self._require_connected()
        self._pulse_flag("takeoff_or_land", flag_hold_time)
        self._pause_sending()

    def _pause_sending(self) -> None:
        """Stop the background sender from transmitting anything further,
        without closing the socket or tearing down the thread. Call
        `_resume_sending()` (done automatically by takeoff()/
        send_rc_control()) to start sending again."""
        self._sending_paused = True

    def _resume_sending(self) -> None:
        self._sending_paused = False

    def land_descend(
        self,
        descend_seconds: float = 3.0,
        throttle_percent: float = -40.0,
    ) -> None:
        """
        EXPERIMENTAL fallback, not the primary landing method.

        Pushes the throttle stick down to `throttle_percent` (default -40)
        for `descend_seconds` (default 3s) while continuously holding the
        one-key land flag. In testing this did NOT land the drone (it
        continued hovering near the ground and eventually drifted into a
        wall), so prefer `land()` with its app-matched 1.5s hold. This
        method is kept only as a documented "what we already tried and it
        didn't help" reference, and as a possible building block if future
        captures reveal a real reason a controlled descent should work.

        Returns the stick to neutral and clears the land flag at the end
        regardless of whether the drone actually touched down -- this
        method cannot detect touchdown since the protocol has no telemetry
        channel back from the drone.
        """
        self._require_connected()
        axis_value = proto.percent_to_axis(throttle_percent)
        with self._state_lock:
            self._state.throttle = axis_value
            self._state.takeoff_or_land = True
        time.sleep(descend_seconds)
        with self._state_lock:
            self._state.throttle = proto.NEUTRAL_AXIS
            self._state.takeoff_or_land = False

    def _pulse_flag(self, flag_name: str, hold_time: float) -> None:
        """Set a one-shot command flag, hold it for `hold_time` seconds
        (so it survives several send ticks even under packet loss), then
        clear it."""
        with self._state_lock:
            setattr(self._state, flag_name, True)
        time.sleep(hold_time)
        with self._state_lock:
            setattr(self._state, flag_name, False)

    def emergency_stop(self, flag_hold_time: float = 1.5) -> None:
        """Cut the motors immediately, regardless of current flight state.

        CONFIRMED by direct real-flight testing on V888 (not just inferred
        from other drones): pressing this while hovering caused the motors
        to cut instantly and the drone to drop immediately -- there is no
        graceful descent. Pressing it while already on the ground (motors
        spinning but not lifting, e.g. after a land() that didn't fully
        disarm) simply cuts the motors with no drop, since there's no
        height to fall from. Both are the SAME underlying behavior
        (unconditional motor cutoff), confirmed on V888 in both states.

        V888 capture confirmed this flag (bit 1) is exactly the app's
        dedicated "Stop" button, observed held ~1.17s in real app capture
        -- default here is 1.5s for a small safety margin, matching
        takeoff()/land(). HOWEVER, a dedicated ground threshold test
        (arm motors, then emergency_stop() with progressively shorter
        flag_hold_time) confirmed the drone stops reliably at every
        duration tested, all the way down to 0.05s (roughly 1-2 packets
        at the ~50Hz send rate) -- no minimum hold time / debounce was
        found. The ~1.2s seen in real app captures appears to just be
        how long a human held the physical button down, not a
        requirement of the drone itself. In other words: 1.5s here is a
        generous default, not a necessary one -- callers needing a
        faster reaction (e.g. automated obstacle-avoidance triggers) can
        safely use a much shorter flag_hold_time.

        DO NOT use this as a landing method. Only use it for a genuine
        emergency (e.g. imminent collision) where an uncontrolled drop is
        preferable to the alternative.

        CRITICAL follow-up safety note: after this returns, do NOT leave
        the background sender running indefinitely while doing nothing
        (e.g. blocked on input() waiting for a person to check the
        drone). Continuing to send neutral packets after the drone has
        stopped/landed has been observed (elsewhere in this project, on
        related drones in the same protocol family) to cause the drone
        to spontaneously take off again. Call end(land_first=False)
        promptly after a stop you intend to be final, rather than
        leaving the connection idling.
        """
        self._require_connected()
        self._pulse_flag("stop", flag_hold_time)

    def calibrate(self, flag_hold_time: float = 1.5) -> None:
        """Trigger gyro calibration / "check" (drone should be level and
        stationary on the ground).

        CONFIRMED by real V888 capture: flag bit 2 (0x04), held ~1.2s,
        reproduced identically across two separate presses. Default hold
        time bumped to 1.5s here (from an earlier, untested 0.5s guess) to
        match the confirmed ~1.2s + the same small safety margin used by
        takeoff()/land()/emergency_stop().
        """
        self._require_connected()
        self._pulse_flag("calibrate", flag_hold_time)

    def flip(self, flag_hold_time: float = 1.5) -> None:
        """Trigger a flip / 360 roll, if supported by the connected
        hardware variant.

        LIKELY NOT USABLE from this library / the WiFi app at all: per
        user confirmation, the 360-degree flip is a physical-RF-remote-
        only feature with no button in the WiFi app. Since the physical
        remote was confirmed to use a completely separate RF channel
        (never WiFi -- see the class docstring's "Physical remote
        control note"), sending flag bit 3 (0x08) over WiFi here may
        simply be ignored by the drone the same way any WiFi command is
        ignored while the RF remote has priority, or may just not be
        wired up to anything on this drone's firmware at all. This is a
        different situation from bits 0/1/2 (all confirmed reachable
        from the WiFi app) -- kept here for protocol completeness and
        in case a future firmware/app variant exposes it, but don't
        expect it to do anything on stock V888 hardware.
        """
        self._require_connected()
        self._pulse_flag("flip", flag_hold_time)

    def set_headless_mode(self, enabled: bool) -> None:
        """Toggle headless mode (0x02=off, 0x03=on in the command sub-field).

        CONFIRMED by real V888 capture: this is a PERSISTENT value (like
        camera index), not a momentary pulse like takeoff/land/stop/
        calibrate/flip -- once set, it's held at that value on every
        outgoing packet until changed again. Matches exactly what was
        predicted in protocol.py before this was captured.
        """
        self._require_connected()
        with self._state_lock:
            self._state.headless = bool(enabled)

    def set_follow_me_mode(self, enabled: bool) -> None:
        """Toggle "Follow Me" mode.

        CONFIRMED by real V888 capture: this shares the SAME offset25
        byte as headless mode (it's not a separate command) -- headless
        off/on/follow-me show up as 0x02/0x03/0x06 respectively, i.e. a
        mode bitfield (see protocol.MODE_* constants) rather than a
        simple binary flag. Like headless, this is a persistent value,
        held on every outgoing packet until changed again.

        During Follow Me, the stock app was observed continuously
        sending small, changing roll/pitch adjustments on its own (no
        user stick input) -- presumably computed client-side from the
        phone's own sensors/location, since this protocol has no
        drone-side telemetry channel. This library does NOT replicate
        that automatic tracking logic; enabling this mode here only
        sets the mode bit; actually implementing "follow the phone" is
        left to the caller (e.g. by calling send_rc_control() based on
        the phone's own GPS/motion, if you want to replicate the stock
        app's behavior).

        Whether headless and follow-me can be combined (0x07) is
        unconfirmed -- no capture has shown that value.
        """
        self._require_connected()
        with self._state_lock:
            self._state.follow_me = bool(enabled)

    # NOTE: a set_camera_tilt() method was tried here based on turbodrone's
    # decompiled findings (a camera-tilt control bit pattern), but
    # real-world testing showed no effect on this drone. Camera tilt is
    # confirmed to exist on the physical RF remote but is apparently not
    # exposed over the WiFi-UAV protocol -- removed rather than keeping
    # a non-functional method around.

    # Per-camera JPEG decode hints -- CONFIRMED for V888 directly (not
    # just inherited from Z908): a real V888 downward-camera capture
    # (120x160 frames, camera_index=1) was decoded offline at quality
    # 50/75/90/100 and compared visually -- quality=50 decoded as pure
    # noise/corruption, quality=100 decoded cleanly. Main camera (index
    # 0, 640x360) decodes correctly with the default (50). Same mapping
    # as Z908, now independently verified on V888.
    _CAMERA_JPEG_QUALITY = {0: 50, 1: 100}

    def set_camera_index(self, index: int) -> None:
        """
        Select which camera feed the drone streams: 0 = main/front
        camera, 1 = secondary/downward camera. Unlike the takeoff/land
        style one-shot flags, this is a held value -- decompiled app
        logic (turbodrone's wifi_uav.md, `nativeSetCameraIndex`) embeds
        the active camera index in every outgoing control/ACK packet, so
        it must keep being sent continuously (which the regular send loop
        already does once this is set) rather than as a single command.

        Also adjusts the video receiver's guessed JPEG quality to match
        what testing showed each camera needs to decode correctly (see
        `_CAMERA_JPEG_QUALITY`), if a video stream is currently running.
        """
        if index not in (0, 1):
            raise ValueError("camera_index must be 0 (main) or 1 (secondary)")
        self._require_connected()
        with self._state_lock:
            self._state.camera_index = index
        if self._video is not None:
            self._video.jpeg_quality = self._CAMERA_JPEG_QUALITY.get(index, 50)

    def arm_motors_test(self, direction: str = "up") -> None:
        """
        V888-specific: replays the brief throttle PULSE observed in a
        separate (non-flight) capture, where a ~0.1s throttle spike to
        near-max (0xb3->0xff) armed the motors, and a ~0.13s pulse to
        0x00 stopped them -- with the command flag byte staying 0x00
        throughout (i.e. NOT the takeoff/land or stop flag bits at all).

        CONFIRMED (user-verified real flight test): this is a GROUND-TEST
        ONLY path, fully independent of takeoff()/land()/emergency_stop().
        direction="up" spins the motors WITHOUT the drone lifting off;
        direction="down" stops them again. It is NOT a prerequisite for
        takeoff() -- takeoff()'s flag toggle arms AND lifts off on its
        own, with no throttle pulse needed first. Use this only if you
        want to spin-test the motors on the ground without flying.

        BUGFIX: an earlier version of this method only mutated
        self._state.throttle and slept, relying on the background
        _send_loop thread to happen to pick up and transmit each value
        within its ~30ms window. That's unreliable -- with a 50Hz send
        interval (20ms) and normal thread/GIL scheduling jitter, it's
        easy for a given throttle value to never actually go out on the
        wire, especially for such short pulses (a real user reported
        the motors not spinning at all, consistent with this). This
        version sends packets directly and explicitly at each stage
        (independent of the background loop's timing) to guarantee the
        pulse is actually transmitted.
        """
        self._require_connected()
        if direction == "up":
            sequence = [(0xB3, 0.05), (0xFF, 0.05)]
        elif direction == "down":
            sequence = [(0x00, 0.15)]
        else:
            raise ValueError("direction must be 'up' or 'down'")

        send_interval = min(self._send_interval, 0.02)  # at least ~50Hz during the pulse
        for throttle_value, hold_seconds in sequence:
            with self._state_lock:
                self._state.throttle = throttle_value
            stage_end = time.monotonic() + hold_seconds
            while time.monotonic() < stage_end:
                self._send_current_state()
                time.sleep(send_interval)

        with self._state_lock:
            self._state.throttle = proto.NEUTRAL_AXIS
        # Send a few explicit neutral packets too, rather than trusting
        # the background loop to get to it before the caller moves on.
        for _ in range(3):
            self._send_current_state()
            time.sleep(send_interval)

    # ------------------------------------------------------------------ #
    # Continuous stick control - djitellopy-style API
    # ------------------------------------------------------------------ #
    def send_rc_control(
        self,
        left_right: float,
        forward_back: float,
        up_down: float,
        yaw: float,
    ) -> None:
        """
        Set continuous stick values, each in the range -100..100, matching
        djitellopy's `send_rc_control(left_right, forward_back, up_down,
        yaw_velocity)` signature. Values are applied immediately and kept
        until changed again; the background thread re-sends them at
        `send_rate_hz`.

        Positive left_right = right, positive forward_back = forward,
        positive up_down = up, positive yaw = clockwise turn.
        """
        self._require_connected()
        self._resume_sending()
        with self._state_lock:
            self._state.roll = proto.percent_to_axis(left_right)
            self._state.pitch = proto.percent_to_axis(forward_back)
            self._state.throttle = proto.percent_to_axis(up_down)
            self._state.yaw = proto.percent_to_axis(yaw)

    def hover(self) -> None:
        """Convenience: set all sticks back to neutral (centred)."""
        self.send_rc_control(0, 0, 0, 0)

    # ------------------------------------------------------------------ #
    # Introspection helpers (no telemetry from the drone is available)
    # ------------------------------------------------------------------ #
    def get_current_state(self) -> dict:
        """Return the locally-tracked control state (not drone telemetry;
        this protocol has no return channel for flight state)."""
        with self._state_lock:
            return {
                "roll": self._state.roll,
                "pitch": self._state.pitch,
                "throttle": self._state.throttle,
                "yaw": self._state.yaw,
                "headless": self._state.headless,
                "camera_index": self._state.camera_index,
            }