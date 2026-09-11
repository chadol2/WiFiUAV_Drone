"""
V888 드론 영상 뷰어
====================
- Z908용 drone_viewer.py를 V888에 맞게 이식 (같은 WiFi-UAV 프로토콜 계열이라
  구조는 거의 동일 -- KY UFO D4의 drone_viewer.py 스타일 Tkinter UI)
- v888 패키지의 V888 클래스로 영상 수신 (UDP 기반, 헤더 없는 JPEG를
  라이브러리 내부에서 합성해 표준 JPEG로 재구성 — RTSP 아님)
- 카메라 전환: V888.set_camera_index(0=전방/1=하방) - envelope offset 86 필드,
  계속 유지해서 보내야 하는 값(원터치 명령 아님). Z908과 정확히 같은 offset,
  같은 방식으로 V888에서도 실측 확인됨.
- 하방 카메라는 원본 해상도가 훨씬 작음(V888 실측: 120x160, 전방은 640x360) -
  항상 640x360으로 업스케일해서 표시(실제 디테일이 늘어나는 건 아니고 화면
  크기만 통일)
- 카메라별 JPEG quality 값(전방=50, 하방=100)도 V888 자체로 실측 검증됨 -
  드론이 실제 양자화표를 안 보내서 수신측이 추측해야 하는데, 하방 카메라는
  quality=50으로 디코딩하면 완전히 깨짐(노이즈), quality=100이라야 정상 디코딩됨.
  V888.set_camera_index()가 이 매핑을 자동 적용함.
- 캡처/녹화/줌 지원

참고: 카메라 각도(틸트) 조절 기능은 물리 조종기 전용으로 확인됨(WiFi
캡처로 조종기+앱 동시 연결 테스트 시, 조종기로는 카메라 각도 조정이 실제로
동작했지만 이 조작이 WiFi 트래픽에는 전혀 나타나지 않았음 — 물리 조종기가
드론과 WiFi가 아닌 별도 RF 채널로 직접 통신하기 때문). 즉 이 뷰어(WiFi 앱
경로)로는 카메라 틸트를 제어할 수 없음 — Z908과 동일한 제약.

참고 2: 물리 조종기가 동시에 켜져 있으면, 먼저 활성 제어를 잡은 쪽이
우선권을 가짐(V888 실측 확인) — 이 뷰어로 먼저 연결해두면 나중에 조종기를
켜도 이 뷰어가 계속 우선권을 유지하지만, 반대로 조종기가 이미 활성 상태에서
연결하면 이 뷰어의 명령(카메라 전환 등)이 무시될 수 있음.
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import os
import datetime
import cv2
import numpy as np
from PIL import Image, ImageTk, ImageDraw, ImageFont
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from v888 import V888

RECORD_DIR = os.path.join(os.path.expanduser("~"), "drone_captures")
os.makedirs(RECORD_DIR, exist_ok=True)

DISPLAY_W = 640
DISPLAY_H = 360

# V888 set_camera_index() 인자: 0=전방(main/front), 1=하방(secondary/downward)
CAM_FRONT = 0
CAM_DOWN = 1


class DroneViewer:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("V888 — 영상 뷰어")
        self.root.configure(bg="#1a1a2e")
        self.root.resizable(False, False)

        self._running = False
        self._recording = False
        self._writer = None
        self._zoom = 1.0
        self._fps = 0.0
        self._fps_count = 0
        self._fps_time = time.time()
        self._cam_index = CAM_FRONT
        self._display_frame = None  # 화면에 그릴 최종(줌 적용) 프레임
        self._raw_w = 0
        self._raw_h = 0
        self._record_w = DISPLAY_W  # locked in at recording start, see _on_record_toggle
        self._record_h = DISPLAY_H
        self._last_frame_count = -1
        self._cam_switch_until = 0.0  # 이 시각까지는 들어오는 프레임을 화면에 반영하지 않음
        self._switching_label = ""
        self._overlay_font = None  # lazily loaded, see _get_overlay_font()

        self._drone = V888()
        self._drone.connect(start_sending=False)
        self._drone.streamon()
        self._drone._resume_sending()
        self._frame_read = self._drone.get_frame_read()

        self._build_ui()
        self._running = True
        threading.Thread(target=self._stream_loop, daemon=True).start()
        self._update_canvas()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ #
    # UI
    # ------------------------------------------------------------------ #
    def _build_ui(self):
        self.canvas = tk.Canvas(self.root, width=DISPLAY_W, height=DISPLAY_H,
                                 bg="#000000", highlightthickness=0)
        self.canvas.grid(row=0, column=0)

        sb = tk.Frame(self.root, bg="#0f0f23", height=28)
        sb.grid(row=1, column=0, sticky="ew")
        self.lbl_fps = tk.Label(sb, text="FPS: --", bg="#0f0f23", fg="#00ff88", font=("Consolas", 9))
        self.lbl_fps.pack(side="left", padx=10)
        self.lbl_res = tk.Label(sb, text="--×--", bg="#0f0f23", fg="#888888", font=("Consolas", 9))
        self.lbl_res.pack(side="left", padx=10)
        self.lbl_zoom = tk.Label(sb, text="줌 1.0×", bg="#0f0f23", fg="#ffaa00", font=("Consolas", 9))
        self.lbl_zoom.pack(side="left", padx=10)
        self.lbl_cam = tk.Label(sb, text="📷 전방", bg="#0f0f23", fg="#aaaaff", font=("Consolas", 9))
        self.lbl_cam.pack(side="left", padx=10)
        self.lbl_rec = tk.Label(sb, text="", bg="#0f0f23", fg="#ff3333", font=("Consolas", 9, "bold"))
        self.lbl_rec.pack(side="right", padx=10)
        self.lbl_status = tk.Label(sb, text="연결 중...", bg="#0f0f23", fg="#aaaaaa", font=("Consolas", 9))
        self.lbl_status.pack(side="right", padx=10)

        ctrl = tk.Frame(self.root, bg="#16213e", pady=10)
        ctrl.grid(row=2, column=0, sticky="ew")
        bs = dict(font=("Segoe UI", 10, "bold"), width=10, height=2,
                  bd=0, cursor="hand2", activeforeground="#ffffff")

        # 카메라 선택 버튼 2개 (전환 토글이 아니라 직접 선택 - KY UFO D4보다 더 단순/직관적)
        self.btn_front = tk.Button(ctrl, text="📷 전방",
                                    bg="#2d4a7a", fg="#ffffff", activebackground="#3d5a8a",
                                    command=lambda: self._on_camera_select(CAM_FRONT), **bs)
        self.btn_front.grid(row=0, column=0, padx=8, pady=4)

        self.btn_down = tk.Button(ctrl, text="📷 하방",
                                   bg="#2a2a4a", fg="#ffffff", activebackground="#3d5a8a",
                                   command=lambda: self._on_camera_select(CAM_DOWN), **bs)
        self.btn_down.grid(row=0, column=1, padx=8, pady=4)

        self.btn_cap = tk.Button(ctrl, text="🖼 캡처",
                                  bg="#2d6a4f", fg="#ffffff", activebackground="#3d7a5f",
                                  command=self._on_capture, **bs)
        self.btn_cap.grid(row=0, column=2, padx=8, pady=4)

        self.btn_rec = tk.Button(ctrl, text="⏺ 녹화\n시작",
                                  bg="#7a2d2d", fg="#ffffff", activebackground="#8a3d3d",
                                  command=self._on_record_toggle, **bs)
        self.btn_rec.grid(row=0, column=3, padx=8, pady=4)

        zf = tk.Frame(ctrl, bg="#16213e")
        zf.grid(row=0, column=4, padx=16)
        tk.Label(zf, text="줌", bg="#16213e", fg="#ffaa00", font=("Segoe UI", 9)).pack()
        self.zoom_var = tk.DoubleVar(value=1.0)
        ttk.Scale(zf, from_=1.0, to=4.0, orient="horizontal",
                  variable=self.zoom_var, length=140,
                  command=self._on_zoom).pack(pady=2)
        zbf = tk.Frame(zf, bg="#16213e")
        zbf.pack()
        for txt, arg in [("−", -0.5), ("원본", 0), ("+", 0.5)]:
            cmd = self._zoom_reset if arg == 0 else (lambda d=arg: self._zoom_step(d))
            tk.Button(zbf, text=txt, bg="#2a2a4a", fg="#ffffff",
                      font=("Segoe UI", 9), width=4, bd=0,
                      cursor="hand2", command=cmd).pack(side="left", padx=2)

        self._update_camera_button_styles()

        tk.Label(self.root, text=f"저장: {RECORD_DIR}",
                 bg="#1a1a2e", fg="#444466",
                 font=("Consolas", 8)).grid(row=3, column=0, pady=(0, 4))

    def _update_camera_button_styles(self):
        active_bg, inactive_bg = "#2d4a7a", "#2a2a4a"
        self.btn_front.config(bg=active_bg if self._cam_index == CAM_FRONT else inactive_bg)
        self.btn_down.config(bg=active_bg if self._cam_index == CAM_DOWN else inactive_bg)

    # ------------------------------------------------------------------ #
    # Video pull loop
    # ------------------------------------------------------------------ #
    def _stream_loop(self):
        """V888's VideoReceiver runs its own background threads already
        (started by streamon()); this loop just polls for new completed
        frames and applies resize/zoom, mirroring drone_viewer.py's
        structure even though there's no RTSP/OpenCV VideoCapture here."""
        self._set_status("스트리밍 중")
        while self._running:
            jpeg_bytes = self._frame_read.frame
            count = self._frame_read.frame_count
            if jpeg_bytes is not None and count != self._last_frame_count:
                self._last_frame_count = count
                # Right after switching cameras, the drone may still be
                # finishing fragments from the OLD camera, or send a
                # transitional/corrupted frame at the wrong resolution
                # for a moment. Drop frames until _cam_switch_until has
                # passed instead of showing whatever briefly comes through.
                if time.time() < self._cam_switch_until:
                    continue
                arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    self._raw_h, self._raw_w = frame.shape[:2]
                    if self._recording and self._writer:
                        # Record at the camera's native resolution (locked
                        # in at _on_record_toggle when recording started),
                        # NOT upscaled to DISPLAY_W x DISPLAY_H -- upscaling
                        # a small source (e.g. downward camera's 120x160)
                        # doesn't add real detail, it just blurs the file.
                        # Camera switching is disabled while recording (see
                        # _on_record_toggle/_update_camera_button_styles),
                        # so the frame size here should already match the
                        # writer's fixed size in the normal case; the
                        # resize is just a safety net in case a frame
                        # arrives at a slightly different size than
                        # expected (e.g. a stray transitional frame).
                        if (self._raw_w, self._raw_h) != (self._record_w, self._record_h):
                            to_write = cv2.resize(
                                frame, (self._record_w, self._record_h),
                                interpolation=cv2.INTER_LINEAR)
                        else:
                            to_write = frame
                        self._writer.write(to_write)
                    self._display_frame = self._apply_zoom(frame)
                    self._fps_count += 1
                    now = time.time()
                    if now - self._fps_time >= 1.0:
                        self._fps = self._fps_count / (now - self._fps_time)
                        self._fps_count = 0
                        self._fps_time = now
            time.sleep(0.01)

    def _apply_zoom(self, frame):
        # Always normalize to DISPLAY_W x DISPLAY_H first -- this is what
        # makes the downward camera's much smaller native resolution
        # (e.g. 120x160) display at the same window size as the front
        # camera (640x360) instead of shrinking the window or leaving
        # black borders.
        h, w = frame.shape[:2]
        if w != DISPLAY_W or h != DISPLAY_H:
            frame = cv2.resize(frame, (DISPLAY_W, DISPLAY_H), interpolation=cv2.INTER_LINEAR)
        if self._zoom <= 1.0:
            return frame
        h, w = DISPLAY_H, DISPLAY_W
        ch, cw = int(h / self._zoom), int(w / self._zoom)
        y1 = (h - ch) // 2
        y2 = y1 + ch
        x1 = (w - cw) // 2
        x2 = x1 + cw
        return cv2.resize(frame[y1:y2, x1:x2], (DISPLAY_W, DISPLAY_H), interpolation=cv2.INTER_LINEAR)

    def _get_overlay_font(self):
        """Lazily load a Korean-capable TrueType font for drawing text
        directly onto video frames with PIL (cv2.putText cannot render
        Korean at all). Falls back through a few common Windows/Linux/
        Mac font paths, and finally to PIL's built-in bitmap font (which
        will show boxes for Korean, but at least won't crash)."""
        if self._overlay_font is not None:
            return self._overlay_font
        candidates = [
            r"C:\Windows\Fonts\malgun.ttf",       # 맑은 고딕 (Windows 기본 한글 폰트)
            r"C:\Windows\Fonts\malgunbd.ttf",
            "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
            "/System/Library/Fonts/AppleSDGothicNeo.ttc",
        ]
        for path in candidates:
            if os.path.exists(path):
                try:
                    self._overlay_font = ImageFont.truetype(path, 24)
                    return self._overlay_font
                except Exception:
                    continue
        self._overlay_font = ImageFont.load_default()
        return self._overlay_font

    def _update_canvas(self):
        if self._display_frame is not None:
            rgb = cv2.cvtColor(self._display_frame, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)
            if time.time() < self._cam_switch_until:
                # cv2.putText only supports its own bitmap font (no
                # Korean/Unicode glyphs -- text comes out as garbled boxes
                # or '?'). Draw the overlay with PIL/ImageDraw instead,
                # which can render Korean using a system font.
                draw = ImageDraw.Draw(pil_img)
                text = f"카메라 전환 중 ({self._switching_label})..."
                font = self._get_overlay_font()
                bbox = draw.textbbox((0, 0), text, font=font)
                tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
                tx = (DISPLAY_W - tw) // 2
                ty = (DISPLAY_H - th) // 2
                # simple outline for readability over any background
                for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    draw.text((tx + dx, ty + dy), text, font=font, fill=(0, 0, 0))
                draw.text((tx, ty), text, font=font, fill=(0, 255, 255))
            img = ImageTk.PhotoImage(pil_img)
            self.canvas.create_image(0, 0, anchor="nw", image=img)
            self.canvas._img = img
            self.lbl_fps.config(text=f"FPS: {self._fps:.1f}")
            self.lbl_res.config(text=f"원본 {self._raw_w}×{self._raw_h}")
            self.lbl_zoom.config(text=f"줌 {self._zoom:.1f}×")
            if self._recording:
                dot = "●" if int(time.time() * 2) % 2 == 0 else "○"
                self.lbl_rec.config(text=f"{dot} 녹화 중")
        if self._running:
            self.root.after(33, self._update_canvas)

    # ------------------------------------------------------------------ #
    # Camera select
    # ------------------------------------------------------------------ #
    def _on_camera_select(self, cam_index: int):
        if self._recording:
            # Buttons are disabled while recording (see _on_record_toggle),
            # but guard here too in case this is ever called some other
            # way -- switching cameras mid-recording would change the
            # frame size out from under the already-opened VideoWriter.
            self._set_status("녹화 중에는 카메라를 전환할 수 없습니다")
            return
        if cam_index == self._cam_index:
            return
        self._cam_index = cam_index
        self._drone.set_camera_index(cam_index)  # also auto-applies the
        # right guessed JPEG quality per camera (V888._CAMERA_JPEG_QUALITY,
        # confirmed by direct V888 testing -- see module docstring above)
        label = "전방" if cam_index == CAM_FRONT else "하방"
        self._switching_label = label
        # Hold the display (drop incoming frames) briefly so any leftover
        # fragments from the old camera, or a transitional/corrupted frame
        # at the wrong resolution, don't flash on screen during the switch.
        self._cam_switch_until = time.time() + 0.5
        self.lbl_cam.config(text=f"📷 {label} (전환 중)")
        self._update_camera_button_styles()
        self._set_status(f"카메라 → {label} 전환 중...")
        self.root.after(550, self._finish_camera_switch_label)

    def _finish_camera_switch_label(self):
        label = "전방" if self._cam_index == CAM_FRONT else "하방"
        self.lbl_cam.config(text=f"📷 {label}")
        self._set_status(f"카메라 → {label}")

    # ------------------------------------------------------------------ #
    # Capture / record
    # ------------------------------------------------------------------ #
    def _on_capture(self):
        if self._display_frame is None:
            self._set_status("캡처 실패 — 영상 없음")
            return
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(RECORD_DIR, f"capture_{ts}.png")
        cv2.imwrite(path, self._display_frame)
        self._set_status(f"캡처: capture_{ts}.png")
        self.btn_cap.config(bg="#52b788")
        self.root.after(400, lambda: self.btn_cap.config(bg="#2d6a4f"))

    def _on_record_toggle(self):
        if not self._recording:
            if self._display_frame is None:
                self._set_status("녹화 실패 — 영상 없음")
                return
            # Lock in the CURRENT camera's native resolution for this
            # recording (e.g. 640x360 for front, 120x160 for down on V888)
            # instead of always upscaling to DISPLAY_W x DISPLAY_H --
            # upscaling a small source doesn't add real detail, just blur.
            self._record_w = self._raw_w or DISPLAY_W
            self._record_h = self._raw_h or DISPLAY_H
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(RECORD_DIR, f"record_{ts}.mp4")
            self._writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*"mp4v"), 20.0,
                (self._record_w, self._record_h))
            self._recording = True
            self.btn_rec.config(text="⏹ 녹화\n중지", bg="#c94040")
            self._set_status(
                f"녹화: record_{ts}.mp4 ({self._record_w}×{self._record_h})")
            # Disable camera switching while recording -- switching mid-
            # recording would change the frame size out from under the
            # already-opened VideoWriter (which is fixed at open time).
            self.btn_front.config(state="disabled")
            self.btn_down.config(state="disabled")
        else:
            self._recording = False
            if self._writer:
                self._writer.release()
                self._writer = None
            self.btn_rec.config(text="⏺ 녹화\n시작", bg="#7a2d2d")
            self.lbl_rec.config(text="")
            self._set_status("녹화 저장 완료")
            self.btn_front.config(state="normal")
            self.btn_down.config(state="normal")

    # ------------------------------------------------------------------ #
    # Zoom
    # ------------------------------------------------------------------ #
    def _on_zoom(self, val):
        self._zoom = round(float(val) * 2) / 2

    def _zoom_step(self, delta):
        self._zoom = max(1.0, min(4.0, self._zoom + delta))
        self.zoom_var.set(self._zoom)

    def _zoom_reset(self):
        self._zoom = 1.0
        self.zoom_var.set(1.0)

    # ------------------------------------------------------------------ #
    def _set_status(self, msg):
        try:
            self.root.after(0, lambda: self.lbl_status.config(text=msg))
        except Exception:
            pass

    def _on_close(self):
        self._running = False
        if self._recording:
            self._on_record_toggle()
        self._drone.streamoff()
        self._drone.end(land_first=False)  # video-only viewer, not flying
        self.root.destroy()


def main():
    root = tk.Tk()
    DroneViewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()