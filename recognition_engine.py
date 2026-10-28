"""
=============================================================================
 ROS Attendance Bot — Recognition Engine (Pi 4B Optimized)
 Multi-threaded, zero-latency camera pipeline for autonomous robotics.
 
 Architecture:
   [CameraGrabber Thread]  --> Always holds latest frame (0 latency, drops old frames)
   [Inference Worker]      --> Processes latest frame with HOG (downscaled 0.33x)
   [UI Stream Thread]      --> Renders frame + cached detections at smooth 30 FPS
=============================================================================
"""

import sys
import time
import pickle
import platform
import logging
import threading
from collections import defaultdict
from pathlib import Path

import cv2
import face_recognition
import numpy as np

import config

log = logging.getLogger(__name__)


class RecognitionEngine:
    """
    High-performance, non-blocking face recognition engine optimized for Raspberry Pi 4B.
    Decouples video acquisition from neural embeddings computation to ensure
    smooth 30 FPS robot camera telemetry without frame backlog lag.
    """

    def __init__(self, attendance_manager, on_face_detected=None, on_frame=None, on_status=None, on_unknown_detected=None):
        self.attendance          = attendance_manager
        self.on_face_detected    = on_face_detected or (lambda *a: None)
        self.on_unknown_detected = on_unknown_detected or (lambda *a: None)
        self.on_frame            = on_frame         or (lambda f: None)
        self.on_status           = on_status        or (lambda s: None)

        self._running         = False
        self._paused          = False
        self._cap             = None

        # Thread synchronization
        self._frame_lock      = threading.Lock()
        self._latest_raw      = None
        self._latest_annotated = None
        self._new_frame_event = threading.Event()

        # Detections cache: list of (box, label, color, txt_col)
        self._det_lock        = threading.Lock()
        self._detections      = []

        # Confirmation gate & alert cooldowns
        self._confirm_buffer: dict[str, list[str]] = defaultdict(list)
        self._confirmed_ids:  set[str]             = set()
        self._last_unknown_alert = 0.0
        self._last_known_alert   = {}  # sid -> float (timestamp)

        # Encodings
        self._known_encodings = []
        self._known_names     = []
        self._known_ids       = []
        self._load_encodings()

        # Threads
        self._capture_thread   = None
        self._inference_thread = None
        self._render_thread    = None

    # ── Public Control ────────────────────────────────────────────────────────

    def start(self):
        """Start camera grabber, inference worker, and telemetry render threads."""
        self._running = True
        self._cap = self._init_camera()

        if self._cap is None or not self._cap.isOpened():
            self.on_status("❌ Camera not accessible (index %d)" % config.CAMERA_INDEX)
            log.error("Camera %d failed to open", config.CAMERA_INDEX)
            return

        self.on_status("✅ Camera started (Pi 4B optimized)")
        log.info("Camera pipeline started (index %d, %dx%d)",
                 config.CAMERA_INDEX, config.FRAME_WIDTH, config.FRAME_HEIGHT)

        # 1. Grabber thread (reads OpenCV at max hardware FPS)
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True, name="CamGrabber")
        self._capture_thread.start()

        # 2. Inference thread (runs HOG face recognition asynchronously)
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True, name="InferenceWorker")
        self._inference_thread.start()

        # 3. Telemetry render thread (smooth display callback)
        self._render_thread = threading.Thread(target=self._render_loop, daemon=True, name="RenderWorker")
        self._render_thread.start()

    def reload_encodings(self):
        self._load_encodings()
        with self._det_lock:
            self._confirmed_ids.clear()
            self._confirm_buffer.clear()
            self._detections.clear()

    def pause(self):
        self._paused = True

    def resume(self):
        self._paused = False

    def stop(self):
        self._running = False
        self._new_frame_event.set()
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass

    # ── Camera Initialization (V4L2 + MJPG for Pi) ───────────────────────────

    def _init_camera(self):
        is_linux = platform.system() == "Linux"
        cap = None

        # On Linux/Raspberry Pi, V4L2 is the lowest-latency capture backend
        if is_linux:
            try:
                cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_V4L2)
                log.info("Attempted cv2.CAP_V4L2 backend")
            except Exception as e:
                log.warning("CAP_V4L2 failed, trying default: %s", e)

        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(config.CAMERA_INDEX)

        if not cap.isOpened():
            return None

        # Raspberry Pi optimization: Enable hardware MJPG compression
        # This prevents USB bus saturation when using external USB webcams on Pi 4B
        if config.USE_MJPG:
            try:
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            except Exception as e:
                log.debug("Could not set MJPG fourcc: %s", e)

        # Set resolution & FPS
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          config.FPS_TARGET)

        # Minimize driver buffer to 1 to guarantee 0-frame latency on robot
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        return cap

    # ── Thread 1: Camera Grabber (Always Fresh Frames) ────────────────────────

    def _capture_loop(self):
        """Continuously reads frames to drain hardware buffers and store the latest."""
        while self._running:
            if self._paused:
                time.sleep(0.05)
                continue

            ret, frame = self._cap.read()
            if not ret or frame is None:
                time.sleep(0.02)
                continue

            with self._frame_lock:
                self._latest_raw = frame
            self._new_frame_event.set()

    # ── Thread 2: Asynchronous Inference Worker ──────────────────────────────

    def _inference_loop(self):
        """
        Executes face detection + recognition asynchronously.
        Does not block the camera telemetry feed.
        """
        scale       = config.PROCESS_SCALE
        inv_scale   = 1.0 / scale
        frame_cycle = 0

        while self._running:
            self._new_frame_event.wait(timeout=0.1)
            self._new_frame_event.clear()

            if self._paused or not self._known_encodings:
                time.sleep(0.05)
                continue

            frame_cycle += 1
            if frame_cycle % config.PROCESS_EVERY_N != 0:
                continue

            with self._frame_lock:
                if self._latest_raw is None:
                    continue
                frame_copy = self._latest_raw.copy()

            # Downsample frame for fast detection on Pi 4B CPU
            small_bgr = cv2.resize(frame_copy, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            small_rgb = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2RGB)

            # 1. Detect face bounding boxes using HOG (upsample=0 for speed on Pi)
            boxes = face_recognition.face_locations(
                small_rgb,
                model=config.DETECTION_MODEL,
                number_of_times_to_upsample=config.UPSAMPLE_TIMES,
            )

            # Filter tiny face noise
            valid_boxes = [b for b in boxes if (b[2] - b[0]) >= config.MIN_FACE_SIZE]

            if not valid_boxes:
                with self._det_lock:
                    self._detections = []
                self._update_confirmation("__none__")
                continue

            # 2. Compute 128-d encodings only for detected faces
            encodings = face_recognition.face_encodings(small_rgb, valid_boxes)

            new_detections = []
            for (top, right, bottom, left), enc in zip(valid_boxes, encodings):
                # Scale bounding box back to full resolution
                full_top    = int(top * inv_scale)
                full_right  = int(right * inv_scale)
                full_bottom = int(bottom * inv_scale)
                full_left   = int(left * inv_scale)
                full_box    = (full_top, full_right, full_bottom, full_left)

                # Distance calculation against known student embeddings
                distances  = face_recognition.face_distance(self._known_encodings, enc)
                best_idx   = int(np.argmin(distances)) if len(distances) else -1
                confidence = 1.0 - distances[best_idx] if best_idx >= 0 else 0.0

                if best_idx >= 0 and distances[best_idx] <= config.RECOGNITION_TOLERANCE:
                    name     = self._known_names[best_idx]
                    sid      = self._known_ids[best_idx]
                    conf_pct = int(confidence * 100)
                    label    = f"{name} ({conf_pct}%)"
                    color    = (57, 255, 20)   # Neon Green
                    txt_col  = (0, 0, 0)

                    # Confirmation gate
                    self._update_confirmation(name)
                    if self._is_confirmed(name):
                        now = time.time()
                        if now - self._last_known_alert.get(sid, 0.0) > config.TFT_FEEDBACK_HOLD:
                            self._last_known_alert[sid] = now
                            self.attendance.mark(sid, name)
                            self._confirmed_ids.add(sid)
                            self.on_face_detected(name, sid, frame_copy, [full_box])
                else:
                    label    = "Unknown"
                    color    = (0, 100, 255)   # Amber/Orange
                    txt_col  = (255, 255, 255)
                    self._update_confirmation("__unknown__")
                    if self._is_confirmed("__unknown__"):
                        now = time.time()
                        if now - self._last_unknown_alert > config.TFT_FEEDBACK_HOLD:
                            self._last_unknown_alert = now
                            self.on_unknown_detected(frame_copy, [full_box])

                new_detections.append((full_box, label, color, txt_col))

            with self._det_lock:
                self._detections = new_detections

    # ── Thread 3: Telemetry Render Loop (Smooth 30 FPS Stream) ────────────────

    def _render_loop(self):
        """Merges latest raw frame with cached detections and emits to GUI."""
        target_delay = 1.0 / config.FPS_TARGET

        while self._running:
            start_t = time.time()

            if self._paused:
                time.sleep(0.05)
                continue

            with self._frame_lock:
                frame = None if self._latest_raw is None else self._latest_raw.copy()

            if frame is not None:
                # Overlay current detections
                with self._det_lock:
                    current_dets = list(self._detections)

                for (top, right, bottom, left), label, color, txt_col in current_dets:
                    # Bounding box
                    cv2.rectangle(frame, (left, top), (right, bottom), color, 2)

                    # Label pill
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                    cv2.rectangle(frame, (left, bottom - th - 10), (left + tw + 6, bottom), color, -1)
                    cv2.putText(frame, label, (left + 3, bottom - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, txt_col, 1, cv2.LINE_AA)

                self.on_frame(frame)

            elapsed = time.time() - start_t
            sleep_time = max(0.005, target_delay - elapsed)
            time.sleep(sleep_time)

    # ── Confirmation Logic ───────────────────────────────────────────────────

    def _update_confirmation(self, name: str):
        buf = self._confirm_buffer[name]
        buf.append(name)
        if len(buf) > config.CONFIRMED_FRAMES * 2:
            buf.pop(0)

    def _is_confirmed(self, name: str) -> bool:
        buf = self._confirm_buffer[name]
        if len(buf) < config.CONFIRMED_FRAMES:
            return False
        recent = buf[-config.CONFIRMED_FRAMES:]
        return all(n == name for n in recent)

    # ── Encodings Storage ────────────────────────────────────────────────────

    def _load_encodings(self):
        enc_path = Path(config.ENCODINGS_FILE)
        if enc_path.exists():
            with open(enc_path, "rb") as f:
                data = pickle.load(f)
            self._known_encodings = data.get("encodings", [])
            self._known_names     = data.get("names",     [])
            self._known_ids       = data.get("ids",       [])
            log.info("Loaded %d face encodings", len(self._known_encodings))
        else:
            log.warning("No encodings file found at %s", enc_path)
            self._known_encodings = []
            self._known_names     = []
            self._known_ids       = []
