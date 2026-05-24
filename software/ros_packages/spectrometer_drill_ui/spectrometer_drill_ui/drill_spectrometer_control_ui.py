#!/usr/bin/env python3
"""
Tk-based drill/spectrometer control UI node.

This module hosts a desktop UI plus a ROS node that:
1) sends drill/linear/cap commands,
2) drives spectrometry requests and image display/saving,
3) subscribes to and renders live mechanical telemetry charts.
"""

from collections import deque
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rover2_spectrometry_interface.srv import SpectrometryInterface
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

SPECTROMETRY_IMAGE_WINDOW = "Spectrometry Image"
TELEMETRY_WINDOW_S = 20.0
TELEMETRY_Y_PADDING_RATIO = 0.10
TELEMETRY_Y_TICKS = 5
TELEMETRY_X_TICKS = 5
LINEAR_ROTATIONS_PER_INCH = 600.0


class ControlUI:
    """Owns Tk widgets/state and delegates ROS interactions to `_ControlNode`."""

    def __init__(self, root):
        self.root = root
        self.root.title("Drill and Spectrometer Controls")

        self.node = _ControlNode()
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        self.cap_engaged = False
        self.drill_auto_enabled = False
        self.linear_auto_enabled = False
        self.mechanical_systems_enabled = False
        self.spectrometry_enabled = False
        self.spectrometry_request_pending = False
        self.displayed_spectrometry_image_count = 0
        self._closing = False

        self._build_ui()
        self._schedule_spectrometry_display()
        self._schedule_mechanical_telemetry_display()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        """Construct top-level UI sections and wire control callbacks."""
        main = tk.Frame(self.root, padx=12, pady=12)
        main.pack(fill=tk.BOTH, expand=True)

        spec_frame = tk.LabelFrame(main, text="Spectrometer", padx=10, pady=10)
        spec_frame.pack(fill=tk.X, pady=(0, 8))
        self.spec_status_var = tk.StringVar(value="Disabled")
        tk.Label(spec_frame, text="Status:").grid(row=0, column=0, sticky="w")
        tk.Label(spec_frame, textvariable=self.spec_status_var, width=24).grid(
            row=0, column=1, sticky="w"
        )

        self.spec_enable_button = tk.Button(
            spec_frame, text="Enable", width=14, command=self.toggle_spectrometry
        )
        self.spec_enable_button.grid(row=0, column=2, padx=(12, 0))

        self.benedicts_button = tk.Button(
            spec_frame,
            text="Run Benedict's",
            width=16,
            command=self.send_benedicts_request,
            state=tk.DISABLED,
        )
        self.benedicts_button.grid(row=1, column=0, pady=(8, 0), sticky="w")

        self.ninhydrin_button = tk.Button(
            spec_frame,
            text="Run Ninhydrin",
            width=16,
            command=self.send_ninhydrin_request,
            state=tk.DISABLED,
        )
        self.ninhydrin_button.grid(row=1, column=1, pady=(8, 0), sticky="w")

        divider = tk.Frame(main, height=2, bg="#666666")
        divider.pack(fill=tk.X, pady=(2, 10))

        mech_frame = tk.LabelFrame(main, text="Mechanical Systems", padx=10, pady=10)
        mech_frame.pack(fill=tk.X, pady=(0, 8))
        self.mech_status_var = tk.StringVar(value="Disabled")
        tk.Label(mech_frame, text="Status:").grid(row=0, column=0, sticky="w")
        tk.Label(mech_frame, textvariable=self.mech_status_var, width=24).grid(
            row=0, column=1, sticky="w"
        )
        self.mech_enable_button = tk.Button(
            mech_frame, text="Enable", width=14, command=self.toggle_mechanical_systems
        )
        self.mech_enable_button.grid(row=0, column=2, padx=(12, 0))

        drill_frame = tk.LabelFrame(main, text="Drill", padx=10, pady=10)
        drill_frame.pack(fill=tk.X, pady=(0, 8))
        self.drill_status_var = tk.StringVar(value="Off")
        tk.Label(drill_frame, text="Status:").grid(row=0, column=0, sticky="w")
        tk.Label(drill_frame, textvariable=self.drill_status_var, width=12).grid(
            row=0, column=1, sticky="w"
        )
        self.drill_button = tk.Button(
            drill_frame, text="Auto Start", width=14, command=self.toggle_drill_auto
        )
        self.drill_button.grid(row=0, column=2, padx=(12, 0))
        self.drill_hold_button = tk.Button(drill_frame, text="Hold to Run", width=14)
        self.drill_hold_button.grid(row=0, column=3, padx=(8, 0))
        self.drill_hold_button.bind("<ButtonPress-1>", self.start_drill_hold)
        self.drill_hold_button.bind("<ButtonRelease-1>", self.stop_drill_hold)

        cap_frame = tk.LabelFrame(main, text="Drill Cap", padx=10, pady=10)
        cap_frame.pack(fill=tk.X, pady=(0, 8))
        self.cap_status_var = tk.StringVar(value="Disengaged")
        tk.Label(cap_frame, text="Status:").grid(row=0, column=0, sticky="w")
        tk.Label(cap_frame, textvariable=self.cap_status_var, width=12).grid(
            row=0, column=1, sticky="w"
        )
        self.cap_button = tk.Button(
            cap_frame, text="Engage", width=14, command=self.toggle_cap
        )
        self.cap_button.grid(row=0, column=2, padx=(12, 0))

        linear_frame = tk.LabelFrame(main, text="Linear Actuator", padx=10, pady=10)
        linear_frame.pack(fill=tk.X, pady=(0, 8))

        tk.Label(linear_frame, text="Velocity (in/s):").grid(row=0, column=0, sticky="w")
        self.linear_speed_var = tk.StringVar(value="0.0")
        tk.Entry(linear_frame, textvariable=self.linear_speed_var, width=10).grid(
            row=0, column=1, sticky="w"
        )

        self.linear_direction_var = tk.StringVar(value="Extend (+)")
        tk.OptionMenu(
            linear_frame,
            self.linear_direction_var,
            "Extend (+)",
            "Retract (-)",
        ).grid(row=0, column=2, padx=(12, 0), sticky="w")

        self.linear_status_var = tk.StringVar(value="Not Publishing")
        tk.Label(linear_frame, text="Status:").grid(row=1, column=0, sticky="w")
        tk.Label(linear_frame, textvariable=self.linear_status_var, width=24).grid(
            row=1, column=1, columnspan=2, sticky="w"
        )

        self.linear_auto_button = tk.Button(
            linear_frame, text="Auto Start", width=14, command=self.toggle_linear_auto
        )
        self.linear_auto_button.grid(row=0, column=3, padx=(12, 0))
        self.linear_hold_button = tk.Button(linear_frame, text="Hold to Run", width=12)
        self.linear_hold_button.grid(row=0, column=4, padx=(8, 0))
        self.linear_hold_button.bind("<ButtonPress-1>", self.start_linear_hold)
        self.linear_hold_button.bind("<ButtonRelease-1>", self.stop_linear_hold)
        self.linear_stop_button = tk.Button(
            linear_frame, text="Stop", width=10, command=self.stop_linear
        )
        self.linear_stop_button.grid(row=1, column=3, padx=(12, 0))

        telemetry_frame = tk.LabelFrame(main, text="Mechanical Telemetry", padx=10, pady=10)
        telemetry_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        self.drill_speed_chart_canvas = tk.Canvas(
            telemetry_frame,
            width=700,
            height=180,
            bg="#111111",
            highlightthickness=1,
            highlightbackground="#444444",
        )
        self.drill_speed_chart_canvas.pack(fill=tk.X, pady=(0, 8))

        self.drill_current_chart_canvas = tk.Canvas(
            telemetry_frame,
            width=700,
            height=180,
            bg="#111111",
            highlightthickness=1,
            highlightbackground="#444444",
        )
        self.drill_current_chart_canvas.pack(fill=tk.X, pady=(0, 8))

        self.linear_position_chart_canvas = tk.Canvas(
            telemetry_frame,
            width=700,
            height=180,
            bg="#111111",
            highlightthickness=1,
            highlightbackground="#444444",
        )
        self.linear_position_chart_canvas.pack(fill=tk.X, pady=(0, 8))

        self.linear_current_chart_canvas = tk.Canvas(
            telemetry_frame,
            width=700,
            height=180,
            bg="#111111",
            highlightthickness=1,
            highlightbackground="#444444",
        )
        self.linear_current_chart_canvas.pack(fill=tk.X)

        self._set_mechanical_controls_state(tk.DISABLED)

    def toggle_mechanical_systems(self):
        """Globally enable/disable drill/linear/cap controls."""
        target = not self.mechanical_systems_enabled
        self.mechanical_systems_enabled = target
        self.mech_status_var.set("Enabled" if target else "Disabled")
        self.mech_enable_button.config(text="Disable" if target else "Enable")

        if target:
            self._set_mechanical_controls_state(tk.NORMAL)
            return

        self._disable_mechanical_outputs()
        self._set_mechanical_controls_state(tk.DISABLED)

    def _set_mechanical_controls_state(self, state):
        self.drill_button.config(state=state)
        self.drill_hold_button.config(state=state)
        self.cap_button.config(state=state)
        self.linear_auto_button.config(state=state)
        self.linear_hold_button.config(state=state)
        self.linear_stop_button.config(state=state)

    def _disable_mechanical_outputs(self):
        """Force motion outputs idle and reset matching button/status state."""
        self.drill_auto_enabled = False
        self.linear_auto_enabled = False
        self.drill_button.config(text="Auto Start")
        self.linear_auto_button.config(text="Auto Start")
        self.node.set_drill_active(False, 0.0)
        self.node.set_linear_active(False, 0.0)
        self.drill_status_var.set("Off")
        self.linear_status_var.set("Not Publishing")

    def toggle_spectrometry(self):
        """Enable/disable spectrometry image subscription and controls."""
        target = not self.spectrometry_enabled
        if not self.node.set_spectrometry_enabled(target):
            messagebox.showerror(
                "Execution Error",
                "Unable to enable spectrometry subscription.",
            )
            return

        self.spectrometry_enabled = target
        self.displayed_spectrometry_image_count = 0
        self.spec_status_var.set("Enabled" if target else "Disabled")
        self.spec_enable_button.config(text="Disable" if target else "Enable")
        button_state = (
            tk.NORMAL
            if target and not self.spectrometry_request_pending
            else tk.DISABLED
        )
        self.benedicts_button.config(state=button_state)
        self.ninhydrin_button.config(state=button_state)

    def _send_spectrometry_request(self, camera_id, reaction_type, reaction_name):
        """Launch a non-blocking spectrometry request from the Tk thread."""
        if not self.spectrometry_enabled:
            messagebox.showwarning(
                "Spectrometry Disabled",
                "Enable spectrometry before sending reaction requests.",
            )
            return

        if self.spectrometry_request_pending:
            messagebox.showwarning(
                "Request Pending",
                "Wait for the current spectrometry request to complete.",
            )
            return

        self.spectrometry_request_pending = True
        self._set_spectrometry_action_state(tk.DISABLED)
        self.spec_status_var.set(f"Sending {reaction_name} request")

        def request_worker():
            ok = self.node.send_spectrometry_request(camera_id, reaction_type)
            if self._closing:
                return
            try:
                self.root.after(
                    0,
                    lambda: self._handle_spectrometry_request_result(
                        ok, reaction_name
                    ),
                )
            except (RuntimeError, tk.TclError):
                pass

        threading.Thread(target=request_worker, daemon=True).start()

    def _handle_spectrometry_request_result(self, ok, reaction_name):
        self.spectrometry_request_pending = False
        if self.spectrometry_enabled:
            self._set_spectrometry_action_state(tk.NORMAL)
        else:
            self._set_spectrometry_action_state(tk.DISABLED)

        if not ok:
            messagebox.showerror(
                "Execution Error",
                f"{reaction_name} request failed or timed out.",
            )
            return
        self.spec_status_var.set(f"{reaction_name} request sent")

    def _set_spectrometry_action_state(self, state):
        self.benedicts_button.config(state=state)
        self.ninhydrin_button.config(state=state)

    def send_benedicts_request(self):
        self._send_spectrometry_request(
            camera_id=1, reaction_type=1, reaction_name="Benedict's"
        )

    def send_ninhydrin_request(self):
        self._send_spectrometry_request(
            camera_id=2, reaction_type=3, reaction_name="Ninhydrin"
        )

    def _schedule_spectrometry_display(self):
        """Pump OpenCV window updates from the latest received image snapshot."""
        try:
            if self.spectrometry_enabled:
                snapshot = self.node.get_spectrometry_image_snapshot()
                if snapshot is not None:
                    image, frame_id, image_count = snapshot
                    if image_count != self.displayed_spectrometry_image_count:
                        cv2.imshow(SPECTROMETRY_IMAGE_WINDOW, image)
                        self.displayed_spectrometry_image_count = image_count
                        if not self.spectrometry_request_pending:
                            self.spec_status_var.set(f"Receiving image: {frame_id}")
                    cv2.waitKey(1)
        except cv2.error as exc:
            self._handle_spectrometry_display_error(exc)

        if not self._closing:
            self.root.after(50, self._schedule_spectrometry_display)

    def _schedule_mechanical_telemetry_display(self):
        """Refresh rolling telemetry plots for drill/linear signals."""
        (
            drill_speed_samples,
            drill_current_samples,
            linear_position_samples,
            linear_current_samples,
        ) = self.node.get_mechanical_telemetry_snapshot()
        self._draw_telemetry_chart(
            self.drill_speed_chart_canvas,
            drill_speed_samples,
            title="Drill Speed Telemetry",
            unit_label="rps",
            line_color="#00C853",
        )
        self._draw_telemetry_chart(
            self.drill_current_chart_canvas,
            drill_current_samples,
            title="Drill Current Telemetry",
            unit_label="A",
            line_color="#FF9800",
        )
        self._draw_telemetry_chart(
            self.linear_position_chart_canvas,
            linear_position_samples,
            title="Linear Position Telemetry",
            unit_label="in",
            line_color="#03A9F4",
        )
        self._draw_telemetry_chart(
            self.linear_current_chart_canvas,
            linear_current_samples,
            title="Linear Current Telemetry",
            unit_label="A",
            line_color="#E91E63",
        )

        if not self._closing:
            self.root.after(100, self._schedule_mechanical_telemetry_display)

    def _draw_telemetry_chart(self, canvas, samples, title, unit_label, line_color):
        """Render one scrolling line chart with dynamic axes and grid ticks."""
        canvas.delete("all")
        width = max(1, int(canvas.winfo_width()))
        height = max(1, int(canvas.winfo_height()))
        left = 110
        right = width - 40
        top = 50
        bottom = height - 45

        canvas.create_text(
            left,
            10,
            text=title,
            fill="#D7D7D7",
            anchor="w",
            font=("TkDefaultFont", 9, "bold"),
        )

        if right <= left or bottom <= top:
            return

        if not samples:
            canvas.create_line(left, top, left, bottom, fill="#888888", width=1)
            canvas.create_line(left, bottom, right, bottom, fill="#888888", width=1)
            canvas.create_text(
                (left + right) / 2,
                (top + bottom) / 2,
                text="No telemetry",
                fill="#A0A0A0",
            )
            return

        values = [value for _, value in samples]
        min_value = min(values)
        max_value = max(values)
        value_range = max_value - min_value
        if value_range <= 1e-9:
            pad = max(abs(max_value) * TELEMETRY_Y_PADDING_RATIO, 1e-3)
        else:
            pad = value_range * TELEMETRY_Y_PADDING_RATIO
        y_min = min_value - pad
        y_max = max_value + pad
        y_span = max(y_max - y_min, 1e-6)

        for tick in range(TELEMETRY_Y_TICKS):
            frac = tick / (TELEMETRY_Y_TICKS - 1)
            y = bottom - frac * (bottom - top)
            tick_value = y_min + frac * y_span
            canvas.create_line(left, y, right, y, fill="#2A2A2A", width=1)
            canvas.create_text(
                left - 8,
                y,
                text=f"{tick_value:.2f}",
                fill="#BFBFBF",
                anchor="e",
            )

        for tick in range(TELEMETRY_X_TICKS):
            frac = tick / (TELEMETRY_X_TICKS - 1)
            x = left + frac * (right - left)
            seconds_ago = int(round((1.0 - frac) * TELEMETRY_WINDOW_S))
            label = "now" if seconds_ago == 0 else f"-{seconds_ago}s"
            canvas.create_line(x, bottom, x, top, fill="#1C1C1C", width=1)
            canvas.create_text(x, bottom + 14, text=label, fill="#A0A0A0", anchor="n")

        canvas.create_line(left, top, left, bottom, fill="#888888", width=1)
        canvas.create_line(left, bottom, right, bottom, fill="#888888", width=1)

        now = time.monotonic()
        window_start = now - TELEMETRY_WINDOW_S
        points = []
        for sample_time, value in samples:
            x_ratio = (sample_time - window_start) / TELEMETRY_WINDOW_S
            x = left + max(0.0, min(1.0, x_ratio)) * (right - left)
            y_ratio = (value - y_min) / y_span
            y = bottom - max(0.0, min(1.0, y_ratio)) * (bottom - top)
            points.extend([x, y])

        if len(points) >= 4:
            canvas.create_line(*points, fill=line_color, width=2)

        canvas.create_text(
            right,
            10,
            text=f"latest {values[-1]:.2f} {unit_label}",
            fill="#D7D7D7",
            anchor="e",
            font=("TkDefaultFont", 9),
        )

    def _handle_spectrometry_display_error(self, exc):
        self.node.get_logger().error(f"Spectrometry image display failed: {exc}")
        self.node.set_spectrometry_enabled(False)
        self.spectrometry_enabled = False
        self.spectrometry_request_pending = False
        self.displayed_spectrometry_image_count = 0
        self.spec_status_var.set("Image display error")
        self.spec_enable_button.config(text="Enable")
        self._set_spectrometry_action_state(tk.DISABLED)

    def toggle_drill_auto(self):
        if not self.mechanical_systems_enabled:
            return
        self.drill_auto_enabled = not self.drill_auto_enabled
        self.node.set_drill_active(self.drill_auto_enabled, 1.0)
        self.drill_status_var.set("On (Auto)" if self.drill_auto_enabled else "Off")
        self.drill_button.config(
            text="Auto Stop" if self.drill_auto_enabled else "Auto Start"
        )

    def start_drill_hold(self, _event):
        if not self.mechanical_systems_enabled:
            return
        if self.drill_auto_enabled:
            self.drill_auto_enabled = False
            self.drill_button.config(text="Auto Start")
        self.node.set_drill_active(True, 1.0)
        self.drill_status_var.set("On (Hold)")

    def stop_drill_hold(self, _event):
        if not self.mechanical_systems_enabled:
            return
        self.node.set_drill_active(False, 0.0)
        self.drill_status_var.set("Off")

    def toggle_cap(self):
        if not self.mechanical_systems_enabled:
            return
        target = not self.cap_engaged
        ok = self.node.set_cap_state(target)
        if not ok:
            messagebox.showerror(
                "Execution Error",
                "Drill cap service call failed or timed out.",
            )
            return
        self.cap_engaged = target
        self.cap_status_var.set("Engaged" if target else "Disengaged")
        self.cap_button.config(text="Disengage" if target else "Engage")

    def _get_linear_command(self):
        speed_text = self.linear_speed_var.get().strip()
        try:
            speed = float(speed_text)
        except ValueError:
            messagebox.showerror("Invalid Input", "Linear velocity must be a number.")
            return None

        direction = 1.0 if self.linear_direction_var.get() == "Extend (+)" else -1.0
        signed_inches_per_s = speed * direction
        return signed_inches_per_s * LINEAR_ROTATIONS_PER_INCH

    def toggle_linear_auto(self):
        if not self.mechanical_systems_enabled:
            return
        if self.linear_auto_enabled:
            self.linear_auto_enabled = False
            self.node.set_linear_active(False, 0.0)
            self.linear_auto_button.config(text="Auto Start")
            self.linear_status_var.set("Not Publishing")
            return

        signed_speed = self._get_linear_command()
        if signed_speed is None:
            return
        self.linear_auto_enabled = True
        self.node.set_linear_active(True, signed_speed)
        self.linear_auto_button.config(text="Auto Stop")
        commanded_inches_per_s = signed_speed / LINEAR_ROTATIONS_PER_INCH
        self.linear_status_var.set(
            f"Commanded {commanded_inches_per_s:.3f} in/s (Auto)"
        )

    def start_linear_hold(self, _event):
        if not self.mechanical_systems_enabled:
            return
        signed_speed = self._get_linear_command()
        if signed_speed is None:
            return
        if self.linear_auto_enabled:
            self.linear_auto_enabled = False
            self.linear_auto_button.config(text="Auto Start")
        self.node.set_linear_active(True, signed_speed)
        commanded_inches_per_s = signed_speed / LINEAR_ROTATIONS_PER_INCH
        self.linear_status_var.set(
            f"Commanded {commanded_inches_per_s:.3f} in/s (Hold)"
        )

    def stop_linear_hold(self, _event):
        if not self.mechanical_systems_enabled:
            return
        self.node.set_linear_active(False, 0.0)
        self.linear_status_var.set("Not Publishing")

    def stop_linear(self):
        if not self.mechanical_systems_enabled:
            return
        self.linear_auto_enabled = False
        self.linear_auto_button.config(text="Auto Start")
        self.node.set_linear_active(False, 0.0)
        self.linear_status_var.set("Not Publishing")

    def on_close(self):
        """Shutdown ROS resources and terminate the Tk application cleanly."""
        self._closing = True
        self.node.set_spectrometry_enabled(False)
        self.node.destroy_node()
        rclpy.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        self.root.destroy()


class _ControlNode(Node):
    """ROS-side command, service, and telemetry integration for the UI."""

    def __init__(self):
        super().__init__("spectrometer_drill_ui")
        self.drill_pub = self.create_publisher(Float32, "/drill/control", 10)
        self.linear_pub = self.create_publisher(Float32, "/linear_actuator/control", 10)
        self.cap_client = self.create_client(SetBool, "/drill/cap/set_engaged")
        self.spectrometry_client = self.create_client(
            SpectrometryInterface, "spectrometry_chart"
        )
        self.bridge = CvBridge()
        self.spectrometry_image = None
        self.spectrometry_frame_id = ""
        self.spectrometry_image_count = 0
        self.spectrometry_image_lock = threading.Lock()
        self.spectrometry_save_dir = Path.cwd() / "spectrometry_folder"
        self.spectrometry_save_dir.mkdir(parents=True, exist_ok=True)
        self.spectrometry_enabled = False
        # Spectrometry image stream (display + autosave in callback).
        self.spectrometry_sub = self.create_subscription(
            Image,
            "camera/image",
            self._spectrometry_image_callback,
            10,
        )
        self.telemetry_lock = threading.Lock()
        self.drill_speed_samples = deque()
        self.drill_current_samples = deque()
        self.linear_position_samples = deque()
        self.linear_current_samples = deque()
        # Mechanical telemetry streams used by the UI charts.
        self.drill_speed_sub = self.create_subscription(
            Float32,
            "drill/speed",
            self._drill_speed_callback,
            10,
        )
        self.drill_current_sub = self.create_subscription(
            Float32,
            "drill/current",
            self._drill_current_callback,
            10,
        )
        self.linear_position_sub = self.create_subscription(
            Float32,
            "linear_actuator/position",
            self._linear_position_callback,
            10,
        )
        self.linear_current_sub = self.create_subscription(
            Float32,
            "linear_actuator/current",
            self._linear_current_callback,
            10,
        )
        self.drill_active = False
        self.drill_value = 0.0
        self.linear_active = False
        self.linear_value = 0.0
        self.publish_timer = self.create_timer(1.0 / 30.0, self._publish_active_commands)

    def publish_drill(self, value):
        msg = Float32()
        msg.data = float(value)
        self.drill_pub.publish(msg)

    def publish_linear(self, value):
        msg = Float32()
        msg.data = float(value)
        self.linear_pub.publish(msg)

    def set_drill_active(self, active, value):
        self.drill_active = bool(active)
        self.drill_value = float(value)
        if self.drill_active:
            self.publish_drill(self.drill_value)
        else:
            self.publish_drill(0.0)

    def set_linear_active(self, active, value):
        self.linear_active = bool(active)
        self.linear_value = float(value)
        if self.linear_active:
            self.publish_linear(self.linear_value)
        else:
            self.publish_linear(0.0)

    def _publish_active_commands(self):
        """Continuously republish active drill/linear commands while enabled."""
        if self.drill_active:
            self.publish_drill(self.drill_value)
        if self.linear_active:
            self.publish_linear(self.linear_value)

    def set_spectrometry_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled == self.spectrometry_enabled:
            return True
        if enabled:
            try:
                cv2.namedWindow(SPECTROMETRY_IMAGE_WINDOW, cv2.WINDOW_NORMAL)
            except cv2.error as exc:
                self.get_logger().error(
                    f"Failed to create spectrometry image window: {exc}"
                )
                return False
            self.spectrometry_enabled = True
            self.get_logger().info("Enabled spectrometry image display")
            return True

        self.spectrometry_enabled = False
        with self.spectrometry_image_lock:
            self.spectrometry_image = None
            self.spectrometry_frame_id = ""
            self.spectrometry_image_count = 0
        try:
            cv2.destroyWindow(SPECTROMETRY_IMAGE_WINDOW)
        except cv2.error:
            pass
        return True

    def _spectrometry_image_callback(self, msg):
        """Convert, persist, and cache each spectrometry image frame."""
        if not self.spectrometry_enabled:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert spectrometry image: {exc}")
            return

        frame_id = msg.header.frame_id or "camera_image"
        image_path = self._write_spectrometry_image(cv_image, frame_id, msg.header.stamp)
        with self.spectrometry_image_lock:
            self.spectrometry_image = cv_image
            self.spectrometry_frame_id = frame_id
            self.spectrometry_image_count += 1

        self.get_logger().info(f"got a new image from frame_id:={frame_id}")
        if image_path is not None:
            self.get_logger().info(f"saved spectrometry image: {image_path.name}")

    def get_latest_spectrometry_image(self):
        with self.spectrometry_image_lock:
            if self.spectrometry_image is None:
                return None
            return self.spectrometry_image.copy()

    def get_spectrometry_image_snapshot(self):
        with self.spectrometry_image_lock:
            if self.spectrometry_image is None:
                return None
            return (
                self.spectrometry_image.copy(),
                self.spectrometry_frame_id or "camera_image",
                self.spectrometry_image_count,
            )

    def _write_spectrometry_image(self, image, frame_id, stamp):
        """Save one image to `spectrometry_folder` using frame/time naming."""
        timestamp = datetime.fromtimestamp(stamp.sec + (stamp.nanosec * 1e-9))
        output_path = self._build_image_output_path(frame_id, timestamp)
        if not cv2.imwrite(str(output_path), image):
            self.get_logger().error(f"Failed to save spectrometry image: {output_path}")
            return None
        return output_path

    def _build_image_output_path(self, frame_id, timestamp):
        reaction_type = self._reaction_type_from_frame_id(frame_id)
        time_text = timestamp.strftime("%H:%M:%S")
        return self.spectrometry_save_dir / f"{reaction_type}_{time_text}.png"

    @staticmethod
    def _reaction_type_from_frame_id(frame_id):
        if frame_id == "camera_1":
            return "benedicts"
        if frame_id == "camera_2":
            return "ninhydrin"
        return "unknown"

    def _drill_speed_callback(self, msg):
        self._append_telemetry_sample(self.drill_speed_samples, float(msg.data))

    def _linear_position_callback(self, msg):
        position_inches = float(msg.data) / LINEAR_ROTATIONS_PER_INCH
        self._append_telemetry_sample(self.linear_position_samples, position_inches)

    def _drill_current_callback(self, msg):
        self._append_telemetry_sample(self.drill_current_samples, float(msg.data))

    def _linear_current_callback(self, msg):
        self._append_telemetry_sample(self.linear_current_samples, float(msg.data))

    def _append_telemetry_sample(self, series, value):
        """Append sample then trim to the configured rolling time window."""
        now = time.monotonic()
        with self.telemetry_lock:
            series.append((now, value))
            self._prune_telemetry_locked(series, now)

    def _prune_telemetry_locked(self, series, now):
        cutoff = now - TELEMETRY_WINDOW_S
        while series and series[0][0] < cutoff:
            series.popleft()

    def get_mechanical_telemetry_snapshot(self):
        """Return thread-safe copies of all four telemetry series."""
        now = time.monotonic()
        with self.telemetry_lock:
            self._prune_telemetry_locked(self.drill_speed_samples, now)
            self._prune_telemetry_locked(self.drill_current_samples, now)
            self._prune_telemetry_locked(self.linear_position_samples, now)
            self._prune_telemetry_locked(self.linear_current_samples, now)
            return (
                list(self.drill_speed_samples),
                list(self.drill_current_samples),
                list(self.linear_position_samples),
                list(self.linear_current_samples),
            )

    def send_spectrometry_request(self, camera_id, reaction_type):
        """Issue async service request and wait with a short timeout budget."""
        if not self.spectrometry_client.wait_for_service(timeout_sec=1.0):
            return False

        req = SpectrometryInterface.Request()
        req.camera_id = int(camera_id)
        req.reaction_type = int(reaction_type)
        future = self.spectrometry_client.call_async(req)

        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.02, remaining))

        if not future.done():
            return False
        if future.exception() is not None:
            return False

        response = future.result()
        return bool(response and response.success)

    def set_cap_state(self, engaged):
        if not self.cap_client.wait_for_service(timeout_sec=1.0):
            return False

        req = SetBool.Request()
        req.data = bool(engaged)
        future = self.cap_client.call_async(req)

        deadline = self.get_clock().now().nanoseconds + int(2.0 * 1e9)
        while rclpy.ok() and not future.done():
            if self.get_clock().now().nanoseconds > deadline:
                return False
            threading.Event().wait(0.02)

        if not future.done():
            return False
        if future.exception() is not None:
            return False

        response = future.result()
        return bool(response and response.success)


def main():
    rclpy.init()
    root = tk.Tk()
    ControlUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
