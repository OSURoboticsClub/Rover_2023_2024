#!/usr/bin/env python3
"""Tk-based spectrometer and science mechanism control UI."""

import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rover2_spectrometry_interface.msg import (
    SpectrometryMechanicalControlMessage,
)
from rover2_spectrometry_interface.msg import (
    SpectrometryMechanicalStatusMessage,
)
from rover2_spectrometry_interface.srv import SpectrometryInterface
from sensor_msgs.msg import Image

SPECTROMETRY_IMAGE_WINDOW = "Spectrometry Image"
MECHANICAL_SYSTEM_NAMES = (
    "valve_1",
    "valve_2",
    "pump",
    "coil_1",
    "coil_2",
    "solenoid",
)
MECHANICAL_SYSTEM_FIELDS = {
    "valve_1": "valve_1_on",
    "valve_2": "valve_2_on",
    "pump": "pump_on",
    "coil_1": "coil_1_on",
    "coil_2": "coil_2_on",
    "solenoid": "solenoid_on",
}
MECHANICAL_STATUS_TIMEOUT_S = 1.0

# Set these to the actual camera ids on the rover.
NINHYDRIN_CAMERA_ID = 0
BENEDICTS_CAMERA_ID = 1


class ControlUI:
    """Own UI widgets and delegate ROS interactions to the node."""

    def __init__(self, root):
        self.root = root
        self.root.title("Spectrometer and Science Controls")

        self.node = _ControlNode()
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self.spin_thread.start()

        self.spectrometry_enabled = False
        self.spectrometry_request_pending = False
        self.displayed_spectrometry_image_count = 0
        self.science_controls_locked = True
        self.mechanical_control_buttons = []
        self._closing = False

        self._build_ui()
        self._schedule_spectrometry_display()
        self._schedule_mechanical_status_display()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        """Construct the spectrometer controls."""
        main = tk.Frame(self.root, padx=12, pady=12)
        main.pack(fill=tk.BOTH, expand=True)

        spec_frame = tk.LabelFrame(main, text="Spectrometer", padx=10, pady=10)
        spec_frame.pack(fill=tk.X)

        self.spec_status_var = tk.StringVar(value="Disabled")
        tk.Label(spec_frame, text="Status:").grid(row=0, column=0, sticky="w")
        tk.Label(spec_frame, textvariable=self.spec_status_var, width=24).grid(
            row=0, column=1, sticky="w"
        )

        self.spec_enable_button = tk.Button(
            spec_frame,
            text="Enable",
            width=14,
            command=self.toggle_spectrometry,
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
        divider.pack(fill=tk.X, pady=(10, 10))

        mechanical_frame = tk.LabelFrame(
            main, text="Mechanical Systems", padx=10, pady=10
        )
        mechanical_frame.pack(fill=tk.BOTH, expand=True)

        self._build_master_control_section(mechanical_frame)
        self._build_valves_section(mechanical_frame)
        self._build_pump_section(mechanical_frame)
        self._build_heating_coils_section(mechanical_frame)
        self._build_dirt_agitator_section(mechanical_frame)
        self._set_mechanical_controls_state(tk.DISABLED)

    def _build_master_control_section(self, parent):
        """Build the lockout controls that gate all mechanical commands."""
        frame = self._new_mechanical_section(parent, "Master Control")
        self.science_lock_status_var = tk.StringVar(value="Locked")

        self.lock_science_button = tk.Button(
            frame,
            text="Lock Science Control",
            width=22,
            command=self.lock_science_control,
            state=tk.DISABLED,
        )
        self.lock_science_button.grid(row=0, column=0, padx=(0, 8), sticky="w")
        self.unlock_science_button = tk.Button(
            frame,
            text="Unlock Science Control",
            width=22,
            command=self.unlock_science_control,
            state=tk.DISABLED,
        )
        self.unlock_science_button.grid(
            row=0, column=1, padx=(0, 16), sticky="w"
        )
        self._add_status_monitor(
            frame,
            row=0,
            column=2,
            label="Control",
            variable=self.science_lock_status_var,
        )

    def _build_valves_section(self, parent):
        """Build individual and grouped valve controls."""
        frame = self._new_mechanical_section(parent, "Valves")
        self.valve_1_status_var = tk.StringVar(value="Unknown")
        self.valve_2_status_var = tk.StringVar(value="Unknown")

        self._add_control_button(
            frame, 0, 0, "Turn On Valve 1", "valve_1", True
        )
        self._add_control_button(
            frame, 0, 1, "Turn On Valve 2", "valve_2", True
        )
        self._add_group_button(
            frame, 0, 2, "Turn All Valves On", ("valve_1", "valve_2"), True
        )
        self._add_control_button(
            frame, 1, 0, "Turn Off Valve 1", "valve_1", False
        )
        self._add_control_button(
            frame, 1, 1, "Turn Off Valve 2", "valve_2", False
        )
        self._add_group_button(
            frame, 1, 2, "Turn All Valves Off", ("valve_1", "valve_2"), False
        )
        self._add_status_monitor(
            frame,
            row=2,
            column=0,
            label="Valve 1",
            variable=self.valve_1_status_var,
        )
        self._add_status_monitor(
            frame,
            row=2,
            column=1,
            label="Valve 2",
            variable=self.valve_2_status_var,
        )

    def _build_pump_section(self, parent):
        """Build pump controls and status monitor."""
        frame = self._new_mechanical_section(parent, "Pump")
        self.pump_status_var = tk.StringVar(value="Unknown")
        self._add_control_button(frame, 0, 0, "Turn On Pump", "pump", True)
        self._add_control_button(frame, 0, 1, "Turn Off Pump", "pump", False)
        self._add_status_monitor(
            frame, row=1, column=0, label="Pump", variable=self.pump_status_var
        )

    def _build_heating_coils_section(self, parent):
        """Build individual and grouped heating-coil controls."""
        frame = self._new_mechanical_section(parent, "Heating Coils")
        self.coil_1_status_var = tk.StringVar(value="Unknown")
        self.coil_2_status_var = tk.StringVar(value="Unknown")

        self._add_control_button(frame, 0, 0, "Turn On Coil 1", "coil_1", True)
        self._add_control_button(frame, 0, 1, "Turn On Coil 2", "coil_2", True)
        self._add_group_button(
            frame, 0, 2, "Turn On All Coils", ("coil_1", "coil_2"), True
        )
        self._add_control_button(
            frame, 1, 0, "Turn Off Coil 1", "coil_1", False
        )
        self._add_control_button(
            frame, 1, 1, "Turn Off Coil 2", "coil_2", False
        )
        self._add_group_button(
            frame, 1, 2, "Turn Off All Coils", ("coil_1", "coil_2"), False
        )
        self._add_status_monitor(
            frame,
            row=2,
            column=0,
            label="Coil 1",
            variable=self.coil_1_status_var,
        )
        self._add_status_monitor(
            frame,
            row=2,
            column=1,
            label="Coil 2",
            variable=self.coil_2_status_var,
        )

    def _build_dirt_agitator_section(self, parent):
        """Build dirt-agitator solenoid controls and status monitor."""
        frame = self._new_mechanical_section(parent, "Dirt Agitator")
        self.solenoid_status_var = tk.StringVar(value="Unknown")
        self._add_control_button(
            frame, 0, 0, "Turn On Solenoid", "solenoid", True
        )
        self._add_control_button(
            frame, 0, 1, "Turn Off Solenoid", "solenoid", False
        )
        self._add_status_monitor(
            frame,
            row=1,
            column=0,
            label="Solenoid",
            variable=self.solenoid_status_var,
        )

    @staticmethod
    def _new_mechanical_section(parent, title):
        frame = tk.LabelFrame(parent, text=title, padx=8, pady=8)
        frame.pack(fill=tk.X, pady=(0, 8))
        return frame

    def _add_control_button(
        self, parent, row, column, text, system_name, enabled
    ):
        button = tk.Button(
            parent,
            text=text,
            width=20,
            command=lambda: self.set_mechanical_system(system_name, enabled),
        )
        button.grid(
            row=row, column=column, padx=(0, 8), pady=(0, 6), sticky="w"
        )
        self.mechanical_control_buttons.append(button)

    def _add_group_button(
        self, parent, row, column, text, system_names, enabled
    ):
        button = tk.Button(
            parent,
            text=text,
            width=20,
            command=lambda: self.set_mechanical_systems(system_names, enabled),
        )
        button.grid(
            row=row, column=column, padx=(0, 8), pady=(0, 6), sticky="w"
        )
        self.mechanical_control_buttons.append(button)

    @staticmethod
    def _add_status_monitor(parent, row, column, label, variable):
        monitor = tk.Frame(parent)
        monitor.grid(
            row=row, column=column, padx=(0, 14), pady=(2, 0), sticky="w"
        )
        tk.Label(monitor, text=f"{label}:").pack(side=tk.LEFT)
        tk.Label(
            monitor,
            textvariable=variable,
            font=("TkDefaultFont", 9, "underline"),
            width=18,
            anchor="w",
        ).pack(side=tk.LEFT, padx=(4, 0))

    def unlock_science_control(self):
        """Allow mechanical commands until the operator locks them again."""
        snapshot = self.node.get_mechanical_status_snapshot()
        controls_ready = (
            snapshot["status_available"]
            and snapshot["can_connected"]
            and snapshot["command_state_valid"]
            and snapshot["command_link_active"]
            and not snapshot["command_pending"]
        )
        if not controls_ready:
            messagebox.showwarning(
                "Science Controls Unavailable",
                "The rover command link and science CAN interface must be "
                "ready before controls can be unlocked.",
            )
            return

        self.node.set_science_locked(False)
        self.science_controls_locked = True
        self.science_lock_status_var.set("Unlock requested")
        self.lock_science_button.config(state=tk.NORMAL)
        self.unlock_science_button.config(state=tk.DISABLED)
        self._set_mechanical_controls_state(tk.DISABLED)

    def lock_science_control(self):
        """Command every mechanical output off, then lock the controls."""
        self.node.set_science_locked(True)
        self._apply_locked_ui("Lock requested", allow_unlock=False)

    def _apply_locked_ui(self, status_text, allow_unlock):
        """Apply the locally safe/locked widget state."""
        self.science_controls_locked = True
        self.science_lock_status_var.set(status_text)
        self.lock_science_button.config(state=tk.DISABLED)
        unlock_state = tk.NORMAL if allow_unlock else tk.DISABLED
        self.unlock_science_button.config(state=unlock_state)
        self._set_mechanical_controls_state(tk.DISABLED)

    def _apply_unlocked_ui(self):
        """Enable output buttons after the rover acknowledges unlock."""
        self.science_controls_locked = False
        self.science_lock_status_var.set("Unlocked (sent)")
        self.lock_science_button.config(state=tk.NORMAL)
        self.unlock_science_button.config(state=tk.DISABLED)
        self._set_mechanical_controls_state(tk.NORMAL)

    def _set_mechanical_controls_state(self, state):
        for button in self.mechanical_control_buttons:
            button.config(state=state)

    def set_mechanical_system(self, system_name, enabled):
        """Send one output request through the ROS command heartbeat."""
        self.set_mechanical_systems((system_name,), enabled)

    def set_mechanical_systems(self, system_names, enabled):
        """Send an atomic output request if controls are unlocked."""
        if self.science_controls_locked:
            return

        states = {system_name: bool(enabled) for system_name in system_names}
        self.node.send_mechanical_states(states)

    def _schedule_mechanical_status_display(self):
        """Refresh monitors from the last rover-side CAN command state."""
        status_variables = {
            "valve_1": self.valve_1_status_var,
            "valve_2": self.valve_2_status_var,
            "pump": self.pump_status_var,
            "coil_1": self.coil_1_status_var,
            "coil_2": self.coil_2_status_var,
            "solenoid": self.solenoid_status_var,
        }
        snapshot = self.node.get_mechanical_status_snapshot()
        status_available = (
            snapshot["status_available"]
            and snapshot["can_connected"]
            and snapshot["command_state_valid"]
        )
        for system_name, variable in status_variables.items():
            if status_available:
                state = snapshot["states"][system_name]
                variable.set("On (sent)" if state else "Off (sent)")
            else:
                variable.set("Unknown")

        if not snapshot["status_available"]:
            self._apply_locked_ui("Status unavailable", allow_unlock=False)
        elif not snapshot["can_connected"]:
            self._apply_locked_ui("CAN unavailable", allow_unlock=False)
        elif not snapshot["command_state_valid"]:
            self._apply_locked_ui("State unknown", allow_unlock=False)
        elif snapshot["command_pending"]:
            if snapshot["desired_controls_unlocked"]:
                self.science_controls_locked = True
                self.science_lock_status_var.set("Unlock requested")
                self.lock_science_button.config(state=tk.NORMAL)
                self.unlock_science_button.config(state=tk.DISABLED)
                self._set_mechanical_controls_state(tk.DISABLED)
            else:
                self._apply_locked_ui("Lock requested", allow_unlock=False)
        elif snapshot["controls_unlocked"]:
            self._apply_unlocked_ui()
        elif not snapshot["command_link_active"]:
            self._apply_locked_ui("Outputs off (link lost)", allow_unlock=False)
        else:
            self._apply_locked_ui("Locked (sent)", allow_unlock=True)

        if not self._closing:
            self.root.after(250, self._schedule_mechanical_status_display)

    def toggle_spectrometry(self):
        """Enable or disable spectrometry image display and controls."""
        target = not self.spectrometry_enabled
        if not self.node.set_spectrometry_enabled(target):
            messagebox.showerror(
                "Execution Error",
                "Unable to enable spectrometry image display.",
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
        self._set_spectrometry_action_state(button_state)

    def _send_spectrometry_request(
        self, camera_id, reaction_type, reaction_name
    ):
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
        state = tk.NORMAL if self.spectrometry_enabled else tk.DISABLED
        self._set_spectrometry_action_state(state)

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
            camera_id=BENEDICTS_CAMERA_ID, reaction_type=1,
            reaction_name="Benedict's"
        )

    def send_ninhydrin_request(self):
        self._send_spectrometry_request(
            camera_id=NINHYDRIN_CAMERA_ID, reaction_type=3,
            reaction_name="Ninhydrin"
        )

    def _schedule_spectrometry_display(self):
        """Update the OpenCV window from the latest image snapshot."""
        try:
            if self.spectrometry_enabled:
                snapshot = self.node.get_spectrometry_image_snapshot()
                if snapshot is not None:
                    image, frame_id, image_count = snapshot
                    if image_count != self.displayed_spectrometry_image_count:
                        cv2.imshow(SPECTROMETRY_IMAGE_WINDOW, image)
                        self.displayed_spectrometry_image_count = image_count
                        if not self.spectrometry_request_pending:
                            self.spec_status_var.set(
                                f"Receiving image: {frame_id}"
                            )
                    cv2.waitKey(1)
        except cv2.error as exc:
            self._handle_spectrometry_display_error(exc)

        if not self._closing:
            self.root.after(50, self._schedule_spectrometry_display)

    def _handle_spectrometry_display_error(self, exc):
        self.node.get_logger().error(
            f"Spectrometry image display failed: {exc}"
        )
        self.node.set_spectrometry_enabled(False)
        self.spectrometry_enabled = False
        self.spectrometry_request_pending = False
        self.displayed_spectrometry_image_count = 0
        self.spec_status_var.set("Image display error")
        self.spec_enable_button.config(text="Enable")
        self._set_spectrometry_action_state(tk.DISABLED)

    def on_close(self):
        """Shut down ROS resources and terminate the Tk application cleanly."""
        self._closing = True
        self.node.set_science_locked(True)
        self.node.set_spectrometry_enabled(False)
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        self.root.destroy()


class _ControlNode(Node):
    """Provide ROS spectrometry and mechanical integration for the UI."""

    def __init__(self):
        super().__init__("spectrometer_ui")
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
        self.spectrometry_sub = self.create_subscription(
            Image,
            "camera/image",
            self._spectrometry_image_callback,
            10,
        )
        self.declare_parameter(
            "mechanical_command_topic", "science/mechanical/control"
        )
        self.declare_parameter(
            "mechanical_status_topic", "science/mechanical/status"
        )
        mechanical_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.mechanical_callback_group = MutuallyExclusiveCallbackGroup()
        self.mechanical_command_lock = threading.Lock()
        self.mechanical_command_sequence = 0
        self.mechanical_command_state = {
            "controls_unlocked": False,
            **{system_name: False for system_name in MECHANICAL_SYSTEM_NAMES},
        }
        self.mechanical_status_lock = threading.Lock()
        self.mechanical_status = None
        self.mechanical_command_publisher = self.create_publisher(
            SpectrometryMechanicalControlMessage,
            self.get_parameter("mechanical_command_topic").value,
            mechanical_qos,
        )
        self.mechanical_status_subscription = self.create_subscription(
            SpectrometryMechanicalStatusMessage,
            self.get_parameter("mechanical_status_topic").value,
            self._mechanical_status_callback,
            mechanical_qos,
            callback_group=self.mechanical_callback_group,
        )
        self.mechanical_publish_timer = self.create_timer(
            1.0 / 30.0,
            self._publish_mechanical_command,
            callback_group=self.mechanical_callback_group,
        )
        self._publish_mechanical_command()

    def set_science_locked(self, locked):
        """Update and immediately publish the complete master-control state."""
        with self.mechanical_command_lock:
            if locked:
                self.mechanical_command_state["controls_unlocked"] = False
                for system_name in MECHANICAL_SYSTEM_NAMES:
                    self.mechanical_command_state[system_name] = False
            else:
                self.mechanical_command_state["controls_unlocked"] = True
            self._increment_mechanical_sequence_locked()
        self._publish_mechanical_command()

    def send_mechanical_states(self, states):
        """Update desired outputs and immediately publish the full snapshot."""
        unknown_systems = set(states) - set(MECHANICAL_SYSTEM_NAMES)
        if unknown_systems:
            self.get_logger().error(
                "Unknown mechanical systems requested: "
                f"{sorted(unknown_systems)}"
            )
            return False

        with self.mechanical_command_lock:
            if (
                any(states.values())
                and not self.mechanical_command_state["controls_unlocked"]
            ):
                self.get_logger().warning(
                    "Ignored mechanical ON command while science is locked"
                )
                return False
            for system_name, state in states.items():
                self.mechanical_command_state[system_name] = bool(state)
            self._increment_mechanical_sequence_locked()
        self._publish_mechanical_command()
        return True

    def _increment_mechanical_sequence_locked(self):
        self.mechanical_command_sequence = (
            self.mechanical_command_sequence + 1
        ) & 0xFFFFFFFF

    def _publish_mechanical_command(self):
        """Publish the desired state heartbeat used by the rover watchdog."""
        with self.mechanical_command_lock:
            sequence = self.mechanical_command_sequence
            state = dict(self.mechanical_command_state)

        msg = SpectrometryMechanicalControlMessage()
        msg.sequence = sequence
        msg.controls_unlocked = state["controls_unlocked"]
        for system_name, field_name in MECHANICAL_SYSTEM_FIELDS.items():
            setattr(msg, field_name, state[system_name])
        self.mechanical_command_publisher.publish(msg)

    def _mechanical_status_callback(self, msg):
        """Cache the latest rover mechanical status."""
        with self.mechanical_status_lock:
            self.mechanical_status = {
                "received_at": time.monotonic(),
                "command_sequence": int(msg.command_sequence),
                "command_link_active": bool(msg.command_link_active),
                "can_connected": bool(msg.can_connected),
                "command_state_valid": bool(msg.command_state_valid),
                "controls_unlocked": bool(msg.controls_unlocked),
                "states": {
                    system_name: bool(getattr(msg, field_name))
                    for system_name, field_name in (
                        MECHANICAL_SYSTEM_FIELDS.items()
                    )
                },
            }

    def get_mechanical_status_snapshot(self):
        """Return a thread-safe rover status and desired-command snapshot."""
        with self.mechanical_command_lock:
            desired_sequence = self.mechanical_command_sequence
            desired_controls_unlocked = self.mechanical_command_state[
                "controls_unlocked"
            ]
        with self.mechanical_status_lock:
            status = (
                dict(self.mechanical_status)
                if self.mechanical_status is not None
                else None
            )

        status_available = bool(
            status
            and time.monotonic() - status["received_at"]
            <= MECHANICAL_STATUS_TIMEOUT_S
        )
        if not status_available:
            return {
                "status_available": False,
                "command_pending": False,
                "command_link_active": False,
                "can_connected": False,
                "command_state_valid": False,
                "controls_unlocked": False,
                "desired_controls_unlocked": desired_controls_unlocked,
                "states": {
                    system_name: None
                    for system_name in MECHANICAL_SYSTEM_NAMES
                },
            }

        status["status_available"] = True
        status["command_pending"] = (
            status["command_sequence"] != desired_sequence
        )
        status["desired_controls_unlocked"] = desired_controls_unlocked
        return status

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
            self.get_logger().error(
                f"Failed to convert spectrometry image: {exc}"
            )
            return

        frame_id = msg.header.frame_id or "camera_image"
        image_path = self._write_spectrometry_image(
            cv_image, frame_id, msg.header.stamp
        )
        with self.spectrometry_image_lock:
            self.spectrometry_image = cv_image
            self.spectrometry_frame_id = frame_id
            self.spectrometry_image_count += 1

        self.get_logger().info(f"got a new image from frame_id:={frame_id}")
        if image_path is not None:
            self.get_logger().info(
                f"saved spectrometry image: {image_path.name}"
            )

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
        """Save one image using frame/time naming."""
        timestamp = datetime.fromtimestamp(stamp.sec + (stamp.nanosec * 1e-9))
        output_path = self._build_image_output_path(frame_id, timestamp)
        if not cv2.imwrite(str(output_path), image):
            self.get_logger().error(
                f"Failed to save spectrometry image: {output_path}"
            )
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

    def send_spectrometry_request(self, camera_id, reaction_type):
        """Issue an asynchronous service request with a short timeout."""
        if not self.spectrometry_client.wait_for_service(timeout_sec=1.0):
            return False

        request = SpectrometryInterface.Request()
        request.camera_id = int(camera_id)
        request.reaction_type = int(reaction_type)
        future = self.spectrometry_client.call_async(request)

        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            time.sleep(min(0.02, remaining))

        if not future.done() or future.exception() is not None:
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
