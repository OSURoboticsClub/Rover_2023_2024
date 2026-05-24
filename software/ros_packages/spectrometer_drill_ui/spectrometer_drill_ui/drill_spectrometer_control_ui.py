#!/usr/bin/env python3

import threading
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool


class ControlUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drill and Spectrometer Controls")

        self.node = _ControlNode()
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        self.cap_engaged = False
        self.drill_auto_enabled = False
        self.linear_auto_enabled = False

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        main = tk.Frame(self.root, padx=12, pady=12)
        main.pack(fill=tk.BOTH, expand=True)

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

        tk.Label(linear_frame, text="Speed (rps):").grid(row=0, column=0, sticky="w")
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
        tk.Button(linear_frame, text="Stop", width=10, command=self.stop_linear).grid(
            row=1, column=3, padx=(12, 0)
        )

        spec_frame = tk.LabelFrame(main, text="Spectrometer", padx=10, pady=10)
        spec_frame.pack(fill=tk.X)
        tk.Label(spec_frame, text="Pending next instructions.").pack(anchor="w")

    def toggle_drill_auto(self):
        self.drill_auto_enabled = not self.drill_auto_enabled
        self.node.set_drill_active(self.drill_auto_enabled, 1.0)
        self.drill_status_var.set("On (Auto)" if self.drill_auto_enabled else "Off")
        self.drill_button.config(
            text="Auto Stop" if self.drill_auto_enabled else "Auto Start"
        )

    def start_drill_hold(self, _event):
        if self.drill_auto_enabled:
            self.drill_auto_enabled = False
            self.drill_button.config(text="Auto Start")
        self.node.set_drill_active(True, 1.0)
        self.drill_status_var.set("On (Hold)")

    def stop_drill_hold(self, _event):
        self.node.set_drill_active(False, 0.0)
        self.drill_status_var.set("Off")

    def toggle_cap(self):
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
            messagebox.showerror("Invalid Input", "Linear speed must be a number.")
            return None

        direction = 1.0 if self.linear_direction_var.get() == "Extend (+)" else -1.0
        return speed * direction

    def toggle_linear_auto(self):
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
        self.linear_status_var.set(f"Commanded {signed_speed:.3f} rps (Auto)")

    def start_linear_hold(self, _event):
        signed_speed = self._get_linear_command()
        if signed_speed is None:
            return
        if self.linear_auto_enabled:
            self.linear_auto_enabled = False
            self.linear_auto_button.config(text="Auto Start")
        self.node.set_linear_active(True, signed_speed)
        self.linear_status_var.set(f"Commanded {signed_speed:.3f} rps (Hold)")

    def stop_linear_hold(self, _event):
        self.node.set_linear_active(False, 0.0)
        self.linear_status_var.set("Not Publishing")

    def stop_linear(self):
        self.linear_auto_enabled = False
        self.linear_auto_button.config(text="Auto Start")
        self.node.set_linear_active(False, 0.0)
        self.linear_status_var.set("Not Publishing")

    def on_close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        self.root.destroy()

 
class _ControlNode(Node):
    def __init__(self):
        super().__init__("spectrometer_drill_ui")
        self.drill_pub = self.create_publisher(Float32, "/drill/control", 10)
        self.linear_pub = self.create_publisher(Float32, "/linear_actuator/control", 10)
        self.cap_client = self.create_client(SetBool, "/drill/cap/set_engaged")
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
        if self.drill_active:
            self.publish_drill(self.drill_value)
        if self.linear_active:
            self.publish_linear(self.linear_value)

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
