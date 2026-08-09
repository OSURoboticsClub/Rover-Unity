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
)
MECHANICAL_SYSTEM_FIELDS = {
    "valve_1": "valve_1_on",
    "valve_2": "valve_2_on",
    "pump": "pump_on",
    "coil_1": "coil_1_on",
    "coil_2": "coil_2_on",
}
MECHANICAL_STATUS_TIMEOUT_S = 1.0
MECHANICAL_ACTIVE_BUTTON_BG = "#242629"
MECHANICAL_ACTIVE_BUTTON_FG = "#ffffff"

# Set these to the actual camera ids on the rover.
NINHYDRIN_CAMERA_ID = 0
BENEDICTS_CAMERA_ID = 1
SPECTROMETRY_REACTIONS = {
    "benedicts": {
        "name": "Benedict's",
        "camera_id": BENEDICTS_CAMERA_ID,
        "reaction_type": 1,
    },
    "ninhydrin": {
        "name": "Ninhydrin",
        "camera_id": NINHYDRIN_CAMERA_ID,
        "reaction_type": 3,
    },
}


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

        self.spectrometry_window_visible = False
        self.spectrometry_request_pending = False
        self.spectrometry_requests_started = set()
        self.displayed_spectrometry_image_count = 0
        self.science_controls_locked = True
        self.mechanical_control_buttons = []
        self.mechanical_button_bindings = []
        self._closing = False

        self._build_ui()
        self._schedule_spectrometry_display()
        self._schedule_spectrometry_status_display()
        self._schedule_mechanical_status_display()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        """Construct the spectrometer controls."""
        main = tk.Frame(self.root, padx=12, pady=12)
        main.pack(fill=tk.BOTH, expand=True)

        spec_frame = tk.LabelFrame(main, text="Spectrometer Image Collection",
                                   padx=10, pady=10)
        spec_frame.pack(fill=tk.X)

        self.spec_window_status_var = tk.StringVar(value="Hidden")
        tk.Label(spec_frame, text="Image window:").grid(
            row=0, column=0, sticky="w"
        )
        tk.Label(
            spec_frame,
            textvariable=self.spec_window_status_var,
            width=12,
            anchor="w",
        ).grid(row=0, column=1, sticky="w")

        self.spec_window_button = tk.Button(
            spec_frame,
            text="Show Window",
            width=14,
            command=self.toggle_spectrometry_window,
        )
        self.spec_window_button.grid(row=0, column=2, padx=(12, 0))

        self.spectrometry_reaction_widgets = {}
        for row, reaction in enumerate(SPECTROMETRY_REACTIONS, start=1):
            self._build_spectrometry_reaction_row(spec_frame, row, reaction)

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
        self._set_mechanical_controls_state(tk.DISABLED)
        self._update_mechanical_button_styles()

    def _build_spectrometry_reaction_row(self, parent, row, reaction):
        """Build collection controls and indicators for one reaction."""
        reaction_name = SPECTROMETRY_REACTIONS[reaction]["name"]
        publishing_var = tk.StringVar(value="Off")
        elapsed_var = tk.StringVar(value="00:00:00")

        start_button = tk.Button(
            parent,
            text=f"Start Collecting {reaction_name} Images",
            width=32,
            command=lambda: self.start_spectrometry_collection(reaction),
        )
        start_button.grid(
            row=row, column=0, columnspan=2, pady=(8, 0), sticky="w"
        )

        stop_button = tk.Button(
            parent,
            text=f"Stop Saving {reaction_name} Images",
            width=30,
            command=lambda: self.stop_spectrometry_collection(reaction),
            state=tk.DISABLED,
        )
        stop_button.grid(
            row=row, column=2, padx=(8, 16), pady=(8, 0), sticky="w"
        )

        tk.Label(parent, text="Publishing:").grid(
            row=row, column=3, pady=(8, 0), sticky="e"
        )
        tk.Label(
            parent, textvariable=publishing_var, width=4, anchor="w"
        ).grid(row=row, column=4, padx=(4, 16), pady=(8, 0), sticky="w")
        tk.Label(parent, text="Collection time:").grid(
            row=row, column=5, pady=(8, 0), sticky="e"
        )
        tk.Label(
            parent, textvariable=elapsed_var, width=8, anchor="w"
        ).grid(row=row, column=6, padx=(4, 0), pady=(8, 0), sticky="w")

        self.spectrometry_reaction_widgets[reaction] = {
            "start_button": start_button,
            "stop_button": stop_button,
            "publishing_var": publishing_var,
            "elapsed_var": elapsed_var,
        }

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
            state=tk.NORMAL,
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
        self._register_mechanical_button(
            button, (system_name,), enabled
        )

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
        self._register_mechanical_button(button, system_names, enabled)

    def _register_mechanical_button(self, button, system_names, enabled):
        """Record the command and default appearance for a button."""
        self.mechanical_button_bindings.append(
            {
                "button": button,
                "system_names": tuple(system_names),
                "enabled": bool(enabled),
                "default_style": {
                    "background": button.cget("background"),
                    "foreground": button.cget("foreground"),
                    "activebackground": button.cget("activebackground"),
                    "activeforeground": button.cget("activeforeground"),
                    "relief": button.cget("relief"),
                },
            }
        )

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
            width=23,
            anchor="w",
        ).pack(side=tk.LEFT, padx=(4, 0))

    def unlock_science_control(self):
        """Allow mechanical commands until the operator locks them again."""
        self.node.set_science_locked(False)
        self._apply_unlocked_ui("Unlocked (unconfirmed)")

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
        self._update_mechanical_button_styles()

    def _apply_unlocked_ui(self, status_text="Unlocked (sent)"):
        """Enable output buttons from the local command state."""
        self.science_controls_locked = False
        self.science_lock_status_var.set(status_text)
        self.lock_science_button.config(state=tk.NORMAL)
        self.unlock_science_button.config(state=tk.DISABLED)
        self._set_mechanical_controls_state(tk.NORMAL)
        self._update_mechanical_button_styles()

    def _set_mechanical_controls_state(self, state):
        for button in self.mechanical_control_buttons:
            button.config(state=state)

    def _update_mechanical_button_styles(self):
        """Highlight buttons matching the locally published command state."""
        states = self.node.get_mechanical_command_states()
        for binding in self.mechanical_button_bindings:
            if self.science_controls_locked:
                binding["button"].config(**binding["default_style"])
                continue
            is_active = all(
                states[system_name] == binding["enabled"]
                for system_name in binding["system_names"]
            )
            if is_active:
                binding["button"].config(
                    background=MECHANICAL_ACTIVE_BUTTON_BG,
                    foreground=MECHANICAL_ACTIVE_BUTTON_FG,
                    activebackground=MECHANICAL_ACTIVE_BUTTON_BG,
                    activeforeground=MECHANICAL_ACTIVE_BUTTON_FG,
                    relief=tk.SUNKEN,
                )
            else:
                binding["button"].config(**binding["default_style"])

    def set_mechanical_system(self, system_name, enabled):
        """Send one output request through the ROS command heartbeat."""
        self.set_mechanical_systems((system_name,), enabled)

    def set_mechanical_systems(self, system_names, enabled):
        """Send an atomic output request if controls are unlocked."""
        if self.science_controls_locked:
            return

        states = {system_name: bool(enabled) for system_name in system_names}
        if self.node.send_mechanical_states(states):
            self._update_mechanical_button_styles()

    def _schedule_mechanical_status_display(self):
        """Refresh monitors from the last rover-side CAN command state."""
        status_variables = {
            "valve_1": self.valve_1_status_var,
            "valve_2": self.valve_2_status_var,
            "pump": self.pump_status_var,
            "coil_1": self.coil_1_status_var,
            "coil_2": self.coil_2_status_var,
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

        if snapshot["desired_controls_unlocked"]:
            if not snapshot["status_available"]:
                status_text = "Unlocked (unconfirmed)"
            elif not snapshot["can_connected"]:
                status_text = "Unlocked (CAN unavailable)"
            elif not snapshot["command_state_valid"]:
                status_text = "Unlocked (state unknown)"
            elif not snapshot["command_link_active"]:
                status_text = "Unlocked (link inactive)"
            elif snapshot["command_pending"]:
                status_text = "Unlock requested"
            elif snapshot["controls_unlocked"]:
                status_text = "Unlocked (sent)"
            else:
                status_text = "Unlocked (unconfirmed)"
            self._apply_unlocked_ui(status_text)
        else:
            if snapshot["status_available"] and snapshot["command_pending"]:
                status_text = "Lock requested"
            elif snapshot["status_available"]:
                status_text = "Locked (sent)"
            else:
                status_text = "Locked (status unavailable)"
            self._apply_locked_ui(status_text, allow_unlock=True)

        if not self._closing:
            self.root.after(250, self._schedule_mechanical_status_display)

    def toggle_spectrometry_window(self):
        """Show or hide the latest spectrometry image window."""
        target = not self.spectrometry_window_visible
        if not self.node.set_spectrometry_window_visible(target):
            messagebox.showerror(
                "Execution Error",
                "Unable to show the spectrometry image window.",
            )
            return

        self.spectrometry_window_visible = target
        self.displayed_spectrometry_image_count = 0
        self.spec_window_status_var.set("Visible" if target else "Hidden")
        self.spec_window_button.config(
            text="Hide Window" if target else "Show Window"
        )

    def start_spectrometry_collection(self, reaction):
        """Start saving one reaction and request its rover feed if needed."""
        if self.spectrometry_request_pending:
            messagebox.showwarning(
                "Request Pending",
                "Wait for the current spectrometry request to complete.",
            )
            return

        self.node.start_spectrometry_collection(reaction)
        if reaction in self.spectrometry_requests_started:
            self._refresh_spectrometry_action_state()
            return

        reaction_config = SPECTROMETRY_REACTIONS[reaction]
        reaction_name = reaction_config["name"]
        self.spectrometry_request_pending = True
        self._refresh_spectrometry_action_state()

        def request_worker():
            ok = self.node.send_spectrometry_request(
                reaction_config["camera_id"],
                reaction_config["reaction_type"],
            )
            if self._closing:
                return
            try:
                self.root.after(
                    0,
                    lambda: self._handle_spectrometry_request_result(
                        ok, reaction, reaction_name
                    ),
                )
            except (RuntimeError, tk.TclError):
                pass

        threading.Thread(target=request_worker, daemon=True).start()

    def _handle_spectrometry_request_result(self, ok, reaction, reaction_name):
        self.spectrometry_request_pending = False

        if not ok:
            self.node.stop_spectrometry_collection(
                reaction, reset_elapsed=True
            )
            self._refresh_spectrometry_action_state()
            messagebox.showerror(
                "Execution Error",
                f"{reaction_name} request failed or timed out.",
            )
            return

        self.spectrometry_requests_started.add(reaction)
        self._refresh_spectrometry_action_state()

    def stop_spectrometry_collection(self, reaction):
        """Stop saving one reaction's images."""
        self.node.stop_spectrometry_collection(reaction)
        self._refresh_spectrometry_action_state()

    def _schedule_spectrometry_status_display(self):
        """Refresh per-reaction publishing and collection-time indicators."""
        snapshot = self.node.get_spectrometry_collection_snapshot()
        for reaction, state in snapshot.items():
            widgets = self.spectrometry_reaction_widgets[reaction]
            publishing = "On" if state["image_received"] else "Off"
            widgets["publishing_var"].set(publishing)
            widgets["elapsed_var"].set(
                self._format_elapsed_time(state["elapsed_s"])
            )
        self._refresh_spectrometry_action_state(snapshot)

        if not self._closing:
            self.root.after(250, self._schedule_spectrometry_status_display)

    def _refresh_spectrometry_action_state(self, snapshot=None):
        if snapshot is None:
            snapshot = self.node.get_spectrometry_collection_snapshot()
        for reaction, state in snapshot.items():
            widgets = self.spectrometry_reaction_widgets[reaction]
            start_state = (
                tk.DISABLED
                if state["active"] or self.spectrometry_request_pending
                else tk.NORMAL
            )
            stop_state = (
                tk.NORMAL
                if state["active"] and not self.spectrometry_request_pending
                else tk.DISABLED
            )
            widgets["start_button"].config(state=start_state)
            widgets["stop_button"].config(state=stop_state)

    @staticmethod
    def _format_elapsed_time(elapsed_s):
        total_seconds = max(0, int(elapsed_s))
        hours, remainder = divmod(total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

    def _schedule_spectrometry_display(self):
        """Update the OpenCV window from the latest image snapshot."""
        try:
            if self.spectrometry_window_visible:
                snapshot = self.node.get_spectrometry_image_snapshot()
                if snapshot is not None:
                    image, _, image_count = snapshot
                    if image_count != self.displayed_spectrometry_image_count:
                        cv2.imshow(SPECTROMETRY_IMAGE_WINDOW, image)
                        self.displayed_spectrometry_image_count = image_count
                    cv2.waitKey(1)
        except cv2.error as exc:
            self._handle_spectrometry_display_error(exc)

        if not self._closing:
            self.root.after(50, self._schedule_spectrometry_display)

    def _handle_spectrometry_display_error(self, exc):
        self.node.get_logger().error(
            f"Spectrometry image display failed: {exc}"
        )
        self.node.set_spectrometry_window_visible(False)
        self.spectrometry_window_visible = False
        self.displayed_spectrometry_image_count = 0
        self.spec_window_status_var.set("Display error")
        self.spec_window_button.config(text="Show Window")

    def on_close(self):
        """Shut down ROS resources and terminate the Tk application cleanly."""
        self._closing = True
        self.node.set_science_locked(True)
        self.node.set_spectrometry_window_visible(False)
        for reaction in SPECTROMETRY_REACTIONS:
            self.node.stop_spectrometry_collection(reaction)
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
        self.spectrometry_window_visible = False
        self.spectrometry_sub = None
        self.spectrometry_frame_reactions = {
            f"camera_{config['camera_id']}": reaction
            for reaction, config in SPECTROMETRY_REACTIONS.items()
        }
        self.spectrometry_collections = {
            reaction: {
                "active": False,
                "image_received": False,
                "started_at": None,
                "elapsed_s": 0.0,
            }
            for reaction in SPECTROMETRY_REACTIONS
        }
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

    def get_mechanical_command_states(self):
        """Return the locally published output states."""
        with self.mechanical_command_lock:
            return {
                system_name: self.mechanical_command_state[system_name]
                for system_name in MECHANICAL_SYSTEM_NAMES
            }

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

    def set_spectrometry_window_visible(self, visible):
        """Show or hide the image window without changing collection."""
        visible = bool(visible)
        if visible == self.spectrometry_window_visible:
            return True
        if visible:
            try:
                cv2.namedWindow(SPECTROMETRY_IMAGE_WINDOW, cv2.WINDOW_NORMAL)
            except cv2.error as exc:
                self.get_logger().error(
                    f"Failed to create spectrometry image window: {exc}"
                )
                return False
            self.spectrometry_window_visible = True
            self.get_logger().info("Showing spectrometry image window")
            return True

        self.spectrometry_window_visible = False
        try:
            cv2.destroyWindow(SPECTROMETRY_IMAGE_WINDOW)
        except cv2.error:
            pass
        return True

    def start_spectrometry_collection(self, reaction):
        """Start saving one reaction and subscribe when first needed."""
        if reaction not in self.spectrometry_collections:
            raise ValueError(f"Unknown spectrometry reaction: {reaction}")

        with self.spectrometry_image_lock:
            state = self.spectrometry_collections[reaction]
            if not state["active"]:
                state["active"] = True
                state["started_at"] = time.monotonic()
                state["elapsed_s"] = 0.0
            if self.spectrometry_sub is None:
                self.spectrometry_sub = self.create_subscription(
                    Image,
                    "science/image",
                    self._spectrometry_image_callback,
                    10,
                )

    def stop_spectrometry_collection(self, reaction, reset_elapsed=False):
        """Stop saving one reaction and unsubscribe when neither is active."""
        if reaction not in self.spectrometry_collections:
            raise ValueError(f"Unknown spectrometry reaction: {reaction}")

        subscription_to_destroy = None
        with self.spectrometry_image_lock:
            state = self.spectrometry_collections[reaction]
            if state["active"] and state["started_at"] is not None:
                state["elapsed_s"] += (
                    time.monotonic() - state["started_at"]
                )
            state["active"] = False
            state["started_at"] = None
            if reset_elapsed:
                state["elapsed_s"] = 0.0
            if (
                self.spectrometry_sub is not None
                and not any(
                    collection["active"]
                    for collection in self.spectrometry_collections.values()
                )
            ):
                subscription_to_destroy = self.spectrometry_sub
                self.spectrometry_sub = None

        if subscription_to_destroy is not None:
            self.destroy_subscription(subscription_to_destroy)

    def get_spectrometry_collection_snapshot(self):
        """Return collection state and live elapsed time for each reaction."""
        now = time.monotonic()
        with self.spectrometry_image_lock:
            snapshot = {}
            for reaction, state in self.spectrometry_collections.items():
                elapsed_s = state["elapsed_s"]
                if state["active"] and state["started_at"] is not None:
                    elapsed_s += now - state["started_at"]
                snapshot[reaction] = {
                    "active": state["active"],
                    "image_received": state["image_received"],
                    "elapsed_s": elapsed_s,
                }
        return snapshot

    def _spectrometry_image_callback(self, msg):
        """Convert, persist, and cache each spectrometry image frame."""
        reaction = self.spectrometry_frame_reactions.get(msg.header.frame_id)
        if reaction is None:
            self.get_logger().warning(
                f"Ignoring image with unknown frame_id: {msg.header.frame_id}"
            )
            return

        with self.spectrometry_image_lock:
            state = self.spectrometry_collections[reaction]
            state["image_received"] = True
            if not state["active"]:
                return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(
                f"Failed to convert spectrometry image: {exc}"
            )
            return

        with self.spectrometry_image_lock:
            if not self.spectrometry_collections[reaction]["active"]:
                return
            image_path = self._write_spectrometry_image(
                cv_image, reaction, msg.header.stamp
            )
            self.spectrometry_image = cv_image
            self.spectrometry_frame_id = msg.header.frame_id
            self.spectrometry_image_count += 1

        self.get_logger().info(
            f"got a new image from frame_id:={msg.header.frame_id}"
        )
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

    def _write_spectrometry_image(self, image, reaction, stamp):
        """Save one image using frame/time naming."""
        timestamp = datetime.fromtimestamp(stamp.sec + (stamp.nanosec * 1e-9))
        output_path = self._build_image_output_path(reaction, timestamp)
        if not cv2.imwrite(str(output_path), image):
            self.get_logger().error(
                f"Failed to save spectrometry image: {output_path}"
            )
            return None
        return output_path

    def _build_image_output_path(self, reaction, timestamp):
        time_text = timestamp.strftime("%H:%M:%S")
        return self.spectrometry_save_dir / f"{reaction}_{time_text}.png"

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
