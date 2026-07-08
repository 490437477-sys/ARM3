# -*- coding: utf-8 -*-
"""
Servo Arm Control System - Python GUI v2.1
==========================================

Enhanced desktop GUI for the 5-servo Arduino UNO R3 robotic arm.

Changes in v2.1:
- Removed the Grab / Release / Relax quick-action buttons
- Added a live 2D arm diagram (Canvas) that mirrors the current angles
  - Side view: base, upper arm, forearm, wrist, gripper
  - S0 base rotation shown as a small compass arrow on the base
  - All five angles labelled in their servo colour

Serial protocol (9600 baud, newline-terminated):
    h | help        Show help on Arduino side
    s | status      Print all 5 servo current angles
    <angle>         Set ALL servos to <angle> (S4 clamped to 0-90)
    <id> <angle>    Set a single servo (0 <= id <= 4)
    a b c d e       Batch set all 5 servo targets at once

Limits: S0-S3 are 0-180, S4 (gripper) is 0-90.
Note: Servo2 direction is reversed internally on the Arduino.
"""

import math
import threading
import queue
import time
import re
import json
import collections
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
import serial
from serial.tools import list_ports

__version__ = "2.2.9"

# ---------------------------------------------------------------------------
# Theme
# ---------------------------------------------------------------------------
BG          = "#1e1e2e"
PANEL_BG    = "#313244"
INPUT_BG    = "#1e1e2e"
TERMINAL_BG = "#11111b"
ACCENT      = "#89b4fa"
GREEN       = "#a6e3a1"
RED         = "#f38ba8"
YELLOW      = "#f9e2af"
MAUVE       = "#cba6f7"
TEXT        = "#cdd6f4"
SUBTEXT     = "#a6adc8"

SERIAL_TIMEOUT = 0.3
WRITE_TIMEOUT  = 0.5
DEBOUNCE_MS    = 60
RX_POLL_MS     = 80
ARM_REFRESH_MS = 60
# The Arduino firmware updates servo positions every moveDelay ms
# (smoothStep deg per update), so the *fastest* the arm can follow a
# playback stream is one frame per moveDelay.  We use 40 ms (= Arduino
# moveDelay) as the floor so playback can keep up with the servo's
# actual running speed rather than the 200 ms conservative cushion.
# The slider's lower bound is clamped to this value.
SERVO_MIN_INTERVAL_MS = 3000


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
class ServoControlApp:

    SERVO_COUNT   = 5
    SERVO_LIMITS  = [180, 180, 180, 180, 90]
    SERVO_NAMES   = [
        "S0 \u0627\u0644\u0642\u0627\u0639\u062f\u0629",
        "S1 \u0627\u0644\u0643\u062a\u0641",
        "S2 \u0627\u0644\u0645\u0631\u0641\u0642",
        "S3 \u0627\u0644\u0645\u0639\u0635\u0645",
        "S4 \u0627\u0644\u0645\u0627\u0633\u0643",
    ]
    SERVO_COLORS  = ["#f38ba8", "#fab387", "#f9e2af", "#a6e3a1", "#89b4fa"]

    STATUS_RE = re.compile(r"(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)")
    SERVO_TARGET_RE = re.compile(r"S(\d+)\s*target\s*=\s*(\d+)")

    def __init__(self):
        self.root = tk.Tk()
        self.root.title(f"Servo Arm Control  v{__version__}")
        self.root.geometry("1280x960")
        self.root.minsize(1100, 820)
        self.root.configure(bg=BG)

        self.ser = None
        self.connected = False
        self._reader_thread = None
        self._reader_alive = False
        self._rx_queue = queue.Queue()
        self._send_lock = threading.Lock()
        self._send_after_id = None
        self._poll_after_id = None
        self._arm_after_id  = None
        self.arm_joints      = []
        self._dragging_joint = None

        self.target_vars   = [tk.IntVar(value=90) for _ in range(self.SERVO_COUNT)]
        self.actual_angles = [90] * self.SERVO_COUNT
        self.auto_send     = tk.BooleanVar(value=True)
        self.autoscroll    = tk.BooleanVar(value=True)

        self.actual_labels = []
        self.sliders       = []
        self.entries       = []
        self._home_btn     = None
        self._cmd_preview  = tk.StringVar(value="90 90 90 90 90")

        # Trajectory recording / playback
        self.trajectory        = []
        self.recording         = False
        self.playing           = False
        self._play_after_id    = None
        self._last_snapshot_ts = 0
        self._suppress_snap    = False
        self.traj_play_speed   = tk.IntVar(value=3500)
        self.traj_loop         = tk.BooleanVar(value=False)
        self.traj_status       = tk.StringVar(value="\u25CF  \u062e\u0627\u0645\u0644 (0 \u0625\u0637\u0627\u0631\u0627\u062a)")
        self.traj_list         = None
        self.traj_record_btn   = None
        self.traj_play_btn     = None
        self._play_index       = 0
        self.SNAPSHOT_DEBOUNCE_MS = 150
        self.TRAIL_MAX_POINTS     = 200

        # Whenever any target angle changes, refresh the Custom-Command
        # live preview and the Home-button state.
        for _v in self.target_vars:
            _v.trace_add("write", lambda *_a: self._update_cmd_preview())

        self._configure_styles()
        self._build_ui()
        self._refresh_ports()
        self._update_cmd_preview()    # prime the live preview / Home button
        self.root.after(RX_POLL_MS,     self._poll_rx_queue)
        self.root.after(ARM_REFRESH_MS, self._arm_render_loop)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._log("\u0628\u062f\u0623 \u0627\u0644\u062a\u062d\u062f\u064a\u062b \u0627\u0644\u062a\u0644\u0642\u0627\u0626\u064a \u0644\u0644\u0645\u062e\u0637\u0637 (\u0643\u0644 " + str(ARM_REFRESH_MS) + " ms)", "info")

    def _configure_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame",            background=BG)
        style.configure("Panel.TFrame",      background=PANEL_BG)
        style.configure("TLabel",            background=BG, foreground=TEXT, font=("Segoe UI", 10))
        style.configure("Panel.TLabel",      background=PANEL_BG, foreground=TEXT, font=("Segoe UI", 10))
        style.configure("PanelMute.TLabel",  background=PANEL_BG, foreground=SUBTEXT, font=("Segoe UI", 9))
        style.configure("TButton",           font=("Segoe UI", 10))
        style.configure("Small.TButton",     font=("Segoe UI", 11), padding=(8, 4))
        style.configure("TCombobox",         font=("Segoe UI", 10))
        style.configure("TCheckbutton",      background=PANEL_BG, foreground=TEXT, font=("Segoe UI", 9), focuscolor=PANEL_BG)
        style.map("TCheckbutton", background=[("active", PANEL_BG)], foreground=[("active", TEXT)])
        style.configure("TLabelframe",       background=PANEL_BG, foreground=ACCENT, borderwidth=0, relief="flat")
        style.configure("TLabelframe.Label", background=PANEL_BG, foreground=ACCENT, font=("Segoe UI", 11, "bold"))
        style.configure("TSeparator",        background=SUBTEXT)

    def _build_ui(self):
        header = tk.Frame(self.root, bg=BG)
        header.pack(pady=(14, 4))
        tk.Label(header, text="\u0627\u0644\u062a\u062d\u0643\u0645 \u0641\u064a \u0630\u0631\u0627\u0639 \u0627\u0644\u0633\u064a\u0631\u0641\u0648",
                 font=("Segoe UI", 20, "bold"), fg=ACCENT, bg=BG).pack(side="left")
        tk.Label(header, text="  \u00b7  Arduino UNO R3  \u00b7  5\u00d7 MG995  \u00b7  \u0645\u0646\u0641\u0630 \u062a\u0633\u0644\u0633\u0644\u064a @ 9600",
                 font=("Segoe UI", 9), fg=SUBTEXT, bg=BG).pack(side="left", padx=8)

        self._build_connection_bar()

        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=14, pady=8)

        left = tk.Frame(main, bg=BG)
        left.pack(side="left", fill="both", expand=True)

        right = tk.Frame(main, bg=BG, width=480)
        right.pack(side="right", fill="y", padx=(12, 0))
        right.pack_propagate(False)

        # Left column: Arm Status Diagram panel now embeds the Servo Control
        # sliders directly below the 2D canvas (one combined panel).
        self._build_arm_diagram(left)

        # Right sidebar: Trajectory -> Custom Command -> Serial Log.
        self._build_right_panel(right)
        self._build_log_panel(right)

    def _build_connection_bar(self):
        bar = tk.Frame(self.root, bg=PANEL_BG)
        bar.pack(fill="x", padx=14, pady=(6, 0), ipady=8)

        inner = tk.Frame(bar, bg=PANEL_BG)
        inner.pack(fill="x", padx=12)

        tk.Label(inner, text="\u0627\u0644\u0627\u062a\u0635\u0627\u0644", font=("Segoe UI", 10, "bold"),
                 fg=ACCENT, bg=PANEL_BG).pack(side="left", padx=(0, 12))

        tk.Label(inner, text="\u0627\u0644\u0645\u0646\u0641\u0630:", fg=TEXT, bg=PANEL_BG).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(inner, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side="left", padx=(4, 4))
        ttk.Button(inner, text="\u27f3", width=3, command=self._refresh_ports).pack(side="left")

        tk.Label(inner, text="  \u0627\u0644\u0633\u0631\u0639\u0629:", fg=TEXT, bg=PANEL_BG).pack(side="left", padx=(8, 0))
        self.baud_var = tk.StringVar(value="9600")
        ttk.Combobox(inner, textvariable=self.baud_var, width=8, state="readonly",
                     values=["9600", "19200", "38400", "57600", "115200"]).pack(side="left", padx=4)

        self.conn_btn = tk.Button(inner, text="\u25cf  \u0627\u062a\u0635\u0627\u0644", font=("Segoe UI", 10, "bold"),
                                  bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT,
                                  relief="flat", padx=14, pady=4, cursor="hand2",
                                  command=self._toggle_connection)
        self.conn_btn.pack(side="left", padx=(14, 10))

        self._status_canvas = tk.Canvas(inner, width=14, height=14, bg=PANEL_BG, highlightthickness=0)
        self._status_canvas.pack(side="left", padx=(6, 4))
        self._status_dot = self._status_canvas.create_oval(2, 2, 12, 12, fill=RED, outline=RED)
        self._status_lbl = tk.Label(inner, text="\u063a\u064a\u0631 \u0645\u062a\u0635\u0644", fg=SUBTEXT, bg=PANEL_BG, font=("Segoe UI", 9))
        self._status_lbl.pack(side="left")

    def _build_servo_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="  \u0627\u0644\u062a\u062d\u0643\u0645 \u0628\u0627\u0644\u0633\u064a\u0631\u0641\u0648  ", padding=(14, 10))
        panel.pack(fill="x", pady=(0, 10))

        hdr = tk.Frame(panel, bg=PANEL_BG)
        hdr.pack(fill="x", pady=(0, 4))
        tk.Label(hdr, text="", width=22, bg=PANEL_BG).pack(side="left")
        tk.Label(hdr, text="\u0627\u0644\u062d\u0627\u0644\u064a", font=("Segoe UI", 9, "bold"),
                 fg=GREEN, bg=PANEL_BG, width=6).pack(side="left")
        tk.Label(hdr, text="\u0627\u0644\u0647\u062f\u0641 (\u0634\u0631\u064a\u0637 / \u062d\u0642\u0644)",
                 font=("Segoe UI", 9, "bold"), fg=SUBTEXT, bg=PANEL_BG).pack(side="left", padx=(4, 0))

        for i in range(self.SERVO_COUNT):
            self._build_servo_row(panel, i)

    def _build_servo_row(self, parent, idx):
        color = self.SERVO_COLORS[idx]
        name  = self.SERVO_NAMES[idx]
        limit = self.SERVO_LIMITS[idx]

        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill="x", pady=3)

        tk.Label(row, text=f" {name} {limit}\u00b0 ",
                 font=("Consolas", 10, "bold"), fg=color, bg=PANEL_BG,
                 width=14, anchor="w").pack(side="left")

        actual_lbl = tk.Label(row, text=" 90\u00b0", font=("Consolas", 13, "bold"),
                              fg=GREEN, bg=PANEL_BG, width=6)
        actual_lbl.pack(side="left", padx=(2, 6))
        self.actual_labels.append(actual_lbl)

        slider = tk.Scale(row, from_=0, to=limit, orient="horizontal",
                          variable=self.target_vars[idx], length=300,
                          bg=PANEL_BG, fg=TEXT, troughcolor=INPUT_BG,
                          highlightthickness=0, showvalue=False,
                          activebackground=color, sliderrelief="flat",
                          command=lambda _v, i=idx: self._on_slider_change(i))
        slider.pack(side="left", padx=4)
        self.sliders.append(slider)

        entry = tk.Entry(row, textvariable=self.target_vars[idx], width=5,
                         font=("Consolas", 11), justify="center",
                         bg=INPUT_BG, fg=TEXT, insertbackground=TEXT, relief="flat")
        entry.pack(side="left", padx=4, ipady=2)
        entry.bind("<Return>",   lambda _e, i=idx: self._on_entry_change(i))
        entry.bind("<FocusOut>", lambda _e, i=idx: self._on_entry_change(i))
        self.entries.append(entry)

        tk.Button(row, text="\u062a\u0639\u064a\u064a\u0646", font=("Segoe UI", 11, "bold"),
                  bg=color, fg="#1e1e2e", activebackground=color,
                  relief="flat", padx=12, pady=5, cursor="hand2",
                  command=lambda i=idx: self._on_entry_change(i)).pack(side="left", padx=2)

        tk.Button(row, text="\u21ba", font=("Segoe UI", 11, "bold"),
                  bg=PANEL_BG, fg=SUBTEXT, activebackground=PANEL_BG,
                  relief="flat", padx=8, pady=5, cursor="hand2",
                  command=lambda i=idx: self._reset_servo(i)).pack(side="left", padx=(2, 0))

    def _build_log_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="  \u0633\u062c\u0644 \u0627\u0644\u0645\u0646\u0641\u0630 \u0627\u0644\u062a\u0633\u0644\u0633\u0644\u064a  ", padding=(10, 8))
        panel.pack(fill="x")

        toolbar = tk.Frame(panel, bg=PANEL_BG)
        toolbar.pack(fill="x")

        # Pack the right-side buttons FIRST so they reserve their natural widths
        # before the long status label takes the remaining space on the left.
        # Otherwise Clear gets squeezed between the label and Copy and its text
        # is truncated to "Cl".
        ttk.Button(toolbar, text="\u0646\u0633\u062e",  style="Small.TButton", command=self._copy_log).pack(side="right", padx=(4, 0))
        ttk.Button(toolbar, text="\u0645\u0633\u062d", style="Small.TButton", command=self._clear_log).pack(side="right")

        tk.Checkbutton(toolbar, text="\u062a\u0645\u0631\u064a\u0631 \u062a\u0644\u0642\u0627\u0626\u064a", variable=self.autoscroll,
                       font=("Segoe UI", 9), fg=SUBTEXT, bg=PANEL_BG,
                       selectcolor=INPUT_BG, activebackground=PANEL_BG).pack(side="left")
        tk.Label(toolbar, text="   \u2191 \u0645\u064f\u0631\u0633\u064e\u0644 (TX)   \u00b7   \u2193 \u0645\u064f\u0633\u062a\u0642\u0628\u064e\u0644 (RX)",
                 font=("Consolas", 8), fg=SUBTEXT, bg=PANEL_BG).pack(side="left")

        self.log_text = scrolledtext.ScrolledText(panel, height=18, bg=TERMINAL_BG, fg=TEXT,
                                                   font=("Consolas", 9), relief="flat", state="disabled",
                                                   insertbackground=TEXT, wrap="word")
        self.log_text.pack(fill="both", expand=True, pady=(6, 0))

        for tag, color in (("info", SUBTEXT), ("ok", GREEN), ("warn", YELLOW),
                           ("err", RED), ("tx", ACCENT), ("rx", MAUVE)):
            self.log_text.tag_configure(tag, foreground=color)

        self._log("\u0627\u0644\u0648\u0627\u062c\u0647\u0629 \u062c\u0627\u0647\u0632\u0629. \u0627\u0636\u063a\u0637 \u25cf \u0627\u062a\u0635\u0627\u0644 \u0644\u0641\u062a\u062d \u0645\u0646\u0641\u0630 \u062a\u0633\u0644\u0633\u0644\u064a.", "info")

    def _build_right_panel(self, parent):
        # Trajectory panel goes at the top of the right column now
        # (swapped with the Serial Log position from v2.2.3).
        self._build_traj_panel(parent)

        # Custom Command panel
        panel = ttk.LabelFrame(parent, text="  \u0623\u0645\u0631 \u0645\u062e\u0635\u0635  ", padding=(14, 10))
        panel.pack(fill="x", pady=(0, 10))

        cmd_frame = tk.Frame(panel, bg=PANEL_BG)
        cmd_frame.pack(fill="x", pady=(4, 0))
        self.custom_var = tk.StringVar()
        self.custom_var.trace_add("write", lambda *_a: self._update_cmd_preview())
        custom_entry = tk.Entry(cmd_frame, textvariable=self.custom_var,
                               font=("Consolas", 10), bg=INPUT_BG, fg=TEXT,
                               insertbackground=TEXT, relief="flat")
        custom_entry.pack(side="left", fill="x", expand=True, ipady=4)
        custom_entry.bind("<Return>", lambda _e: self._send_custom())
        tk.Button(cmd_frame, text="\u0625\u0631\u0633\u0627\u0644", font=("Segoe UI", 9, "bold"),
                  bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT,
                  relief="flat", padx=10, pady=2, cursor="hand2",
                  command=self._send_custom).pack(side="left", padx=(4, 0))

        # Live preview: shows the current batch command that auto-send
        # would transmit, mirroring the slider state in real time.
        preview_frame = tk.Frame(panel, bg=PANEL_BG)
        preview_frame.pack(fill="x", pady=(6, 0))
        tk.Label(preview_frame, text="\u2192", font=("Consolas", 10, "bold"),
                 fg=SUBTEXT, bg=PANEL_BG).pack(side="left")
        tk.Label(preview_frame, textvariable=self._cmd_preview,
                 font=("Consolas", 10, "bold"), fg=ACCENT, bg=PANEL_BG
                 ).pack(side="left", padx=(4, 0))

        tk.Label(panel, text="\u0623\u0645\u062b\u0644\u0629:  90   |   0 60   |   0 90 90 90 40",
                 font=("Consolas", 8), fg=SUBTEXT, bg=PANEL_BG).pack(anchor="w", pady=(4, 0))

        ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=10)

        tk.Checkbutton(panel, text="\u0625\u0631\u0633\u0627\u0644 \u062a\u0644\u0642\u0627\u0626\u064a \u0639\u0646\u062f \u062a\u062d\u0631\u064a\u0643 \u0627\u0644\u0634\u0631\u064a\u0637",
                       variable=self.auto_send, font=("Segoe UI", 9),
                       fg=TEXT, bg=PANEL_BG, selectcolor=INPUT_BG,
                       activebackground=PANEL_BG).pack(anchor="w")

        ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=10)

        tk.Label(panel, text="\u0627\u0644\u062d\u062f\u0648\u062f", font=("Segoe UI", 10, "bold"),
                 fg=ACCENT, bg=PANEL_BG).pack(anchor="w")
        tk.Label(panel, text="S0\u2013S3: 0\u2013180\u00b0\nS4 \u0627\u0644\u0645\u0627\u0633\u0643: 0\u201390\u00b0\n\u0627\u062a\u062c\u0627\u0647 S2 \u0645\u0639\u0643\u0648\u0633 \u0641\u064a Arduino",
                 font=("Consolas", 9), fg=SUBTEXT, bg=PANEL_BG, justify="left").pack(anchor="w", pady=(4, 0))

    def _build_arm_diagram(self, parent):
        """Build the combined Arm Status Diagram + Servo Control panel.
        The 2D arm canvas sits on top and the servo sliders are embedded
        directly below it (single panel).
        """
        panel = ttk.LabelFrame(parent, text="  \u0645\u062e\u0637\u0637 \u062d\u0627\u0644\u0629 \u0627\u0644\u0630\u0631\u0627\u0639  ", padding=(10, 8))
        panel.pack(fill="both", expand=True, pady=(0, 10))

        self.arm_canvas = tk.Canvas(panel, height=420, bg=TERMINAL_BG, highlightthickness=0)
        # Canvas can grow with the panel; the arm drawing inside uses a
        # SCALE factor derived from the actual canvas size, so the arm
        # fills the available space at any window size.
        self.arm_canvas.pack(fill="both", expand=True)
        self.arm_canvas.bind("<Configure>", lambda _e: self._draw_arm())
        self._setup_arm_canvas_events()

        # Separator between canvas and embedded servo controls
        ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=(10, 6))

        # Servo Control sub-section (no nested LabelFrame, just a label)
        inner = tk.Frame(panel, bg=PANEL_BG)
        inner.pack(fill="x")
        tk.Label(inner, text="\u0627\u0644\u062a\u062d\u0643\u0645 \u0628\u0627\u0644\u0633\u064a\u0631\u0641\u0648", font=("Segoe UI", 11, "bold"),
                 fg=ACCENT, bg=PANEL_BG).pack(anchor="w", padx=(2, 0), pady=(0, 4))

        # Column header: Actual | Target (slider / entry)
        hdr = tk.Frame(inner, bg=PANEL_BG)
        hdr.pack(fill="x", pady=(0, 2))
        tk.Label(hdr, text="", bg=PANEL_BG, width=14).pack(side="left")
        tk.Label(hdr, text="\u0627\u0644\u062d\u0627\u0644\u064a", font=("Segoe UI", 9, "bold"),
                 fg=GREEN, bg=PANEL_BG, width=6).pack(side="left", padx=(2, 6))
        tk.Label(hdr, text="\u0627\u0644\u0647\u062f\u0641 (\u0634\u0631\u064a\u0637 /\u062d\u0642\u0644)",
                 font=("Segoe UI", 9, "bold"), fg=SUBTEXT, bg=PANEL_BG).pack(side="left", padx=(4, 0))

        for i in range(self.SERVO_COUNT):
            self._build_servo_row(inner, i)

        # Reset All button (sends all 5 servos to 90° home)
        reset_row = tk.Frame(inner, bg=PANEL_BG); reset_row.pack(fill="x", pady=(8, 0))
        self._reset_all_btn = tk.Button(
            reset_row, text="\u21BA \u0625\u0639\u0627\u062f\u0629 \u062a\u0639\u064a\u064a\u0646 \u0627\u0644\u0643\u0644", font=("Segoe UI", 11, "bold"),
            bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT,
            relief="flat", padx=14, pady=6, cursor="hand2",
            command=self._reset_all)
        self._reset_all_btn.pack(side="right")

    def _reset_all(self):
        """Send all 5 servos back to the 90° home position."""
        self._send_batch_and_sync([90] * self.SERVO_COUNT)
        self._log("\u062a\u0645\u062a \u0625\u0639\u0627\u062f\u0629 \u062c\u0645\u064a\u0639 \u0627\u0644\u0633\u064a\u0631\u0641\u0648 \u0625\u0644\u0649 90\u00b0", "info")

    def _arm_render_loop(self):
        try:
            self._draw_arm()
        except Exception as exc:
            # Never let a transient drawing error kill the auto-refresh chain.
            try:
                self._log("\u062e\u0637\u0623 \u0641\u064a \u0631\u0633\u0645 \u0627\u0644\u0645\u062e\u0637\u0637: " + str(exc), "err")
            except Exception:
                pass
        self._arm_after_id = self.root.after(ARM_REFRESH_MS, self._arm_render_loop)

    def _draw_arm(self):
        # Angle convention (chosen so 90 deg = straight up, matching the
        # reference image and the Arduino's home pose):
        #   0   = horizontal right
        #   90  = vertical up
        #   180 = horizontal left
        # For S2 (elbow) and S3 (wrist), 90 means "no bend" -- the segment
        # continues in the current direction.  Smaller values bend one way,
        # larger values bend the other.
        c = self.arm_canvas
        c.delete("all")
        w = max(c.winfo_width(), 280)
        h = max(c.winfo_height(), 320)
        # Cache canvas size for the trail renderer to use
        self._traj_canvas_w = w
        self._traj_canvas_h = h

        # SCALE = how much to enlarge the arm so it fills the available
        # canvas at any window size.  Base design was 280 x 420.
        BASE_W, BASE_H = 280, 420
        SCALE = min(w / BASE_W, h / BASE_H)

        cx      = w / 2
        cy_base = h - 60 * SCALE

        a = [v.get() for v in self.target_vars]
        s0, s1, s2, s3, s4 = a

        # ground line
        c.create_line(0, cy_base + 18 * SCALE, w, cy_base + 18 * SCALE,
                      fill=SUBTEXT, dash=(2, 4))

        # mount block on top of the base (responsive)
        mw = 52 * SCALE
        mh_top = 13 * SCALE
        mh_bot = 26 * SCALE
        c.create_rectangle(cx - mw, cy_base - mh_top, cx + mw, cy_base + mh_bot,
                           fill=SUBTEXT, outline="")

        # arm geometry -- cos / sin pair, screen-y inverted (up = -y)
        # Responsive: arm length scales with the canvas size.
        L1, L2, L3 = 137 * SCALE, 83 * SCALE, 42 * SCALE   # L2 shortened by 1/3

        sx, sy = cx, cy_base - mh_top
        s1_rad = math.radians(s1)
        ex     = sx + L1 * math.cos(s1_rad)
        ey     = sy - L1 * math.sin(s1_rad)

        fdir_rad = math.radians(s1 + (s2 - 90))
        fx       = ex + L2 * math.cos(fdir_rad)
        fy       = ey - L2 * math.sin(fdir_rad)

        wdir_rad = math.radians(s1 + (s2 - 90) + (s3 - 90))
        wx       = fx + L3 * math.cos(wdir_rad)
        wy       = fy - L3 * math.sin(wdir_rad)

        # segments (thicker at the shoulder, tapering to the wrist, responsive)
        seg_w = lambda w0: max(2, int(w0 * SCALE))
        c.create_line(sx, sy, ex, ey, fill=ACCENT, width=seg_w(20), capstyle=tk.ROUND)
        c.create_line(ex, ey, fx, fy, fill=GREEN,  width=seg_w(17), capstyle=tk.ROUND)
        c.create_line(fx, fy, wx, wy, fill=YELLOW, width=seg_w(12), capstyle=tk.ROUND)

        # joints (responsive, hollow rings)
        jr = lambda r0: max(3, int(r0 * SCALE))
        c.create_oval(sx - jr(13), sy - jr(13), sx + jr(13), sy + jr(13),
                      fill=TERMINAL_BG, outline=ACCENT, width=3)
        c.create_oval(ex - jr(12), ey - jr(12), ex + jr(12), ey + jr(12),
                      fill=TERMINAL_BG, outline=GREEN,  width=3)
        c.create_oval(fx - jr(8),  fy - jr(8),  fx + jr(8),  fy + jr(8),
                      fill=TERMINAL_BG, outline=YELLOW, width=3)
        # S3 wrist joint: hollow ring matching the other joint style.
        # Outlined RED to mark the start of the gripper fingers.
        c.create_oval(wx - jr(7), wy - jr(7), wx + jr(7), wy + jr(7),
                      fill=TERMINAL_BG, outline=RED, width=3)

        # gripper (S4: 0 = closed, 90 = wide open with 90° total spread)
        # Each finger rotates out from the wrist direction; at s4=90 the two
        # fingers are at ±45° from forward, so the total spread is 90°
        open_ratio  = max(0.0, min(1.0, s4 / 90.0))
        half_spread = math.radians(45.0) * open_ratio
        grip_len    = 31 * SCALE
        left_angle  = wdir_rad - half_spread
        right_angle = wdir_rad + half_spread
        left_x      = wx + grip_len * math.cos(left_angle)
        left_y      = wy - grip_len * math.sin(left_angle)
        right_x     = wx + grip_len * math.cos(right_angle)
        right_y     = wy - grip_len * math.sin(right_angle)
        c.create_line(wx, wy, left_x,  left_y,  fill=RED, width=seg_w(3), capstyle=tk.ROUND)
        c.create_line(wx, wy, right_x, right_y, fill=RED, width=seg_w(3), capstyle=tk.ROUND)
        # S4 angle label above the gripper tip
        c.create_text(wx, wy - grip_len - 12 * SCALE, text=f"S4 {int(round(s4))}°",
                      fill=RED, font=("Consolas", max(9, int(11 * SCALE)), "bold"))

        # angle labels (top, in servo colours) -- responsive
        label_x = 10
        for i, (name, val) in enumerate(zip(["S0", "S1", "S2", "S3", "S4"], a)):
            color = self.SERVO_COLORS[i]
            c.create_text(label_x, 8, text=name, fill=color,
                          font=("Consolas", max(9, int(12 * SCALE)), "bold"), anchor="nw")
            c.create_text(label_x + 22 * SCALE, 8, text=f"{val:>3}\u00b0", fill=TEXT,
                          font=("Consolas", max(9, int(12 * SCALE)), "bold"), anchor="nw")
            label_x += 60 * SCALE

        if self.connected:
            note = "target angles (live \u00b7 click Query Status to refresh actuals)"
        else:
            note = "target angles \u00b7 not connected"
        # Draw trajectory trail in the background layer
        try:
            self._render_traj_trail(c)
        except Exception:
            pass
        c.create_text(w / 2, h - 8, text=note, fill=SUBTEXT,
                      font=("Segoe UI", max(8, int(10 * SCALE)), "italic"))

        # ----- S0 base pointer is drawn LAST so it sits in front of the
        # arm segments and joint circles (z-order: topmost).
        s0_rad = math.radians(s0)
        s0r = 14 * SCALE
        c.create_oval(cx - s0r, cy_base - (s0r + 13 * SCALE), cx + s0r, cy_base - 1,
                      fill=PANEL_BG, outline="white", width=2)
        ind_len = 75 * SCALE
        s0_anchor_y = cy_base - mh_top
        ix = cx + ind_len * math.cos(s0_rad)
        iy = s0_anchor_y - ind_len * math.sin(s0_rad)
        c.create_line(cx, s0_anchor_y, ix, iy,
                      fill="white", width=max(2, int(3 * SCALE)))
        # S0 angle label that follows the pointer
        label_r = ind_len + 14 * SCALE
        lx = cx + label_r * math.cos(s0_rad)
        ly = s0_anchor_y - label_r * math.sin(s0_rad) + 4 * SCALE
        c.create_text(lx, ly, text=f"S0 {int(round(s0))}°", fill="white",
                      font=("Consolas", max(9, int(12 * SCALE)), "bold"))

        # Cache joint positions for the drag handler.  Format:
        #   (canvas_x, canvas_y, servo_id, kind, anchor_x, anchor_y, ...)
        # kind = "polar" for atan2-based drags (S0, S1, S2, S3)
        # kind = "perp"  for perpendicular-distance drag (S4)
        # The anchor for S0/S1 is the base (sx, sy); for S2 it's (ex, ey);
        # for S3 it's (fx, fy); for S4 we store the forward unit vector.
        # Per-joint hit radius (7th tuple element) scales with SCALE so the
        # drag handle always covers the visible joint / gripper area.
        # S4's hit radius is intentionally large to cover the entire gripper
        # (the user drags the wrist / fingers to open the claw).
        s0_hit_r  = max(22, int(28 * SCALE))
        s1_hit_r  = max(18, int(22 * SCALE))
        s2_hit_r  = max(14, int(18 * SCALE))
        s3_hit_r  = max(12, int(16 * SCALE))
        s4_hit_r  = max(34, int(46 * SCALE))
        self.arm_joints = [
            (int(ix), int(iy), 0, "polar", int(sx), int(sy), s0_hit_r),  # S0 pointer tip
            (int(ex), int(ey), 1, "polar", int(sx), int(sy), s1_hit_r),  # S1 shoulder
            (int(fx), int(fy), 2, "polar", int(ex), int(ey), s2_hit_r),  # S2 elbow
            (int(wx), int(wy), 3, "polar", int(fx), int(fy), s3_hit_r),  # S3 wrist
            (int(wx), int(wy), 4, "perp",  math.cos(wdir_rad), -math.sin(wdir_rad), s4_hit_r),  # S4 gripper
        ]

    # ------------------------------------------------------------------
    # Trajectory trail rendering
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # Trajectory trail rendering
    # ------------------------------------------------------------------
    def _fk_tip(self, s0, s1, s2, s3, s4, cx, cy_base, w=0, h=0):
        """Forward kinematics returning (gripper tip wx, wy) for the
        given servo angles, using the same geometry as _draw_arm.

        cy_base here is the *uncorrected* base line (h - 60) and the
        SCALE is derived from the canvas size so the trail aligns with
        the arm.  If w/h are 0 we fall back to the legacy 1.0 scale
        (preserves compatibility for any external caller that does not
        pass canvas dimensions).
        """
        if w <= 0 or h <= 0:
            s = 1.0
        else:
            s = min(w / 280, h / 420)
        L1 = 137 * s
        L2 = 83 * s
        L3 = 42  * s
        sx, sy = cx, cy_base - 13 * s
        s1_rad = math.radians(s1)
        ex = sx + L1 * math.cos(s1_rad)
        ey = sy - L1 * math.sin(s1_rad)
        fdir_rad = math.radians(s1 + (s2 - 90))
        fx = ex + L2 * math.cos(fdir_rad)
        fy = ey - L2 * math.sin(fdir_rad)
        wdir_rad = math.radians(s1 + (s2 - 90) + (s3 - 90))
        wx = fx + L3 * math.cos(wdir_rad)
        wy = fy - L3 * math.sin(wdir_rad)
        return (wx, wy)

    def _render_traj_trail(self, c):
        """Render the recorded / playing trajectory on the arm canvas.

        During recording: draw the path the gripper tip has already taken
        in the current recording session.
        During playback:  draw the full planned path in faint gray, plus
        a bright playhead dot at the current frame.
        When idle:        nothing.
        """
        if not self.trajectory and not (self.recording or self.playing):
            return
        if not (self.recording or self.playing):
            return
        cw = getattr(self, "_traj_canvas_w", 0)
        ch = getattr(self, "_traj_canvas_h", 320)
        cx = cw / 2
        cy_base = ch - 60
        if cx <= 0:
            return
        # Cap how many frames we draw so performance stays smooth
        frames = self.trajectory
        if len(frames) > self.TRAIL_MAX_POINTS:
            step = max(1, len(frames) // self.TRAIL_MAX_POINTS)
            frames = frames[::step]
        pts = []
        for f in frames:
            try:
                wx, wy = self._fk_tip(f[0], f[1], f[2], f[3], f[4], cx, cy_base, cw, ch)
                pts.append((wx, wy))
            except Exception:
                continue
        if len(pts) < 2:
            return
        # Polyline
        for i in range(len(pts) - 1):
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            c.create_line(x1, y1, x2, y2, fill=SUBTEXT, width=1, dash=(2, 3))
        # Playhead at the currently-playing frame
        if self.playing and 0 <= self._play_index < len(self.trajectory):
            cur = self.trajectory[self._play_index]
            try:
                wx, wy = self._fk_tip(cur[0], cur[1], cur[2], cur[3], cur[4], cx, cy_base, cw, ch)
                c.create_oval(wx - 6, wy - 6, wx + 6, wy + 6,
                              outline=MAUVE, fill=MAUVE, width=1)
                c.create_oval(wx - 10, wy - 10, wx + 10, wy + 10,
                              outline=MAUVE, width=1)
            except Exception:
                pass


    # ------------------------------------------------------------------
    # Drag-to-control: let the user grab a joint on the arm diagram and
    # drag it to drive the corresponding servo.
    #
    # Conventions (must match _draw_arm):
    #   0   = horizontal right
    #   90  = vertical up
    #   180 = horizontal left
    # For S2 (elbow) and S3 (wrist), 90 means "no bend".
    # ------------------------------------------------------------------
    def _setup_arm_canvas_events(self):
        """Bind mouse events on the arm canvas for joint dragging."""
        c = self.arm_canvas
        c.bind("<ButtonPress-1>",   self._on_arm_press)
        c.bind("<B1-Motion>",       self._on_arm_drag)
        c.bind("<ButtonRelease-1>", self._on_arm_release)
        c.bind("<Motion>",          self._on_arm_hover)

    def _find_joint_under(self, x, y, radius=24):
        """Return (idx, servo_id) for the closest joint whose hit-radius
        contains (x, y).  arm_joints tuples are 6- or 7-element:
            (jx, jy, sid, kind, anchor_x, anchor_y, [hit_radius])
        Per-joint hit-radius (index 6) lets the gripper have a larger grab
        area than the small round joints.  We return the first joint that
        is within ITS radius; the caller is responsible for ordering.
        """
        for i, joint in enumerate(getattr(self, "arm_joints", [])):
            jx, jy, sid = joint[0], joint[1], joint[2]
            jr = joint[6] if len(joint) > 6 else radius
            dx, dy = jx - x, jy - y
            if dx * dx + dy * dy <= jr * jr:
                return (i, sid)
        return (None, None)

    def _on_arm_press(self, event):
        """Begin dragging the joint under the cursor, if any."""
        j_idx, sid = self._find_joint_under(event.x, event.y)
        if j_idx is None:
            return
        self._dragging_joint = j_idx
        try:
            self.arm_canvas.configure(cursor="hand1")
        except Exception:
            pass

    def _on_arm_drag(self, event):
        """While dragging, recompute the servo angle from the cursor.

        arm_joints tuples:
            S0/S1/S2/S3 (polar): (jx, jy, sid, "polar", anchor_x, anchor_y)
            S4         (perp):   (jx, jy, sid, "perp",  fwd_x,    fwd_y)
        For polar joints we use atan2(anchor_y - y, x - anchor_x) directly,
        except S2/S3 which need an offset because s2/s3 are *bend* angles
        relative to the upstream segment.
        For S4 we project the cursor onto the wrist-forward axis and use
        the perpendicular distance to set the open angle.
        """
        j_idx = getattr(self, "_dragging_joint", None)
        if j_idx is None:
            return
        joints = getattr(self, "arm_joints", [])
        if not joints or j_idx >= len(joints):
            return
        try:
            a = [v.get() for v in self.target_vars]
        except Exception:
            return
        s0, s1, s2, s3, s4 = a

        # Canvas geometry (must match _draw_arm's SCALE formula)
        try:
            c = self.arm_canvas
            w = max(c.winfo_width(), 280)
            h = max(c.winfo_height(), 320)
            BASE_W, BASE_H = 280, 420
            SCALE = min(w / BASE_W, h / BASE_H)
            cx      = w / 2
            cy_base = h - 60 * SCALE
            mh_top  = 13 * SCALE
            grip_len = 31 * SCALE
        except Exception:
            return
        sx, sy = cx, cy_base - mh_top
        L1 = 137 * SCALE
        s1_rad = math.radians(s1)
        ex = sx + L1 * math.cos(s1_rad)
        ey = sy - L1 * math.sin(s1_rad)

        joint = joints[j_idx]
        jx, jy, sid = joint[0], joint[1], joint[2]
        kind = joint[3] if len(joint) > 3 else "polar"
        anchor_x = joint[4] if len(joint) > 4 else jx
        anchor_y = joint[5] if len(joint) > 5 else jy

        x, y = event.x, event.y
        if kind == "perp":
            # S4 gripper: perpendicular distance from wrist-forward axis.
            fwd_x, fwd_y = anchor_x, anchor_y
            dx, dy = x - jx, y - jy
            perp = dx * (-fwd_y) + dy * fwd_x
            # At s4=90 the finger tips sit at grip_len * sin(45deg) from the
            # forward axis; map that perpendicular distance to angle 90.
            max_perp = grip_len * math.sin(math.radians(45.0))
            if max_perp < 1:
                max_perp = 1.0
            new_a = abs(perp) / max_perp * 90.0
        else:
            # Polar drag: atan2 from the cached anchor point.
            new_a = math.degrees(math.atan2(anchor_y - y, x - anchor_x))
            if sid == 2:
                # s2 = 90 + (angle from elbow to cursor) - s1
                new_a = 90 + new_a - s1
            elif sid == 3:
                # s3 = 180 + (angle from forearm-tip to cursor) - s1 - s2
                new_a = 180 + new_a - s1 - s2

        # Normalize and clamp
        new_a = float(new_a)
        if kind != "perp":
            new_a = new_a % 360.0
            if new_a > 180.0:
                new_a -= 360.0
        limit = self.SERVO_LIMITS[sid]
        lo, hi = 0, limit
        new_a = max(lo, min(hi, new_a))
        new_int = int(round(new_a))
        # Skip if the angle did not change -- avoid spamming serial sends
        # while the cursor hovers over the same position.
        try:
            if self.target_vars[sid].get() == new_int:
                return
        except Exception:
            pass
        try:
            self.target_vars[sid].set(new_int)
            self.sliders[sid].set(new_int)
        except Exception:
            return
        # Route through the same path the slider widget uses so that
        # auto-send fires and the actual-angle label updates.  Setting
        # an IntVar programmatically does NOT trigger the slider's
        # <Command> callback, so we have to call this by hand.
        try:
            self._on_slider_change(sid)
        except Exception:
            pass

    def _on_arm_release(self, _event):
        """Stop dragging and reset the cursor."""
        self._dragging_joint = None
        try:
            self.arm_canvas.configure(cursor="")
        except Exception:
            pass

    def _on_arm_hover(self, event):
        """Show a grab cursor when hovering over a draggable joint."""
        if getattr(self, "_dragging_joint", None) is not None:
            return
        j_idx, _ = self._find_joint_under(event.x, event.y)
        try:
            self.arm_canvas.configure(cursor="hand1" if j_idx is not None else "")
        except Exception:
            pass


    # ------------------------------------------------------------------
    # Trajectory panel + recording / playback
    # ------------------------------------------------------------------
    def _build_traj_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="  \u0627\u0644\u0645\u0633\u0627\u0631  ", padding=(14, 10))
        panel.pack(fill="both", expand=True, pady=(10, 0))

        # Status line
        tk.Label(panel, textvariable=self.traj_status,
                 font=("Consolas", 10, "bold"), fg=SUBTEXT, bg=PANEL_BG,
                 anchor="w").pack(fill="x")

        # Row 1: Record / Play / Stop / Clear
        row1 = tk.Frame(panel, bg=PANEL_BG); row1.pack(fill="x", pady=(8, 0))
        self.traj_record_btn = tk.Button(
            row1, text="\u25CF \u062a\u0633\u062c\u064a\u0644", font=("Segoe UI", 11, "bold"),
            bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT,
            relief="flat", padx=14, pady=6, cursor="hand2",
            command=self._toggle_record)
        self.traj_record_btn.pack(side="left")
        self.traj_play_btn = tk.Button(
            row1, text="\u25B6 \u062a\u0634\u063a\u064a\u0644", font=("Segoe UI", 11, "bold"),
            bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT,
            relief="flat", padx=14, pady=6, cursor="hand2",
            command=self._toggle_play)
        self.traj_play_btn.pack(side="left", padx=(6, 0))
        tk.Button(row1, text="\u23F9 \u0625\u064a\u0642\u0627\u0641", font=("Segoe UI", 11, "bold"),
                  bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT,
                  relief="flat", padx=14, pady=6, cursor="hand2",
                  command=self._stop_play).pack(side="left", padx=(6, 0))
        tk.Button(row1, text="\u0645\u0633\u062d", font=("Segoe UI", 11, "bold"),
                  bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT,
                  relief="flat", padx=14, pady=6, cursor="hand2",
                  command=self._clear_trajectory).pack(side="left", padx=(6, 0))

        # Row 2: Capture / Save / Load
        # Row 2: Save / Load / Run Example (icons removed for short text)
        row2 = tk.Frame(panel, bg=PANEL_BG); row2.pack(fill="x", pady=(6, 0))
        tk.Button(row2, text="\u062d\u0641\u0638", font=("Segoe UI", 11, "bold"),
                  bg=GREEN, fg="#1e1e2e", activebackground=GREEN,
                  relief="flat", padx=14, pady=6, cursor="hand2",
                  command=self._save_trajectory).pack(side="left", fill="x", expand=True, padx=(0, 3))
        tk.Button(row2, text="\u062a\u062d\u0645\u064a\u0644", font=("Segoe UI", 11, "bold"),
                  bg=GREEN, fg="#1e1e2e", activebackground=GREEN,
                  relief="flat", padx=14, pady=6, cursor="hand2",
                  command=self._load_trajectory).pack(side="left", fill="x", expand=True, padx=3)
        tk.Button(row2, text="\u0645\u062b\u0627\u0644", font=("Segoe UI", 11, "bold"),
                  bg=MAUVE, fg="#1e1e2e", activebackground=MAUVE,
                  relief="flat", padx=14, pady=6, cursor="hand2",
                  command=self._load_example_trajectory).pack(side="left", fill="x", expand=True, padx=(3, 0))

        # Row 3: Speed slider
        row3 = tk.Frame(panel, bg=PANEL_BG); row3.pack(fill="x", pady=(8, 0))
        tk.Label(row3, text="\u0627\u0644\u0633\u0631\u0639\u0629:", font=("Segoe UI", 9),
                 fg=SUBTEXT, bg=PANEL_BG).pack(side="left")
        speed = tk.Scale(row3, from_=SERVO_MIN_INTERVAL_MS, to=5000, orient="horizontal",
                        variable=self.traj_play_speed, resolution=50,
                        length=180, showvalue=True,
                        bg=PANEL_BG, fg=TEXT, troughcolor=INPUT_BG,
                        highlightthickness=0, sliderrelief="flat")
        speed.pack(side="left", padx=(6, 0))
        tk.Label(row3, text="\u0645\u0644/\u0625\u0637\u0627\u0631", font=("Segoe UI", 8),
                 fg=SUBTEXT, bg=PANEL_BG).pack(side="left", padx=(4, 0))

        # Row 4: Loop checkbox
        row4 = tk.Frame(panel, bg=PANEL_BG); row4.pack(fill="x", pady=(4, 0))
        tk.Checkbutton(row4, text="\u062a\u0643\u0631\u0627\u0631", variable=self.traj_loop,
                       font=("Segoe UI", 9), fg=TEXT, bg=PANEL_BG,
                       selectcolor=INPUT_BG, activebackground=PANEL_BG).pack(side="left")

        # Row 5: Frame listbox (stretches to fill remaining vertical space)
        list_frame = tk.Frame(panel, bg=PANEL_BG); list_frame.pack(fill="both", expand=True, pady=(8, 0))
        sb = tk.Scrollbar(list_frame); sb.pack(side="right", fill="y")
        self.traj_list = tk.Listbox(
            list_frame, yscrollcommand=sb.set,
            font=("Consolas", 9), bg=INPUT_BG, fg=TEXT,
            selectbackground=ACCENT, selectforeground="#1e1e2e",
            relief="flat", highlightthickness=1,
            highlightbackground=PANEL_BG, highlightcolor=ACCENT)
        self.traj_list.pack(side="left", fill="both", expand=True)
        sb.config(command=self.traj_list.yview)
        self.traj_list.bind("<<ListboxSelect>>", self._on_traj_list_select)
        self.traj_list.bind("<Double-Button-1>", self._on_traj_list_edit)
        # Click anywhere outside the listbox to clear its selection so
        # the trajectory "controls the servos" mode is released.
        self.root.bind_all("<Button-1>", self._on_traj_list_outside_click, add="+")

    def _toggle_record(self):
        if self.recording:
            self.recording = False
            self.traj_record_btn.configure(text="\u25CF \u062a\u0633\u062c\u064a\u0644", bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT)
            self._update_traj_status()
            self._log(f"recording stopped -- {len(self.trajectory)} frames captured", "info")
        else:
            self.trajectory = []
            self.recording = True
            self.traj_record_btn.configure(text="\u25A0 \u0625\u064a\u0642\u0627\u0641", fg=TEXT)
            self._refresh_traj_list()
            self._update_traj_status()
            self._snapshot(force=True)
            self._log("\u0628\u062f\u0623 \u0627\u0644\u062a\u0633\u062c\u064a\u0644", "info")

    def _toggle_play(self):
        if self.playing:
            self._stop_play()
            return
        if not self.trajectory:
            self._log("\u0644\u0627 \u064a\u0648\u062c\u062f \u0645\u0627 \u064a\u064f\u0634\u063a\u0651\u064e\u0644 \u2014 \u0633\u062c\u0651\u064e\u0644 \u0623\u0648 \u062d\u0645\u0651\u064e\u0644 \u0645\u0633\u0627\u0631\u0627\u064b \u0623\u0648\u0644\u0627\u064b", "warn")
            return
        self.playing = True
        self._suppress_snap = True
        self._play_index = 0
        self.traj_play_btn.configure(text="\u23F9 \u0625\u064a\u0642\u0627\u0641 \u0645\u0624\u0642\u062a", fg=YELLOW)
        self._update_traj_status()
        self._log(f"playback started -- {len(self.trajectory)} frames", "info")
        self._play_tick()

    def _stop_play(self):
        if self._play_after_id is not None:
            try:
                self.root.after_cancel(self._play_after_id)
            except Exception:
                pass
            self._play_after_id = None
        was_playing = self.playing
        self.playing = False
        self._suppress_snap = False
        if self.traj_play_btn is not None:
            self.traj_play_btn.configure(text="\u25B6 \u062a\u0634\u063a\u064a\u0644", bg=ACCENT, fg="#1e1e2e", activebackground=ACCENT)
        self._update_traj_status()
        if was_playing:
            self._log("\u062a\u0648\u0642\u0641 \u0627\u0644\u062a\u0634\u063a\u064a\u0644", "info")

    def _play_tick(self):
        if not self.playing:
            return
        if not self.trajectory:
            self._stop_play()
            return
        if self._play_index >= len(self.trajectory):
            if self.traj_loop.get():
                self._play_index = 0
            else:
                self._stop_play()
                return
        frame = self.trajectory[self._play_index]
        try:
            angles = [int(a) for a in frame]
            for i, a in enumerate(angles):
                self.target_vars[i].set(a)
                try:
                    self.sliders[i].set(a)
                except Exception:
                    pass
        except Exception as exc:
            self._log(f"playback error: {exc}", "err")
            self._stop_play()
            return
        # Actually transmit the frame to the Arduino.  target_vars.set()
        # alone does not fire the slider's <Command>, so the auto-send
        # path is bypassed -- we have to send the batch explicitly.
        try:
            cmd = " ".join(str(a) for a in angles)
            self._send_command(cmd)
        except Exception as exc:
            self._log(f"playback send error: {exc}", "err")
        try:
            self.traj_list.selection_clear(0, "end")
            self.traj_list.selection_set(self._play_index)
            self.traj_list.see(self._play_index)
        except Exception:
            pass
        self._update_traj_status()
        self._play_index += 1
        delay = max(SERVO_MIN_INTERVAL_MS, int(self.traj_play_speed.get()))
        self._play_after_id = self.root.after(delay, self._play_tick)

    def _clear_trajectory(self):
        if self.playing:
            self._stop_play()
        self.trajectory = []
        self._play_index = 0
        self._refresh_traj_list()
        self._update_traj_status()
        self._log("\u062a\u0645 \u0645\u0633\u062d \u0627\u0644\u0645\u0633\u0627\u0631", "info")

    def _snapshot(self, force=False):
        """Append the current 5 angles to the trajectory (debounced)."""
        now = int(time.time() * 1000)
        if not force and (now - self._last_snapshot_ts) < self.SNAPSHOT_DEBOUNCE_MS:
            return
        self._last_snapshot_ts = now
        try:
            angles = [int(v.get()) for v in self.target_vars]
        except Exception:
            return
        if self.trajectory and list(self.trajectory[-1]) == angles:
            return
        self.trajectory.append(angles)
        if len(self.trajectory) > self.TRAIL_MAX_POINTS * 4:
            self.trajectory = self.trajectory[-self.TRAIL_MAX_POINTS * 2:]
        self._refresh_traj_list()
        self._update_traj_status()

    def _save_trajectory(self):
        if not self.trajectory:
            self._log("\u0644\u0627 \u064a\u0648\u062c\u062f \u0645\u0627 \u064a\u064f\u062d\u0641\u064e\u0638 \u2014 \u0627\u0644\u0645\u0633\u0627\u0631 \u0641\u0627\u0631\u063a", "warn")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("Servo trajectory", "*.json"), ("All files", "*.*")],
            title="\u062d\u0641\u0638 \u0627\u0644\u0645\u0633\u0627\u0631")
        if not path:
            return
        try:
            payload = {
                "format": "servo_control.trajectory.v1",
                "version": __version__,
                "frame_count": len(self.trajectory),
                "frames": self.trajectory,
            }
            with open(path, "w", encoding="utf-8") as fp:
                json.dump(payload, fp, indent=2)
            self._log(f"saved {len(self.trajectory)} frames to {path}", "info")
        except Exception as exc:
            self._log(f"save failed: {exc}", "err")

    def _load_trajectory(self):
        path = filedialog.askopenfilename(
            filetypes=[("Servo trajectory", "*.json"), ("All files", "*.*")],
            title="\u062a\u062d\u0645\u064a\u0644 \u0627\u0644\u0645\u0633\u0627\u0631")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fp:
                payload = json.load(fp)
        except Exception as exc:
            self._log(f"load failed: {exc}", "err")
            return
        frames = payload.get("frames") if isinstance(payload, dict) else None
        if not frames or not isinstance(frames, list):
            self._log("\u0627\u0644\u0645\u0644\u0641 \u0644\u064a\u0633 \u0645\u0633\u0627\u0631\u0627\u064b \u0635\u0627\u0644\u062d\u0627\u064b (\u0644\u0627 \u062a\u0648\u062c\u062f \u0645\u0635\u0641\u0648\u0641\u0629 \u0625\u0637\u0627\u0631\u0627\u062a)", "err")
            return
        if self.playing:
            self._stop_play()
        cleaned = []
        for f in frames:
            if not isinstance(f, (list, tuple)) or len(f) != 5:
                continue
            try:
                row = [max(0, min(self.SERVO_LIMITS[i], int(f[i]))) for i in range(5)]
                cleaned.append(row)
            except (ValueError, TypeError):
                continue
        if not cleaned:
            self._log("\u0644\u0627 \u062a\u0648\u062c\u062f \u0625\u0637\u0627\u0631\u0627\u062a \u0635\u0627\u0644\u062d\u0629 \u0641\u064a \u0627\u0644\u0645\u0644\u0641", "err")
            return
        self.trajectory = cleaned
        self._play_index = 0
        self._refresh_traj_list()
        self._update_traj_status()
        self._log(f"loaded {len(cleaned)} frames from {path}", "info")

    def _load_example_trajectory(self):
        """Load the bundled example trajectory (hard-coded) and play it.

        The example is defined entirely in source so the button works
        without any external file on disk.  Update the `frames` list
        below to change the demo motion.
        """
        example = {
            "format": "servo_control.trajectory.v1",
            "version": __version__,
            "frame_count": 12,
            "frames": [
                [ 20,  80,  90,   0, 80],
                [  0,  80, 130,  20, 40],
                [ 20,  80,  90,   0, 80],
                [  0,  80, 130,  20, 40],
                [ 90,  90,  40,  50, 80],
                [120,  90,  60,  90, 40],
                [ 90,  90,  40,  50, 80],
                [120,  90,  60,  90, 40],
                [180,  80, 130,  20, 40],
                [160,  80,  90,   0, 80],
                [180,  80, 130,  20, 40],
                [ 90,  90,  90,  90, 90],
            ],
        }

        if self.playing:
            self._stop_play()
        cleaned = []
        for f in example["frames"]:
            if not isinstance(f, (list, tuple)) or len(f) != 5:
                continue
            try:
                row = [max(0, min(self.SERVO_LIMITS[i], int(f[i]))) for i in range(5)]
                cleaned.append(row)
            except (ValueError, TypeError):
                continue
        if not cleaned:
            self._log("\u0645\u0633\u0627\u0631 \u0627\u0644\u0645\u062b\u0627\u0644 \u0627\u0644\u0645\u062f\u0645\u062c \u0644\u0627 \u064a\u062d\u062a\u0648\u064a \u0639\u0644\u0649 \u0625\u0637\u0627\u0631\u0627\u062a \u0635\u0627\u0644\u062d\u0629", "err")
            return
        self.trajectory = cleaned
        self._play_index = 0
        self._refresh_traj_list()
        self._update_traj_status()
        self._log(f"loaded bundled example -- {len(cleaned)} frames", "info")
        self._toggle_play()

    def _refresh_traj_list(self):
        if self.traj_list is None:
            return
        self.traj_list.delete(0, "end")
        for i, f in enumerate(self.trajectory):
            line = "{:>3}:  {:>3} {:>3} {:>3} {:>3} {:>3}".format(
                i + 1, f[0], f[1], f[2], f[3], f[4])
            self.traj_list.insert("end", line)

    def _on_traj_list_select(self, _event):
        """Click a frame: all 5 servos run to the angles in that row.
        Does nothing during playback so the user does not accidentally
        fight the auto-runner. Works while recording (the user can
        still click old frames to pose the arm), and gracefully no-ops
        on the serial line when not connected.
        """
        if self.playing:
            return
        sel = self.traj_list.curselection()
        if not sel:
            return
        idx = sel[0]
        if idx < 0 or idx >= len(self.trajectory):
            return
        frame = [int(a) for a in self.trajectory[idx]]
        for i, a in enumerate(frame):
            try:
                self.sliders[i].set(a)
            except Exception:
                pass
        # Send the command to actually move the servos (handles
        # "not connected" gracefully and syncs target_vars + canvas).
        self._send_batch_and_sync(frame)
        self._log(
            "jumped to frame " + str(idx + 1) + "/" + str(len(self.trajectory))
            + ": " + " ".join(str(a) for a in frame),
            "info")

    def _on_traj_list_outside_click(self, event):
        """Click anywhere outside the trajectory listbox to clear its
        selection. This releases the "frame controls the servos" mode so
        that moving sliders afterwards is not interpreted as jumping
        between frames.
        """
        if self.traj_list is None or self.playing:
            return
        # Find the click target and walk up its parent chain.  If we
        # land on the listbox itself, its frame, or its scrollbar, the
        # click is "inside" and we keep the selection.
        w = event.widget
        inside = False
        while w is not None:
            if w is self.traj_list or w is self.traj_list.master:
                inside = True
                break
            w = getattr(w, "master", None)
        if inside:
            return
        try:
            self.traj_list.selection_clear(0, "end")
        except Exception:
            pass


    def _on_traj_list_edit(self, event):
        """Double-click a frame to edit its 5 servo angles in-place.

        A small Toplevel with an Entry is positioned over the selected
        row.  Using a Toplevel (instead of an Entry inside the Listbox)
        avoids the right edge clipping the last value.  Pressing Enter
        commits, Escape cancels.  Only edits while NOT playing or
        recording so we never lose live frames.
        """
        if self.playing or self.recording:
            self._log("\u0644\u0627 \u064a\u0645\u0643\u0646 \u062a\u0639\u062f\u064a\u0644 \u0625\u0637\u0627\u0631 \u0623\u062b\u0646\u0627\u0621 \u0627\u0644\u062a\u0634\u063a\u064a\u0644 \u0623\u0648 \u0627\u0644\u062a\u0633\u062c\u064a\u0644", "warn")
            return
        sel = self.traj_list.curselection()
        if not sel:
            return
        idx = sel[0]
        if idx < 0 or idx >= len(self.trajectory):
            return
        bbox = self.traj_list.bbox(idx)
        if not bbox:
            return
        # bbox is (x, y, w, h) inside the Listbox.  Convert to screen
        # coordinates so the Toplevel can sit on top of the row.
        rx, ry, rw, rh = bbox
        lx = self.traj_list.winfo_rootx() + rx
        ly = self.traj_list.winfo_rooty() + ry
        # Make the popup a bit wider than the row so the last number is
        # never clipped, and a touch taller so the Entry text is centred.
        popup_w = rw + 24
        popup_h = rh + 4
        top = tk.Toplevel(self.root)
        top.wm_overrideredirect(True)
        top.geometry(f"{popup_w}x{popup_h}+{lx}+{ly - 2}")
        top.configure(bg=ACCENT)
        top.attributes("-topmost", True)
        original = self.traj_list.get(idx)
        entry = tk.Entry(
            top, font=("Consolas", 9),
            bg=INPUT_BG, fg=TEXT,
            insertbackground=TEXT, relief="flat",
            highlightthickness=0, bd=0)
        entry.insert(0, original)
        entry.select_range(0, tk.END)
        entry.focus_set()
        entry.pack(fill="both", expand=True, padx=2, pady=2)

        def commit(_e=None):
            try:
                # If focus moved to the Toplevel itself (not the Entry),
                # ignore -- the user just clicked the border.
                if _e is not None and getattr(_e, "widget", None) is top:
                    return
                txt = entry.get().strip()
                # Strip a leading "N: " or "N:" prefix if the user kept it
                if ':' in txt:
                    txt = txt.split(':', 1)[1].strip()
                parts = txt.split()
                if len(parts) != 5:
                    raise ValueError("expected 5 angle values")
                new_angles = []
                for i, p in enumerate(parts):
                    a = int(p)
                    a = max(0, min(self.SERVO_LIMITS[i], a))
                    new_angles.append(a)
                self.trajectory[idx] = new_angles
                self._refresh_traj_list()
                self.traj_list.selection_set(idx)
                self.traj_list.see(idx)
                for i, a in enumerate(new_angles):
                    self.target_vars[i].set(a)
                    try:
                        self.sliders[i].set(a)
                    except Exception:
                        pass
                try:
                    self._draw_arm()
                except Exception:
                    pass
                self._log(f"frame {idx+1} edited -> {new_angles}", "info")
            except ValueError as exc:
                self._log(f"edit cancelled: {exc}", "warn")
            finally:
                try:
                    top.destroy()
                except Exception:
                    pass

        def cancel(_e=None):
            try:
                top.destroy()
            except Exception:
                pass

        def on_popup_focus_out(_e=None):
            # Slight delay so commit() on <Return> can run first.
            self.root.after(60, commit)

        entry.bind("<Return>", commit)
        entry.bind("<KP_Enter>", commit)
        entry.bind("<Escape>", cancel)
        top.bind("<FocusOut>", on_popup_focus_out)
        # Ensure clicking outside the popup also commits and closes.
        top.bind("<Button-1>", lambda _e: entry.focus_set())

    def _update_traj_status(self):
        n = len(self.trajectory)
        if self.playing:
            idx = min(self._play_index, n)
            self.traj_status.set(f"\u25B6 Playing  {idx}/{n}")
        elif self.recording:
            self.traj_status.set(f"\u25CF  Recording  ({n} frames)")
        else:
            self.traj_status.set(f"\u25CB  Idle  ({n} frames)")


    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports if ports else ["(no ports)"]
        if ports:
            self.port_combo.current(0)

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            port = self.port_var.get()
            if port and port != "(no ports)":
                try:
                    baud = int(self.baud_var.get())
                except ValueError:
                    baud = 9600
                self._connect(port, baud)

    def _connect(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT, write_timeout=WRITE_TIMEOUT)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self._set_status(False, f"Failed: {e}")
            self._log(f"Connection failed: {e}", "err")
            return

        self.connected = True
        self._reader_alive = True
        self._reader_thread = threading.Thread(target=self._reader_loop, name="serial-reader", daemon=True)
        self._reader_thread.start()

        self._set_status(True, f"Connected {port} @ {baud}")
        self.conn_btn.configure(text="\u25cf  \u0642\u0637\u0639 \u0627\u0644\u0627\u062a\u0635\u0627\u0644")
        self._log(f"Connected to {port} @ {baud}", "ok")

    def _disconnect(self):
        self._reader_alive = False
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False
        self._set_status(False, "\u063a\u064a\u0631 \u0645\u062a\u0635\u0644")
        self.conn_btn.configure(text="\u25cf  \u0627\u062a\u0635\u0627\u0644")
        self._log("\u062a\u0645 \u0642\u0637\u0639 \u0627\u0644\u0627\u062a\u0635\u0627\u0644.", "info")

    def _on_connection_lost(self):
        self.root.after(0, self._handle_connection_lost_ui)

    def _handle_connection_lost_ui(self):
        if self.connected:
            self._log("\u0641\u064f\u0642\u062f \u0627\u0644\u0627\u062a\u0635\u0627\u0644.", "err")
            self._disconnect()

    def _reader_loop(self):
        ser = self.ser
        buf = ""
        try:
            while self._reader_alive and ser and ser.is_open:
                try:
                    chunk = ser.read(256)
                except (serial.SerialException, OSError):
                    break
                if not chunk:
                    continue
                buf += chunk.decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.rstrip("\r").strip()
                    if line:
                        self._rx_queue.put(line)
        finally:
            self._rx_queue.put("__DISCONNECTED__")

    def _send_command(self, cmd):
        if not cmd:
            return
        if not self.connected or self.ser is None:
            self._log(f"[not connected]  would send: {cmd}", "warn")
            return
        payload = (cmd.rstrip() + "\n").encode("utf-8")
        ser = self.ser
        with self._send_lock:
            try:
                ser.write(payload)
                ser.flush()
            except (serial.SerialTimeoutException, serial.SerialException, OSError) as e:
                self._log(f"Send failed: {e}", "err")
                self._on_connection_lost()
                return
        self._log(f"\u2192 {cmd.rstrip()}", "tx")

    def _send_custom(self):
        cmd = self.custom_var.get().strip()
        if not cmd:
            return
        # If the command is a numeric pattern, sync sliders / entries
        # so the GUI matches what the Arduino is being told.
        self._parse_and_sync_sliders(cmd)
        self._send_command(cmd)
        # Auto-clear the input after sending so the next command is ready
        # to type. (v2.2.9+)
        self.custom_var.set("")

    def _poll_rx_queue(self):
        try:
            while True:
                item = self._rx_queue.get_nowait()
                if item == "__DISCONNECTED__":
                    self._handle_connection_lost_ui()
                else:
                    self._on_rx_line(item)
        except queue.Empty:
            pass
        self._poll_after_id = self.root.after(RX_POLL_MS, self._poll_rx_queue)

    def _on_rx_line(self, line):
        self._log(f"\u2190 {line}", "rx")
        upper = line.upper()

        m = self.STATUS_RE.search(line)
        if m:
            angles = [int(m.group(i)) for i in range(1, 6)]
            if "STATUS" in upper or "INIT OK" in upper:
                self.actual_angles = angles
                self._refresh_actual_display()
            elif "BATCH OK TARGET" in upper:
                for i, ang in enumerate(angles):
                    self.target_vars[i].set(ang)
                self._log("  \u21b3 \u062a\u0645 \u062a\u0623\u0643\u064a\u062f \u0627\u0644\u0625\u0637\u0627\u0631 \u0627\u0644\u0647\u062f\u0641", "ok")

        m2 = self.SERVO_TARGET_RE.search(line)
        if m2:
            try:
                idx    = int(m2.group(1))
                target = int(m2.group(2))
                if 0 <= idx < self.SERVO_COUNT:
                    self.target_vars[idx].set(target)
            except Exception:
                pass

    def _refresh_actual_display(self):
        for i, ang in enumerate(self.actual_angles):
            tgt = self.target_vars[i].get()
            marker = "\u2713" if ang == tgt else "\u2192"
            color = GREEN if ang == tgt else YELLOW
            self.actual_labels[i].configure(text=f"{marker} {ang:>3}\u00b0", fg=color)

    def _on_slider_change(self, idx):
        if self.auto_send.get() and self.connected:
            self._schedule_send()
        try:
            tgt = self.target_vars[idx].get()
            cur = self.actual_angles[idx]
            color = GREEN if tgt == cur else YELLOW
            self.actual_labels[idx].configure(text=f"\u2026 {tgt:>3}\u00b0", fg=color)
        except tk.TclError:
            pass
        # Force an immediate repaint so the arm diagram feels live, not 60ms-laggy.
        try:
            self._draw_arm()
        except Exception:
            pass
        # If recording is on, append this angle change to the trajectory.
        # Suppressed during playback so replayed frames do not pollute the recording.
        if self.recording and not self._suppress_snap:
            try:
                self._snapshot()
            except Exception:
                pass

    def _on_entry_change(self, idx):
        try:
            val = int(self.target_vars[idx].get())
        except (tk.TclError, ValueError):
            return
        limit = self.SERVO_LIMITS[idx]
        val = max(0, min(limit, val))
        self.target_vars[idx].set(val)
        self.sliders[idx].set(val)
        if self.connected:
            self._send_command(f"{idx} {val}")
        # Keep the arm diagram in sync with the new value
        try:
            self._draw_arm()
        except Exception:
            pass

    def _reset_servo(self, idx):
        self.target_vars[idx].set(90)
        self.sliders[idx].set(90)
        if self.connected:
            self._send_command(f"{idx} 90")
        try:
            self._draw_arm()
        except Exception:
            pass

    # ---------- live preview & quick-action sync ----------
    def _update_cmd_preview(self, *_args):
        """Refresh the live preview label and Home-button state.
        Called by traces on every target-angle change AND on every
        Custom-Command keystroke.
        """
        try:
            angles = [v.get() for v in self.target_vars]
        except Exception:
            return
        slider_cmd = " ".join(str(a) for a in angles)
        try:
            custom_text = self.custom_var.get().strip()
        except Exception:
            custom_text = ""
        # The preview mirrors whatever the user is about to send:
        #   * if there is text in the Custom-Command box, show that
        #   * otherwise show the current slider state
        if hasattr(self, "_cmd_preview") and self._cmd_preview is not None:
            self._cmd_preview.set(custom_text if custom_text else slider_cmd)
        if self._home_btn is not None:
            try:
                if all(a == 90 for a in angles):
                    self._home_btn.configure(
                        text="\u2713  At Home (all 90\u00b0)",
                        bg=GREEN,
                    )
                else:
                    self._home_btn.configure(
                        text="\u2302  Home (All 90\u00b0)",
                        bg=ACCENT,
                    )
            except Exception:
                pass

    def _send_batch_and_sync(self, angles):
        """Send a batch command AND sync the local sliders / entries /
        target-vars so the UI matches what the Arduino is being told.
        """
        for i, a in enumerate(angles):
            self.target_vars[i].set(a)
        cmd = " ".join(str(a) for a in angles)
        self._send_command(cmd)
        try:
            self._draw_arm()
        except Exception:
            pass


    def _parse_and_sync_sliders(self, cmd):
        """If the command is a numeric pattern (5 / 1 / 2 ints), sync
        sliders/entries to match so the GUI reflects what was sent.
        Keywords like h / help / s / status are ignored.
        """
        parts = cmd.split()
        if not parts:
            return
        # Skip keywords
        if parts[0].lower() in ("h", "help", "s", "status"):
            return
        try:
            nums = [int(p) for p in parts]
        except ValueError:
            return
        def _clamp(i, a):
            return max(0, min(self.SERVO_LIMITS[i], a))
        if len(nums) == 5:
            for i in range(5):
                a = _clamp(i, nums[i])
                self.target_vars[i].set(a)
                self.sliders[i].set(a)
        elif len(nums) == 1:
            for i in range(5):
                a = _clamp(i, nums[0])
                self.target_vars[i].set(a)
                self.sliders[i].set(a)
        elif len(nums) == 2:
            sid, a = nums
            if 0 <= sid < 5:
                a = _clamp(sid, a)
                self.target_vars[sid].set(a)
                self.sliders[sid].set(a)
        try:
            self._draw_arm()
        except Exception:
            pass

    def _quick_action(self, btn, action):
        """Run a quick action and flash the button so the user sees it
        respond.  action in {"home", "status", "help"}.

        Order matters: we run the action FIRST so the button's bg is
        already updated (e.g. to "At Home" green) before we capture
        it as orig_bg, then we flash yellow on top, then restore to
        the new state.  This way the flash is always visible.
        """
        if action == "home":
            self._send_batch_and_sync([90, 90, 90, 90, 90])
        elif action == "status":
            self._send_command("s")
        elif action == "help":
            self._send_command("h")
        # Brief yellow flash on top of whatever state the button is in
        try:
            orig_bg = btn.cget("bg")
            btn.configure(bg=YELLOW, fg="#1e1e2e")
            self.root.after(180, lambda: btn.configure(bg=orig_bg, fg="#1e1e2e"))
        except Exception:
            pass

    def _schedule_send(self):
        if self._send_after_id is not None:
            try:
                self.root.after_cancel(self._send_after_id)
            except Exception:
                pass
        self._send_after_id = self.root.after(DEBOUNCE_MS, self._do_send)

    def _do_send(self):
        self._send_after_id = None
        if not (self.connected and self.ser is not None):
            return
        angles = [v.get() for v in self.target_vars]
        cmd = " ".join(str(a) for a in angles)
        self._send_command(cmd)

    def _set_status(self, connected, text):
        color = GREEN if connected else RED
        self._status_canvas.itemconfigure(self._status_dot, fill=color, outline=color)
        self._status_lbl.configure(text=text)

    def _log(self, msg, level="info"):
        ts = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{ts}] {msg}\n", level)
        if self.autoscroll.get():
            self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _copy_log(self):
        self.log_text.configure(state="normal")
        text = self.log_text.get("1.0", "end-1c")
        self.log_text.configure(state="disabled")
        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    def _on_close(self):
        if self.playing:
            self._stop_play()
        self._disconnect()
        for aid in (self._poll_after_id, self._arm_after_id, self._send_after_id, self._play_after_id):
            if aid is not None:
                try:
                    self.root.after_cancel(aid)
                except Exception:
                    pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = ServoControlApp()
    app.run()
