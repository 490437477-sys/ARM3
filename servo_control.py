# -*- coding: utf-8 -*-
"""
Servo Arm Control System - Python GUI
"""

import threading
import tkinter as tk
from tkinter import ttk
import serial
from serial.tools import list_ports

__version__ = "1.1.0"

SERIAL_TIMEOUT = 0.5
WRITE_TIMEOUT = 0.5


class ServoControlApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Servo Arm Control v1.1")
        self.root.geometry("800x650")
        self.root.configure(bg="#2b2b2b")

        self.ser = None
        self.connected = False
        self.servo_vars = [tk.IntVar(value=90) for _ in range(5)]
        self.auto_send = tk.BooleanVar(value=True)
        self.btn1_var = tk.BooleanVar()
        self.btn2_var = tk.BooleanVar()

        self._send_lock = threading.Lock()
        self._send_after_id = None
        self._send_debounce_ms = 50

        self.create_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_ui(self):
        # Title
        title_frame = tk.Frame(self.root, bg="#2b2b2b")
        title_frame.pack(pady=15)
        tk.Label(title_frame, text="Servo Arm Control System",
                font=("Arial", 22, "bold"), fg="white", bg="#2b2b2b").pack()

        # Main container
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        # ===== LEFT: Servo Controls =====
        servo_frame = tk.LabelFrame(main, text="Servo Angle Control",
                                    font=("Arial", 12, "bold"), fg="white",
                                    bg="#3c3c3c", padx=15, pady=15)
        servo_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        self.servo_sliders = []
        self.servo_entries = []
        self.servo_labels = []
        servo_colors = ["#f38ba8", "#fab387", "#f9e2af", "#a6e3a1", "#89b4fa"]

        for i in range(5):
            row = ttk.Frame(servo_frame)
            row.pack(fill=tk.X, pady=6)

            lbl = tk.Label(row, text=f"Servo {i}:",
                          font=("Arial", 11, "bold"), fg=servo_colors[i],
                          bg="#3c3c3c", width=10, anchor="w")
            lbl.pack(side=tk.LEFT)

            max_val = 90 if i == 4 else 180
            slider = ttk.Scale(row, from_=0, to=max_val, orient=tk.HORIZONTAL,
                            variable=self.servo_vars[i], length=250,
                            command=lambda v, idx=i: self.on_slider_change(idx))
            slider.pack(side=tk.LEFT, padx=10)
            self.servo_sliders.append(slider)

            entry = ttk.Entry(row, textvariable=self.servo_vars[i],
                           width=5, font=("Arial", 11), justify="center")
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind('<Return>', lambda e, idx=i: self.on_entry_change(idx))
            entry.bind('<FocusOut>', lambda e, idx=i: self.on_entry_change(idx))
            self.servo_entries.append(entry)

            val_lbl = tk.Label(row, text="90\xba",
                             font=("Arial", 11, "bold"), fg="#6f6",
                             bg="#3c3c3c", width=6)
            val_lbl.pack(side=tk.LEFT, padx=(5, 0))
            self.servo_labels.append(val_lbl)

        # ===== RIGHT: Control Panel =====
        ctrl_frame = tk.LabelFrame(main, text="Control Panel",
                                   font=("Arial", 12, "bold"), fg="white",
                                   bg="#3c3c3c", padx=15, pady=15)
        ctrl_frame.pack(side=tk.LEFT, fill=tk.BOTH)

        self.conn_lbl = tk.Label(ctrl_frame, text="Disconnected",
                                 font=("Arial", 12, "bold"), fg="#f66",
                                 bg="#3c3c3c")
        self.conn_lbl.pack(pady=(0, 10))

        port_row = ttk.Frame(ctrl_frame)
        port_row.pack(fill=tk.X, pady=5)
        tk.Label(port_row, text="COM Port:", font=("Arial", 10),
                fg="white", bg="#3c3c3c").pack(side=tk.LEFT)

        self.port_combo = ttk.Combobox(port_row, width=10, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()

        ttk.Button(port_row, text="Refresh",
                  command=self.refresh_ports, width=7).pack(side=tk.LEFT)

        self.conn_btn = ttk.Button(ctrl_frame, text="Connect",
                                  command=self.toggle_connection, width=12)
        self.conn_btn.pack(pady=5)

        ttk.Separator(ctrl_frame, orient='horizontal').pack(fill=tk.X, pady=10)

        btn_lbl = tk.Label(ctrl_frame, text="Servo 4 Control:",
                          font=("Arial", 11, "bold"), fg="#cdd6f4", bg="#3c3c3c")
        btn_lbl.pack()

        btn_row = ttk.Frame(ctrl_frame)
        btn_row.pack(pady=8)

        tk.Checkbutton(btn_row, text="+ (Button1)",
                      font=("Arial", 10), fg="#6f6", bg="#3c3c3c",
                      selectcolor="#45475a", variable=self.btn1_var,
                      command=self.on_btn1_press).pack(side=tk.LEFT, padx=5)

        tk.Checkbutton(btn_row, text="- (Button2)",
                      font=("Arial", 10), fg="#f66", bg="#3c3c3c",
                      selectcolor="#45475a", variable=self.btn2_var,
                      command=self.on_btn2_press).pack(side=tk.LEFT, padx=5)

        ttk.Separator(ctrl_frame, orient='horizontal').pack(fill=tk.X, pady=10)

        tk.Checkbutton(ctrl_frame, text="Auto Send",
                      variable=self.auto_send).pack(pady=5)

        preset_lbl = tk.Label(ctrl_frame, text="Presets:",
                             font=("Arial", 10), fg="white", bg="#3c3c3c")
        preset_lbl.pack()

        preset_row = ttk.Frame(ctrl_frame)
        preset_row.pack(pady=5)
        for angle in [0, 45, 90, 135, 180]:
            ttk.Button(preset_row, text=f"{angle}\xba",
                      command=lambda a=angle: self.set_all(a),
                      width=5).pack(side=tk.LEFT, padx=2)

        ttk.Button(ctrl_frame, text="Send",
                  command=self.send_data, width=12).pack(pady=5)
        ttk.Button(ctrl_frame, text="Reset (90\xba)",
                  command=lambda: self.set_all(90), width=12).pack()

        self.status = tk.Text(ctrl_frame, height=8, width=28,
                             font=("Consolas", 9), bg="#1a1a1a", fg="#0f0",
                             relief=tk.FLAT)
        self.status.pack(pady=10)

        self.update_display()

    def on_btn1_press(self):
        if self.btn1_var.get():
            current = self.servo_vars[4].get()
            self.servo_vars[4].set(min(90, current + 2))
            self.btn1_var.set(False)
            self.update_display()

    def on_btn2_press(self):
        if self.btn2_var.get():
            current = self.servo_vars[4].get()
            self.servo_vars[4].set(max(0, current - 2))
            self.btn2_var.set(False)
            self.update_display()

    def on_slider_change(self, index):
        try:
            val = self.servo_vars[index].get()
            self.servo_labels[index].config(text=f"{val}\xba")
            self.update_display()
        except tk.TclError:
            pass

    def on_entry_change(self, index):
        try:
            val = self.servo_vars[index].get()
            max_val = 90 if index == 4 else 180
            val = max(0, min(max_val, val))
            self.servo_vars[index].set(val)
            self.servo_sliders[index].set(val)
            self.servo_labels[index].config(text=f"{val}\xba")
            self.update_display()
        except (tk.TclError, ValueError):
            pass

    def update_display(self):
        angles = [v.get() for v in self.servo_vars]

        self.status.delete(1.0, tk.END)
        self.status.insert(tk.END, "Servo Status:\n\n")
        for i, a in enumerate(angles):
            self.status.insert(tk.END, f"  S{i}: {a}\xba\n")
        self.status.insert(tk.END, f"\n{'Connected' if self.connected else 'Disconnected'}")

        if self.auto_send.get() and self.connected:
            self._schedule_send()

    def refresh_ports(self):
        ports = list(list_ports.comports())
        self.port_combo['values'] = [p.device for p in ports] if ports else ['No Port']
        if ports:
            self.port_combo.current(0)

    def toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            port = self.port_combo.get()
            if port and port != 'No Port':
                self._connect(port)

    def _connect(self, port):
        try:
            self.ser = serial.Serial(
                port, 9600,
                timeout=SERIAL_TIMEOUT,
                write_timeout=WRITE_TIMEOUT,
            )
            self.connected = True
            self.conn_lbl.config(text=f"Connected {port}", fg="#6f6")
            self.conn_btn.config(text="Disconnect")
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.connected = False
            self.conn_lbl.config(text=f"Failed: {e}", fg="#f66")
        self.update_display()

    def _disconnect(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False
        self.conn_lbl.config(text="Disconnected", fg="#f66")
        self.conn_btn.config(text="Connect")
        self.update_display()

    def _on_connection_lost(self):
        # Called from worker thread; bounce back to UI thread.
        self.root.after(0, self._disconnect)

    def _schedule_send(self):
        # Debounce: collapse a burst of UI events into a single send
        # 50ms after the last one. Prevents flooding the Arduino's
        # 64-byte serial buffer when a slider is dragged.
        if self._send_after_id is not None:
            try:
                self.root.after_cancel(self._send_after_id)
            except Exception:
                pass
        self._send_after_id = self.root.after(
            self._send_debounce_ms, self._do_send)

    def _do_send(self):
        self._send_after_id = None
        self._send_async()

    def _send_async(self):
        if not self.connected or self.ser is None:
            return
        ser = self.ser
        # Snapshot all 5 angles under the lock so the frame is
        # internally consistent (no mixing across UI events).
        with self._send_lock:
            angles = [v.get() for v in self.servo_vars]
        payload = f"{angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]}\n".encode()

        def worker():
            with self._send_lock:
                try:
                    ser.write(payload)
                except (serial.SerialTimeoutException, serial.SerialException, OSError):
                    self._on_connection_lost()

        threading.Thread(target=worker, daemon=True).start()

    def send_data(self):
        self._send_async()

    def set_all(self, angle):
        for i, v in enumerate(self.servo_vars):
            max_val = 90 if i == 4 else 180
            v.set(min(angle, max_val))
        for s in self.servo_sliders:
            s.set(angle)
        self.update_display()

    def on_close(self):
        self._disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = ServoControlApp()
    app.run()
