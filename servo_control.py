# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import ttk
import serial
from serial.tools import list_ports

class ServoControlApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Servo Control System")
        self.root.geometry("950x720")
        self.root.configure(bg="#2b2b2b")
        
        self.ser = None
        self.servo_vars = [tk.IntVar(value=90) for _ in range(5)]
        self.auto_send = tk.BooleanVar(value=True)
        
        self.joy1_x = 512
        self.joy1_y = 512
        self.joy2_x = 512
        self.joy2_y = 512
        
        self.create_ui()
        
    def create_ui(self):
        tk.Label(self.root, text="Servo Control System", font=("Arial", 20, "bold"),
                fg="white", bg="#2b2b2b").pack(pady=15)
        
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Left: Servo Control
        servo_frame = tk.LabelFrame(main, text="Servo Angle Control (0-180)", 
                                    font=("Arial", 12), fg="white",
                                    bg="#3c3c3c", padx=10, pady=10)
        servo_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        self.servo_sliders = []
        self.servo_entries = []
        self.servo_value_labels = []
        
        for i in range(5):
            row = ttk.Frame(servo_frame)
            row.pack(fill=tk.X, pady=5)
            
            tk.Label(row, text=f"Servo {i}:", font=("Arial", 11), 
                    fg="#6af", bg="#3c3c3c", width=8).pack(side=tk.LEFT)
            
            slider = ttk.Scale(row, from_=0, to=180, orient=tk.HORIZONTAL,
                             variable=self.servo_vars[i], length=180,
                             command=lambda v, idx=i: self.on_slider_change(idx))
            slider.pack(side=tk.LEFT, padx=5)
            self.servo_sliders.append(slider)
            
            entry = ttk.Entry(row, textvariable=self.servo_vars[i], 
                            width=5, font=("Arial", 11), justify="center")
            entry.pack(side=tk.LEFT, padx=5)
            entry.bind('<Return>', lambda e, idx=i: self.on_entry_change(idx))
            entry.bind('<FocusOut>', lambda e, idx=i: self.on_entry_change(idx))
            self.servo_entries.append(entry)
            
            lbl = tk.Label(row, text="90°", font=("Arial", 11, "bold"),
                          fg="#6f6", bg="#3c3c3c", width=6)
            lbl.pack(side=tk.LEFT, padx=5)
            self.servo_value_labels.append(lbl)
        
        # Middle: Virtual Joystick
        joystick_frame = tk.LabelFrame(main, text="Virtual Joystick", font=("Arial", 12),
                                      fg="white", bg="#3c3c3c", padx=10, pady=10)
        joystick_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 10))
        
        self.setup_joystick(joystick_frame, "Joystick 1", 0)
        self.setup_joystick(joystick_frame, "Joystick 2", 1)
        
        # Right: Control Panel
        ctrl_frame = tk.LabelFrame(main, text="Control Panel", font=("Arial", 12),
                                   fg="white", bg="#3c3c3c", padx=10, pady=10)
        ctrl_frame.pack(side=tk.LEFT, fill=tk.BOTH)
        
        # Connection Status
        conn_frame = tk.Frame(ctrl_frame, bg="#3c3c3c")
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.conn_status_lbl = tk.Label(conn_frame, text="● Disconnected", 
                                        font=("Arial", 12, "bold"),
                                        fg="#f66", bg="#3c3c3c")
        self.conn_status_lbl.pack()
        
        # Serial Port
        port_row = ttk.Frame(ctrl_frame)
        port_row.pack(fill=tk.X, pady=5)
        
        tk.Label(port_row, text="Port:", font=("Arial", 10), 
                fg="white", bg="#3c3c3c").pack(side=tk.LEFT)
        
        self.port_combo = ttk.Combobox(port_row, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()
        
        ttk.Button(port_row, text="Refresh", command=self.refresh_ports,
                  width=6).pack(side=tk.LEFT, padx=2)
        
        self.conn_btn = ttk.Button(port_row, text="Connect", 
                                   command=self.toggle_connection, width=6)
        self.conn_btn.pack(side=tk.LEFT)
        
        # Button Control
        btn_frame = ttk.Frame(ctrl_frame)
        btn_frame.pack(pady=10)
        
        tk.Label(btn_frame, text="Servo4 Control:", font=("Arial", 11, "bold"),
                fg="#cdd6f4", bg="#3c3c3c").pack()
        
        btn_row = ttk.Frame(btn_frame)
        btn_row.pack(pady=5)
        
        self.btn1_var = tk.BooleanVar()
        self.btn2_var = tk.BooleanVar()
        
        btn1 = tk.Checkbutton(btn_row, text="Button1 (+)", font=("Arial", 10),
                             fg="#6f6", bg="#3c3c3c", selectcolor="#45475a",
                             variable=self.btn1_var, command=self.on_btn1_press)
        btn1.pack(side=tk.LEFT, padx=5)
        
        btn2 = tk.Checkbutton(btn_row, text="Button2 (-)", font=("Arial", 10),
                             fg="#f66", bg="#3c3c3c", selectcolor="#45475a",
                             variable=self.btn2_var, command=self.on_btn2_press)
        btn2.pack(side=tk.LEFT, padx=5)
        
        # Auto Send
        ttk.Checkbutton(ctrl_frame, text="Auto Send Data", 
                       variable=self.auto_send).pack(pady=5)
        
        # Preset
        preset_frame = ttk.Frame(ctrl_frame)
        preset_frame.pack(pady=10)
        
        tk.Label(preset_frame, text="Preset:", font=("Arial", 10),
                fg="white", bg="#3c3c3c").pack()
        
        preset_btn_frame = ttk.Frame(preset_frame)
        preset_btn_frame.pack(pady=5)
        
        for angle in [0, 45, 90, 135, 180]:
            ttk.Button(preset_btn_frame, text=f"{angle}°",
                      command=lambda a=angle: self.set_all(a),
                      width=6).pack(side=tk.LEFT, padx=2)
        
        # Send Button
        ttk.Button(ctrl_frame, text="Send Data", 
                  command=self.send_data, width=15).pack(pady=5)
        
        ttk.Button(ctrl_frame, text="Reset 90°", 
                  command=lambda: self.set_all(90), width=15).pack()
        
        # Status Display
        self.status = tk.Text(ctrl_frame, height=6, width=25,
                             font=("Courier", 9), bg="#1a1a1a", fg="#0f0")
        self.status.pack(pady=10)
        
        self.update_display()
        
    def setup_joystick(self, parent, name, index):
        frame = ttk.Frame(parent)
        frame.pack(pady=10)
        
        tk.Label(frame, text=name, font=("Arial", 11, "bold"),
                fg="white", bg="#3c3c3c").pack()
        
        canvas = tk.Canvas(frame, width=150, height=150, bg="#45475a",
                          highlightthickness=0)
        canvas.pack()
        
        canvas.create_oval(20, 20, 130, 130, outline="#6c7086", width=2)
        stick = canvas.create_oval(60, 60, 90, 90, fill="#89b4fa")
        
        if index == 0:
            self.joy1_canvas = canvas
            self.joy1_stick = stick
        else:
            self.joy2_canvas = canvas
            self.joy2_stick = stick
        
        val_frame = ttk.Frame(frame)
        val_frame.pack()
        
        tk.Label(val_frame, text="X:", font=("Consolas", 10), fg="#f9e2af",
                bg="#3c3c3c").grid(row=0, column=0)
        x_lbl = tk.Label(val_frame, text="512", font=("Consolas", 10), fg="white",
                        bg="#3c3c3c", width=5)
        x_lbl.grid(row=0, column=1)
        
        tk.Label(val_frame, text=" Y:", font=("Consolas", 10), fg="#f9e2af",
                bg="#3c3c3c").grid(row=0, column=2)
        y_lbl = tk.Label(val_frame, text="512", font=("Consolas", 10), fg="white",
                        bg="#3c3c3c", width=5)
        y_lbl.grid(row=0, column=3)
        
        if index == 0:
            self.joy1_x_lbl = x_lbl
            self.joy1_y_lbl = y_lbl
        else:
            self.joy2_x_lbl = x_lbl
            self.joy2_y_lbl = y_lbl
        
        canvas.bind("<B1-Motion>", lambda e, idx=index: self.on_joystick_drag(e, idx))
        canvas.bind("<ButtonRelease-1>", lambda e, idx=index: self.on_joystick_release(e, idx))
    
    def on_joystick_drag(self, event, index):
        canvas = self.joy1_canvas if index == 0 else self.joy2_canvas
        stick = self.joy1_stick if index == 0 else self.joy2_stick
        
        center_x, center_y = 75, 75
        dx = event.x - center_x
        dy = event.y - center_y
        
        distance = (dx**2 + dy**2)**0.5
        if distance > 50:
            dx = dx / distance * 50
            dy = dy / distance * 50
        
        canvas.coords(stick, center_x + dx - 15, center_y + dy - 15,
                     center_x + dx + 15, center_y + dy + 15)
        
        x_val = int(512 + dx / 50 * 512)
        y_val = int(512 - dy / 50 * 512)
        
        if index == 0:
            self.joy1_x = max(0, min(1023, x_val))
            self.joy1_y = max(0, min(1023, y_val))
            self.joy1_x_lbl.config(text=str(self.joy1_x))
            self.joy1_y_lbl.config(text=str(self.joy1_y))
        else:
            self.joy2_x = max(0, min(1023, x_val))
            self.joy2_y = max(0, min(1023, y_val))
            self.joy2_x_lbl.config(text=str(self.joy2_x))
            self.joy2_y_lbl.config(text=str(self.joy2_y))
    
    def on_joystick_release(self, event, index):
        canvas = self.joy1_canvas if index == 0 else self.joy2_canvas
        stick = self.joy1_stick if index == 0 else self.joy2_stick
        
        canvas.coords(stick, 60, 60, 90, 90)
        
        if index == 0:
            self.joy1_x = 512
            self.joy1_y = 512
            self.joy1_x_lbl.config(text="512")
            self.joy1_y_lbl.config(text="512")
        else:
            self.joy2_x = 512
            self.joy2_y = 512
            self.joy2_x_lbl.config(text="512")
            self.joy2_y_lbl.config(text="512")
    
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
    
    def on_slider_change(self, idx):
        try:
            val = self.servo_vars[idx].get()
            self.servo_value_labels[idx].config(text=f"{val}°")
            self.update_display()
        except:
            pass
    
    def on_entry_change(self, idx):
        try:
            val = self.servo_vars[idx].get()
            val = max(0, min(180, val))
            self.servo_vars[idx].set(val)
            self.servo_sliders[idx].set(val)
            self.servo_value_labels[idx].config(text=f"{val}°")
            self.update_display()
        except:
            pass
    
    def update_display(self):
        angles = [v.get() for v in self.servo_vars]
        self.status.delete("1.0", tk.END)
        self.status.insert(tk.END, f"Servo Angles:\n")
        for i, a in enumerate(angles):
            self.status.insert(tk.END, f"  Servo{i}: {a}°\n")
        
        if self.ser and self.ser.is_open:
            self.status.insert(tk.END, f"\nConnected: {self.ser.port}")
        else:
            self.status.insert(tk.END, f"\nDisconnected")
        
        if self.auto_send.get() and self.ser and self.ser.is_open:
            self.send_data()
    
    def refresh_ports(self):
        ports = list(list_ports.comports())
        self.port_combo['values'] = [p.device for p in ports] if ports else ['No port']
        if ports:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.conn_status_lbl.config(text="● Disconnected", fg="#f66")
            self.conn_btn.config(text="Connect")
        else:
            port = self.port_combo.get()
            if port and port != 'No port':
                try:
                    self.ser = serial.Serial(port, 9600, timeout=1)
                    self.conn_status_lbl.config(text=f"● Connected {port}", fg="#6f6")
                    self.conn_btn.config(text="Disconnect")
                    self.update_display()
                except:
                    self.conn_status_lbl.config(text="● Failed", fg="#f66")
    
    def send_data(self):
        if self.ser and self.ser.is_open:
            try:
                angles = [v.get() for v in self.servo_vars]
                data = f"{angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]}\n"
                self.ser.write(data.encode())
            except:
                pass
    
    def set_all(self, angle):
        for v in self.servo_vars:
            v.set(angle)
        for s in self.servo_sliders:
            s.set(angle)
        self.update_display()
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = ServoControlApp()
    app.run()
