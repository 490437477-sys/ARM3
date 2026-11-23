#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ARM4 — настольный пульт управления (порт / tkinter)

Прошивка: servo_control_ru.ino (Arduino Uno / Nano, 9600 8N1)

Протокол команд прошивки
    90                  все пять серво в 90
    0 60                задать серво 0 в 60
    0 90 90 90 40       задать все пять серво одной строкой
    s                   запрос текущих углов, ответ: [Текущие углы S0..S4]: 90,90,90,90,90
    h                   вывести справку

Внимание: прошивка принимает только цифры / пробелы / h / s, прочие символы отбрасываются,
      а главный цикл читает порт раз в 40 мс (буфер приёма 64 байта),
      поэтому отправка идёт через очередь: по каждому серво хранится последнее значение, минимум 45 мс.

Зависимости: pyserial (pip install pyserial), tkinter входит в стандартную библиотеку Python.
"""

import json
import math
import queue
import re
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
    HAS_PYSERIAL = True
except Exception:                                                   # noqa: BLE001
    serial = None
    list_ports = None
    HAS_PYSERIAL = False

import tkinter as tk
from tkinter import filedialog, simpledialog, ttk

# ===================== Константы =====================
LIMITS = [180, 180, 180, 180, 90]          # совпадает с MAX_ANGLE_NORMAL / MAX_ANGLE_GRAB прошивки
NAMES = ["S0 база", "S1 плечо", "S2 локоть", "S3 кисть", "S4 захват"]
COLORS = ["#f38ba8", "#fab387", "#f9e2af", "#a6e3a1", "#89b4fa"]

BG = "#1e1e2e"
PANEL = "#313244"
INPUT = "#1e1e2e"
TERM = "#11111b"
ACCENT = "#89b4fa"
GREEN = "#a6e3a1"
RED = "#f38ba8"
YELLOW = "#f9e2af"
MAUVE = "#cba6f7"
TEXT = "#cdd6f4"
SUB = "#a6adc8"
ARM_BLUE = "#4d96ff"

UI_FONT = "Microsoft YaHei UI"
MONO = "Consolas"

MIN_GAP = 0.045                            # прошивка опрашивает раз в 40 мс, небольшой запас
BAUD = 9600                                 # фиксировано 9600 8N1, выбора скорости нет


# ===================== Чистые функции (для отдельных тестов) =====================
def clamp(idx, value):
    """Ограничить угол допустимым диапазоном сустава."""
    return max(0, min(LIMITS[idx], int(round(float(value)))))


def parse_status_angles(line):
    """Взять последние 5 чисел (текущие углы) из ответа о статусе.

    Строка статуса: [Текущие углы S0,S1,S2,S3,S4]: 90,80,70,60,45
    Обозначения S0~S4 сами являются цифрами, поэтому берём "последние 5", а не с начала.
    Строка подтверждения цели (с "целевые углы") не считается фактическим углом.
    """
    if "Текущие углы" not in line:
        return None
    if "целевые углы" in line:
        return None
    nums = re.findall(r"\d+", line)
    if len(nums) < 5:
        return None
    tail = nums[-5:]
    return [clamp(i, v) for i, v in enumerate(tail)]


def arm_geometry(angles, w, h):
    """Прямая кинематика 2D: пиксельные координаты основания / плеча / локтя / кисти и направления звеньев."""
    s = min(w / 280.0, h / 420.0)
    cx = w / 2.0
    cy_base = h - 60 * s
    l1 = 137 * (2 / 3) * s
    l2 = 83 * (2 / 3) * s
    l3 = 42 * s
    sx, sy = cx, cy_base - 13 * s

    a1 = math.radians(angles[1])
    ex = sx + l1 * math.cos(a1)
    ey = sy - l1 * math.sin(a1)

    d2 = math.radians(angles[1] + angles[2] - 90)
    fx = ex + l2 * math.cos(d2)
    fy = ey - l2 * math.sin(d2)

    d3 = math.radians(angles[1] + angles[2] - 90 + angles[3] - 90)
    wx = fx + l3 * math.cos(d3)
    wy = fy - l3 * math.sin(d3)

    return dict(s=s, cx=cx, cy_base=cy_base, sx=sx, sy=sy,
                ex=ex, ey=ey, fx=fx, fy=fy, wx=wx, wy=wy,
                d2=d2, d3=d3, a1=a1, l1=l1, l2=l2, l3=l3)


def build_command(values):
    """Пять углов -> '0 90 90 90 40'"""
    return " ".join(str(int(v)) for v in values)


# ===================== Порт (очередь отправки + поток приёма) =====================
class SerialLink:
    """Чтение и запись порта в фоновом потоке, интерфейс получает события только через post(event).

    Формат события: (kind, text), kind ∈ {"tx", "rx", "err", "warn", "closed"}
    """

    def __init__(self, post):
        self.post = post
        self.ser = None
        self._pending = {}          # номер серво -> последний угол (объединение по суставу)
        self._raw = []              # сырые команды, порядок которых важен
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._alive = False
        self._last_tx = 0.0
        self.port = ""
        self.baud = BAUD

    # ---- Состояние ----
    @property
    def connected(self):
        return self._alive and self.ser is not None

    @property
    def idle(self):
        with self._lock:
            return not self._pending and not self._raw

    # ---- Подключение / отключение ----
    def connect(self, port, baud=BAUD):
        if not HAS_PYSERIAL:
            raise RuntimeError("Нет pyserial, выполните: pip install pyserial")
        self.port = port
        self.baud = int(baud)
        self._last_tx = 0.0
        self.ser = serial.Serial(port=port, baudrate=self.baud,
                                 timeout=0.1, write_timeout=0.5)
        self._alive = True
        self._start_threads()

    def _start_threads(self):
        threading.Thread(target=self._write_loop, daemon=True).start()
        threading.Thread(target=self._read_loop, daemon=True).start()

    def disconnect(self):
        self._alive = False
        with self._cv:
            self._pending.clear()
            self._raw.clear()
            self._cv.notify_all()
        ser = self.ser
        self.ser = None
        try:
            if ser is not None:
                ser.close()
        except Exception:                                       # noqa: BLE001
            pass

    # ---- Отправка ----
    def send_servo(self, idx, value):
        if not self.connected:
            return False
        with self._cv:
            self._pending[idx] = int(value)
            self._cv.notify()
        return True

    def send_raw(self, text):
        if not self.connected:
            return False
        with self._cv:
            if len(self._raw) >= 8:
                self._raw.pop(0)
                self.post(("warn", "Очередь переполнена, самая старая команда отброшена"))
            self._raw.append(str(text))
            self._cv.notify()
        return True

    def send_all(self, values):
        return self.send_raw(build_command(values))

    # ---- Поток ----
    def _write_loop(self):
        while self._alive:
            with self._cv:
                while self._alive and not self._pending and not self._raw:
                    self._cv.wait(0.2)
                if not self._alive:
                    break
                if self._pending:
                    idx = min(self._pending)
                    text = "%d %d" % (idx, self._pending.pop(idx))
                else:
                    text = self._raw.pop(0)
            gap = MIN_GAP - (time.monotonic() - self._last_tx)
            if gap > 0:
                time.sleep(gap)
            ser = self.ser
            if ser is None:
                break
            try:
                ser.write((text + "\n").encode("ascii"))
                self._last_tx = time.monotonic()
                self.post(("tx", text))
            except Exception as exc:                             # noqa: BLE001
                self.post(("err", "Ошибка записи: %s" % exc))
                self._alive = False
                self.post(("closed", ""))
                break

    def _read_loop(self):
        while self._alive:
            ser = self.ser
            if ser is None:
                break
            try:
                data = ser.readline()
            except Exception as exc:                             # noqa: BLE001
                if self._alive:
                    self.post(("err", "Ошибка порта: %s" % exc))
                self._alive = False
                self.post(("closed", ""))
                break
            if not data:
                continue
            text = data.decode("utf-8", errors="replace").strip()
            if text:
                self.post(("rx", text))

    def close(self):
        self.disconnect()


# ===================== Главное окно =====================
class ArmApp:
    def __init__(self, root):
        self.root = root
        self.uiq = queue.Queue()
        self.link = SerialLink(self.uiq.put)

        self.target = [90, 90, 90, 90, 90]
        self.actual = [90, 90, 90, 90, 90]

        self.traj = []
        self.playing = False
        self.play_index = 0
        self.play_speed = 2000
        self.play_loop = False
        self._play_job = None

        self.auto_send = True
        self.autoscroll = True
        self.drag_joint = None
        self.joints = []
        self.port_dev = ""
        self.port_map = {}
        self._sync = False
        self._last_warn = 0.0
        self.cw, self.ch = 430, 420

        self.slider = []
        self.entry = []
        self.act_lbl = []
        self.chip_lbl = []

        self._build_ui()
        self._refresh_ports()
        self._update_actual_labels()
        self._update_chips()
        self._update_preview()
        self._set_connected(False)
        self._redraw()

        if not HAS_PYSERIAL:
            self.log("Нет pyserial: выполните pip install pyserial и перезапустите программу", "err")
        else:
            self.log("Интерфейс готов. Выберите порт и подключитесь", "info")

        self.root.after(60, self._pump)
        self.root.after(1200, self._poll_status)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- Вспомогательные компоненты ----------
    def _button(self, parent, text, cmd, bg=ACCENT, fg=None, padx=12, pady=6):
        fg = fg or ("#1e1e2e" if bg not in (PANEL,) else SUB)
        return tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg,
                         activebackground=bg, activeforeground=fg,
                         relief="flat", bd=0, padx=padx, pady=pady,
                         font=(UI_FONT, 9, "bold"), cursor="hand2", takefocus=0)

    def _panel(self, parent, col, wcol=1):
        frame = tk.Frame(parent, bg=PANEL, padx=12, pady=12)
        frame.grid(row=0, column=col, sticky="nsew", padx=(0 if col == 0 else 12, 0))
        # uniform распределяет три колонки по одной группе: равные веса -> равная ширина панелей
        parent.columnconfigure(col, weight=wcol, uniform="panels")
        return frame

    def _section_title(self, parent, text, pady=(0, 8)):
        tk.Label(parent, text=text, bg=PANEL, fg=SUB,
                 font=(UI_FONT, 9)).pack(anchor="w", pady=pady)

    def _checkbox(self, parent, text, var, cmd, bg=PANEL):
        """Флажок для тёмной темы: без системной белой рамки, вместо неё ☐/☑ на светлом фоне."""
        cb = tk.Checkbutton(parent, bg=bg, fg=TEXT, activebackground=bg,
                            activeforeground=TEXT, selectcolor="#45475a",
                            indicatoron=0, relief="flat", bd=0,
                            highlightthickness=0, font=(UI_FONT, 9),
                            padx=6, pady=2, cursor="hand2", takefocus=0,
                            variable=var)
        cb.configure(command=lambda: self._on_check(cb, text, var, cmd))
        self._paint_check(cb, text, var.get())
        return cb

    def _on_check(self, cb, text, var, cmd):
        self._paint_check(cb, text, var.get())
        if cmd:
            cmd()

    def _paint_check(self, cb, text, on):
        cb.configure(text=("☑ " if on else "☐ ") + text)

    def _hint(self, parent, text, pady=(4, 0)):
        """Пояснительный текст с автопереносом (ширина следует за панелью)."""
        lbl = tk.Label(parent, text=text, bg=PANEL, fg=SUB, font=(UI_FONT, 8),
                       justify="left", anchor="w", wraplength=320)
        lbl.pack(anchor="w", fill="x", pady=pady)

        def _fix(event, _l=lbl):
            wrap = max(160, event.width - 6)
            try:
                cur = int(float(_l.cget("wraplength") or 0))
            except (TypeError, ValueError, tk.TclError):
                cur = -1
            if cur != wrap:
                _l.configure(wraplength=wrap)

        lbl.bind("<Configure>", _fix)
        return lbl
    # ---------- Построение интерфейса ----------
    def _build_ui(self):
        self.root.title("ARM4 — пульт управления")
        self.root.configure(bg=BG)
        self.root.geometry("1320x900")
        self.root.minsize(1200, 780)

        self._build_header()

        main = tk.Frame(self.root, bg=BG, padx=14, pady=12)
        main.pack(fill="both", expand=True)
        main.rowconfigure(0, weight=1)

        self._build_arm_panel(main)
        self._build_traj_panel(main)
        self._build_servo_panel(main)

        self._build_footer()

    def _build_header(self):
        outer = tk.Frame(self.root, bg=PANEL)
        outer.pack(fill="x")
        bar = tk.Frame(outer, bg=PANEL, padx=14, pady=10)
        bar.pack(fill="x")

        tk.Label(bar, text="ARM4 — пульт управления", bg=PANEL, fg=TEXT,
                 font=(UI_FONT, 12, "bold")).pack(side="left")

        self.dot = tk.Label(bar, text="●", bg=PANEL, fg=RED, font=(UI_FONT, 11))
        self.dot.pack(side="left", padx=(18, 5))
        self.status_lbl = tk.Label(bar, text="Не подключено", bg=PANEL, fg=SUB,
                                   font=(UI_FONT, 9))
        self.status_lbl.pack(side="left")

        self.conn_btn = self._button(bar, "🔌 Подключить", self._toggle_connect, bg=GREEN)
        self.conn_btn.pack(side="right", padx=(10, 0))
        self.home_btn = self._button(bar, "В центр 90°", self._home_all, bg=GREEN)
        self.home_btn.pack(side="right", padx=(10, 0))

        self.refresh_btn = self._button(bar, "⟳", self._refresh_ports, bg=PANEL, padx=8)
        self.refresh_btn.pack(side="right", padx=6)

        self.port_combo = ttk.Combobox(bar, width=28, state="readonly")
        self.port_combo.pack(side="right")
        tk.Label(bar, text="Порт (9600 8N1)", bg=PANEL, fg=SUB).pack(side="right", padx=(16, 4))

    def _build_arm_panel(self, main):
        panel = self._panel(main, 0, 1)
        self._section_title(panel, "Схема руки (перетаскивание суставов)")

        self.canvas = tk.Canvas(panel, bg=TERM, highlightthickness=0,
                                width=380, height=420)
        self.canvas.pack(fill="both", expand=True, pady=(2, 8))
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self.canvas.bind("<Button-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Motion>", self._on_hover)

        chips = tk.Frame(panel, bg=PANEL)
        chips.pack(fill="x")
        for i in range(5):
            lb = tk.Label(chips, text="S%d" % i, bg=INPUT, fg=COLORS[i],
                          font=(MONO, 7), padx=4, pady=3)
            lb.pack(side="left", padx=(0, 3))
            self.chip_lbl.append(lb)

        self._hint(panel, "Целевой / фактический угол; перетащите кольцо сустава — серво пойдёт за ним", pady=(8, 0))

    def _build_traj_panel(self, main):
        panel = self._panel(main, 1, 1)
        self._section_title(panel, "Траектория")

        row1 = tk.Frame(panel, bg=PANEL)
        row1.pack(fill="x")
        self.play_btn = self._button(row1, "▶ Пуск", self._toggle_play)
        self.play_btn.pack(side="left")
        self.stop_btn = self._button(row1, "⏹ Стоп", self._stop_play, bg=PANEL)
        self.stop_btn.pack(side="left", padx=6)
        self.clear_btn = self._button(row1, "Очистить", self._clear_traj, bg=PANEL)
        self.clear_btn.pack(side="left")

        row2 = tk.Frame(panel, bg=PANEL)
        row2.pack(fill="x", pady=(6, 0))
        self.snap_btn = self._button(row2, "⊚ Снять", lambda: self._snapshot(True), bg=GREEN)
        self.snap_btn.pack(side="left")
        self.save_btn = self._button(row2, "⇩ Сохранить", self._save_traj, bg=GREEN, padx=10)
        self.save_btn.pack(side="left", padx=6)
        self.load_btn = self._button(row2, "⇧ Загрузить", self._load_traj, bg=GREEN, padx=10)
        self.load_btn.pack(side="left")
        self.example_btn = self._button(row2, "Пример", self._load_example, bg=MAUVE, padx=10)
        self.example_btn.pack(side="left", padx=6)

        speed_row = tk.Frame(panel, bg=PANEL)
        speed_row.pack(fill="x", pady=(6, 0))
        tk.Label(speed_row, text="Скорость", bg=PANEL, fg=SUB, font=(UI_FONT, 9)).pack(side="left")
        self.speed_scale = tk.Scale(speed_row, from_=1200, to=6000, resolution=100,
                                    orient="horizontal", showvalue=0, bg=ACCENT,
                                    troughcolor=INPUT, highlightthickness=0, bd=0,
                                    sliderrelief="flat", activebackground=ACCENT,
                                    sliderlength=18, command=self._on_speed)
        self.speed_scale.set(2000)
        self.speed_scale.pack(side="left", fill="x", expand=True, padx=8)
        self.speed_lbl = tk.Label(speed_row, text="2000 мс/кадр", bg=PANEL, fg=SUB,
                                  font=(MONO, 8), width=12, anchor="e")
        self.speed_lbl.pack(side="right")

        self.loop_var = tk.BooleanVar(value=False)
        self._checkbox(panel, "Цикл", self.loop_var,
                       self._on_loop_toggle).pack(anchor="w", pady=(4, 4))

        box = tk.Frame(panel, bg=PANEL)
        box.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(box, columns=("i", "s0", "s1", "s2", "s3", "s4"),
                                 show="headings", height=6, selectmode="browse")
        for cid, text, width in (("i", "#", 44), ("s0", "S0", 44), ("s1", "S1", 44),
                                 ("s2", "S2", 44), ("s3", "S3", 44), ("s4", "S4", 44)):
            self.tree.heading(cid, text=text)
            self.tree.column(cid, width=width, anchor="center", stretch=True)
        vsb = ttk.Scrollbar(box, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        self.tree.bind("<ButtonRelease-1>", self._on_tree_click)
        self.tree.bind("<Double-1>", self._on_tree_dbl)

        self._style_tree()

        self._section_title(panel, "Своя команда", pady=(12, 4))
        cmd_row = tk.Frame(panel, bg=PANEL)
        cmd_row.pack(fill="x")
        self.cmd_var = tk.StringVar()
        cmd_entry = tk.Entry(cmd_row, textvariable=self.cmd_var, bg=INPUT, fg=TEXT,
                             insertbackground=TEXT, relief="flat", font=(MONO, 9))
        cmd_entry.pack(side="left", fill="x", expand=True, ipady=5, padx=(0, 8))
        cmd_entry.bind("<Return>", lambda _e: self._send_custom())
        self._button(cmd_row, "Отправить", self._send_custom, padx=12).pack(side="right")

        self.preview_lbl = tk.Label(panel, text="", bg=PANEL, fg=ACCENT, font=(MONO, 9))
        self.preview_lbl.pack(anchor="w", pady=(6, 0))

        self._hint(panel,
                   "Пример: 90 | 0 60 | 0 90 90 90 40 | s\n"
                   "Пределы: S0–S3 0–180°, S4 захват 0–90°, S2 инвертировано (в интерфейсе логический угол)\n"
                   "Прошивка принимает только цифры, пробелы, h, s; прочее отбрасывается", pady=(4, 0))

    def _style_tree(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Treeview", background=INPUT, fieldbackground=INPUT,
                        foreground=TEXT, rowheight=24, borderwidth=0, relief="flat")
        style.map("Treeview", background=[("selected", ACCENT)],
                  foreground=[("selected", "#1e1e2e")])
        style.configure("Treeview.Heading", background=PANEL, foreground=SUB,
                        relief="flat", borderwidth=0, font=(UI_FONT, 8, "bold"))
        style.map("Treeview.Heading",
                  background=[("active", PANEL), ("pressed", PANEL)],
                  foreground=[("active", TEXT)])
        # для readonly-списка нужен явный fieldbackground, иначе остаётся белый фон системы
        style.configure("TCombobox", fieldbackground=INPUT, background=PANEL,
                        foreground=TEXT, arrowcolor=SUB, insertcolor=TEXT,
                        bordercolor=PANEL, lightcolor=PANEL, darkcolor=PANEL,
                        padding=3)
        style.map("TCombobox",
                  fieldbackground=[("readonly", INPUT), ("disabled", PANEL)],
                  foreground=[("readonly", TEXT), ("disabled", SUB)],
                  selectbackground=[("readonly", INPUT)],
                  selectforeground=[("readonly", TEXT)],
                  background=[("readonly", PANEL), ("disabled", PANEL)],
                  arrowcolor=[("readonly", SUB), ("active", TEXT)],
                  bordercolor=[("readonly", PANEL)])
        style.configure("TScrollbar", background=PANEL, troughcolor=INPUT,
                        arrowcolor=SUB, bordercolor=PANEL, darkcolor=PANEL,
                        lightcolor=PANEL)
        style.map("TScrollbar", background=[("active", "#45475a")])

    def _build_servo_panel(self, main):
        panel = self._panel(main, 2, 1)
        self._section_title(panel, "Управление серво")

        for i in range(5):
            row = tk.Frame(panel, bg=PANEL)
            row.pack(fill="x", pady=5)

            name = "%s %d°" % (NAMES[i], LIMITS[i])
            if i == 2:
                name += " ↔"
            tk.Label(row, text=name, bg=PANEL, fg=COLORS[i], width=17, anchor="w",
                     font=(UI_FONT, 8, "bold")).pack(side="left")

            act = tk.Label(row, text="✓ 90°", bg=PANEL, fg=GREEN, width=7, anchor="e",
                           font=(MONO, 9, "bold"))
            act.pack(side="right", padx=(6, 0))
            self.act_lbl.append(act)

            rst = self._button(row, "↺", lambda _i=i: self._reset_servo(_i),
                               bg=PANEL, padx=8, pady=2)
            rst.pack(side="right", padx=(6, 0))
            rst.configure(font=(UI_FONT, 9))

            num = tk.Entry(row, width=4, bg=INPUT, fg=TEXT, insertbackground=TEXT,
                           relief="flat", justify="center", font=(MONO, 9))
            num.pack(side="right", ipady=3)
            num.insert(0, "90")
            num.bind("<Return>", lambda _e, _i=i: self._apply_entry(_i))
            num.bind("<FocusOut>", lambda _e, _i=i: self._apply_entry(_i))
            self.entry.append(num)

            scale = tk.Scale(row, from_=0, to=LIMITS[i], resolution=1,
                             orient="horizontal", showvalue=0, bg=ACCENT,
                             troughcolor=INPUT, highlightthickness=0, bd=0,
                             sliderrelief="flat", activebackground=ACCENT,
                             sliderlength=18, length=120,
                             command=lambda v, _i=i: self._on_slider(_i, v))
            scale.set(90)
            scale.pack(side="left", fill="x", expand=True, padx=8)
            # клик по дорожке сразу переносит ползунок, удержание ведёт его за курсором
            scale.bind("<Button-1>", lambda e, _s=scale: self._scale_jump(_s, e))
            scale.bind("<B1-Motion>", lambda e, _s=scale: self._scale_jump(_s, e))
            scale.bind("<ButtonRelease-1>", lambda _e, _i=i: self._send_servo(_i))
            self.slider.append(scale)

        self.live_var = tk.BooleanVar(value=True)
        self._checkbox(panel, "Отправлять при перетаскивании", self.live_var,
                       self._on_live_toggle).pack(anchor="w", pady=(14, 6))

        self._hint(panel,
                   "Главный цикл прошивки читает порт раз в 40 мс, интерполяция около 25°/с.\n"
                   "Программа объединяет отправку (по серво только последнее значение) и держит интервал 45 мс,"
                   "чтобы не переполнить буфер приёма 64 байта.\n"
                   "После подключения раз в 1,2 с отправляется s для обновления фактических углов.", pady=(0, 0))

    def _build_footer(self):
        foot = tk.Frame(self.root, bg=BG)
        foot.pack(fill="x", padx=14, pady=(0, 12))

        bar = tk.Frame(foot, bg=BG)
        bar.pack(fill="x", pady=(0, 6))
        tk.Label(bar, text="Журнал порта", bg=BG, fg=SUB,
                 font=(UI_FONT, 9, "bold")).pack(side="left")

        self._button(bar, "Очистить", self._clear_log, bg=PANEL, padx=10, pady=3).pack(side="right")
        self._button(bar, "Копировать", self._copy_log, bg=PANEL, padx=10, pady=3).pack(
            side="right", padx=6)
        self.auto_var = tk.BooleanVar(value=True)
        self._checkbox(bar, "Автопрокрутка", self.auto_var, self._on_auto_toggle,
                       bg=BG).pack(side="right", padx=10)

        self.logbox = tk.Text(foot, height=8, bg=TERM, fg=TEXT, relief="flat",
                              font=(MONO, 9), padx=10, pady=8, wrap="word",
                              state="disabled", insertbackground=TEXT)
        self.logbox.pack(fill="x")
        for tag, color in (("info", SUB), ("ok", GREEN), ("warn", YELLOW),
                           ("err", RED), ("tx", ACCENT), ("rx", MAUVE)):
            self.logbox.tag_configure(tag, foreground=color)

    # ---------- Журнал ----------
    def log(self, msg, level="info"):
        self.logbox.configure(state="normal")
        self.logbox.insert("end", "[%s] %s\n" % (time.strftime("%H:%M:%S"), msg), level)
        lines = int(self.logbox.index("end-1c").split(".")[0])
        if lines > 220:
            self.logbox.delete("1.0", "%d.0" % (lines - 200))
        self.logbox.configure(state="disabled")
        if self.autoscroll:
            self.logbox.see("end")

    def _clear_log(self):
        self.logbox.configure(state="normal")
        self.logbox.delete("1.0", "end")
        self.logbox.configure(state="disabled")

    def _copy_log(self):
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(self.logbox.get("1.0", "end-1c"))
        except tk.TclError:
            pass

    def _on_auto_toggle(self):
        self.autoscroll = self.auto_var.get()

    # ---------- Порт ----------
    def _refresh_ports(self):
        ports = []
        if HAS_PYSERIAL:
            for p in list_ports.comports():
                ports.append((p.device, p.description or ""))
        self.port_map = {}
        items = []
        for dev, desc in ports:
            label = "%s  %s" % (dev, desc) if desc else dev
            self.port_map[label] = dev
            items.append(label)
        self.port_combo.configure(values=items)
        if not items:
            self.port_combo.set("")
            return
        prefer = None
        for label in items:
            low = label.lower()
            if any(k in low for k in ("ch340", "cp210", "arduino", "usb-serial",
                                      "usb serial", "ft232", "wchusbserial")):
                prefer = label
                break
        current = self.port_combo.get()
        self.port_combo.set(current if current in items else (prefer or items[0]))

    def _toggle_connect(self):
        if self.link.connected:
            self.link.disconnect()
            self._set_connected(False)
            self.log("Порт отключён", "info")
            return

        if not HAS_PYSERIAL:
            self.log("Нет pyserial: выполните pip install pyserial", "err")
            return

        label = self.port_combo.get().strip()
        dev = self.port_map.get(label)
        if not dev:
            dev = label.split()[0] if label else ""
        if not dev:
            self.log("Сначала выберите порт (⟳ обновляет список)", "warn")
            return

        try:
            self.link.connect(dev, BAUD)
        except Exception as exc:                                # noqa: BLE001
            self.log("Ошибка подключения: %s" % exc, "err")
            self._set_connected(False)
            return

        self.port_dev = dev
        self._set_connected(True)
        self.log("Порт подключён %s @%d" % (dev, BAUD), "ok")
        self.link.send_raw("s")

    def _set_connected(self, ok):
        self.dot.configure(fg=GREEN if ok else RED)
        if ok:
            self.status_lbl.configure(text="Подключено %d %s" % (BAUD, self.port_dev), fg=SUB)
        else:
            self.status_lbl.configure(text="Не подключено", fg=SUB)
        self.conn_btn.configure(text="✕ Отключить" if ok else "🔌 Подключить",
                                bg=ACCENT if ok else GREEN)

    def _poll_status(self):
        if self.link.connected and self.link.idle:
            self.link.send_raw("s")
        self.root.after(1200, self._poll_status)

    def _pump(self):
        try:
            while True:
                kind, text = self.uiq.get_nowait()
                if kind == "tx":
                    self.log("→ " + text, "tx")
                elif kind == "rx":
                    self.log("← " + text, "rx")
                    self._handle_rx(text)
                elif kind == "warn":
                    self.log(text, "warn")
                elif kind == "err":
                    self.log(text, "err")
                elif kind == "closed":
                    if self.link.connected:
                        self.link.disconnect()
                    self._set_connected(False)
                    self.log("Соединение потеряно", "warn")
        except queue.Empty:
            pass
        self.root.after(60, self._pump)

    def _handle_rx(self, line):
        angles = parse_status_angles(line)
        if angles is not None:
            self.actual = list(angles)
            self._update_actual_labels()
            self._update_chips()

    # ---------- Панель серво ----------
    def _warn_once(self, msg):
        now = time.monotonic()
        if now - self._last_warn > 1.0:
            self._last_warn = now
            self.log(msg, "warn")

    def _set_slider(self, idx, value):
        self._sync = True
        self.slider[idx].set(value)
        self.entry[idx].delete(0, "end")
        self.entry[idx].insert(0, str(int(value)))
        self._sync = False

    def _send_servo(self, idx, quiet=False):
        if self.link.send_servo(idx, self.target[idx]):
            return True
        if not quiet:
            self._warn_once("Порт не подключён, команда не отправлена")
        return False

    def _scale_jump(self, scale, event):
        """Перенести ползунок под курсор: с обоих концов дорожки оставляем половину длины ползунка."""
        w = scale.winfo_width()
        half = float(scale.cget("sliderlength")) / 2.0
        lo, hi = float(scale.cget("from")), float(scale.cget("to"))
        res = float(scale.cget("resolution") or 1)
        ratio = (event.x - half) / max(1.0, w - 2 * half)
        ratio = min(1.0, max(0.0, ratio))
        val = lo + round(ratio * (hi - lo) / res) * res
        if val != float(scale.get()):
            scale.set(val)
        return "break"

    def _on_slider(self, idx, raw):
        if self._sync:
            return
        value = clamp(idx, float(raw))
        self.target[idx] = value
        self.entry[idx].delete(0, "end")
        self.entry[idx].insert(0, str(value))
        if self.auto_send:
            self._send_servo(idx, quiet=True)
        self._after_target()

    def _apply_entry(self, idx):
        try:
            value = int(float(self.entry[idx].get()))
        except ValueError:
            value = self.target[idx]
        value = clamp(idx, value)
        self.target[idx] = value
        self._set_slider(idx, value)
        self._send_servo(idx)
        self._after_target()

    def _reset_servo(self, idx):
        self.target[idx] = 90
        self._set_slider(idx, 90)
        self._send_servo(idx)
        self._after_target()

    def _home_all(self):
        for i in range(5):
            self.target[i] = 90
            self._set_slider(i, 90)
        if not self.link.send_all(self.target):
            self._warn_once("Порт не подключён, команда не отправлена")
        else:
            self.log("Отправлен центр: %s" % build_command(self.target), "info")
        self._after_target()

    def _on_live_toggle(self):
        self.auto_send = self.live_var.get()

    def _after_target(self):
        self._update_actual_labels()
        self._update_chips()
        self._redraw()
        self._update_preview()

    def _update_actual_labels(self):
        for i in range(5):
            same = self.actual[i] == self.target[i]
            self.act_lbl[i].configure(
                text=("✓ %d°" if same else "→ %d°") % self.actual[i],
                fg=GREEN if same else YELLOW)

    def _update_chips(self):
        for i in range(5):
            self.chip_lbl[i].configure(
                text="S%d  %d°/%d°" % (i, self.target[i], self.actual[i]))

    def _update_preview(self):
        custom = self.cmd_var.get().strip()
        self.preview_lbl.configure(text="→ " + (custom or build_command(self.target)))

    # ---------- Своя команда ----------
    def _send_custom(self):
        cmd = self.cmd_var.get().strip()
        if not cmd:
            return
        self._sync_from_command(cmd)
        if not self.link.send_raw(cmd):
            self._warn_once("Порт не подключён, команда не отправлена")
        self.cmd_var.set("")
        self._update_preview()

    def _sync_from_command(self, cmd):
        parts = [p for p in re.split(r"\s+", cmd) if p]
        if not parts:
            return
        if parts[0].lower() in ("h", "help", "s", "status"):
            return
        nums = []
        for p in parts:
            if not re.fullmatch(r"\d+", p):
                return
            nums.append(int(p))
        if len(nums) == 5:
            for i in range(5):
                self.target[i] = clamp(i, nums[i])
        elif len(nums) == 1:
            for i in range(5):
                self.target[i] = clamp(i, nums[0])
        elif len(nums) == 2 and 0 <= nums[0] < 5:
            self.target[nums[0]] = clamp(nums[0], nums[1])
        for i in range(5):
            self._set_slider(i, self.target[i])
        self._after_target()

    # ---------- Холст ----------
    def _on_canvas_resize(self, event):
        self.cw, self.ch = max(event.width, 40), max(event.height, 40)
        self._redraw()

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        w, h = self.cw, self.ch
        if w < 40 or h < 40:
            return

        s = min(w / 280.0, h / 420.0)
        g = arm_geometry(self.target, w, h)

        # пол
        c.create_line(0, g["cy_base"] + 18 * s, w, g["cy_base"] + 18 * s,
                      fill=SUB, dash=(2, 4))
        # основание
        mw, mht, mhb = 52 * s, 13 * s, 26 * s
        c.create_rectangle(g["cx"] - mw, g["cy_base"] - mht,
                           g["cx"] + mw, g["cy_base"] + mhb,
                           fill=ARM_BLUE, outline="")

        # три звена
        for x1, y1, x2, y2, lw in ((g["sx"], g["sy"], g["ex"], g["ey"], 20),
                                   (g["ex"], g["ey"], g["fx"], g["fy"], 17),
                                   (g["fx"], g["fy"], g["wx"], g["wy"], 12)):
            c.create_line(x1, y1, x2, y2, fill=ARM_BLUE,
                          width=max(2, int(lw * s)), capstyle=tk.ROUND)

        # кольца суставов
        for x, y, r in ((g["sx"], g["sy"], 13 * s), (g["ex"], g["ey"], 12 * s),
                        (g["fx"], g["fy"], 8 * s), (g["wx"], g["wy"], 7 * s)):
            r = max(3, r)
            c.create_oval(x - r, y - r, x + r, y + r, fill=TERM, outline="#ffffff",
                          width=3)

        # захват: S4 0~90 -> степень раскрытия
        open_ratio = max(0.0, min(1.0, self.target[4] / 90.0))
        half = math.radians(45) * open_ratio
        grip = 31 * s
        for sign in (-1, 1):
            ang = g["d3"] + sign * half
            c.create_line(g["wx"], g["wy"],
                          g["wx"] + grip * math.cos(ang),
                          g["wy"] - grip * math.sin(ang), fill=RED, width=3)

        # указатель основания S0
        s0r = math.radians(self.target[0])
        indlen = 75 * (2 / 3) * s
        c.create_line(g["cx"], g["sy"],
                      g["cx"] + indlen * math.cos(s0r),
                      g["sy"] - indlen * math.sin(s0r),
                      fill="#ffffff", width=max(2, int(3 * s)))

        # подписи целевых углов сверху
        lx = 10
        for i in range(5):
            c.create_text(lx, 8, text="S%d" % i, anchor="nw", fill=COLORS[i],
                          font=(MONO, 8, "bold"))
            c.create_text(lx + 22 * s, 8, text="%d°" % self.target[i], anchor="nw",
                          fill=TEXT, font=(MONO, 8, "bold"))
            lx += 60 * s

        # данные попадания при перетаскивании
        self.joints = [
            dict(id=0, kind="polar", ax=g["sx"], ay=g["sy"],
                 jx=g["cx"] + indlen * math.cos(s0r),
                 jy=g["sy"] - indlen * math.sin(s0r), r=max(22, 28 * s)),
            dict(id=1, kind="polar", ax=g["sx"], ay=g["sy"],
                 jx=g["ex"], jy=g["ey"], r=max(18, 22 * s)),
            dict(id=2, kind="polar", ax=g["ex"], ay=g["ey"],
                 jx=g["fx"], jy=g["fy"], r=max(14, 18 * s)),
            dict(id=3, kind="polar", ax=g["fx"], ay=g["fy"],
                 jx=g["wx"], jy=g["wy"], r=max(12, 16 * s)),
            dict(id=4, kind="perp", jx=g["wx"], jy=g["wy"],
                 nx=math.cos(g["d3"]), ny=-math.sin(g["d3"]),
                 r=max(34, 46 * s)),
        ]

    def _joint_at(self, x, y, pad=0):
        for i, j in enumerate(self.joints):
            dx, dy = j["jx"] - x, j["jy"] - y
            rr = j["r"] + pad
            if dx * dx + dy * dy <= rr * rr:
                return i
        return None

    def _on_press(self, event):
        idx = self._joint_at(event.x, event.y)
        if idx is not None:
            self.drag_joint = idx
            self.canvas.configure(cursor="hand2")

    def _on_hover(self, event):
        if self.drag_joint is not None:
            return
        self.canvas.configure(cursor="hand2" if self._joint_at(event.x, event.y) is not None else "")

    def _on_drag(self, event):
        if self.drag_joint is None:
            return
        j = self.joints[self.drag_joint]
        sid = j["id"]

        if j["kind"] == "perp":
            dx, dy = event.x - j["jx"], event.y - j["jy"]
            perp = dx * (-j["ny"]) + dy * j["nx"]
            max_perp = 31 * math.sin(math.radians(45))
            if max_perp < 1:
                max_perp = 1
            value = abs(perp) / max_perp * 90.0
        else:
            value = math.degrees(math.atan2(j["ay"] - event.y, event.x - j["ax"]))
            if sid == 2:
                value = 90 + value - self.target[1]
            elif sid == 3:
                value = 180 + value - self.target[1] - self.target[2]
            value = (value % 360 + 360) % 360
            if value > 180:
                value -= 360

        value = clamp(sid, value)
        if self.target[sid] == value:
            return
        self.target[sid] = value
        self._set_slider(sid, value)
        if self.auto_send:
            self._send_servo(sid, quiet=True)
        self._after_target()

    def _on_release(self, _event):
        if self.drag_joint is not None and self.auto_send:
            self._send_servo(self.joints[self.drag_joint]["id"])
        self.drag_joint = None
        self.canvas.configure(cursor="")

    # ---------- Траектория ----------
    def _snapshot(self, force=False):
        frame = list(self.target)
        if not force and self.traj and self.traj[-1] == frame:
            return
        if self.traj and self.traj[-1] == frame:
            if force:
                self.log("Углы совпадают с предыдущим кадром, пропущено", "warn")
            return
        self.traj.append(frame)
        self._refresh_tree()
        self.log("Снят кадр %d: %s" % (len(self.traj), build_command(frame)), "info")

    def _refresh_tree(self):
        for iid in self.tree.get_children():
            self.tree.delete(iid)
        for i, frame in enumerate(self.traj):
            self.tree.insert("", "end", iid=str(i),
                             values=("#%d" % (i + 1),) + tuple(frame))

    def _on_tree_click(self, event):
        iid = self.tree.identify_row(event.y)
        if iid:
            self._jump(int(iid))

    def _on_tree_dbl(self, event):
        iid = self.tree.identify_row(event.y)
        col = self.tree.identify_column(event.x)
        if not iid or not col:
            return
        c = int(col.replace("#", ""))
        if c <= 1:
            return
        sid = c - 2
        idx = int(iid)
        cur = self.traj[idx][sid]
        value = simpledialog.askinteger(
            "Изменить угол кадра",
            "Кадр %d · %s (0–%d)" % (idx + 1, NAMES[sid], LIMITS[sid]),
            parent=self.root, initialvalue=cur, minvalue=0, maxvalue=LIMITS[sid])
        if value is None:
            return
        self.traj[idx][sid] = clamp(sid, value)
        self._refresh_tree()
        self._jump(idx)

    def _jump(self, idx):
        if self.playing or idx < 0 or idx >= len(self.traj):
            return
        frame = list(self.traj[idx])
        self.target = frame
        for i in range(5):
            self._set_slider(i, frame[i])
        self._after_target()
        self.link.send_all(frame)
        self.tree.selection_set(str(idx))
        self.tree.see(str(idx))
        self.log("Переход к кадру %d/%d: %s" % (idx + 1, len(self.traj), build_command(frame),), "info")

    def _delete_frame(self, idx):
        if self.playing:
            self.log("Во время воспроизведения кадр не удаляется", "warn")
            return
        if 0 <= idx < len(self.traj):
            self.traj.pop(idx)
            self._refresh_tree()
            self._redraw()
            self.log("Кадр %d удалён" % (idx + 1), "info")

    def _on_speed(self, raw):
        self.play_speed = int(float(raw))
        self.speed_lbl.configure(text="%d мс/кадр" % self.play_speed)

    def _on_loop_toggle(self):
        self.play_loop = self.loop_var.get()

    def _toggle_play(self):
        if self.playing:
            self._stop_play()
            return
        if not self.link.connected:
            self.log("Сначала подключите порт", "warn")
            return
        if not self.traj:
            self.log("Нечего воспроизводить — снять или загрузить траекторию", "warn")
            return
        self.playing = True
        self.play_index = 0
        self.play_btn.configure(text="⏸ Пауза")
        self.log("Воспроизведение — кадров: %d" % len(self.traj), "info")
        self._play_tick()

    def _play_tick(self):
        if not self.playing:
            return
        if not self.traj:
            self._stop_play()
            return
        if self.play_index >= len(self.traj):
            if self.play_loop:
                self.play_index = 0
            else:
                self._stop_play()
                return
        frame = list(self.traj[self.play_index])
        self.target = frame
        for i in range(5):
            self._set_slider(i, frame[i])
        self._after_target()
        self.link.send_all(frame)
        self.tree.selection_set(str(self.play_index))
        self.tree.see(str(self.play_index))
        self.play_index += 1
        self._play_job = self.root.after(self.play_speed, self._play_tick)

    def _stop_play(self):
        if self._play_job is not None:
            self.root.after_cancel(self._play_job)
            self._play_job = None
        if self.playing:
            self.log("Воспроизведение остановлено", "info")
        self.playing = False
        self.play_btn.configure(text="▶ Пуск")

    def _clear_traj(self):
        self._stop_play()
        self.traj = []
        self._refresh_tree()
        self._redraw()
        self.log("Траектория очищена", "info")

    def _save_traj(self):
        if not self.traj:
            self.log("Нечего сохранять — траектория пуста", "warn")
            return
        path = filedialog.asksaveasfilename(
            parent=self.root, title="Сохранить траекторию", defaultextension=".json",
            initialfile="arm4_trajectory.json",
            filetypes=[("JSON траектория", "*.json"), ("Все файлы", "*.*")])
        if not path:
            return
        payload = {"format": "arm4.servo_control.trajectory.v1",
                   "frame_count": len(self.traj), "frames": self.traj}
        try:
            with open(path, "w", encoding="utf-8") as fp:
                json.dump(payload, fp, ensure_ascii=False, indent=2)
            self.log("Сохранено кадров %d → %s" % (len(self.traj), path), "info")
        except Exception as exc:                                # noqa: BLE001
            self.log("Ошибка сохранения: %s" % exc, "err")

    def _load_traj(self):
        path = filedialog.askopenfilename(
            parent=self.root, title="Загрузить траекторию",
            filetypes=[("JSON траектория", "*.json"), ("Все файлы", "*.*")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fp:
                payload = json.load(fp)
        except Exception as exc:                               # noqa: BLE001
            self.log("Ошибка загрузки: %s" % exc, "err")
            return
        frames = payload.get("frames") if isinstance(payload, dict) else None
        if not isinstance(frames, list):
            self.log("Файл не является траекторией (нет массива frames)", "err")
            return
        cleaned = []
        for frame in frames:
            if not isinstance(frame, list) or len(frame) != 5:
                continue
            try:
                cleaned.append([clamp(i, frame[i]) for i in range(5)])
            except (TypeError, ValueError):
                continue
        if not cleaned:
            self.log("В файле нет корректных кадров", "err")
            return
        self._stop_play()
        self.traj = cleaned
        self._refresh_tree()
        self._redraw()
        self.log("Загружено кадров %d ← %s" % (len(cleaned), path), "info")

    def _load_example(self):
        self._stop_play()
        example = [
            [90, 90, 90, 90, 45],
            [35, 115, 70, 75, 70],
            [90, 90, 90, 90, 45],
            [145, 115, 70, 105, 70],
            [90, 90, 90, 90, 45],
            [25, 125, 60, 70, 80],
            [90, 90, 90, 90, 45],
            [155, 125, 60, 110, 80],
            [40, 65, 115, 95, 25],
            [140, 65, 115, 85, 25],
            [90, 105, 70, 90, 60],
            [90, 90, 90, 90, 45],
        ]
        self.traj = [list(f) for f in example]
        self._refresh_tree()
        self._redraw()
        self.log("Загружен встроенный пример — кадров: %d" % len(self.traj), "info")
        self._toggle_play()

    # ---------- Выход ----------
    def _on_close(self):
        try:
            self._stop_play()
        except Exception:                                      # noqa: BLE001
            pass
        try:
            self.link.disconnect()
        except Exception:                                      # noqa: BLE001
            pass
        self.root.destroy()


def _enable_dpi_awareness():
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:                                      # noqa: BLE001
            pass


def main():
    _enable_dpi_awareness()
    root = tk.Tk()
    ArmApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()






