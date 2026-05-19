# coding:UTF-8
import asyncio
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

import bleak
import device_model


COLORS = {
    "bg": "#0b1120",
    "panel": "#111827",
    "panel_2": "#172033",
    "border": "#2b3a52",
    "text": "#e5e7eb",
    "muted": "#94a3b8",
    "accent": "#38bdf8",
    "accent_2": "#34d399",
    "danger": "#fb7185",
    "warning": "#fbbf24",
    "x": "#38bdf8",
    "y": "#34d399",
    "z": "#f472b6",
}


ANGLE_KEYS = ("AngX", "AngY", "AngZ")
STABILITY_WINDOW_SECONDS = 1.5
MIN_STABILITY_SAMPLES = 12
STABILITY_LIMITS = {
    "AngX": 2.0,
    "AngY": 2.0,
    "AngZ": 4.0,
}
AXIS_LABELS = {
    "AngX": ("X 方向夹角", COLORS["x"]),
    "AngY": ("Y 方向夹角", COLORS["y"]),
    "AngZ": ("Z 方向夹角", COLORS["z"]),
}


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def circular_mean(values):
    if not values:
        return 0.0
    sin_sum = sum(math.sin(math.radians(value)) for value in values)
    cos_sum = sum(math.cos(math.radians(value)) for value in values)
    return math.degrees(math.atan2(sin_sum, cos_sum))


class DualBleWorker:
    def __init__(self, event_queue):
        self.event_queue = event_queue
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        self.models = {}

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def scan(self, timeout=8):
        future = self._submit(self._scan(timeout))
        future.add_done_callback(lambda done: self._done("scan", done))

    async def _scan(self, timeout):
        devices = await bleak.BleakScanner.discover(timeout=timeout)
        devices = [
            device
            for device in devices
            if device.name and "WT" in device.name.upper()
        ]
        devices.sort(key=lambda device: (device.name.upper(), device.address))
        return devices

    def connect(self, slot, ble_device):
        self.disconnect(slot)
        future = self._submit(self._connect(slot, ble_device))
        future.add_done_callback(lambda done: self._done("connect", done, slot))

    async def _connect(self, slot, ble_device):
        model = device_model.DeviceModel(f"IMU {slot}", ble_device, lambda data_model: self._on_data(slot, data_model))
        self.models[slot] = model
        self.event_queue.put(("status", slot, f"IMU {slot} 正在连接 {ble_device.name or 'Unknown'}"))
        try:
            await model.openDevice()
            return f"IMU {slot} 已断开"
        finally:
            if self.models.get(slot) is model:
                self.models.pop(slot, None)

    def _on_data(self, slot, model):
        self.event_queue.put(("data", slot, dict(model.deviceData)))

    def disconnect(self, slot):
        model = self.models.get(slot)
        if model is not None:
            model.closeDevice()

    def disconnect_all(self):
        for slot in list(self.models):
            self.disconnect(slot)

    def stop(self):
        self.disconnect_all()
        self.loop.call_soon_threadsafe(self.loop.stop)

    def _done(self, kind, done, slot=None):
        try:
            result = done.result()
            self.event_queue.put((kind, slot, result))
        except Exception as exc:
            self.event_queue.put((f"{kind}_error", slot, str(exc)))


class OrientationCube(tk.Canvas):
    def __init__(self, parent, slot, **kwargs):
        super().__init__(
            parent,
            bg=COLORS["panel"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            **kwargs,
        )
        self.slot = slot
        self.device_name = "未连接"
        self.status = "等待连接"
        self.last_update = None
        self.ang_x = 0.0
        self.ang_y = 0.0
        self.ang_z = 0.0
        self.bind("<Configure>", lambda _event: self.draw())

    def set_status(self, status, device_name=None):
        self.status = status
        if device_name is not None:
            self.device_name = device_name
        self.draw()

    def update_data(self, data):
        self.ang_x = float(data.get("AngX", self.ang_x) or 0.0)
        self.ang_y = float(data.get("AngY", self.ang_y) or 0.0)
        self.ang_z = float(data.get("AngZ", self.ang_z) or 0.0)
        self.status = "正在接收数据"
        self.last_update = time.time()
        self.draw()

    def refresh_age(self):
        if self.last_update is not None:
            age = time.time() - self.last_update
            if age > 2.5:
                self.status = f"{age:.0f}s 未收到新数据"
                self.draw()

    def _rotate_point(self, point):
        x, y, z = point
        rx = math.radians(self.ang_x)
        ry = math.radians(self.ang_y)
        rz = math.radians(self.ang_z)

        cos_x, sin_x = math.cos(rx), math.sin(rx)
        y, z = y * cos_x - z * sin_x, y * sin_x + z * cos_x

        cos_y, sin_y = math.cos(ry), math.sin(ry)
        x, z = x * cos_y + z * sin_y, -x * sin_y + z * cos_y

        cos_z, sin_z = math.cos(rz), math.sin(rz)
        x, y = x * cos_z - y * sin_z, x * sin_z + y * cos_z
        return x, y, z

    @staticmethod
    def _project(point, cx, cy, scale):
        x, y, z = point
        distance = 4.4
        factor = scale / (distance - z)
        return cx + x * factor, cy - y * factor

    def _screen_point(self, point, cx, cy, scale):
        return self._project(self._rotate_point(point), cx, cy, scale)

    def draw(self):
        self.delete("all")
        width = max(self.winfo_width(), 420)
        height = max(self.winfo_height(), 420)
        cx, cy = width / 2, height / 2 + 22
        scale = min(width, height) * 1.32

        self.create_rectangle(0, 0, width, height, fill=COLORS["panel"], outline="")
        self.create_text(
            24,
            24,
            text=f"IMU {self.slot}",
            fill=COLORS["text"],
            anchor="w",
            font=("Helvetica Neue", 28, "bold"),
        )
        self.create_text(
            24,
            58,
            text=self.device_name,
            fill=COLORS["muted"],
            anchor="w",
            font=("Helvetica Neue", 13),
        )
        status_color = COLORS["accent_2"] if self.last_update else COLORS["warning"]
        self.create_text(
            width - 24,
            30,
            text=self.status,
            fill=status_color,
            anchor="e",
            font=("Helvetica Neue", 13, "bold"),
        )

        vertices = {
            "000": (-1, -1, -1),
            "001": (-1, -1, 1),
            "010": (-1, 1, -1),
            "011": (-1, 1, 1),
            "100": (1, -1, -1),
            "101": (1, -1, 1),
            "110": (1, 1, -1),
            "111": (1, 1, 1),
        }
        rotated = {key: self._rotate_point(value) for key, value in vertices.items()}
        projected = {key: self._project(value, cx, cy, scale) for key, value in rotated.items()}

        faces = [
            ("001", "101", "111", "011", "#164e63"),
            ("100", "101", "111", "110", "#14532d"),
            ("010", "011", "111", "110", "#4c1d95"),
            ("000", "100", "110", "010", "#1e293b"),
            ("000", "001", "011", "010", "#1f2937"),
            ("000", "100", "101", "001", "#334155"),
        ]
        faces.sort(key=lambda face: sum(rotated[key][2] for key in face[:4]) / 4)
        for face in faces:
            points = []
            for key in face[:4]:
                points.extend(projected[key])
            self.create_polygon(points, fill=face[4], outline=COLORS["border"], width=1)

        edges = (
            ("000", "001"), ("000", "010"), ("000", "100"),
            ("111", "101"), ("111", "011"), ("111", "110"),
            ("001", "011"), ("001", "101"), ("010", "011"),
            ("010", "110"), ("100", "101"), ("100", "110"),
        )
        for a, b in edges:
            self.create_line(*projected[a], *projected[b], fill="#dbeafe", width=2)

        origin = self._screen_point((0, 0, 0), cx, cy, scale)
        axes = (
            ((1.7, 0, 0), COLORS["x"], "X"),
            ((0, 1.7, 0), COLORS["y"], "Y"),
            ((0, 0, 1.7), COLORS["z"], "Z"),
        )
        for end, color, label in axes:
            p2 = self._screen_point(end, cx, cy, scale)
            self.create_line(*origin, *p2, fill=color, width=5, arrow=tk.LAST, arrowshape=(14, 17, 6))
            self.create_text(p2[0] + 12, p2[1], text=label, fill=color, font=("Helvetica Neue", 16, "bold"))
        self.create_oval(origin[0] - 5, origin[1] - 5, origin[0] + 5, origin[1] + 5, fill=COLORS["warning"], outline="")

        y = height - 72
        values = (
            ("Roll X", self.ang_x, COLORS["x"]),
            ("Pitch Y", self.ang_y, COLORS["y"]),
            ("Yaw Z", self.ang_z, COLORS["z"]),
        )
        block_width = width / 3
        for index, (label, value, color) in enumerate(values):
            x = block_width * index + block_width / 2
            self.create_text(x, y, text=label, fill=COLORS["muted"], font=("Helvetica Neue", 12, "bold"))
            self.create_text(x, y + 30, text=f"{value:8.2f}°", fill=color, font=("Helvetica Neue", 22, "bold"))


class AngleGauge(tk.Canvas):
    def __init__(self, parent, title, color, **kwargs):
        super().__init__(
            parent,
            bg=COLORS["panel"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            **kwargs,
        )
        self.title = title
        self.color = color
        self.angle = 0.0
        self.calibrated = False
        self.bind("<Configure>", lambda _event: self.draw())

    def update_angle(self, angle, calibrated):
        self.angle = angle
        self.calibrated = calibrated
        self.draw()

    def draw(self):
        self.delete("all")
        width = max(self.winfo_width(), 260)
        height = max(self.winfo_height(), 150)
        self.create_rectangle(0, 0, width, height, fill=COLORS["panel"], outline="")
        self.create_text(18, 18, text=self.title, fill=COLORS["muted"], anchor="w", font=("Helvetica Neue", 12, "bold"))

        display = self.angle if self.calibrated else 0.0
        text = f"{display:+7.2f}°" if self.calibrated else "--"
        self.create_text(width / 2, 56, text=text, fill=self.color, font=("Helvetica Neue", 30, "bold"))

        bar_x0, bar_y0 = 24, 100
        bar_x1 = width - 24
        bar_h = 14
        self.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y0 + bar_h, fill=COLORS["panel_2"], outline=COLORS["border"])
        center = (bar_x0 + bar_x1) / 2
        self.create_line(center, bar_y0 - 8, center, bar_y0 + bar_h + 8, fill=COLORS["muted"])
        if self.calibrated:
            clamped = max(min(display, 90.0), -90.0)
            marker = center + (clamped / 90.0) * ((bar_x1 - bar_x0) / 2)
            self.create_rectangle(min(center, marker), bar_y0, max(center, marker), bar_y0 + bar_h, fill=self.color, outline="")
            self.create_oval(marker - 7, bar_y0 - 4, marker + 7, bar_y0 + bar_h + 4, fill=self.color, outline="")
        self.create_text(bar_x0, bar_y0 + 34, text="-90°", fill=COLORS["muted"], anchor="w", font=("Helvetica Neue", 10))
        self.create_text(center, bar_y0 + 34, text="0°", fill=COLORS["muted"], font=("Helvetica Neue", 10))
        self.create_text(bar_x1, bar_y0 + 34, text="+90°", fill=COLORS["muted"], anchor="e", font=("Helvetica Neue", 10))


class DualImuDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("双 IMU 3DoF 姿态显示")
        self.geometry("1320x900")
        self.minsize(1120, 760)
        self.configure(bg=COLORS["bg"])

        self.event_queue = queue.Queue()
        self.worker = DualBleWorker(self.event_queue)
        self.devices = []
        self.connected_names = {"A": None, "B": None}
        self.latest_angles = {"A": None, "B": None}
        self.angle_history = {"A": deque(maxlen=60), "B": deque(maxlen=60)}
        self.calibrated = False
        self.calibration_pending = False
        self.calibration_offsets = {key: 0.0 for key in ANGLE_KEYS}
        self.relative_angles = {key: 0.0 for key in ANGLE_KEYS}

        self._setup_style()
        self._build_layout()
        self.after(50, self._process_events)
        self.after(500, self._refresh_cubes)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _setup_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background=COLORS["bg"], foreground=COLORS["text"], font=("Helvetica Neue", 12))
        style.configure("Root.TFrame", background=COLORS["bg"])
        style.configure("Panel.TFrame", background=COLORS["panel"])
        style.configure("Title.TLabel", background=COLORS["bg"], foreground=COLORS["text"], font=("Helvetica Neue", 25, "bold"))
        style.configure("Subtitle.TLabel", background=COLORS["bg"], foreground=COLORS["muted"], font=("Helvetica Neue", 12))
        style.configure("PanelTitle.TLabel", background=COLORS["panel"], foreground=COLORS["text"], font=("Helvetica Neue", 15, "bold"))
        style.configure("Status.TLabel", background=COLORS["panel"], foreground=COLORS["accent"], font=("Helvetica Neue", 12, "bold"))
        style.configure("Muted.TLabel", background=COLORS["panel"], foreground=COLORS["muted"], font=("Helvetica Neue", 11))
        style.configure("Primary.TButton", background=COLORS["accent"], foreground="#082f49", borderwidth=0, focusthickness=0, padding=(14, 10), font=("Helvetica Neue", 12, "bold"))
        style.map("Primary.TButton", background=[("active", "#7dd3fc"), ("disabled", "#334155")], foreground=[("disabled", "#94a3b8")])
        style.configure("Secondary.TButton", background="#263244", foreground=COLORS["text"], borderwidth=0, padding=(14, 10), font=("Helvetica Neue", 12, "bold"))
        style.map("Secondary.TButton", background=[("active", "#334155"), ("disabled", "#1f2937")])
        style.configure("Danger.TButton", background=COLORS["danger"], foreground="#fff1f2", borderwidth=0, padding=(14, 10), font=("Helvetica Neue", 12, "bold"))
        style.map("Danger.TButton", background=[("active", "#fda4af"), ("disabled", "#334155")])
        style.configure("Calibrate.TButton", background=COLORS["accent_2"], foreground="#052e16", borderwidth=0, padding=(14, 10), font=("Helvetica Neue", 12, "bold"))
        style.map("Calibrate.TButton", background=[("active", "#86efac"), ("disabled", "#334155")], foreground=[("disabled", "#94a3b8")])
        style.configure("Treeview", background=COLORS["panel"], foreground=COLORS["text"], fieldbackground=COLORS["panel"], borderwidth=0, rowheight=34)
        style.configure("Treeview.Heading", background=COLORS["panel_2"], foreground=COLORS["muted"], relief="flat", font=("Helvetica Neue", 11, "bold"))
        style.map("Treeview", background=[("selected", "#0e7490")], foreground=[("selected", "#ecfeff")])

    def _build_layout(self):
        header = ttk.Frame(self, style="Root.TFrame")
        header.pack(fill="x", padx=24, pady=(18, 12))
        ttk.Label(header, text="双 IMU 3DoF 姿态显示", style="Title.TLabel").pack(anchor="w")
        ttk.Label(header, text="同时连接两个 WT901BLE，实时显示两个 3D 姿态立方体", style="Subtitle.TLabel").pack(anchor="w", pady=(2, 0))

        root = ttk.Frame(self, style="Root.TFrame")
        root.pack(fill="both", expand=True, padx=24, pady=(0, 24))
        root.columnconfigure(0, minsize=350)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        sidebar = ttk.Frame(root, style="Panel.TFrame")
        sidebar.grid(row=0, column=0, sticky="nsew", padx=(0, 16))
        sidebar.columnconfigure(0, weight=1)

        ttk.Label(sidebar, text="设备选择", style="PanelTitle.TLabel").grid(row=0, column=0, sticky="w", padx=16, pady=(18, 6))
        self.status_label = ttk.Label(sidebar, text="未扫描", style="Status.TLabel")
        self.status_label.grid(row=1, column=0, sticky="w", padx=16, pady=(0, 14))

        self.scan_button = ttk.Button(sidebar, text="扫描蓝牙设备", style="Primary.TButton", command=self._scan)
        self.scan_button.grid(row=2, column=0, sticky="ew", padx=16, pady=(0, 14))

        columns = ("name", "address")
        self.device_tree = ttk.Treeview(sidebar, columns=columns, show="headings", height=12)
        self.device_tree.heading("name", text="名称")
        self.device_tree.heading("address", text="地址 / UUID")
        self.device_tree.column("name", width=120, minwidth=90, stretch=False)
        self.device_tree.column("address", width=210, minwidth=180, stretch=True)
        self.device_tree.grid(row=3, column=0, sticky="nsew", padx=16, pady=(0, 14))
        sidebar.rowconfigure(3, weight=1)

        connect_grid = ttk.Frame(sidebar, style="Panel.TFrame")
        connect_grid.grid(row=4, column=0, sticky="ew", padx=16, pady=(0, 10))
        connect_grid.columnconfigure(0, weight=1)
        connect_grid.columnconfigure(1, weight=1)
        ttk.Button(connect_grid, text="连接到 A", style="Secondary.TButton", command=lambda: self._connect_selected("A")).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        ttk.Button(connect_grid, text="连接到 B", style="Secondary.TButton", command=lambda: self._connect_selected("B")).grid(row=0, column=1, sticky="ew")

        disconnect_grid = ttk.Frame(sidebar, style="Panel.TFrame")
        disconnect_grid.grid(row=5, column=0, sticky="ew", padx=16, pady=(0, 16))
        disconnect_grid.columnconfigure(0, weight=1)
        disconnect_grid.columnconfigure(1, weight=1)
        ttk.Button(disconnect_grid, text="断开 A", style="Danger.TButton", command=lambda: self._disconnect("A")).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        ttk.Button(disconnect_grid, text="断开 B", style="Danger.TButton", command=lambda: self._disconnect("B")).grid(row=0, column=1, sticky="ew")

        self.calibrate_button = ttk.Button(sidebar, text="等待稳定并校准", style="Calibrate.TButton", command=self._request_calibration)
        self.calibrate_button.grid(row=6, column=0, sticky="ew", padx=16, pady=(0, 10))
        self.calibration_label = ttk.Label(sidebar, text="校准状态：等待两个 IMU 连接", style="Muted.TLabel", wraplength=310)
        self.calibration_label.grid(row=7, column=0, sticky="ew", padx=16, pady=(0, 16))

        ttk.Label(
            sidebar,
            text="用法：先连接 A/B，保持两个 IMU 静止，点击“等待稳定并校准”。校准后下方显示 B 相对 A 的三个方向夹角。",
            style="Muted.TLabel",
            wraplength=310,
        ).grid(row=8, column=0, sticky="ew", padx=16, pady=(0, 18))

        main = ttk.Frame(root, style="Root.TFrame")
        main.grid(row=0, column=1, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)
        main.rowconfigure(1, weight=0)

        self.cubes = {
            "A": OrientationCube(main, "A"),
            "B": OrientationCube(main, "B"),
        }
        self.cubes["A"].grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.cubes["B"].grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        gauge_panel = ttk.Frame(main, style="Root.TFrame")
        gauge_panel.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(16, 0))
        for index in range(3):
            gauge_panel.columnconfigure(index, weight=1)
        self.gauges = {}
        for index, key in enumerate(ANGLE_KEYS):
            title, color = AXIS_LABELS[key]
            gauge = AngleGauge(gauge_panel, title, color, height=150)
            gauge.grid(row=0, column=index, sticky="ew", padx=(0 if index == 0 else 10, 0))
            self.gauges[key] = gauge

    def _scan(self):
        self.status_label.configure(text="正在扫描...")
        self.scan_button.configure(state="disabled")
        self.device_tree.delete(*self.device_tree.get_children())
        self.worker.scan(timeout=8)

    def _connect_selected(self, slot):
        selection = self.device_tree.selection()
        if not selection:
            messagebox.showinfo("请选择设备", "先扫描，然后在设备列表里选中一个设备。")
            return
        index = int(selection[0])
        ble_device = self.devices[index]
        other_slot = "B" if slot == "A" else "A"
        if self.connected_names.get(other_slot) == ble_device.address:
            messagebox.showwarning("设备已连接", f"这个设备已经连接到 IMU {other_slot}，请选择另一个设备。")
            return

        self.connected_names[slot] = ble_device.address
        display_name = f"{ble_device.name or 'Unknown'}  {ble_device.address}"
        self.cubes[slot].set_status("正在连接...", display_name)
        self.worker.connect(slot, ble_device)

    def _disconnect(self, slot):
        self.worker.disconnect(slot)
        self.connected_names[slot] = None
        self.latest_angles[slot] = None
        self.angle_history[slot].clear()
        self.calibrated = False
        self.calibration_pending = False
        self._update_gauges()
        self._update_calibration_label()
        self.cubes[slot].last_update = None
        self.cubes[slot].set_status("已断开", "未连接")

    def _request_calibration(self):
        if self.latest_angles["A"] is None or self.latest_angles["B"] is None:
            messagebox.showinfo("等待数据", "请先连接两个 IMU，并等待两个立方体都开始接收数据。")
            return
        self.calibration_pending = True
        self.calibrated = False
        self.calibration_label.configure(text="校准状态：等待两个 IMU 稳定...")
        self._try_calibrate()

    def _process_events(self):
        try:
            while True:
                kind, slot, payload = self.event_queue.get_nowait()
                if kind == "scan":
                    self._handle_scan(payload)
                elif kind == "scan_error":
                    self.scan_button.configure(state="normal")
                    self.status_label.configure(text="扫描失败")
                    messagebox.showerror("扫描失败", payload)
                elif kind == "connect":
                    if slot is not None:
                        self.connected_names[slot] = None
                        self.cubes[slot].last_update = None
                        self.cubes[slot].set_status(str(payload), "未连接")
                elif kind == "connect_error":
                    if slot is not None:
                        self.connected_names[slot] = None
                        self.cubes[slot].last_update = None
                        self.cubes[slot].set_status("连接失败", "未连接")
                    messagebox.showerror("连接失败", payload)
                elif kind == "data":
                    self.cubes[slot].update_data(payload)
                    self._record_angles(slot, payload)
                    self._try_calibrate()
                    self._update_relative_angles()
                elif kind == "status":
                    self.status_label.configure(text=payload)
        except queue.Empty:
            pass
        self.after(50, self._process_events)

    def _handle_scan(self, devices):
        self.devices = devices
        self.scan_button.configure(state="normal")
        self.device_tree.delete(*self.device_tree.get_children())
        for index, device in enumerate(self.devices):
            self.device_tree.insert("", "end", iid=str(index), values=(device.name or "Unknown", device.address))
        if self.devices:
            self.device_tree.selection_set("0")
            self.status_label.configure(text=f"找到 {len(self.devices)} 个设备")
        else:
            self.status_label.configure(text="没有找到设备")

    def _refresh_cubes(self):
        for cube in self.cubes.values():
            cube.refresh_age()
        self._try_calibrate()
        self._update_calibration_label()
        self.after(500, self._refresh_cubes)

    def _record_angles(self, slot, data):
        if slot not in self.latest_angles:
            return
        if any(not isinstance(data.get(key), (int, float)) for key in ANGLE_KEYS):
            return
        angles = {key: float(data[key]) for key in ANGLE_KEYS}
        self.latest_angles[slot] = angles
        self.angle_history[slot].append((time.time(), angles))

    def _is_slot_stable(self, slot):
        stable, _detail = self._slot_stability(slot)
        return stable

    def _slot_stability(self, slot):
        history = list(self.angle_history[slot])
        now = time.time()
        history = [(ts, angles) for ts, angles in history if now - ts <= STABILITY_WINDOW_SECONDS]
        detail = {
            "samples": len(history),
            "ranges": {key: None for key in ANGLE_KEYS},
            "fresh": bool(history and now - history[-1][0] <= 1.0),
        }
        if len(history) < MIN_STABILITY_SAMPLES:
            return False, detail
        if now - history[-1][0] > 1.0:
            return False, detail

        for key in ANGLE_KEYS:
            values = [angles[key] for _ts, angles in history]
            mean = circular_mean(values)
            max_deviation = max(abs(normalize_angle(value - mean)) for value in values)
            angle_range = max_deviation * 2
            detail["ranges"][key] = angle_range
            if angle_range > STABILITY_LIMITS[key]:
                return False, detail
        return True, detail

    def _both_stable(self):
        return self._is_slot_stable("A") and self._is_slot_stable("B")

    def _mean_angles(self, slot):
        history = list(self.angle_history[slot])
        now = time.time()
        recent = [angles for ts, angles in history if now - ts <= STABILITY_WINDOW_SECONDS]
        return {
            key: circular_mean([angles[key] for angles in recent])
            for key in ANGLE_KEYS
        }

    def _try_calibrate(self):
        if not self.calibration_pending:
            return
        if not self._both_stable():
            return

        mean_a = self._mean_angles("A")
        mean_b = self._mean_angles("B")
        for key in ANGLE_KEYS:
            self.calibration_offsets[key] = normalize_angle(mean_b[key] - mean_a[key])
        self.calibrated = True
        self.calibration_pending = False
        self._update_relative_angles()
        self.calibration_label.configure(text="校准状态：已校准，正在显示 B 相对 A 的夹角")

    def _update_relative_angles(self):
        if not self.calibrated:
            self._update_gauges()
            return
        angles_a = self.latest_angles["A"]
        angles_b = self.latest_angles["B"]
        if angles_a is None or angles_b is None:
            return
        for key in ANGLE_KEYS:
            raw_delta = normalize_angle(angles_b[key] - angles_a[key])
            self.relative_angles[key] = normalize_angle(raw_delta - self.calibration_offsets[key])
        self._update_gauges()

    def _update_gauges(self):
        for key, gauge in self.gauges.items():
            gauge.update_angle(self.relative_angles[key], self.calibrated)

    def _update_calibration_label(self):
        if self.calibrated:
            self.calibration_label.configure(text="校准状态：已校准，正在显示 B 相对 A 的夹角")
            return
        if self.calibration_pending:
            stable_a, detail_a = self._slot_stability("A")
            stable_b, detail_b = self._slot_stability("B")
            self.calibration_label.configure(
                text=(
                    f"校准状态：等待稳定  "
                    f"A:{self._format_stability(stable_a, detail_a)}  "
                    f"B:{self._format_stability(stable_b, detail_b)}"
                )
            )
            return
        if self.latest_angles["A"] is None or self.latest_angles["B"] is None:
            self.calibration_label.configure(text="校准状态：等待两个 IMU 连接")
        else:
            self.calibration_label.configure(text="校准状态：两个 IMU 已连接，可开始校准")

    @staticmethod
    def _format_stability(stable, detail):
        if not detail["fresh"]:
            return "无新数据"
        ranges = detail["ranges"]
        if any(ranges[key] is None for key in ANGLE_KEYS):
            return f"{detail['samples']}/{MIN_STABILITY_SAMPLES}帧"
        state = "稳" if stable else "未稳"
        return (
            f"{state} "
            f"X{ranges['AngX']:.1f}° "
            f"Y{ranges['AngY']:.1f}° "
            f"Z{ranges['AngZ']:.1f}°"
        )

    def _on_close(self):
        self.worker.stop()
        self.destroy()


if __name__ == "__main__":
    app = DualImuDashboard()
    app.mainloop()
