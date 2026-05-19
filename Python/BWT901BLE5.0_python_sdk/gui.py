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
    "bg": "#0f172a",
    "panel": "#111827",
    "panel_2": "#172033",
    "border": "#293548",
    "text": "#e5e7eb",
    "muted": "#94a3b8",
    "accent": "#38bdf8",
    "accent_2": "#34d399",
    "danger": "#fb7185",
    "warning": "#fbbf24",
    "grid": "#263244",
    "x": "#38bdf8",
    "y": "#34d399",
    "z": "#f472b6",
    "q": "#fbbf24",
}


DATA_GROUPS = {
    "加速度 g": ("AccX", "AccY", "AccZ"),
    "角速度 deg/s": ("AsX", "AsY", "AsZ"),
    "角度 deg": ("AngX", "AngY", "AngZ"),
    "纯IMU位置 m": ("PosX", "PosY", "PosZ"),
    "纯IMU速度 m/s": ("VelX", "VelY", "VelZ"),
    "磁场": ("HX", "HY", "HZ"),
    "四元数": ("Q0", "Q1", "Q2", "Q3"),
}


class ImuDeadReckoner:
    GRAVITY = 9.80665
    CALIBRATION_FRAMES = 80
    MIN_ACC_DEADBAND_G = 0.018
    MAX_ACC_DEADBAND_G = 0.16
    MIN_STATIC_ACCEL_G = 0.05
    MAX_STATIC_ACCEL_G = 0.24
    MIN_STATIC_GYRO_DPS = 2.0
    MAX_STATIC_GYRO_DPS = 12.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.linear_accel = [0.0, 0.0, 0.0]
        self.path = deque(maxlen=1200)
        self.path.append(tuple(self.position))
        self.last_time = None
        self.gravity_reference_g = None
        self.static_frames = 0
        self.is_stationary = False
        self.calibration_frames = self.CALIBRATION_FRAMES
        self.calibration_sum_g = [0.0, 0.0, 0.0]
        self.calibration_samples = []
        self.calibration_gyro_norms = []
        self.acc_deadband_g = self.MIN_ACC_DEADBAND_G
        self.static_accel_g = self.MIN_STATIC_ACCEL_G
        self.static_gyro_dps = self.MIN_STATIC_GYRO_DPS
        self.last_acc_world_g = None
        self.force_stationary = False

    @staticmethod
    def _euler_body_to_world(vector, roll, pitch, yaw):
        x, y, z = vector
        rx, ry, rz = math.radians(roll), math.radians(pitch), math.radians(yaw)

        cos_x, sin_x = math.cos(rx), math.sin(rx)
        y, z = y * cos_x - z * sin_x, y * sin_x + z * cos_x

        cos_y, sin_y = math.cos(ry), math.sin(ry)
        x, z = x * cos_y + z * sin_y, -x * sin_y + z * cos_y

        cos_z, sin_z = math.cos(rz), math.sin(rz)
        x, y = x * cos_z - y * sin_z, x * sin_z + y * cos_z
        return [x, y, z]

    @staticmethod
    def _quaternion_body_to_world(vector, q0, q1, q2, q3):
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm < 1e-9:
            return None
        w, x, y, z = q0 / norm, q1 / norm, q2 / norm, q3 / norm
        vx, vy, vz = vector

        # Rotate vector by q * v * q_conjugate.
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ]

    def update(self, data, timestamp):
        required = ("AccX", "AccY", "AccZ", "AngX", "AngY", "AngZ")
        if any(not isinstance(data.get(key), (int, float)) for key in required):
            return self.snapshot()

        previous_time = self.last_time
        self.last_time = timestamp

        acc_body_g = [float(data["AccX"]), float(data["AccY"]), float(data["AccZ"])]
        quaternion = [data.get(key) for key in ("Q0", "Q1", "Q2", "Q3")]
        if all(isinstance(value, (int, float)) for value in quaternion):
            acc_world_g = self._quaternion_body_to_world(acc_body_g, *[float(v) for v in quaternion])
        else:
            acc_world_g = None
        if acc_world_g is None:
            acc_world_g = self._euler_body_to_world(
                acc_body_g,
                float(data["AngX"]),
                float(data["AngY"]),
                float(data["AngZ"]),
            )

        if self.gravity_reference_g is None:
            self.gravity_reference_g = list(acc_world_g)

        if self.calibration_frames > 0:
            gyro_norm = math.sqrt(sum(float(data.get(key, 0.0)) ** 2 for key in ("AsX", "AsY", "AsZ")))
            self.calibration_samples.append(tuple(acc_world_g))
            self.calibration_gyro_norms.append(gyro_norm)
            for index in range(3):
                self.calibration_sum_g[index] += acc_world_g[index]
                sample_count = self.CALIBRATION_FRAMES + 1 - self.calibration_frames
                self.gravity_reference_g[index] = self.calibration_sum_g[index] / sample_count
            self.calibration_frames -= 1
            if self.calibration_frames == 0:
                self._finish_calibration()
            self.is_stationary = True
            self.velocity = [0.0, 0.0, 0.0]
            self.linear_accel = [0.0, 0.0, 0.0]
            self.last_acc_world_g = list(acc_world_g)
            return self.snapshot()

        if previous_time is None:
            return self.snapshot()

        dt = timestamp - previous_time
        if dt <= 0 or dt > 0.35:
            self.last_acc_world_g = list(acc_world_g)
            return self.snapshot()
        dt = min(dt, 0.08)

        linear_g = [
            acc_world_g[index] - self.gravity_reference_g[index]
            for index in range(3)
        ]
        gyro_norm = math.sqrt(sum(float(data.get(key, 0.0)) ** 2 for key in ("AsX", "AsY", "AsZ")))
        accel_norm = math.sqrt(sum(value * value for value in linear_g))
        jerk_g = 0.0
        if self.last_acc_world_g is not None:
            jerk_g = math.sqrt(sum((acc_world_g[index] - self.last_acc_world_g[index]) ** 2 for index in range(3)))
        self.last_acc_world_g = list(acc_world_g)

        maybe_stationary = self.force_stationary or (
            accel_norm < self.static_accel_g
            and gyro_norm < self.static_gyro_dps
            and jerk_g < self.static_accel_g * 1.8
        )
        if maybe_stationary:
            self.static_frames += 1
        else:
            self.static_frames = 0
        self.is_stationary = self.static_frames >= 4

        if self.is_stationary:
            for index in range(3):
                self.gravity_reference_g[index] = self.gravity_reference_g[index] * 0.985 + acc_world_g[index] * 0.015
            linear_g = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.path.append(tuple(self.position))
            self.linear_accel = [0.0, 0.0, 0.0]
            return self.snapshot()
        else:
            linear_g = [self._soft_deadband(value, self.acc_deadband_g) for value in linear_g]

        self.linear_accel = [
            max(min(value * self.GRAVITY, 25.0), -25.0)
            for value in linear_g
        ]
        for index in range(3):
            self.position[index] += self.velocity[index] * dt + 0.5 * self.linear_accel[index] * dt * dt
            self.velocity[index] += self.linear_accel[index] * dt
            self.velocity[index] *= math.exp(-dt / 3.5)
            if abs(self.velocity[index]) < 0.006 and accel_norm < self.static_accel_g * 1.4:
                self.velocity[index] = 0.0
        self.path.append(tuple(self.position))
        return self.snapshot()

    def _finish_calibration(self):
        if not self.calibration_samples:
            return

        mean_g = [
            sum(sample[index] for sample in self.calibration_samples) / len(self.calibration_samples)
            for index in range(3)
        ]
        residuals = [
            math.sqrt(sum((sample[index] - mean_g[index]) ** 2 for index in range(3)))
            for sample in self.calibration_samples
        ]
        mean_residual = sum(residuals) / len(residuals)
        variance = sum((value - mean_residual) ** 2 for value in residuals) / max(len(residuals), 1)
        accel_noise_g = math.sqrt(variance) + mean_residual

        gyro_mean = sum(self.calibration_gyro_norms) / max(len(self.calibration_gyro_norms), 1)
        gyro_var = sum((value - gyro_mean) ** 2 for value in self.calibration_gyro_norms) / max(len(self.calibration_gyro_norms), 1)
        gyro_noise = math.sqrt(gyro_var) + gyro_mean

        self.gravity_reference_g = mean_g
        self.acc_deadband_g = min(max(accel_noise_g * 3.0 + 0.006, self.MIN_ACC_DEADBAND_G), self.MAX_ACC_DEADBAND_G)
        self.static_accel_g = min(max(accel_noise_g * 7.0 + 0.018, self.MIN_STATIC_ACCEL_G), self.MAX_STATIC_ACCEL_G)
        self.static_gyro_dps = min(max(gyro_noise * 4.0 + 0.8, self.MIN_STATIC_GYRO_DPS), self.MAX_STATIC_GYRO_DPS)

    @staticmethod
    def _soft_deadband(value, deadband):
        magnitude = abs(value)
        if magnitude <= deadband:
            return 0.0
        return math.copysign((magnitude - deadband) * 0.72, value)

    def snapshot(self):
        drift = math.sqrt(sum(value * value for value in self.position))
        speed = math.sqrt(sum(value * value for value in self.velocity))
        accel_norm = math.sqrt(sum(value * value for value in self.linear_accel))
        return {
            "position": tuple(self.position),
            "velocity": tuple(self.velocity),
            "linear_accel": tuple(self.linear_accel),
            "path": list(self.path),
            "drift": drift,
            "speed": speed,
            "accel_norm": accel_norm,
            "stationary": self.is_stationary,
            "calibrating": self.calibration_frames > 0,
            "acc_deadband_g": self.acc_deadband_g,
            "static_accel_g": self.static_accel_g,
            "static_gyro_dps": self.static_gyro_dps,
            "force_stationary": self.force_stationary,
        }


class BleWorker:
    def __init__(self, event_queue):
        self.event_queue = event_queue
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        self.device = None
        self.connect_future = None

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def submit(self, kind, coro):
        future = asyncio.run_coroutine_threadsafe(coro, self.loop)

        def done_callback(done):
            try:
                self.event_queue.put((kind, "ok", done.result()))
            except Exception as exc:
                self.event_queue.put((kind, "error", str(exc)))

        future.add_done_callback(done_callback)
        return future

    def scan(self, timeout=8):
        return self.submit("scan", self._scan(timeout))

    async def _scan(self, timeout):
        devices = await bleak.BleakScanner.discover(timeout=timeout)
        devices.sort(key=lambda d: ((d.name or "").upper(), d.address))
        return devices

    def connect(self, ble_device):
        if self.device is not None:
            self.disconnect()
        self.connect_future = self.submit("connect", self._connect(ble_device))
        return self.connect_future

    async def _connect(self, ble_device):
        self.device = device_model.DeviceModel("WT901BLE", ble_device, self._on_data)
        self.event_queue.put(("status", "ok", "正在连接设备..."))
        try:
            await self.device.openDevice()
            return "设备已断开"
        finally:
            self.device = None

    def _on_data(self, model):
        self.event_queue.put(("data", "ok", dict(model.deviceData)))

    def disconnect(self):
        if self.device is not None:
            self.device.closeDevice()
        self.event_queue.put(("status", "ok", "正在断开..."))

    def stop(self):
        self.disconnect()
        self.loop.call_soon_threadsafe(self.loop.stop)


class AxisChart(tk.Canvas):
    def __init__(self, parent, title, keys, unit, y_hint, **kwargs):
        super().__init__(
            parent,
            bg=COLORS["panel"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            **kwargs,
        )
        self.title = title
        self.keys = keys
        self.unit = unit
        self.y_hint = y_hint
        self.history = {key: deque(maxlen=180) for key in keys}
        self.bind("<Configure>", lambda _event: self.draw())

    def add_sample(self, data):
        for key in self.keys:
            value = data.get(key)
            if isinstance(value, (int, float)):
                self.history[key].append(float(value))
            elif self.history[key]:
                self.history[key].append(self.history[key][-1])
            else:
                self.history[key].append(0.0)
        self.draw()

    def draw(self):
        self.delete("all")
        width = max(self.winfo_width(), 320)
        height = max(self.winfo_height(), 180)
        pad_left, pad_right, pad_top, pad_bottom = 46, 20, 40, 30
        plot_w = width - pad_left - pad_right
        plot_h = height - pad_top - pad_bottom

        self.create_rectangle(0, 0, width, height, fill=COLORS["panel"], outline="")
        self.create_text(
            18,
            18,
            text=self.title,
            fill=COLORS["text"],
            anchor="w",
            font=("Helvetica Neue", 14, "bold"),
        )
        self.create_text(
            width - 18,
            18,
            text=self.unit,
            fill=COLORS["muted"],
            anchor="e",
            font=("Helvetica Neue", 11),
        )

        for i in range(5):
            y = pad_top + plot_h * i / 4
            self.create_line(pad_left, y, width - pad_right, y, fill=COLORS["grid"])
        for i in range(6):
            x = pad_left + plot_w * i / 5
            self.create_line(x, pad_top, x, height - pad_bottom, fill=COLORS["grid"])

        values = []
        for key in self.keys:
            values.extend(self.history[key])
        max_abs = max([abs(v) for v in values], default=self.y_hint)
        max_abs = max(max_abs, self.y_hint, 0.001)
        y_max = max_abs * 1.15
        y_min = -y_max

        self.create_text(
            10,
            pad_top,
            text=f"{y_max:.1f}",
            fill=COLORS["muted"],
            anchor="w",
            font=("Helvetica Neue", 10),
        )
        self.create_text(
            10,
            height - pad_bottom,
            text=f"{y_min:.1f}",
            fill=COLORS["muted"],
            anchor="w",
            font=("Helvetica Neue", 10),
        )
        zero_y = pad_top + (y_max / (y_max - y_min)) * plot_h
        self.create_line(pad_left, zero_y, width - pad_right, zero_y, fill="#526078")

        color_map = {
            self.keys[0]: COLORS["x"],
            self.keys[1]: COLORS["y"] if len(self.keys) > 1 else COLORS["x"],
            self.keys[2]: COLORS["z"] if len(self.keys) > 2 else COLORS["x"],
        }

        legend_x = pad_left
        for key in self.keys:
            color = color_map.get(key, COLORS["q"])
            self.create_line(legend_x, 23, legend_x + 16, 23, fill=color, width=3)
            self.create_text(
                legend_x + 22,
                23,
                text=key,
                fill=COLORS["muted"],
                anchor="w",
                font=("Helvetica Neue", 10),
            )
            legend_x += 78

        for key in self.keys:
            series = list(self.history[key])
            if len(series) < 2:
                continue
            points = []
            for index, value in enumerate(series):
                x = pad_left + (index / (len(series) - 1)) * plot_w
                y = pad_top + ((y_max - value) / (y_max - y_min)) * plot_h
                points.extend((x, y))
            self.create_line(
                *points,
                fill=color_map.get(key, COLORS["q"]),
                width=2,
                smooth=True,
            )


class OrientationView(tk.Canvas):
    def __init__(self, parent, **kwargs):
        super().__init__(
            parent,
            bg=COLORS["panel"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            **kwargs,
        )
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        self.bind("<Configure>", lambda _event: self.draw())

    def update_angles(self, data):
        self.angle_x = float(data.get("AngX", self.angle_x) or 0.0)
        self.angle_y = float(data.get("AngY", self.angle_y) or 0.0)
        self.angle_z = float(data.get("AngZ", self.angle_z) or 0.0)
        self.draw()

    @staticmethod
    def _rotate(points, cx, cy, degrees):
        rad = math.radians(degrees)
        sin_v, cos_v = math.sin(rad), math.cos(rad)
        rotated = []
        for x, y in points:
            dx, dy = x - cx, y - cy
            rotated.append((cx + dx * cos_v - dy * sin_v, cy + dx * sin_v + dy * cos_v))
        return rotated

    def draw(self):
        self.delete("all")
        width = max(self.winfo_width(), 260)
        height = max(self.winfo_height(), 220)
        cx, cy = width / 2, height / 2 + 10
        radius = min(width, height) * 0.34

        self.create_rectangle(0, 0, width, height, fill=COLORS["panel"], outline="")
        self.create_text(
            18,
            18,
            text="实时姿态",
            fill=COLORS["text"],
            anchor="w",
            font=("Helvetica Neue", 14, "bold"),
        )

        pitch_offset = max(min(self.angle_y, 45), -45) / 45 * radius * 0.72
        roll = max(min(self.angle_x, 90), -90)
        horizon_y = cy + pitch_offset
        base = [
            (cx - radius * 2, horizon_y),
            (cx + radius * 2, horizon_y),
            (cx + radius * 2, cy - radius * 2),
            (cx - radius * 2, cy - radius * 2),
        ]
        ground = [
            (cx - radius * 2, horizon_y),
            (cx + radius * 2, horizon_y),
            (cx + radius * 2, cy + radius * 2),
            (cx - radius * 2, cy + radius * 2),
        ]
        sky = self._rotate(base, cx, cy, roll)
        earth = self._rotate(ground, cx, cy, roll)

        self.create_oval(
            cx - radius - 8,
            cy - radius - 8,
            cx + radius + 8,
            cy + radius + 8,
            fill=COLORS["panel_2"],
            outline=COLORS["border"],
            width=2,
        )
        self.create_polygon(*sum(sky, ()), fill="#0ea5e9", outline="")
        self.create_polygon(*sum(earth, ()), fill="#a16207", outline="")
        self.create_oval(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            outline=COLORS["border"],
            width=3,
        )

        for offset in (-40, -20, 20, 40):
            y = cy + pitch_offset + offset / 45 * radius
            line = self._rotate([(cx - 38, y), (cx + 38, y)], cx, cy, roll)
            self.create_line(*line[0], *line[1], fill="#dbeafe", width=2)

        self.create_line(cx - 54, cy, cx - 12, cy, fill=COLORS["warning"], width=3)
        self.create_line(cx + 12, cy, cx + 54, cy, fill=COLORS["warning"], width=3)
        self.create_polygon(
            cx,
            cy - 8,
            cx - 10,
            cy + 8,
            cx + 10,
            cy + 8,
            fill=COLORS["warning"],
            outline="",
        )

        self.create_text(
            width / 2,
            height - 36,
            text=f"Roll {self.angle_x:7.2f}   Pitch {self.angle_y:7.2f}   Yaw {self.angle_z:7.2f}",
            fill=COLORS["text"],
            font=("Helvetica Neue", 12, "bold"),
        )


class Cube3DView(tk.Canvas):
    def __init__(self, parent, **kwargs):
        super().__init__(
            parent,
            bg=COLORS["panel"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            **kwargs,
        )
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        self.bind("<Configure>", lambda _event: self.draw())

    def update_angles(self, data):
        self.angle_x = float(data.get("AngX", self.angle_x) or 0.0)
        self.angle_y = float(data.get("AngY", self.angle_y) or 0.0)
        self.angle_z = float(data.get("AngZ", self.angle_z) or 0.0)
        self.draw()

    def _rotate_point(self, point):
        x, y, z = point
        rx = math.radians(self.angle_x)
        ry = math.radians(self.angle_y)
        rz = math.radians(self.angle_z)

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
        distance = 4.2
        factor = scale / (distance - z)
        return cx + x * factor, cy - y * factor

    def _screen_point(self, point, cx, cy, scale):
        return self._project(self._rotate_point(point), cx, cy, scale)

    def draw(self):
        self.delete("all")
        width = max(self.winfo_width(), 300)
        height = max(self.winfo_height(), 220)
        cx, cy = width / 2, height / 2 + 14
        scale = min(width, height) * 1.18

        self.create_rectangle(0, 0, width, height, fill=COLORS["panel"], outline="")
        self.create_text(
            18,
            18,
            text="三轴立方体",
            fill=COLORS["text"],
            anchor="w",
            font=("Helvetica Neue", 14, "bold"),
        )
        self.create_text(
            width - 18,
            18,
            text="X / Y / Z",
            fill=COLORS["muted"],
            anchor="e",
            font=("Helvetica Neue", 11),
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
            self.create_line(*projected[a], *projected[b], fill="#cbd5e1", width=2)

        axis_specs = (
            ((0, 0, 0), (1.65, 0, 0), COLORS["x"], "X"),
            ((0, 0, 0), (0, 1.65, 0), COLORS["y"], "Y"),
            ((0, 0, 0), (0, 0, 1.65), COLORS["z"], "Z"),
        )
        origin = self._screen_point((0, 0, 0), cx, cy, scale)
        for start, end, color, label in axis_specs:
            p1 = self._screen_point(start, cx, cy, scale)
            p2 = self._screen_point(end, cx, cy, scale)
            self.create_line(*p1, *p2, fill=color, width=4, arrow=tk.LAST, arrowshape=(12, 14, 5))
            label_x = p2[0] + (p2[0] - origin[0]) * 0.08
            label_y = p2[1] + (p2[1] - origin[1]) * 0.08
            self.create_text(label_x, label_y, text=label, fill=color, font=("Helvetica Neue", 13, "bold"))

        self.create_oval(origin[0] - 4, origin[1] - 4, origin[0] + 4, origin[1] + 4, fill=COLORS["warning"], outline="")
        self.create_text(
            width / 2,
            height - 24,
            text=f"X {self.angle_x:6.2f}   Y {self.angle_y:6.2f}   Z {self.angle_z:6.2f}",
            fill=COLORS["muted"],
            font=("Helvetica Neue", 11, "bold"),
        )


class Trajectory3DView(tk.Canvas):
    def __init__(self, parent, **kwargs):
        super().__init__(
            parent,
            bg=COLORS["panel"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            **kwargs,
        )
        self.state = None
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        self.bind("<Configure>", lambda _event: self.draw())

    @staticmethod
    def _view_project(point, cx, cy, scale):
        x, y, z = point
        rz = math.radians(-38)
        rx = math.radians(58)

        cos_z, sin_z = math.cos(rz), math.sin(rz)
        x, y = x * cos_z - y * sin_z, x * sin_z + y * cos_z

        cos_x, sin_x = math.cos(rx), math.sin(rx)
        y, z = y * cos_x - z * sin_x, y * sin_x + z * cos_x
        return cx + x * scale, cy - y * scale

    def update_state(self, state, data):
        self.state = state
        self.angle_x = float(data.get("AngX", self.angle_x) or 0.0)
        self.angle_y = float(data.get("AngY", self.angle_y) or 0.0)
        self.angle_z = float(data.get("AngZ", self.angle_z) or 0.0)
        self.draw()

    def draw(self):
        self.delete("all")
        width = max(self.winfo_width(), 360)
        height = max(self.winfo_height(), 210)
        cx, cy = width / 2, height / 2 + 18

        self.create_rectangle(0, 0, width, height, fill=COLORS["panel"], outline="")
        self.create_text(
            18,
            18,
            text="纯IMU 6DoF轨迹",
            fill=COLORS["text"],
            anchor="w",
            font=("Helvetica Neue", 14, "bold"),
        )
        self.create_text(
            width - 18,
            18,
            text="无外部校正",
            fill=COLORS["danger"],
            anchor="e",
            font=("Helvetica Neue", 11, "bold"),
        )

        state = self.state or {
            "position": (0.0, 0.0, 0.0),
            "velocity": (0.0, 0.0, 0.0),
            "path": [(0.0, 0.0, 0.0)],
            "drift": 0.0,
            "speed": 0.0,
            "accel_norm": 0.0,
            "stationary": False,
            "calibrating": False,
            "acc_deadband_g": 0.0,
            "static_accel_g": 0.0,
            "static_gyro_dps": 0.0,
            "force_stationary": False,
        }
        path = state["path"]
        max_extent = 0.15
        for point in path:
            max_extent = max(max_extent, abs(point[0]), abs(point[1]), abs(point[2]))
        scale = min(width * 0.34, height * 0.38) / max_extent

        grid_size = max(max_extent, 0.5)
        for value in (-grid_size, 0, grid_size):
            a = self._view_project((-grid_size, value, 0), cx, cy, scale)
            b = self._view_project((grid_size, value, 0), cx, cy, scale)
            c = self._view_project((value, -grid_size, 0), cx, cy, scale)
            d = self._view_project((value, grid_size, 0), cx, cy, scale)
            self.create_line(*a, *b, fill=COLORS["grid"])
            self.create_line(*c, *d, fill=COLORS["grid"])

        axes = (
            ((0, 0, 0), (grid_size, 0, 0), COLORS["x"], "X"),
            ((0, 0, 0), (0, grid_size, 0), COLORS["y"], "Y"),
            ((0, 0, 0), (0, 0, grid_size), COLORS["z"], "Z"),
        )
        for start, end, color, label in axes:
            p1 = self._view_project(start, cx, cy, scale)
            p2 = self._view_project(end, cx, cy, scale)
            self.create_line(*p1, *p2, fill=color, width=3, arrow=tk.LAST, arrowshape=(10, 12, 5))
            self.create_text(p2[0] + 10, p2[1], text=label, fill=color, font=("Helvetica Neue", 12, "bold"))

        if len(path) >= 2:
            points = []
            for point in path:
                points.extend(self._view_project(point, cx, cy, scale))
            self.create_line(*points, fill=COLORS["accent"], width=2, smooth=True)

        px, py, pz = state["position"]
        marker = self._view_project((px, py, pz), cx, cy, scale)
        self.create_oval(marker[0] - 7, marker[1] - 7, marker[0] + 7, marker[1] + 7, fill=COLORS["warning"], outline="")
        if state["force_stationary"]:
            state_label = "强制静止"
            state_color = COLORS["accent_2"]
        elif state["calibrating"]:
            state_label = "校准中"
            state_color = COLORS["accent"]
        elif state["stationary"]:
            state_label = "静止锁定"
            state_color = COLORS["accent_2"]
        else:
            state_label = "积分中"
            state_color = COLORS["warning"]
        self.create_text(width - 18, height - 48, text=state_label, fill=state_color, anchor="e", font=("Helvetica Neue", 12, "bold"))
        self.create_text(
            18,
            height - 50,
            text=f"位置  X {px:8.3f} m   Y {py:8.3f} m   Z {pz:8.3f} m",
            fill=COLORS["text"],
            anchor="w",
            font=("Helvetica Neue", 12, "bold"),
        )
        self.create_text(
            18,
            height - 24,
            text=f"漂移 |p| {state['drift']:.3f} m    速度 |v| {state['speed']:.3f} m/s    线加速度 {state['accel_norm']:.3f} m/s²    阈值 {state['static_accel_g']:.3f}g/{state['static_gyro_dps']:.1f}dps",
            fill=COLORS["muted"],
            anchor="w",
            font=("Helvetica Neue", 11),
        )


class DataCard(ttk.Frame):
    def __init__(self, parent, title, keys):
        super().__init__(parent, style="Card.TFrame")
        self.keys = keys
        self.values = {}
        ttk.Label(self, text=title, style="CardTitle.TLabel").pack(anchor="w", padx=14, pady=(12, 6))
        body = ttk.Frame(self, style="Card.TFrame")
        body.pack(fill="x", padx=14, pady=(0, 12))
        for index, key in enumerate(keys):
            item = ttk.Frame(body, style="Card.TFrame")
            item.grid(row=0, column=index, sticky="ew", padx=(0, 10 if index < len(keys) - 1 else 0))
            body.columnconfigure(index, weight=1)
            ttk.Label(item, text=key, style="MetricName.TLabel").pack(anchor="w")
            value = ttk.Label(item, text="--", style="MetricValue.TLabel")
            value.pack(anchor="w")
            self.values[key] = value

    def update_values(self, data):
        for key, label in self.values.items():
            value = data.get(key)
            if isinstance(value, (int, float)):
                label.configure(text=f"{value:.3f}")
            elif value is not None:
                label.configure(text=str(value))
            else:
                label.configure(text="--")


class WitBleDashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("WT901BLE 实时监控")
        self.geometry("1320x880")
        self.minsize(1120, 760)
        self.configure(bg=COLORS["bg"])

        self.event_queue = queue.Queue()
        self.worker = BleWorker(self.event_queue)
        self.devices = []
        self.latest_data = {}
        self.last_packet_time = None
        self.scan_running = False
        self.connected = False
        self.dead_reckoner = ImuDeadReckoner()
        self.raw_charts_visible = False

        self._setup_style()
        self._build_layout()
        self.after(50, self._process_events)
        self.after(500, self._refresh_connection_age)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _setup_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background=COLORS["bg"], foreground=COLORS["text"], font=("Helvetica Neue", 12))
        style.configure("Panel.TFrame", background=COLORS["panel"])
        style.configure("Card.TFrame", background=COLORS["panel_2"], relief="flat")
        style.configure("Title.TLabel", background=COLORS["bg"], foreground=COLORS["text"], font=("Helvetica Neue", 24, "bold"))
        style.configure("Subtitle.TLabel", background=COLORS["bg"], foreground=COLORS["muted"], font=("Helvetica Neue", 12))
        style.configure("PanelTitle.TLabel", background=COLORS["panel"], foreground=COLORS["text"], font=("Helvetica Neue", 14, "bold"))
        style.configure("Muted.TLabel", background=COLORS["panel"], foreground=COLORS["muted"], font=("Helvetica Neue", 11))
        style.configure("Status.TLabel", background=COLORS["panel"], foreground=COLORS["accent"], font=("Helvetica Neue", 12, "bold"))
        style.configure("CardTitle.TLabel", background=COLORS["panel_2"], foreground=COLORS["muted"], font=("Helvetica Neue", 11, "bold"))
        style.configure("MetricName.TLabel", background=COLORS["panel_2"], foreground=COLORS["muted"], font=("Helvetica Neue", 10))
        style.configure("MetricValue.TLabel", background=COLORS["panel_2"], foreground=COLORS["text"], font=("Helvetica Neue", 15, "bold"))
        style.configure("Primary.TButton", background=COLORS["accent"], foreground="#082f49", borderwidth=0, focusthickness=0, padding=(14, 9), font=("Helvetica Neue", 12, "bold"))
        style.map("Primary.TButton", background=[("active", "#7dd3fc"), ("disabled", "#334155")], foreground=[("disabled", "#94a3b8")])
        style.configure("Secondary.TButton", background="#263244", foreground=COLORS["text"], borderwidth=0, padding=(14, 9), font=("Helvetica Neue", 12, "bold"))
        style.map("Secondary.TButton", background=[("active", "#334155"), ("disabled", "#1f2937")])
        style.configure("Danger.TButton", background=COLORS["danger"], foreground="#fff1f2", borderwidth=0, padding=(14, 9), font=("Helvetica Neue", 12, "bold"))
        style.map("Danger.TButton", background=[("active", "#fda4af"), ("disabled", "#334155")])
        style.configure("Compact.TButton", background=COLORS["panel_2"], foreground=COLORS["text"], borderwidth=0, padding=(12, 7), font=("Helvetica Neue", 11, "bold"))
        style.map("Compact.TButton", background=[("active", "#263244")])
        style.configure("Lock.TCheckbutton", background=COLORS["panel"], foreground=COLORS["text"], font=("Helvetica Neue", 11, "bold"))
        style.map("Lock.TCheckbutton", background=[("active", COLORS["panel"])], foreground=[("selected", COLORS["accent_2"])])
        style.configure("Treeview", background=COLORS["panel"], foreground=COLORS["text"], fieldbackground=COLORS["panel"], borderwidth=0, rowheight=32)
        style.configure("Treeview.Heading", background=COLORS["panel_2"], foreground=COLORS["muted"], relief="flat", font=("Helvetica Neue", 11, "bold"))
        style.map("Treeview", background=[("selected", "#0e7490")], foreground=[("selected", "#ecfeff")])

    def _build_layout(self):
        header = ttk.Frame(self, style="TFrame")
        header.pack(fill="x", padx=22, pady=(18, 12))
        ttk.Label(header, text="WT901BLE 实时监控", style="Title.TLabel").pack(anchor="w")
        ttk.Label(header, text="蓝牙连接、实时数据和三轴可视化", style="Subtitle.TLabel").pack(anchor="w", pady=(2, 0))

        root = ttk.Frame(self, style="TFrame")
        root.pack(fill="both", expand=True, padx=22, pady=(0, 22))
        root.columnconfigure(0, minsize=330)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        sidebar = ttk.Frame(root, style="Panel.TFrame")
        sidebar.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        sidebar.columnconfigure(0, weight=1)

        ttk.Label(sidebar, text="设备", style="PanelTitle.TLabel").grid(row=0, column=0, sticky="w", padx=16, pady=(16, 4))
        self.status_label = ttk.Label(sidebar, text="未连接", style="Status.TLabel")
        self.status_label.grid(row=1, column=0, sticky="w", padx=16, pady=(0, 12))

        buttons = ttk.Frame(sidebar, style="Panel.TFrame")
        buttons.grid(row=2, column=0, sticky="ew", padx=16, pady=(0, 12))
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        self.scan_button = ttk.Button(buttons, text="扫描", style="Primary.TButton", command=self._scan)
        self.scan_button.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        self.connect_button = ttk.Button(buttons, text="连接", style="Secondary.TButton", command=self._connect_selected)
        self.connect_button.grid(row=0, column=1, sticky="ew")

        self.disconnect_button = ttk.Button(sidebar, text="断开连接", style="Danger.TButton", command=self._disconnect, state="disabled")
        self.disconnect_button.grid(row=3, column=0, sticky="ew", padx=16, pady=(0, 14))

        self.reset_imu_button = ttk.Button(sidebar, text="重置/校准6DoF", style="Secondary.TButton", command=self._reset_dead_reckoning)
        self.reset_imu_button.grid(row=4, column=0, sticky="ew", padx=16, pady=(0, 14))

        self.force_stationary_var = tk.BooleanVar(value=False)
        self.force_stationary_check = ttk.Checkbutton(
            sidebar,
            text="强制静止锁定",
            variable=self.force_stationary_var,
            style="Lock.TCheckbutton",
        )
        self.force_stationary_check.grid(row=5, column=0, sticky="w", padx=16, pady=(0, 14))

        columns = ("name", "address")
        self.device_tree = ttk.Treeview(sidebar, columns=columns, show="headings", height=9)
        self.device_tree.heading("name", text="名称")
        self.device_tree.heading("address", text="地址 / UUID")
        self.device_tree.column("name", width=120, minwidth=90, stretch=False)
        self.device_tree.column("address", width=190, minwidth=170, stretch=True)
        self.device_tree.grid(row=6, column=0, sticky="nsew", padx=16, pady=(0, 14))
        sidebar.rowconfigure(6, weight=1)

        ttk.Label(sidebar, text="提示：静止测试时可打开强制静止锁定；移动前请关闭。macOS 上显示的通常是 UUID。", style="Muted.TLabel", wraplength=286).grid(row=7, column=0, sticky="ew", padx=16, pady=(0, 16))

        self.orientation = OrientationView(sidebar, height=250)
        self.orientation.grid(row=8, column=0, sticky="ew", padx=16, pady=(0, 16))

        content = ttk.Frame(root, style="TFrame")
        content.grid(row=0, column=1, sticky="nsew")
        content.columnconfigure(0, weight=1)
        content.rowconfigure(1, weight=1)

        top_panel = ttk.Frame(content, style="TFrame")
        top_panel.grid(row=0, column=0, sticky="ew")
        top_panel.columnconfigure(0, weight=3)
        top_panel.columnconfigure(1, weight=2)

        cards = ttk.Frame(top_panel, style="TFrame")
        cards.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        for i in range(3):
            cards.columnconfigure(i, weight=1)
        self.cards = {}
        card_items = list(DATA_GROUPS.items())
        for index, (title, keys) in enumerate(card_items):
            row = index // 3
            col = index % 3
            cards.rowconfigure(row, weight=1)
            card = DataCard(cards, title, keys)
            card.grid(row=row, column=col, sticky="ew", padx=(0 if col == 0 else 10, 0), pady=(0 if row == 0 else 10, 10))
            self.cards[title] = card

        self.cube = Cube3DView(top_panel, height=252)
        self.cube.grid(row=0, column=1, sticky="nsew", pady=(0, 10))

        charts = ttk.Frame(content, style="TFrame")
        charts.grid(row=1, column=0, sticky="nsew")
        charts.columnconfigure(0, weight=1)
        charts.rowconfigure(0, weight=1)
        charts.rowconfigure(1, weight=0)
        charts.rowconfigure(2, weight=0)

        self.trajectory = Trajectory3DView(charts, height=360)
        self.trajectory.grid(row=0, column=0, sticky="nsew", pady=(0, 10))

        raw_header = ttk.Frame(charts, style="TFrame")
        raw_header.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        raw_header.columnconfigure(0, weight=1)
        ttk.Label(raw_header, text="原始曲线", style="Subtitle.TLabel").grid(row=0, column=0, sticky="w")
        self.toggle_charts_button = ttk.Button(raw_header, text="展开原始曲线", style="Compact.TButton", command=self._toggle_raw_charts)
        self.toggle_charts_button.grid(row=0, column=1, sticky="e")

        self.raw_charts = ttk.Frame(charts, style="TFrame")
        self.raw_charts.columnconfigure(0, weight=1)
        self.raw_charts.rowconfigure(0, weight=1)
        self.raw_charts.rowconfigure(1, weight=1)
        self.raw_charts.rowconfigure(2, weight=1)
        self.acc_chart = AxisChart(self.raw_charts, "加速度", ("AccX", "AccY", "AccZ"), "g", 1.0, height=150)
        self.gyro_chart = AxisChart(self.raw_charts, "角速度", ("AsX", "AsY", "AsZ"), "deg/s", 50.0, height=150)
        self.angle_chart = AxisChart(self.raw_charts, "角度", ("AngX", "AngY", "AngZ"), "deg", 30.0, height=150)
        self.acc_chart.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        self.gyro_chart.grid(row=1, column=0, sticky="nsew", pady=(0, 8))
        self.angle_chart.grid(row=2, column=0, sticky="nsew")

    def _scan(self):
        if self.scan_running:
            return
        self.scan_running = True
        self.status_label.configure(text="正在扫描蓝牙设备...")
        self.scan_button.configure(state="disabled")
        self.device_tree.delete(*self.device_tree.get_children())
        self.worker.scan(timeout=8)

    def _connect_selected(self):
        selection = self.device_tree.selection()
        if not selection:
            messagebox.showinfo("请选择设备", "先点击扫描，然后在列表里选择一个设备。")
            return
        index = int(selection[0])
        ble_device = self.devices[index]
        self._reset_dead_reckoning()
        self.status_label.configure(text=f"连接 {ble_device.name or 'Unknown'}...")
        self.connect_button.configure(state="disabled")
        self.scan_button.configure(state="disabled")
        self.worker.connect(ble_device)

    def _disconnect(self):
        self.worker.disconnect()
        self.disconnect_button.configure(state="disabled")

    def _toggle_raw_charts(self):
        self.raw_charts_visible = not self.raw_charts_visible
        if self.raw_charts_visible:
            self.raw_charts.grid(row=2, column=0, sticky="nsew")
            self.raw_charts.master.rowconfigure(2, weight=1)
            self.toggle_charts_button.configure(text="收起原始曲线")
        else:
            self.raw_charts.grid_remove()
            self.raw_charts.master.rowconfigure(2, weight=0)
            self.toggle_charts_button.configure(text="展开原始曲线")

    def _reset_dead_reckoning(self):
        self.dead_reckoner.reset()
        self.dead_reckoner.force_stationary = self.force_stationary_var.get()
        self.latest_data.update({
            "PosX": 0.0,
            "PosY": 0.0,
            "PosZ": 0.0,
            "VelX": 0.0,
            "VelY": 0.0,
            "VelZ": 0.0,
        })
        state = self.dead_reckoner.snapshot()
        self.trajectory.update_state(state, self.latest_data)
        for card in self.cards.values():
            card.update_values(self.latest_data)

    def _process_events(self):
        try:
            while True:
                kind, status, payload = self.event_queue.get_nowait()
                if kind == "scan":
                    self._handle_scan(status, payload)
                elif kind == "connect":
                    self._handle_connect_done(status, payload)
                elif kind == "data":
                    self._handle_data(payload)
                elif kind == "status":
                    self.status_label.configure(text=payload)
        except queue.Empty:
            pass
        self.after(50, self._process_events)

    def _handle_scan(self, status, payload):
        self.scan_running = False
        self.scan_button.configure(state="normal")
        if status == "error":
            self.status_label.configure(text="扫描失败")
            messagebox.showerror("扫描失败", payload)
            return

        self.devices = [device for device in payload if device.name]
        wt_devices = [device for device in self.devices if "WT" in (device.name or "").upper()]
        if wt_devices:
            self.devices = wt_devices

        self.device_tree.delete(*self.device_tree.get_children())
        for index, device in enumerate(self.devices):
            self.device_tree.insert("", "end", iid=str(index), values=(device.name or "Unknown", device.address))

        if self.devices:
            self.device_tree.selection_set("0")
            self.status_label.configure(text=f"找到 {len(self.devices)} 个设备")
        else:
            self.status_label.configure(text="没有找到设备")

    def _handle_connect_done(self, status, payload):
        self.connected = False
        self.connect_button.configure(state="normal")
        self.scan_button.configure(state="normal")
        self.disconnect_button.configure(state="disabled")
        if status == "error":
            self.status_label.configure(text="连接失败")
            messagebox.showerror("连接失败", payload)
        else:
            self.status_label.configure(text=str(payload))

    def _handle_data(self, data):
        if not self.connected:
            self.connected = True
            self.disconnect_button.configure(state="normal")
            self.connect_button.configure(state="disabled")
            self.scan_button.configure(state="disabled")
        self.latest_data.update(data)
        self.last_packet_time = time.time()
        self.status_label.configure(text="已连接，正在接收数据")
        self.dead_reckoner.force_stationary = self.force_stationary_var.get()
        inertial_state = self.dead_reckoner.update(self.latest_data, self.last_packet_time)
        position = inertial_state["position"]
        velocity = inertial_state["velocity"]
        self.latest_data.update({
            "PosX": position[0],
            "PosY": position[1],
            "PosZ": position[2],
            "VelX": velocity[0],
            "VelY": velocity[1],
            "VelZ": velocity[2],
        })

        for card in self.cards.values():
            card.update_values(self.latest_data)
        self.acc_chart.add_sample(self.latest_data)
        self.gyro_chart.add_sample(self.latest_data)
        self.angle_chart.add_sample(self.latest_data)
        self.orientation.update_angles(self.latest_data)
        self.cube.update_angles(self.latest_data)
        self.trajectory.update_state(inertial_state, self.latest_data)

    def _refresh_connection_age(self):
        if self.connected and self.last_packet_time is not None:
            age = time.time() - self.last_packet_time
            if age > 3:
                self.status_label.configure(text=f"已连接，{age:.0f}s 未收到新数据")
        self.after(500, self._refresh_connection_age)

    def _on_close(self):
        self.worker.stop()
        self.destroy()


if __name__ == "__main__":
    app = WitBleDashboard()
    app.mainloop()
