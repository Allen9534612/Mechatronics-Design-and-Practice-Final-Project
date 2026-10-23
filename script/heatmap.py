"""平衡球平台監控介面（OpenGL，可縮放視窗）。

左側顯示資料流數值（感測器、位置估測、PID、伺服），中間為距離熱圖，右側為
可調項與提示。指令經 pyserial 送給 ESP32，顯示值以韌體回報的 STAT 為準。

用法：
    python script/heatmap.py [--port COM5] [--demo] [--gpu high]

快捷鍵：4/8 解析度、R 自動平衡、L 回水平、/ 輸入指令、F11 全螢幕、Esc 離開。
"""

import argparse
import ctypes
import math
import os
import random
import re
import subprocess
import sys
import threading
import time
import traceback
from collections import deque

# 必須在 pygame 初始化前設定，否則 Windows 會把視窗位圖放大而變模糊。
os.environ.setdefault("SDL_WINDOWS_DPI_AWARENESS", "permonitorv2")

import moderngl
import numpy as np
import pygame
import serial
import serial.tools.list_ports

BAUD_RATE = 115200

# 顯示用常數，需與 include/Config.h 一致。
SERVO_MIN_DEG = 40.0
SERVO_MAX_DEG = 140.0
SERVO_CHANNELS = (0, 1, 2)
# 平台高度範圍（mm），由 PlatformKinematics 的 minHeight/maxHeight 算出，
# 需與機構尺寸一致；超出範圍韌體會拒絕。
HEIGHT_MIN_MM = 61.0
HEIGHT_MAX_MM = 102.0
MIN_SPAN_MM = 40  # 色階最小跨距，避免畫面接近平坦時對比被放大成雜訊

# 欄位順序需與 src/main.cpp 的 printStat() 一致。
STAT_FIELDS = [
    "mode",
    "present",
    "x",
    "y",
    "theta",
    "phi",
    "s1",
    "s2",
    "s3",
    "kp",
    "ki",
    "kd",
    "tx",
    "ty",
    "height",
    "max_tilt",
    "dist",
    "zones",
    "int_x",
    "int_y",
]
MODE_NAMES = ["PID 自動平衡", "指定傾角", "直接角度"]

DESIGN_W, DESIGN_H = 1280, 760

# --------------------------------------------------------------------------
# 顯卡與 DPI
# --------------------------------------------------------------------------

GPU_PREF_KEY = r"Software\Microsoft\DirectX\UserGpuPreferences"
GPU_LEVELS = {"default": 0, "low": 1, "high": 2}
INTEGRATED_HINTS = (
    "intel",
    "uhd",
    "iris",
    "radeon graphics",
    "microsoft basic",
    "llvmpipe",
    "software",
)
DISCRETE_HINTS = ("nvidia", "geforce", "quadro", "radeon rx", "radeon pro")


def dpi_scale():
    """回傳系統 DPI 縮放倍率（非 Windows 為 1.0）。"""
    if sys.platform != "win32":
        return 1.0
    try:
        return ctypes.windll.user32.GetDpiForSystem() / 96.0
    except (AttributeError, OSError):
        return 1.0


def detect_gpus():
    """列出系統上的顯示卡名稱，失敗時回傳空 list。"""
    try:
        if sys.platform == "win32":
            cmd = [
                "powershell",
                "-NoProfile",
                "-Command",
                "Get-CimInstance Win32_VideoController | ForEach-Object { $_.Name }",
            ]
        elif sys.platform == "darwin":
            cmd = [
                "sh",
                "-c",
                "system_profiler SPDisplaysDataType | sed -n 's/.*Chipset Model: //p'",
            ]
        else:
            cmd = ["sh", "-c", "lspci | grep -Ei 'vga|3d|display' | cut -d: -f3-"]
        out = subprocess.run(
            cmd,
            capture_output=True,
            check=False,
            timeout=10,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        text = out.stdout.decode("utf-8", errors="replace")
        return [ln.strip() for ln in text.splitlines() if ln.strip()]
    except (OSError, subprocess.SubprocessError):
        return []


def is_integrated(name):
    name = re.sub(r"\((tm|r)\)", "", name.lower())
    return any(k in name for k in INTEGRATED_HINTS) and not any(
        k in name for k in DISCRETE_HINTS
    )


def get_gpu_preference(exe):
    """讀取 Windows 為 exe 設定的圖形偏好（0 預設、1 省電、2 高效能）。"""
    if sys.platform != "win32":
        return None
    import winreg

    try:
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, GPU_PREF_KEY) as key:
            value, _ = winreg.QueryValueEx(key, exe)
    except OSError:
        return None
    for part in str(value).split(";"):
        if part.startswith("GpuPreference="):
            try:
                return int(part.split("=")[1])
            except ValueError:
                return None
    return None


def set_gpu_preference(exe, level):
    """設定 Windows 的圖形偏好，下次啟動該 exe 時生效。"""
    import winreg

    with winreg.CreateKey(winreg.HKEY_CURRENT_USER, GPU_PREF_KEY) as key:
        winreg.SetValueEx(key, exe, 0, winreg.REG_SZ, f"GpuPreference={level};")


def apply_gpu_option(choice):
    """依 --gpu 設定圖形偏好；設定有變更時以新偏好重新啟動自己。"""
    if choice == "auto" or sys.platform != "win32":
        if choice != "auto":
            print("--gpu 只支援 Windows，已忽略。")
        return
    level = GPU_LEVELS[choice]
    exe = sys.executable
    if get_gpu_preference(exe) == level or os.environ.get("HEATMAP_GPU_SET"):
        return
    set_gpu_preference(exe, level)
    print(f"已將 {exe} 的圖形偏好設為 {choice}，重新啟動中…")
    sys.exit(
        subprocess.call(
            [sys.executable] + sys.argv, env=dict(os.environ, HEATMAP_GPU_SET="1")
        )
    )


# --------------------------------------------------------------------------
# 資料與序列埠
# --------------------------------------------------------------------------


class Telemetry:
    """序列埠執行緒寫入、GUI 讀取的共用狀態。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.grid_n = 0
        self.dist = np.zeros((8, 8))
        self.cx = -1.0
        self.cy = -1.0
        self.frame_times = deque(maxlen=64)
        self.stat = None
        self.stat_time = 0.0
        self.log = deque(maxlen=200)
        self.status = "尚未連線"
        self.connected = False

    def add_log(self, text):
        # 序列埠雜訊可能夾帶空字元等控制字元，繪製文字時會拋出例外。
        text = "".join(ch for ch in text if ch.isprintable())
        if not text:
            return
        with self.lock:
            self.log.append(text)

    def push_data(self, n, dist, cx, cy):
        with self.lock:
            self.grid_n = n
            self.dist[:n, :n] = np.asarray(dist).reshape(n, n)
            self.cx, self.cy = cx, cy
            self.frame_times.append(time.time())

    def push_stat(self, stat):
        with self.lock:
            self.stat = stat
            self.stat_time = time.time()

    def snapshot(self):
        """回傳目前狀態的複本，供 GUI 一次讀取。"""
        with self.lock:
            now = time.time()
            recent = [t for t in self.frame_times if now - t <= 1.0]
            fps = 0.0
            if len(recent) >= 2:
                fps = (len(recent) - 1) / max(recent[-1] - recent[0], 1e-3)
            return {
                "n": self.grid_n,
                "dist": self.dist.copy(),
                "cx": self.cx,
                "cy": self.cy,
                "fps": fps,
                "stat": dict(self.stat) if self.stat else None,
                "stat_age": now - self.stat_time if self.stat else None,
                "log": list(self.log)[-40:],
                "status": self.status,
                "connected": self.connected,
            }


def parse_line(tel, line):
    """解析韌體的一行：DATA 與 STAT 更新狀態，其餘記為訊息。"""
    line = line.strip()
    if not line:
        return
    try:
        if line.startswith("DATA,"):
            p = line.split(",")
            n = int(p[1])
            if n in (4, 8) and len(p) == 2 + n * n + 2:
                tel.push_data(
                    n, [float(v) for v in p[2 : 2 + n * n]], float(p[-2]), float(p[-1])
                )
            return
        if line.startswith("STAT,"):
            p = line.split(",")
            if len(p) == 1 + len(STAT_FIELDS):
                tel.push_stat(dict(zip(STAT_FIELDS, map(float, p[1:]))))
            return
    except ValueError:
        return
    tel.add_log(line)


def find_port():
    keywords = ["CP210", "CH340", "UART", "SILICON LABS", "USB-SERIAL"]
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if any(k in p.description.upper() for k in keywords):
            return p.device
    return ports[0].device if ports else None


class SerialLink:
    """以背景執行緒收發序列埠資料，斷線時自動重連。"""

    def __init__(self, tel, port=None, baud=BAUD_RATE):
        self.tel, self.port, self.baud = tel, port, baud
        self.ser = None
        self.running = True
        self.wlock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass

    def send(self, cmd):
        """送出一行指令，成功回傳 True。"""
        with self.wlock:
            if not (self.ser and self.ser.is_open):
                self.tel.add_log(f"[未連線] 無法送出: {cmd}")
                return False
            try:
                self.ser.write((cmd + "\n").encode("utf-8"))
                self.ser.flush()
            except serial.SerialException as e:
                self.tel.add_log(f"[送出失敗] {e}")
                return False
        self.tel.add_log(f"→ {cmd}")
        return True

    def _run(self):
        tel = self.tel
        while self.running:
            if not self.ser:
                port = self.port or find_port()
                if not port:
                    tel.status, tel.connected = "找不到序列埠，重試中…", False
                    time.sleep(1.0)
                    continue
                try:
                    self.ser = serial.Serial(
                        port, self.baud, timeout=0.1, write_timeout=0.5
                    )
                    tel.status, tel.connected = f"已連線 {port}", True
                    tel.add_log(f"[已開啟 {port}]")
                except serial.SerialException as e:
                    tel.status, tel.connected = f"無法開啟 {port}", False
                    tel.add_log(f"[錯誤] {e}")
                    time.sleep(1.0)
                    continue
            try:
                raw = self.ser.readline()
                if raw:
                    parse_line(tel, raw.decode("utf-8", errors="ignore"))
            except (serial.SerialException, OSError):
                tel.status, tel.connected = "連線中斷，重試中…", False
                try:
                    self.ser.close()
                except (serial.SerialException, OSError):
                    pass
                self.ser = None
                time.sleep(1.0)


class DemoLink:
    """模擬裝置：產生與韌體相同格式的資料，並回應同樣的指令。"""

    LEG_ANGLES = (120.0, 0.0, 240.0)
    LEVEL = (90.0, 85.5, 86.0)

    def __init__(self, tel):
        self.tel = tel
        self.running = True
        self.lock = threading.Lock()
        self.n = 4
        self.mode = 0
        self.kp, self.ki, self.kd = 0.12, 0.01, 0.06
        self.tx = self.ty = 0.0
        self.height = 82.6
        self.max_tilt = 15.0
        self.theta = self.phi = 0.0
        self.bx, self.by, self.vx, self.vy = 40.0, -25.0, 0.0, 0.0
        self.ix = self.iy = 0.0
        self.off_until = 0.0
        self.raw = None  # 直接角度模式下的三顆伺服角度。
        self.offsets = [0.0, -4.5, -4.0]
        self.thread = threading.Thread(target=self._run, daemon=True)
        tel.status, tel.connected = "DEMO 模擬資料", True

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)

    def send(self, cmd):
        self.tel.add_log(f"→ {cmd}")
        p = cmd.split()
        try:
            with self.lock:
                c = p[0].lower() if p else ""
                if c == "4":
                    self.n = 4
                elif c == "8":
                    self.n = 8
                elif c == "r":
                    self.mode, self.ix, self.iy = 0, 0.0, 0.0
                    self.raw = None
                    self.tel.add_log(">> 恢復紅外線陣列自動控制")
                elif c == "l":
                    self.mode, self.theta, self.phi = 1, 0.0, 0.0
                    self.raw = None
                    self.tel.add_log(">> 回水平")
                elif c == "n":
                    self.off_until = time.time() + 2.0
                    self.tel.add_log(">> 球已移開，平台慢慢回水平")
                elif c == "p" and len(p) == 4:
                    self.kp, self.ki, self.kd = map(float, p[1:])
                    self.ix = self.iy = 0.0
                    self.tel.add_log(
                        f">> PID kp={self.kp:.4f} ki={self.ki:.4f} kd={self.kd:.4f}"
                    )
                elif c == "g" and len(p) == 3:
                    self.tx, self.ty = float(p[1]), float(p[2])
                    self.tel.add_log(f">> 目標點 ({self.tx:.1f}, {self.ty:.1f}) mm")
                elif c == "h" and len(p) == 2:
                    self.height = float(p[1])
                    self.tel.add_log(f">> 高度 {self.height:.1f} mm")
                elif c == "t" and len(p) == 3:
                    th, ph = float(p[1]), float(p[2])
                    if 0 <= th <= self.max_tilt:
                        self.mode, self.theta, self.phi = 1, th, ph
                        self.raw = None
                        self.tel.add_log(f">> 傾斜 {th:.1f} 度，往 {ph:.1f} 度方向往下")
                    else:
                        self.tel.add_log(
                            f">> 錯誤：傾角需在 0 ~ {self.max_tilt:.1f} 度之間"
                        )
                elif c == "d" and len(p) == 4:
                    v = [float(x) for x in p[1:]]
                    if all(SERVO_MIN_DEG <= a <= SERVO_MAX_DEG for a in v):
                        self.mode, self.raw, self.theta = 2, v, 0.0
                        self.tel.add_log(
                            f">> 直接角度 {v[0]:.1f} / {v[1]:.1f} / {v[2]:.1f}"
                        )
                    else:
                        self.tel.add_log(
                            f">> 錯誤：角度需在 {SERVO_MIN_DEG:.0f} ~ {SERVO_MAX_DEG:.0f} 之間"
                        )
                elif c == "o":
                    if len(p) > 1 and p[1].lower() == "r":
                        self.offsets = [0.0, -4.5, -4.0]
                        self.tel.add_log(">> 已還原預設原點並清除儲存值")
                    elif self.mode != 2 or self.raw is None:
                        self.tel.add_log(">> 請先用 d 指令把平台調到水平，再寫入原點")
                    else:
                        self.offsets = [a - 90.0 for a in self.raw]
                        self.tel.add_log(
                            f">> 原點已寫入（斷電保留）：水平角度 {self.raw[0]:.1f} / {self.raw[1]:.1f} / {self.raw[2]:.1f}"
                        )
                else:
                    self.tel.add_log(">> (demo) 不支援這個指令")
        except ValueError:
            self.tel.add_log(">> (demo) 格式錯誤")
        return True

    def _servos(self):
        if self.raw:
            return list(self.raw)
        return [
            b + 3.0 * self.theta * math.cos(math.radians(self.phi - a))
            for a, b in zip(self.LEG_ANGLES, self.LEVEL)
        ]

    def _run(self):
        dt = 1.0 / 30
        last_stat = 0.0
        while self.running:
            t0 = time.time()
            with self.lock:
                present = (
                    t0 > self.off_until and abs(self.bx) < 70 and abs(self.by) < 70
                )
                if self.mode == 0:
                    if present:
                        ex, ey = self.bx - self.tx, self.by - self.ty
                        self.ix = max(-300, min(300, self.ix + ex * dt))
                        self.iy = max(-300, min(300, self.iy + ey * dt))
                        ux = -(self.kp * ex + self.ki * self.ix + self.kd * self.vx)
                        uy = -(self.kp * ey + self.ki * self.iy + self.kd * self.vy)
                        mag = math.hypot(ux, uy)
                        self.theta = min(mag, self.max_tilt)
                        self.phi = (
                            math.degrees(math.atan2(uy, ux)) % 360
                            if mag > 1e-4
                            else 0.0
                        )
                    else:
                        self.theta *= 0.9
                a = 9810.0 * 5.0 / 7.0 * math.sin(math.radians(self.theta)) * 0.15
                self.vx = (self.vx + a * math.cos(math.radians(self.phi)) * dt) * 0.99
                self.vy = (self.vy + a * math.sin(math.radians(self.phi)) * dt) * 0.99
                self.bx += self.vx * dt
                self.by += self.vy * dt
                if (
                    not present
                    and t0 > self.off_until
                    and (abs(self.bx) >= 70 or abs(self.by) >= 70)
                ):
                    self.off_until = t0 + 1.5
                    self.bx, self.by = random.uniform(-45, 45), random.uniform(-45, 45)
                    self.vx = self.vy = 0.0
                n = self.n
                pitch = 60.0 / n
                gx = (n - 1) / 2 + math.degrees(math.atan2(self.bx, 300)) / pitch
                gy = (n - 1) / 2 + math.degrees(math.atan2(self.by, 300)) / pitch
                d = np.full((n, n), 300.0) + np.random.normal(0, 1.5, (n, n))
                yy, xx = np.mgrid[0:n, 0:n]
                if present:
                    d = d - 55.0 * np.exp(-((xx - gx) ** 2 + (yy - gy) ** 2) / 2.0)
                d[np.random.rand(n, n) < 0.03] = 0
                cx, cy = (gx, gy) if present else (-1.0, -1.0)
                stat = None
                if t0 - last_stat >= 0.1:
                    last_stat = t0
                    s = self._servos()
                    stat = dict(
                        zip(
                            STAT_FIELDS,
                            [
                                self.mode,
                                1 if present else 0,
                                self.bx,
                                self.by,
                                self.theta,
                                self.phi,
                                s[0],
                                s[1],
                                s[2],
                                self.kp,
                                self.ki,
                                self.kd,
                                self.tx,
                                self.ty,
                                self.height,
                                self.max_tilt,
                                245.0 if present else 300.0,
                                int((d > 0).sum()),
                                self.ix,
                                self.iy,
                            ],
                        )
                    )
            self.tel.push_data(n, d, cx, cy)
            if stat:
                self.tel.push_stat(stat)
            time.sleep(max(0.0, dt - (time.time() - t0)))


# --------------------------------------------------------------------------
# OpenGL 2D 繪圖
# --------------------------------------------------------------------------

# 配色：中性灰階 + 單一強調色，僅在警示時使用紅色。
BG = (15, 16, 19)
CARD = (24, 26, 31)
CARD_HI = (36, 39, 47)
HAIR = (44, 48, 58)
TEXT = (238, 240, 244)
DIM = (125, 131, 144)
ACCENT = (86, 156, 255)
DANGER = (255, 99, 99)

VS_COLOR = """
#version 330
uniform vec2 u_res;
in vec2 in_pos;
in vec4 in_col;
out vec4 v_col;
void main() {
    vec2 p = in_pos / u_res * 2.0 - 1.0;
    gl_Position = vec4(p.x, -p.y, 0.0, 1.0);
    v_col = in_col;
}
"""
FS_COLOR = """
#version 330
in vec4 v_col;
out vec4 f_col;
void main() { f_col = v_col; }
"""
VS_TEX = """
#version 330
uniform vec2 u_res;
in vec2 in_pos;
in vec2 in_uv;
out vec2 v_uv;
void main() {
    vec2 p = in_pos / u_res * 2.0 - 1.0;
    gl_Position = vec4(p.x, -p.y, 0.0, 1.0);
    v_uv = in_uv;
}
"""
FS_TEX = """
#version 330
uniform sampler2D tex;
uniform vec4 u_tint;
in vec2 v_uv;
out vec4 f_col;
void main() {
    float a = texture(tex, v_uv).a;
    f_col = vec4(u_tint.rgb, u_tint.a * a);
}
"""


def rgba(c, a=1.0):
    return (c[0] / 255.0, c[1] / 255.0, c[2] / 255.0, a)


def find_font_paths():
    """回傳 (一般, 粗體) 字型路徑，優先選有中文字形的字型。"""
    regular = [
        "C:/Windows/Fonts/msjh.ttc",
        "C:/Windows/Fonts/msyh.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    ]
    bold = [
        "C:/Windows/Fonts/msjhbd.ttc",
        "C:/Windows/Fonts/msyhbd.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    ]
    reg = next((p for p in regular if os.path.exists(p)), None)
    if reg is None:
        reg = pygame.font.match_font(
            "microsoftjhenghei,microsoftyahei,notosanscjktc,simsun,arial"
        )
    return reg, next((p for p in bold if os.path.exists(p)), None) or reg


class Renderer:
    """批次繪製色塊、線條與圓；文字轉成貼圖後以 OpenGL 繪出。"""

    CACHE_LIMIT = 4000  # 文字貼圖快取上限（每幀開頭檢查，約數十 MB 顯示記憶體）。

    def __init__(self, ctx):
        self.ctx = ctx
        self.prog = ctx.program(vertex_shader=VS_COLOR, fragment_shader=FS_COLOR)
        self.tprog = ctx.program(vertex_shader=VS_TEX, fragment_shader=FS_TEX)
        self.tvbo = ctx.buffer(reserve=6 * 4 * 4, dynamic=True)
        self.tvao = ctx.vertex_array(
            self.tprog, [(self.tvbo, "2f 2f", "in_pos", "in_uv")]
        )
        self.verts = []
        self.texts = []
        self.tex_cache = {}
        self.fonts = {}
        self.font_reg, self.font_bold = find_font_paths()
        self.size = (1, 1)
        ctx.enable(moderngl.BLEND)
        ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA

    def begin(self, w, h):
        self.size = (w, h)
        self.ctx.viewport = (0, 0, w, h)
        self.ctx.clear(*rgba(BG))
        self.verts.clear()
        self.texts.clear()
        # 只在幀開頭整理快取；繪製途中釋放貼圖會讓本幀已排入的文字失效。
        if len(self.tex_cache) > self.CACHE_LIMIT:
            for tex, _, _ in self.tex_cache.values():
                tex.release()
            self.tex_cache.clear()

    def _tri(self, pts, col):
        for x, y in pts:
            self.verts.extend((x, y, *col))

    def rect(self, x, y, w, h, col, a=1.0):
        self._tri(
            [(x, y), (x + w, y), (x, y + h), (x + w, y), (x + w, y + h), (x, y + h)],
            rgba(col, a),
        )

    def rrect(self, x, y, w, h, r, col, a=1.0):
        """圓角矩形。"""
        r = min(r, w / 2, h / 2)
        if r < 0.75:
            self.rect(x, y, w, h, col, a)
            return
        self.rect(x + r, y, w - 2 * r, h, col, a)
        self.rect(x, y + r, r, h - 2 * r, col, a)
        self.rect(x + w - r, y + r, r, h - 2 * r, col, a)
        c = rgba(col, a)
        corners = (
            (x + r, y + r, math.pi),
            (x + w - r, y + r, 1.5 * math.pi),
            (x + w - r, y + h - r, 0.0),
            (x + r, y + h - r, 0.5 * math.pi),
        )
        for cx, cy, a0 in corners:
            for i in range(4):
                t0 = a0 + (math.pi / 2) * i / 4
                t1 = a0 + (math.pi / 2) * (i + 1) / 4
                self._tri(
                    [
                        (cx, cy),
                        (cx + r * math.cos(t0), cy + r * math.sin(t0)),
                        (cx + r * math.cos(t1), cy + r * math.sin(t1)),
                    ],
                    c,
                )

    def line(self, x1, y1, x2, y2, t, col, a=1.0):
        dx, dy = x2 - x1, y2 - y1
        ln = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / ln * t / 2, dx / ln * t / 2
        self._tri(
            [
                (x1 + nx, y1 + ny),
                (x1 - nx, y1 - ny),
                (x2 + nx, y2 + ny),
                (x2 + nx, y2 + ny),
                (x1 - nx, y1 - ny),
                (x2 - nx, y2 - ny),
            ],
            rgba(col, a),
        )

    def circle(self, cx, cy, r, col, a=1.0, seg=32):
        c = rgba(col, a)
        for i in range(seg):
            a0, a1 = 2 * math.pi * i / seg, 2 * math.pi * (i + 1) / seg
            self._tri(
                [
                    (cx, cy),
                    (cx + r * math.cos(a0), cy + r * math.sin(a0)),
                    (cx + r * math.cos(a1), cy + r * math.sin(a1)),
                ],
                c,
            )

    def ring(self, cx, cy, r, t, col, a=1.0, seg=40):
        c = rgba(col, a)
        ri, ro = r - t / 2, r + t / 2

        def pt(rr, aa):
            return (cx + rr * math.cos(aa), cy + rr * math.sin(aa))

        for i in range(seg):
            a0, a1 = 2 * math.pi * i / seg, 2 * math.pi * (i + 1) / seg
            self._tri(
                [
                    pt(ri, a0),
                    pt(ro, a0),
                    pt(ro, a1),
                    pt(ri, a0),
                    pt(ro, a1),
                    pt(ri, a1),
                ],
                c,
            )

    def arrow(self, x1, y1, x2, y2, t, col, head, a=1.0):
        ang = math.atan2(y2 - y1, x2 - x1)
        bx, by = x2 - head * math.cos(ang), y2 - head * math.sin(ang)
        self.line(x1, y1, bx, by, t, col, a)
        w = head * 0.55
        nx, ny = -math.sin(ang) * w, math.cos(ang) * w
        self._tri([(x2, y2), (bx + nx, by + ny), (bx - nx, by - ny)], rgba(col, a))

    def font(self, size, bold=False):
        size = max(8, round(size))
        key = (size, bold)
        if key not in self.fonts:
            self.fonts[key] = pygame.font.Font(
                self.font_bold if bold else self.font_reg, size
            )
        return self.fonts[key]

    def measure(self, s, size, bold=False):
        return self.font(size, bold).size(s)

    def text(self, s, x, y, size, col=TEXT, anchor="tl", bold=False, a=1.0):
        """繪製文字並回傳寬度。anchor 第一字為垂直 t/m/b，第二字為水平 l/c/r。"""
        s = s.replace("\x00", "")
        if not s:
            return 0
        f = self.font(size, bold)
        key = (s, round(size), bold)
        ent = self.tex_cache.get(key)
        if ent is None:
            surf = f.render(s, True, (255, 255, 255))
            w, h = surf.get_size()
            tex = self.ctx.texture((w, h), 4, pygame.image.tostring(surf, "RGBA"))
            tex.filter = (moderngl.LINEAR, moderngl.LINEAR)
            ent = self.tex_cache[key] = (tex, w, h)
        tex, w, h = ent
        if anchor[1] == "c":
            x -= w / 2
        elif anchor[1] == "r":
            x -= w
        if anchor[0] == "m":
            y -= h / 2
        elif anchor[0] == "b":
            y -= h
        self.texts.append((tex, round(x), round(y), w, h, rgba(col, a)))
        return w

    def flush(self):
        w, h = self.size
        if self.verts:
            data = np.array(self.verts, dtype="f4")
            vbo = self.ctx.buffer(data.tobytes())
            vao = self.ctx.vertex_array(self.prog, [(vbo, "2f 4f", "in_pos", "in_col")])
            self.prog["u_res"].value = (w, h)
            vao.render(moderngl.TRIANGLES)
            vao.release()
            vbo.release()
            self.verts.clear()
        for tex, x, y, tw, th, col in self.texts:
            q = np.array(
                [
                    x,
                    y,
                    0,
                    0,
                    x + tw,
                    y,
                    1,
                    0,
                    x,
                    y + th,
                    0,
                    1,
                    x + tw,
                    y,
                    1,
                    0,
                    x + tw,
                    y + th,
                    1,
                    1,
                    x,
                    y + th,
                    0,
                    1,
                ],
                dtype="f4",
            )
            self.tvbo.write(q.tobytes())
            tex.use(0)
            self.tprog["tex"].value = 0
            self.tprog["u_res"].value = (w, h)
            self.tprog["u_tint"].value = col
            self.tvao.render(moderngl.TRIANGLES)
        self.texts.clear()


# --------------------------------------------------------------------------
# 立即模式 UI
# --------------------------------------------------------------------------


class UI:
    """立即模式的按鈕與數值調整元件，每幀重新建立。"""

    def __init__(self, r):
        self.r = r
        self.mouse = (-1, -1)
        self.down = False
        self.clicks = []
        self.wheel = 0
        self.hint = None
        self.press_id = None
        self.next_rep = 0.0
        self.s = 1.0

    def inside(self, rect, pos=None):
        x, y, w, h = rect
        px, py = pos if pos else self.mouse
        return x <= px < x + w and y <= py < y + h

    def button(
        self,
        bid,
        rect,
        label,
        size,
        hint=None,
        active=False,
        enabled=True,
        repeat=False,
    ):
        """繪製按鈕，本幀被點擊（或按住連發）時回傳 True。"""
        hover = enabled and self.inside(rect)
        if hover and hint:
            self.hint = hint
        fired = False
        if enabled:
            for c in self.clicks:
                if self.inside(rect, c):
                    fired = True
                    self.press_id, self.next_rep = bid, time.time() + 0.4
            if (
                repeat
                and self.down
                and self.press_id == bid
                and hover
                and time.time() >= self.next_rep
            ):
                fired = True
                self.next_rep = time.time() + 0.1
        x, y, w, h = rect
        if active:
            col = ACCENT
        elif not enabled:
            col = CARD
        else:
            col = HAIR if hover else CARD_HI
        self.r.rrect(x, y, w, h, 6 * self.s, col)
        txt = (BG if active else TEXT) if enabled else DIM
        self.r.text(label, x + w / 2, y + h / 2, size, txt, "mc", bold=active)
        return fired

    def stepper(self, sid, rect, label, value, size, hint, enabled=True):
        """標籤 / 數值 / [-] [+]；滑鼠滾輪放在該列也可調整。回傳步數。"""
        r = self.r
        x, y, w, h = rect
        delta = 0
        r.text(label, x, y + h / 2, size, DIM, "ml")
        r.text(
            value,
            x + w - 2 * h - 12 * self.s,
            y + h / 2,
            size,
            TEXT if enabled else DIM,
            "mr",
            bold=True,
        )
        if self.button(
            sid + "-",
            (x + w - 2 * h - 4 * self.s, y, h, h),
            "-",
            size * 1.1,
            hint,
            enabled=enabled,
            repeat=True,
        ):
            delta -= 1
        if self.button(
            sid + "+",
            (x + w - h, y, h, h),
            "+",
            size * 1.1,
            hint,
            enabled=enabled,
            repeat=True,
        ):
            delta += 1
        if enabled and self.inside(rect):
            self.hint = hint
            delta += int(self.wheel)
        return delta


def wrap_text(r, text, size, max_w):
    """依像素寬度換行（中文逐字、標點不置於行首）。"""
    no_lead = "。，、；：！？）」』,.;:!?)"
    lines = []
    for para in text.split("\n"):
        cur = ""
        for ch in para:
            if r.measure(cur + ch, size)[0] > max_w and cur and ch not in no_lead:
                lines.append(cur)
                cur = ch
            else:
                cur += ch
        lines.append(cur)
    return lines


def dist_color(d, lo, hi):
    """距離色階：lo 到 hi 之間近為亮、遠為暗的單色漸層。"""
    if d <= 0:
        return (20, 22, 26)
    t = 1.0 - min(max((d - lo) / (hi - lo), 0), 1)
    stops = [(26, 34, 52), (44, 82, 148), ACCENT, (232, 242, 255)]
    pos = t * (len(stops) - 1)
    i = min(int(pos), len(stops) - 2)
    f = pos - i
    return tuple(
        int(stops[i][k] + (stops[i + 1][k] - stops[i][k]) * f) for k in range(3)
    )


HINTS = {
    "res4": "4×4 解析度，60 Hz。更新快、格子少，適合高速控制。\n指令：4（快捷鍵 4）",
    "res8": "8×8 解析度，15 Hz。格子多、位置較精細，但更新慢。\n指令：8（快捷鍵 8）",
    "auto": "恢復 PID 自動平衡並清除積分。下過 t / l / a 之後必須按這個才會繼續自動控制。\n指令：r（快捷鍵 R）",
    "level": "平台回水平並暫停自動控制，校正伺服時使用。\n指令：l（快捷鍵 L）",
    "kp": "比例增益 Kp（度/mm）：球偏離越遠，平台傾越多。太大會震盪，太小反應慢。滾輪或按住 +/- 可連續調整。\n指令：p kp ki kd",
    "ki": "積分增益 Ki（度/(mm·s)）：消除穩態誤差，太大會越晃越大。調整時會清除積分。\n指令：p kp ki kd",
    "kd": "微分增益 Kd（度/(mm/s)）：依球速度提供阻尼以抑制震盪，雜訊大時不宜太大。\n指令：p kp ki kd",
    "tx": "目標點 X（mm，盤面中心為原點）：球要停的位置。\n指令：g x y",
    "ty": "目標點 Y（mm，盤面中心為原點）：球要停的位置。\n指令：g x y",
    "center": "把目標點設回盤面中心 (0, 0)。\n指令：g 0 0",
    "height": "平台中心高度（mm）。會改變最大傾角，超出機構範圍會被韌體拒絕。\n指令：h 高度",
    "sv": "伺服實際輸出角度（不含 offset 運算）。調整會切換為「直接角度」模式、暫停自動平衡（按 R 恢復）。滾輪或按住 +/- 可連續調整。\n指令：d a1 a2 a3",
    "step": "每次調整伺服角度的步進（0.5° / 1° / 2°），按一下切換。",
    "origin": "把目前三顆伺服的角度寫入為平台水平的原點（存進 ESP32，斷電保留）。請先把平台調到實際水平；需在「直接角度」模式且 PCA9685 有連線。為避免誤按，3 秒內要再按一次確認。\n指令：o",
    "origin_reset": "還原為 Config.h 的預設原點，並清除 ESP32 已儲存的值。\n指令：o r",
    "cmd": "直接輸入韌體指令，Enter 送出。例：s 顯示狀態、? 說明、c 1 90 1 0 校正伺服。快捷鍵 /",
}
DEFAULT_HINT = (
    "滑鼠移到按鈕或數值上會顯示說明。\n"
    "4 / 8 解析度　R 自動平衡　L 回水平\n/ 輸入指令　F11 全螢幕　Esc 離開"
)


class App:
    """主視窗：處理輸入事件並繪製整個介面。"""

    def __init__(self, link, tel, size, gpu_info, snapshot=None, frames=0):
        pygame.init()
        pygame.display.set_caption("平衡球平台監控")
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
        self.size = size
        pygame.display.set_mode(
            size, pygame.OPENGL | pygame.DOUBLEBUF | pygame.RESIZABLE, vsync=1
        )
        self.ctx = moderngl.create_context()
        self.gl_renderer = self.ctx.info["GL_RENDERER"]
        self.gpu_info = gpu_info
        self.r = Renderer(self.ctx)
        self.ui = UI(self.r)
        self.link, self.tel = link, tel
        self.snapshot, self.frames = snapshot, frames
        self.running = True
        self.fullscreen = False
        self.cmd_text, self.cmd_focus = "", False
        self.ov = {}
        self.servo_step = 0.5
        self.origin_armed_until = 0.0
        self.s = 1.0

    # --- 設定值 ---
    def val(self, key, stat):
        """回傳設定值；剛送出的值會暫時覆蓋，避免連點時讀到舊遙測。"""
        o = self.ov.get(key)
        if o and time.time() < o[1]:
            return o[0]
        return stat[key] if stat else None

    def set_ov(self, key, v):
        self.ov[key] = (v, time.time() + 0.6)

    def send_pid(self, stat, **chg):
        v = {k: self.val(k, stat) for k in ("kp", "ki", "kd")}
        v.update(chg)
        for k, x in v.items():
            self.set_ov(k, x)
        self.link.send(f"p {v['kp']:.4f} {v['ki']:.4f} {v['kd']:.4f}")

    def send_target(self, tx, ty):
        self.set_ov("tx", tx)
        self.set_ov("ty", ty)
        self.link.send(f"g {tx:.1f} {ty:.1f}")

    # --- 事件 ---
    def handle_events(self):
        ui = self.ui
        ui.clicks, ui.wheel = [], 0
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                self.running = False
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                ui.clicks.append(e.pos)
                ui.down = True
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                ui.down = False
                ui.press_id = None
            elif e.type in (pygame.WINDOWFOCUSLOST, pygame.WINDOWLEAVE):
                # 放開滑鼠的事件可能送不到視窗，避免按住連發卡住。
                ui.down = False
                ui.press_id = None
            elif e.type == pygame.MOUSEWHEEL:
                ui.wheel += e.y
            elif e.type == pygame.KEYDOWN:
                self.handle_key(e)
        ui.mouse = pygame.mouse.get_pos()

    def handle_key(self, e):
        if self.cmd_focus:
            if e.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                if self.cmd_text.strip():
                    self.link.send(self.cmd_text.strip())
                self.cmd_text, self.cmd_focus = "", False
            elif e.key == pygame.K_ESCAPE:
                self.cmd_text, self.cmd_focus = "", False
            elif e.key == pygame.K_BACKSPACE:
                self.cmd_text = self.cmd_text[:-1]
            elif e.unicode and e.unicode.isprintable():
                self.cmd_text += e.unicode
            return
        k = e.key
        if k == pygame.K_ESCAPE:
            self.running = False
        elif k in (pygame.K_4, pygame.K_KP4):
            self.link.send("4")
        elif k in (pygame.K_8, pygame.K_KP8):
            self.link.send("8")
        elif k == pygame.K_r:
            self.link.send("r")
        elif k == pygame.K_l:
            self.link.send("l")
        elif k in (pygame.K_SLASH, pygame.K_KP_DIVIDE):
            self.cmd_focus = True
        elif k == pygame.K_F11:
            self.fullscreen = not self.fullscreen
            flags = pygame.OPENGL | pygame.DOUBLEBUF
            flags |= pygame.FULLSCREEN if self.fullscreen else pygame.RESIZABLE
            pygame.display.set_mode(
                (0, 0) if self.fullscreen else self.size, flags, vsync=1
            )

    # --- 版面元件 ---
    def card(self, x, y, w, h, title):
        """繪製卡片並回傳內容起點的 y。"""
        s = self.s
        self.r.rrect(x, y, w, h, 10 * s, CARD)
        self.r.text(title, x + 14 * s, y + 11 * s, 11.5 * s, DIM, "tl", bold=True)
        return y + 34 * s

    def row(self, x, y, w, label, value, col=TEXT):
        s = self.s
        self.r.text(label, x, y, 12.5 * s, DIM, "tl")
        self.r.text(value, x + w, y, 12.5 * s, col, "tr", bold=True)
        return y + 21 * s

    # --- 左側：資料流 ---
    def left_panel(self, d, x, w):
        r, s = self.r, self.s
        stat = d["stat"]
        pad = 14 * s
        bx, iw = x + pad, w - 2 * pad
        ix, iiw = bx + 14 * s, iw - 28 * s
        y = 14 * s
        r.text("Ball Balancer", bx, y, 17 * s, TEXT, "tl", bold=True)
        col = ACCENT if d["connected"] else DANGER
        r.circle(bx + iw - 4 * s, y + 11 * s, 4 * s, col)
        r.text(d["status"], bx + iw - 14 * s, y + 3 * s, 11.5 * s, DIM, "tr")
        y += 40 * s

        present = bool(stat and stat["present"])
        n = d["n"]
        has = stat is not None

        def card_rows(title, rows):
            nonlocal y
            h = 34 * s + len(rows) * 21 * s + 10 * s
            yy = self.card(bx, y, iw, h, title)
            for lab, val in rows:
                yy = self.row(ix, yy, iiw, lab, val)
            y += h + 10 * s

        card_rows(
            "01  感測器",
            [
                ("解析度", f"{n}×{n}" if n else "—"),
                ("更新率", f"{d['fps']:.1f} Hz"),
                ("有效區域", f"{int(stat['zones'])} / {n * n}" if has and n else "—"),
                ("平均距離", f"{stat['dist']:.0f} mm" if has else "—"),
            ],
        )
        card_rows(
            "02  球位置",
            [
                ("偵測", "有球" if present else "無球"),
                ("格點", f"({d['cx']:.2f}, {d['cy']:.2f})" if present else "—"),
                (
                    "位置 (mm)",
                    f"({stat['x']:+.1f}, {stat['y']:+.1f})" if present else "—",
                ),
                (
                    "離中心",
                    f"{math.hypot(stat['x'], stat['y']):.1f} mm" if present else "—",
                ),
            ],
        )

        if has:
            tx, ty = self.val("tx", stat), self.val("ty", stat)
            ex, ey = stat["x"] - tx, stat["y"] - ty
            pid_rows = [
                ("目標 (mm)", f"({tx:+.1f}, {ty:+.1f})"),
                ("誤差 (mm)", f"({ex:+.1f}, {ey:+.1f})" if present else "—"),
                ("積分", f"({stat['int_x']:+.1f}, {stat['int_y']:+.1f})"),
                ("輸出 傾角 θ", f"{stat['theta']:.2f}°"),
                ("輸出 方向 φ", f"{stat['phi']:.1f}°"),
            ]
        else:
            pid_rows = [
                (k, "—")
                for k in (
                    "目標 (mm)",
                    "誤差 (mm)",
                    "積分",
                    "輸出 傾角 θ",
                    "輸出 方向 φ",
                )
            ]
        h = 34 * s + 5 * 21 * s + 22 * s
        yy = self.card(bx, y, iw, h, "03  PID  (mm → 度)")
        for lab, val in pid_rows:
            yy = self.row(ix, yy, iiw, lab, val)
        if has:
            frac = min(stat["theta"] / max(stat["max_tilt"], 1e-3), 1.0)
            r.rrect(ix, yy + 2 * s, iiw, 5 * s, 2.5 * s, BG)
            if frac > 0.01:
                r.rrect(
                    ix,
                    yy + 2 * s,
                    iiw * frac,
                    5 * s,
                    2.5 * s,
                    DANGER if frac > 0.9 else ACCENT,
                )
        y += h + 10 * s

        h = 34 * s + 3 * 28 * s + 4 * s
        yy = self.card(bx, y, iw, h, "04  伺服角度")
        for i in range(3):
            ang = stat[f"s{i + 1}"] if has else None
            r.text(f"腳 {i + 1}  CH{SERVO_CHANNELS[i]}", ix, yy, 12.5 * s, DIM, "tl")
            r.text(
                f"{ang:.1f}°" if ang is not None else "—",
                ix + iiw,
                yy,
                12.5 * s,
                TEXT,
                "tr",
                bold=True,
            )
            r.rrect(ix, yy + 19 * s, iiw, 4 * s, 2 * s, BG)
            if ang is not None:
                f = (ang - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG)
                r.rrect(
                    ix,
                    yy + 19 * s,
                    max(iiw * min(max(f, 0), 1), 4 * s),
                    4 * s,
                    2 * s,
                    DANGER if f < 0.04 or f > 0.96 else ACCENT,
                )
            yy += 28 * s
        y += h + 12 * s

        mode = (
            MODE_NAMES[int(stat["mode"])] if has and 0 <= int(stat["mode"]) < 3 else "—"
        )
        r.text(f"模式  {mode}", bx, y, 12.5 * s, TEXT, "tl", bold=True)

        # 顯卡資訊（左下角）
        gy = self.h - 14 * s
        renderer = self.gl_renderer
        if self.gpu_warning():
            r.text(
                "使用內顯繪圖，可用 --gpu high 改用獨顯", bx, gy, 11 * s, DANGER, "bl"
            )
            gy -= 15 * s
        r.text(f"GPU  {renderer}", bx, gy, 11 * s, DIM, "bl")

    def gpu_warning(self):
        return is_integrated(self.gl_renderer) and any(
            not is_integrated(g) for g in self.gpu_info.get("all", [])
        )

    # --- 中間：熱圖 ---
    def center(self, d, x0, x1, h):
        r, s = self.r, self.s
        n, stat = d["n"], d["stat"]
        r.text("VL53L7CX  距離圖", x0, 18 * s, 13 * s, DIM, "tl", bold=True)
        log_h = 112 * s
        y0 = 46 * s
        y1 = h - log_h - 54 * s
        side = max(min(x1 - x0, y1 - y0), 40)
        gx0, gy0 = x0 + (x1 - x0 - side) / 2, y0
        grid = d["dist"][:n, :n]
        valid = grid[grid > 0]
        lo = float(valid.min()) if valid.size else 0.0
        hi = max(float(valid.max()) if valid.size else 0.0, lo + MIN_SPAN_MM)
        if n:
            cell = side / n
            gap = max(2.0, 3 * s)
            for yy in range(n):
                for xx in range(n):
                    dv = d["dist"][yy, xx]
                    px, py = gx0 + xx * cell, gy0 + yy * cell
                    r.rrect(
                        px + gap / 2,
                        py + gap / 2,
                        cell - gap,
                        cell - gap,
                        4 * s,
                        dist_color(dv, lo, hi),
                    )
                    if cell >= 26 * s:
                        bright = dv > 0 and dist_color(dv, lo, hi)[2] > 200
                        r.text(
                            f"{int(dv)}" if dv > 0 else "–",
                            px + 9 * s,
                            py + 8 * s,
                            min(cell * 0.18, 15 * s),
                            BG if bright else (TEXT if dv > 0 else DIM),
                            "tl",
                        )
            cx, cy = gx0 + side / 2, gy0 + side / 2
            r.line(cx - 9 * s, cy, cx + 9 * s, cy, 1.2, TEXT, 0.35)
            r.line(cx, cy - 9 * s, cx, cy + 9 * s, 1.2, TEXT, 0.35)
            if stat and stat["theta"] > 0.05:
                L = (
                    side * 0.36 * min(stat["theta"] / max(stat["max_tilt"], 1e-3), 1)
                    + 10 * s
                )
                ph = math.radians(stat["phi"])
                r.arrow(
                    cx,
                    cy,
                    cx + L * math.cos(ph),
                    cy + L * math.sin(ph),
                    3 * s,
                    TEXT,
                    12 * s,
                    0.9,
                )
            if d["cx"] >= 0:
                bx = gx0 + (d["cx"] + 0.5) * cell
                by = gy0 + (d["cy"] + 0.5) * cell
                r.ring(bx, by, max(cell * 0.17, 7 * s), 3 * s, TEXT)
        else:
            r.rrect(gx0, gy0, side, side, 10 * s, CARD)
            r.text("等待資料…", gx0 + side / 2, gy0 + side / 2, 14 * s, DIM, "mc")

        if n and not valid.size:
            mx, my = gx0 + side / 2, gy0 + side / 2
            r.rrect(
                gx0 + side * 0.08, my - 34 * s, side * 0.84, 68 * s, 10 * s, BG, 0.88
            )
            r.text("沒有有效區域", mx, my - 12 * s, 15 * s, TEXT, "mc", bold=True)
            why = "沒有區域通過距離、訊號強度或狀態碼的篩選"
            r.text(why, mx, my + 12 * s, 12 * s, DIM, "mc")

        ly = gy0 + side + 12 * s
        steps = 60
        for i in range(steps):
            dv = hi - (hi - lo) * i / (steps - 1)
            r.rect(
                gx0 + side * i / steps,
                ly,
                side / steps + 1,
                6 * s,
                dist_color(dv, lo, hi),
            )
        r.text(f"{lo:.0f} mm", gx0, ly + 10 * s, 11 * s, DIM, "tl")
        r.text(f"{hi:.0f} mm", gx0 + side, ly + 10 * s, 11 * s, DIM, "tr")
        r.text(
            "圓圈：球　箭頭：平台下傾方向",
            gx0 + side / 2,
            ly + 10 * s,
            11 * s,
            DIM,
            "tc",
        )

        ly0 = h - log_h - 14 * s
        r.rrect(x0, ly0, x1 - x0, log_h, 10 * s, CARD)
        r.text("韌體訊息", x0 + 14 * s, ly0 + 10 * s, 11.5 * s, DIM, "tl", bold=True)
        lh = 15 * s
        rows = int((log_h - 36 * s) // lh)
        for i, ln in enumerate(d["log"][-rows:] if rows > 0 else []):
            col = TEXT if ln.startswith(">>") else DIM
            r.text(ln[:140], x0 + 14 * s, ly0 + 30 * s + i * lh, 11.5 * s, col, "tl")

    # --- 右側：控制 ---
    def right_panel(self, d, x, w, h):
        r, s, ui = self.r, self.s, self.ui
        stat = d["stat"]
        ok = stat is not None
        pad = 14 * s
        bx, iw = x + pad, w - 2 * pad
        ix, iiw = bx + 14 * s, iw - 28 * s
        rh = 24 * s
        gap = 6 * s
        half = (iiw - 6 * s) / 2
        n = d["n"]
        y = 14 * s
        r.text("控制", bx, y, 17 * s, TEXT, "tl", bold=True)
        y += 40 * s

        h1 = 34 * s + 2 * 30 * s + 4 * s
        yy = self.card(bx, y, iw, h1, "感測器與模式")
        if ui.button(
            "res4",
            (ix, yy, half, 26 * s),
            "4×4  60Hz",
            12.5 * s,
            HINTS["res4"],
            active=(n == 4),
        ):
            self.link.send("4")
        if ui.button(
            "res8",
            (ix + half + 6 * s, yy, half, 26 * s),
            "8×8  15Hz",
            12.5 * s,
            HINTS["res8"],
            active=(n == 8),
        ):
            self.link.send("8")
        yy += 30 * s
        if ui.button(
            "auto",
            (ix, yy, half, 26 * s),
            "自動平衡",
            12.5 * s,
            HINTS["auto"],
            active=bool(ok and stat["mode"] == 0),
        ):
            self.link.send("r")
        if ui.button(
            "level",
            (ix + half + 6 * s, yy, half, 26 * s),
            "回水平",
            12.5 * s,
            HINTS["level"],
        ):
            self.link.send("l")
        y += h1 + 10 * s

        h2 = 34 * s + 3 * (rh + gap) + 2 * s
        yy = self.card(bx, y, iw, h2, "PID 增益")
        for key, label, step in (
            ("kp", "Kp  度/mm", 0.01),
            ("ki", "Ki  度/(mm·s)", 0.002),
            ("kd", "Kd  度/(mm/s)", 0.01),
        ):
            v = self.val(key, stat)
            dl = ui.stepper(
                "pid" + key,
                (ix, yy, iiw, rh),
                label,
                f"{v:.3f}" if ok else "—",
                12.5 * s,
                HINTS[key],
                enabled=ok,
            )
            if dl and ok:
                self.send_pid(stat, **{key: round(max(0.0, v + dl * step), 4)})
            yy += rh + gap
        y += h2 + 10 * s

        h3 = 34 * s + 3 * (rh + gap) + 2 * s
        yy = self.card(bx, y, iw, h3, "目標點與高度")
        tx, ty = self.val("tx", stat), self.val("ty", stat)
        if ui.button(
            "center",
            (bx + iw - 66 * s, y + 7 * s, 54 * s, 22 * s),
            "置中",
            11.5 * s,
            HINTS["center"],
            enabled=ok,
        ):
            self.send_target(0.0, 0.0)
        dl = ui.stepper(
            "tx",
            (ix, yy, iiw, rh),
            "目標 X (mm)",
            f"{tx:+.1f}" if ok else "—",
            12.5 * s,
            HINTS["tx"],
            enabled=ok,
        )
        if dl and ok:
            self.send_target(tx + dl, ty)
        yy += rh + gap
        dl = ui.stepper(
            "ty",
            (ix, yy, iiw, rh),
            "目標 Y (mm)",
            f"{ty:+.1f}" if ok else "—",
            12.5 * s,
            HINTS["ty"],
            enabled=ok,
        )
        if dl and ok:
            self.send_target(tx, ty + dl)
        yy += rh + gap
        hv = self.val("height", stat)
        dl = ui.stepper(
            "h",
            (ix, yy, iiw, rh),
            "平台高度 (mm)",
            f"{hv:.1f}" if ok else "—",
            12.5 * s,
            HINTS["height"],
            enabled=ok,
        )
        if dl and ok:
            nh = min(max(round(hv + dl, 1), HEIGHT_MIN_MM), HEIGHT_MAX_MM)
            if nh != hv:
                self.set_ov("height", nh)
                self.link.send(f"h {nh:.1f}")
        y += h3 + 10 * s

        h4 = 34 * s + 3 * (rh + gap) + 30 * s
        yy = self.card(bx, y, iw, h4, "伺服角度與原點")
        if ui.button(
            "origin_reset",
            (bx + iw - 66 * s, y + 7 * s, 54 * s, 22 * s),
            "還原",
            11.5 * s,
            HINTS["origin_reset"],
            enabled=ok,
        ):
            self.link.send("o r")
        cur = [self.val(f"s{i + 1}", stat) for i in range(3)]
        for i in range(3):
            dl = ui.stepper(
                f"sv{i}",
                (ix, yy, iiw, rh),
                f"腳 {i + 1}  CH{SERVO_CHANNELS[i]}",
                f"{cur[i]:.1f}°" if ok else "—",
                12.5 * s,
                HINTS["sv"],
                enabled=ok,
            )
            if dl and ok:
                new = list(cur)
                new[i] = round(
                    min(
                        max(cur[i] + dl * self.servo_step, SERVO_MIN_DEG), SERVO_MAX_DEG
                    ),
                    1,
                )
                if new[i] != cur[i]:
                    for k, v in enumerate(new):
                        self.set_ov(f"s{k + 1}", v)
                    self.link.send(f"d {new[0]:.1f} {new[1]:.1f} {new[2]:.1f}")
                    self.origin_armed_until = 0.0
                    cur = new
            yy += rh + gap
        half = (iiw - 6 * s) / 2
        if ui.button(
            "step",
            (ix, yy, half, 24 * s),
            f"步進 {self.servo_step:g}°",
            12.5 * s,
            HINTS["step"],
        ):
            steps = (0.5, 1.0, 2.0)
            self.servo_step = steps[(steps.index(self.servo_step) + 1) % len(steps)]
        raw_mode = ok and int(stat["mode"]) == 2
        armed = time.time() < self.origin_armed_until
        if ui.button(
            "origin",
            (ix + half + 6 * s, yy, half, 24 * s),
            "再按一次確認" if armed else "寫入原點",
            12.5 * s,
            HINTS["origin"],
            enabled=raw_mode,
            active=armed,
        ):
            if armed:
                self.origin_armed_until = 0.0
                self.link.send("o")
            else:
                self.origin_armed_until = time.time() + 3.0
        y += h4 + 10 * s

        box = (bx, y, iw, 30 * s)
        if ui.inside(box):
            ui.hint = HINTS["cmd"]
        for c in ui.clicks:
            self.cmd_focus = ui.inside(box, c)
        r.rrect(bx, y, iw, 30 * s, 8 * s, ACCENT if self.cmd_focus else HAIR)
        r.rrect(bx + 1.5, y + 1.5, iw - 3, 30 * s - 3, 7 * s, BG)
        if self.cmd_text or self.cmd_focus:
            caret = "|" if self.cmd_focus and int(time.time() * 2) % 2 == 0 else ""
            r.text(self.cmd_text + caret, bx + 12 * s, y + 15 * s, 12.5 * s, TEXT, "ml")
        else:
            r.text(
                "指令  按 / 或點此輸入，Enter 送出",
                bx + 12 * s,
                y + 15 * s,
                12 * s,
                DIM,
                "ml",
            )
        y += 30 * s + 10 * s

        hh = h - y - 14 * s
        if hh > 50 * s:
            r.rrect(bx, y, iw, hh, 10 * s, CARD)
            r.text("提示", bx + 14 * s, y + 11 * s, 11.5 * s, DIM, "tl", bold=True)
            text = ui.hint or DEFAULT_HINT
            ly = y + 32 * s
            for ln in wrap_text(r, text, 12 * s, iw - 28 * s):
                if ly + 15 * s > y + hh:
                    break
                r.text(ln, bx + 14 * s, ly, 12 * s, TEXT if ui.hint else DIM, "tl")
                ly += 17 * s

    def frame(self):
        w, h = pygame.display.get_window_size()
        self.h = h
        if not self.fullscreen:
            self.size = (w, h)
        self.s = min(max(min(w / DESIGN_W, h / DESIGN_H), 0.55), 3.0)
        s = self.s
        self.ui.s = s
        r, ui = self.r, self.ui
        r.begin(w, h)
        ui.hint = None
        d = self.tel.snapshot()
        lw, rw = 318 * s, 330 * s
        if w < lw + rw + 220:
            lw, rw = max((w - 220) * 0.49, 200), max((w - 220) * 0.51, 200)
        self.left_panel(d, 0, lw)
        self.center(d, lw + 8 * s, w - rw - 8 * s, h)
        self.right_panel(d, w - rw, rw, h)
        r.flush()

    def run(self):
        clock = pygame.time.Clock()
        count = 0
        try:
            while self.running:
                self.handle_events()
                self.frame()
                count += 1
                if self.snapshot and count >= self.frames:
                    self.save_snapshot()
                    break
                pygame.display.flip()
                clock.tick(60)
        finally:
            pygame.quit()

    def save_snapshot(self):
        w, h = pygame.display.get_window_size()
        data = self.ctx.screen.read(viewport=(0, 0, w, h), components=3)
        img = pygame.image.frombuffer(data, (w, h), "RGB")
        pygame.image.save(pygame.transform.flip(img, False, True), self.snapshot)
        print(f"已儲存畫面 {self.snapshot} ({w}x{h})")


def default_window_size():
    """依 DPI 縮放與螢幕大小決定初始視窗大小。"""
    try:
        pygame.display.init()  # 先初始化，SDL 才會套用 DPI 設定。
    except pygame.error:
        pass
    k = dpi_scale()
    w, h = int(DESIGN_W * k), int(DESIGN_H * k)
    try:
        dw, dh = pygame.display.get_desktop_sizes()[0]
        w, h = min(w, int(dw * 0.94)), min(h, int(dh * 0.88))
    except pygame.error:
        pass
    return w, h


def main():
    ap = argparse.ArgumentParser(description="平衡球平台監控介面")
    ap.add_argument("--port", help="序列埠（預設自動偵測）")
    ap.add_argument("--baud", type=int, default=BAUD_RATE)
    ap.add_argument("--demo", action="store_true", help="使用模擬資料")
    ap.add_argument("--size", help="初始視窗大小，例如 1600x900")
    ap.add_argument(
        "--gpu",
        choices=["auto", "high", "low", "default"],
        default="auto",
        help="設定 Windows 圖形偏好（high=獨顯）並重新啟動；預設不變更",
    )
    ap.add_argument("--snapshot", help="渲染後存成 PNG 並結束（除錯用）")
    ap.add_argument("--frames", type=int, default=90)
    ap.add_argument("--list-ports", action="store_true")
    args = ap.parse_args()

    if args.list_ports:
        for p in serial.tools.list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return
    apply_gpu_option(args.gpu)
    if args.size:
        try:
            size = tuple(int(v) for v in args.size.lower().split("x"))
            assert len(size) == 2
        except (ValueError, AssertionError):
            sys.exit("--size 格式應為 寬x高，例如 1280x760")
    else:
        size = default_window_size()

    gpu_info = {"all": []}
    threading.Thread(
        target=lambda: gpu_info.update(all=detect_gpus()), daemon=True
    ).start()

    tel = Telemetry()
    link = DemoLink(tel) if args.demo else SerialLink(tel, args.port, args.baud)
    link.start()
    try:
        app = App(link, tel, size, gpu_info, args.snapshot, args.frames)
        print(f"OpenGL: {app.gl_renderer}")
        app.run()
    except Exception:
        # pythonw 啟動時沒有主控台，崩潰原因寫入檔案方便查看。
        log = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "heatmap_crash.log"
        )
        with open(log, "w", encoding="utf-8") as f:
            f.write(traceback.format_exc())
        raise
    finally:
        link.stop()


if __name__ == "__main__":
    main()
