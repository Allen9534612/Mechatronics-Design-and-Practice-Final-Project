"""平衡球平台監控介面（OpenGL，可任意縮放視窗）。

左側：資料流（感測器 → 球位置估測 → PID → 逆向運動學/伺服）的即時數值
中間：VL53L7CX 距離熱圖 + 球位置 + 平台傾斜方向 + 韌體訊息
右側：可調項按鈕（解析度、模式、PID、目標點、高度、傾斜測試）與操作提示
下指令一律透過 pyserial 送給 ESP32，數值以韌體回報的 STAT 為準。

用法：
    python script/heatmap.py                 # 自動偵測 ESP32 序列埠
    python script/heatmap.py --port COM5
    python script/heatmap.py --demo          # 沒接硬體，用模擬資料操作介面
快捷鍵：4 / 8 切換解析度、R 自動平衡、L 回水平、/ 輸入指令、F11 全螢幕、Esc 或 C 離開

需要：pip install -r script/requirements.txt
"""

import argparse
import math
import os
import random
import sys
import threading
import time
from collections import deque

import moderngl
import numpy as np
import pygame
import serial
import serial.tools.list_ports

BAUD_RATE = 115200
MIN_DISTANCE = 150  # 熱圖色階範圍（mm），下限同 Config.h 的 TOF_FILTER_MIN_DISTANCE
MAX_DISTANCE = 500

# 以下需與 include/Config.h 一致（只用在顯示）
SERVO_MIN_DEG = 40.0
SERVO_MAX_DEG = 140.0
SERVO_CHANNELS = (0, 1, 2)

STAT_FIELDS = [
    "mode", "present", "x", "y", "theta", "phi", "s1", "s2", "s3", "kp", "ki",
    "kd", "tx", "ty", "height", "max_tilt", "dist", "zones", "int_x", "int_y",
]  # 對應 main.cpp 的 printStat()
MODE_NAMES = ["PID 自動平衡", "指定傾角", "直接角度"]

# ---------------------------------------------------------------------------
# 資料與序列埠
# ---------------------------------------------------------------------------


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
        with self.lock:
            now = time.time()
            fps = 0.0
            recent = [t for t in self.frame_times if now - t <= 1.0]
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
    """解析韌體送來的一行；DATA / STAT 更新狀態，其餘當成訊息記錄。"""
    line = line.strip()
    if not line:
        return
    try:
        if line.startswith("DATA,"):
            p = line.split(",")
            n = int(p[1])
            if n in (4, 8) and len(p) == 2 + n * n + 2:
                tel.push_data(n, [float(v) for v in p[2:2 + n * n]],
                              float(p[-2]), float(p[-1]))
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
    """pyserial 背景執行緒：持續讀取並解析，send() 送出指令；斷線自動重連。"""

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
                    self.ser = serial.Serial(port, self.baud, timeout=0.1)
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
    """沒接硬體時的模擬器：產生與韌體相同格式的 DATA / STAT，並回應同樣的指令。"""

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
                    self.tel.add_log(">> 恢復紅外線陣列自動控制")
                elif c == "l":
                    self.mode, self.theta, self.phi = 1, 0.0, 0.0
                    self.tel.add_log(">> 回水平")
                elif c == "n":
                    self.off_until = time.time() + 2.0
                    self.tel.add_log(">> 球已移開，平台慢慢回水平")
                elif c == "p" and len(p) == 4:
                    self.kp, self.ki, self.kd = map(float, p[1:])
                    self.ix = self.iy = 0.0
                    self.tel.add_log(f">> PID kp={self.kp:.4f} ki={self.ki:.4f} kd={self.kd:.4f}")
                elif c == "g" and len(p) == 3:
                    self.tx, self.ty = float(p[1]), float(p[2])
                    self.tel.add_log(f">> 目標點 ({self.tx:.1f}, {self.ty:.1f}) mm")
                elif c == "h" and len(p) == 2:
                    self.height = float(p[1])
                    self.tel.add_log(f">> 高度 {self.height:.1f} mm，最大傾角 {self.max_tilt:.1f} 度")
                elif c == "t" and len(p) == 3:
                    th, ph = float(p[1]), float(p[2])
                    if 0 <= th <= self.max_tilt:
                        self.mode, self.theta, self.phi = 1, th, ph
                        self.tel.add_log(f">> 傾斜 {th:.1f} 度，往 {ph:.1f} 度方向往下")
                    else:
                        self.tel.add_log(f">> 錯誤：傾角需在 0 ~ {self.max_tilt:.1f} 度之間")
                else:
                    self.tel.add_log(">> (demo) 不支援這個指令")
        except ValueError:
            self.tel.add_log(">> (demo) 格式錯誤")
        return True

    def _servos(self):
        return [b + 3.0 * self.theta * math.cos(math.radians(self.phi - a))
                for a, b in zip(self.LEG_ANGLES, self.LEVEL)]

    def _run(self):
        dt = 1.0 / 30
        last_stat = 0.0
        while self.running:
            t0 = time.time()
            with self.lock:
                present = t0 > self.off_until and abs(self.bx) < 70 and abs(self.by) < 70
                if self.mode == 0:
                    if present:
                        ex, ey = self.bx - self.tx, self.by - self.ty
                        self.ix = max(-300, min(300, self.ix + ex * dt))
                        self.iy = max(-300, min(300, self.iy + ey * dt))
                        ux = -(self.kp * ex + self.ki * self.ix + self.kd * self.vx)
                        uy = -(self.kp * ey + self.ki * self.iy + self.kd * self.vy)
                        mag = math.hypot(ux, uy)
                        self.theta = min(mag, self.max_tilt)
                        self.phi = math.degrees(math.atan2(uy, ux)) % 360 if mag > 1e-4 else 0.0
                    else:
                        self.theta *= 0.9
                a = 9810.0 * 5.0 / 7.0 * math.sin(math.radians(self.theta)) * 0.15
                self.vx = (self.vx + a * math.cos(math.radians(self.phi)) * dt) * 0.99
                self.vy = (self.vy + a * math.sin(math.radians(self.phi)) * dt) * 0.99
                self.bx += self.vx * dt
                self.by += self.vy * dt
                if not present and t0 > self.off_until and (abs(self.bx) >= 70 or abs(self.by) >= 70):
                    self.off_until = t0 + 1.5  # 掉出盤面，稍後重新放回
                    self.bx, self.by = random.uniform(-45, 45), random.uniform(-45, 45)
                    self.vx = self.vy = 0.0
                n = self.n
                pitch = 60.0 / n
                gx = (n - 1) / 2 + math.degrees(math.atan2(self.bx, 300)) / pitch
                gy = (n - 1) / 2 + math.degrees(math.atan2(self.by, 300)) / pitch
                d = np.full((n, n), 300.0) + np.random.normal(0, 1.5, (n, n))
                yy, xx = np.mgrid[0:n, 0:n]
                g = np.exp(-((xx - gx) ** 2 + (yy - gy) ** 2) / 2.0)
                if present:
                    d = d - 55.0 * g
                d[np.random.rand(n, n) < 0.03] = 0
                valid = int((d > 0).sum())
                cx, cy = (gx, gy) if present else (-1.0, -1.0)
                stat = None
                if t0 - last_stat >= 0.1:
                    last_stat = t0
                    s = self._servos()
                    stat = dict(zip(STAT_FIELDS, [
                        self.mode, 1 if present else 0, self.bx, self.by, self.theta,
                        self.phi, s[0], s[1], s[2], self.kp, self.ki, self.kd, self.tx,
                        self.ty, self.height, self.max_tilt, 245.0 if present else 300.0,
                        valid, self.ix, self.iy]))
            self.tel.push_data(n, d, cx, cy)
            if stat:
                self.tel.push_stat(stat)
            time.sleep(max(0.0, dt - (time.time() - t0)))


# ---------------------------------------------------------------------------
# OpenGL 2D 繪圖
# ---------------------------------------------------------------------------

BG = (13, 15, 20)
PANEL = (20, 24, 32)
BOX = (28, 33, 44)
BORDER = (52, 60, 78)
TEXT = (232, 236, 245)
DIM = (140, 150, 172)
ACCENT = (84, 172, 255)
GOOD = (92, 214, 134)
WARN = (255, 192, 72)
BAD = (255, 104, 104)
BTN = (46, 55, 72)
BTN_HOT = (66, 80, 108)
BTN_ACTIVE = (38, 112, 196)
BTN_OFF = (34, 38, 48)

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
    vec4 t = texture(tex, v_uv);
    f_col = vec4(u_tint.rgb, u_tint.a * t.a);
}
"""


def rgba(c, a=1.0):
    return (c[0] / 255.0, c[1] / 255.0, c[2] / 255.0, a)


def find_font_paths():
    cands = [
        "C:/Windows/Fonts/msjh.ttc", "C:/Windows/Fonts/msyh.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    ]
    bold = [
        "C:/Windows/Fonts/msjhbd.ttc", "C:/Windows/Fonts/msyhbd.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    ]
    reg = next((p for p in cands if os.path.exists(p)), None)
    if reg is None:
        reg = pygame.font.match_font("microsoftjhenghei,microsoftyahei,notosanscjktc,simsun,arial")
    bld = next((p for p in bold if os.path.exists(p)), None) or reg
    return reg, bld


class Renderer:
    """批次繪製色塊/線條/圓，文字轉成貼圖後以 OpenGL 繪出。"""

    def __init__(self, ctx):
        self.ctx = ctx
        self.prog = ctx.program(vertex_shader=VS_COLOR, fragment_shader=FS_COLOR)
        self.tprog = ctx.program(vertex_shader=VS_TEX, fragment_shader=FS_TEX)
        self.tvbo = ctx.buffer(reserve=6 * 4 * 4, dynamic=True)
        self.tvao = ctx.vertex_array(self.tprog, [(self.tvbo, "2f 2f", "in_pos", "in_uv")])
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

    # --- 圖形 ---
    def _tri(self, pts, col):
        for x, y in pts:
            self.verts.extend((x, y, *col))

    def rect(self, x, y, w, h, col, a=1.0):
        c = rgba(col, a)
        self._tri([(x, y), (x + w, y), (x, y + h), (x + w, y), (x + w, y + h), (x, y + h)], c)

    def frame(self, x, y, w, h, col, t=1.0, a=1.0):
        self.rect(x, y, w, t, col, a)
        self.rect(x, y + h - t, w, t, col, a)
        self.rect(x, y + t, t, h - 2 * t, col, a)
        self.rect(x + w - t, y + t, t, h - 2 * t, col, a)

    def line(self, x1, y1, x2, y2, t, col, a=1.0):
        dx, dy = x2 - x1, y2 - y1
        ln = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / ln * t / 2, dx / ln * t / 2
        c = rgba(col, a)
        self._tri([(x1 + nx, y1 + ny), (x1 - nx, y1 - ny), (x2 + nx, y2 + ny),
                   (x2 + nx, y2 + ny), (x1 - nx, y1 - ny), (x2 - nx, y2 - ny)], c)

    def circle(self, cx, cy, r, col, a=1.0, seg=32):
        c = rgba(col, a)
        for i in range(seg):
            a0, a1 = 2 * math.pi * i / seg, 2 * math.pi * (i + 1) / seg
            self._tri([(cx, cy), (cx + r * math.cos(a0), cy + r * math.sin(a0)),
                       (cx + r * math.cos(a1), cy + r * math.sin(a1))], c)

    def ring(self, cx, cy, r, t, col, a=1.0, seg=40):
        c = rgba(col, a)
        ri, ro = r - t / 2, r + t / 2
        for i in range(seg):
            a0, a1 = 2 * math.pi * i / seg, 2 * math.pi * (i + 1) / seg
            p = lambda rr, aa: (cx + rr * math.cos(aa), cy + rr * math.sin(aa))
            self._tri([p(ri, a0), p(ro, a0), p(ro, a1), p(ri, a0), p(ro, a1), p(ri, a1)], c)

    def arrow(self, x1, y1, x2, y2, t, col, head, a=1.0):
        ang = math.atan2(y2 - y1, x2 - x1)
        bx, by = x2 - head * math.cos(ang), y2 - head * math.sin(ang)
        self.line(x1, y1, bx, by, t, col, a)
        w = head * 0.55
        nx, ny = -math.sin(ang) * w, math.cos(ang) * w
        self._tri([(x2, y2), (bx + nx, by + ny), (bx - nx, by - ny)], rgba(col, a))

    # --- 文字 ---
    def font(self, size, bold=False):
        size = max(8, int(round(size)))
        key = (size, bold)
        if key not in self.fonts:
            self.fonts[key] = pygame.font.Font(self.font_bold if bold else self.font_reg, size)
        return self.fonts[key]

    def measure(self, s, size, bold=False):
        return self.font(size, bold).size(s)

    def text(self, s, x, y, size, col=TEXT, anchor="tl", bold=False, a=1.0):
        """anchor：第一個字母 t/m/b（垂直）、第二個 l/c/r（水平）。回傳文字寬度。"""
        if not s:
            return 0
        f = self.font(size, bold)
        key = (s, int(round(size)), bold)
        ent = self.tex_cache.get(key)
        if ent is None:
            if len(self.tex_cache) > 800:
                for t, _, _ in self.tex_cache.values():
                    t.release()
                self.tex_cache.clear()
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
            q = np.array([x, y, 0, 0, x + tw, y, 1, 0, x, y + th, 0, 1,
                          x + tw, y, 1, 0, x + tw, y + th, 1, 1, x, y + th, 0, 1], dtype="f4")
            self.tvbo.write(q.tobytes())
            tex.use(0)
            self.tprog["tex"].value = 0
            self.tprog["u_res"].value = (w, h)
            self.tprog["u_tint"].value = col
            self.tvao.render(moderngl.TRIANGLES)
        self.texts.clear()


# ---------------------------------------------------------------------------
# 立即模式 UI
# ---------------------------------------------------------------------------


class UI:
    def __init__(self, r):
        self.r = r
        self.mouse = (-1, -1)
        self.down = False
        self.clicks = []
        self.wheel = 0
        self.hint = None
        self.press_id = None
        self.next_rep = 0.0

    def inside(self, rect, pos=None):
        x, y, w, h = rect
        px, py = pos if pos else self.mouse
        return x <= px < x + w and y <= py < y + h

    def button(self, bid, rect, label, size, hint=None, active=False, enabled=True, repeat=False):
        r = self.r
        hover = enabled and self.inside(rect)
        if hover and hint:
            self.hint = hint
        fired = False
        if enabled:
            for c in self.clicks:
                if self.inside(rect, c):
                    fired = True
                    self.press_id, self.next_rep = bid, time.time() + 0.4
            if repeat and self.down and self.press_id == bid and hover and time.time() >= self.next_rep:
                fired = True
                self.next_rep = time.time() + 0.1
        col = BTN_OFF if not enabled else BTN_ACTIVE if active else BTN_HOT if hover else BTN
        x, y, w, h = rect
        r.rect(x, y, w, h, col)
        r.frame(x, y, w, h, ACCENT if active else BORDER, 1)
        r.text(label, x + w / 2, y + h / 2, size, TEXT if enabled else DIM, "mc")
        return fired

    def stepper(self, sid, rect, label, value, size, hint, enabled=True, unit=""):
        """一列：標籤 / 數值 / [−] [+]；滑鼠滾輪放在這一列也可調整。回傳 -n..+n 步數。"""
        r = self.r
        x, y, w, h = rect
        bw = h
        delta = 0
        r.text(label, x, y + h / 2, size, DIM, "ml")
        r.text(value + unit, x + w - 2 * bw - 8, y + h / 2, size, TEXT if enabled else DIM, "mr", bold=True)
        if self.button(sid + "-", (x + w - 2 * bw - 4, y, bw, h), "-", size * 1.1, hint, enabled=enabled, repeat=True):
            delta -= 1
        if self.button(sid + "+", (x + w - bw, y, bw, h), "+", size * 1.1, hint, enabled=enabled, repeat=True):
            delta += 1
        if enabled and self.inside(rect):
            self.hint = hint
            if self.wheel:
                delta += int(self.wheel)
        return delta


def wrap_text(r, text, size, max_w):
    lines = []
    for para in text.split("\n"):
        cur = ""
        for ch in para:
            if r.measure(cur + ch, size)[0] > max_w and cur and ch not in "。，、；：！？）」』,.;:!?)":
                lines.append(cur)
                cur = ch
            else:
                cur += ch
        lines.append(cur)
    return lines


def dist_color(d):
    if d <= 0:
        return (24, 27, 34)
    t = min(max((d - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE), 0.0), 1.0)
    stops = [(255, 214, 90), (232, 90, 70), (70, 60, 150), (18, 34, 90)]
    pos = t * (len(stops) - 1)
    i = min(int(pos), len(stops) - 2)
    f = pos - i
    return tuple(int(stops[i][k] + (stops[i + 1][k] - stops[i][k]) * f) for k in range(3))


HINTS = {
    "res4": "4×4 解析度，60 Hz。更新快、格子少，適合高速控制。\n指令：4（快捷鍵 4）",
    "res8": "8×8 解析度，15 Hz。格子多、位置較精細，但更新慢。\n指令：8（快捷鍵 8）",
    "auto": "恢復 PID 自動平衡並清掉積分。下過 t / l / a 指令後必須按這個才會繼續自動控制。\n指令：r（快捷鍵 R）",
    "level": "平台回水平並暫停自動控制，校正伺服時使用。\n指令：l（快捷鍵 L）",
    "kp": "比例增益 Kp（度/mm）：球偏離越遠，平台傾越多。太大會震盪，太小反應慢。滾輪或按住 +/− 可連續調整。\n指令：p kp ki kd",
    "ki": "積分增益 Ki（度/(mm·s)）：消除穩態誤差（球停在偏離處）。太大會越晃越大。調整時會清掉積分。\n指令：p kp ki kd",
    "kd": "微分增益 Kd（度/(mm/s)）：依球速度提供阻尼，抑制震盪。雜訊大時不宜太大。\n指令：p kp ki kd",
    "tx": "目標點 X（mm，盤面中心為原點）：球要停的位置。\n指令：g x y",
    "ty": "目標點 Y（mm，盤面中心為原點）：球要停的位置。\n指令：g x y",
    "center": "把目標點設回盤面中心 (0, 0)。\n指令：g 0 0",
    "height": "平台中心高度（mm）。會改變最大傾角，超出機構範圍會被韌體拒絕。\n指令：h 高度",
    "theta": "傾斜測試的傾角（度），按「送出傾斜」才會動作。\n指令：t 傾角 方向",
    "phi": "傾斜測試的方向（度）：平台往這個方向那一側往下傾。0 = +X，90 = +Y。\n指令：t 傾角 方向",
    "tilt": "送出傾斜測試：平台切換為「指定傾角」模式，自動控制暫停（按 R 恢復）。\n指令：t 傾角 方向",
    "cmd": "直接輸入韌體序列埠指令，Enter 送出，例：s 顯示狀態、? 說明、c 1 90 1 0 校正伺服。快捷鍵 /",
}
DEFAULT_HINT = ("操作提示：滑鼠移到按鈕或數值上會在這裡顯示說明。\n"
                "4 / 8 切換解析度　R 自動平衡　L 回水平\n/ 輸入指令　F11 全螢幕　Esc 離開")


class App:
    def __init__(self, link, tel, size, snapshot=None, frames=0):
        pygame.init()
        pygame.display.set_caption("平衡球平台監控 (OpenGL)")
        self.size = size
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
        self.screen = pygame.display.set_mode(
            size, pygame.OPENGL | pygame.DOUBLEBUF | pygame.RESIZABLE, vsync=1)
        self.ctx = moderngl.create_context()
        self.r = Renderer(self.ctx)
        self.ui = UI(self.r)
        self.link, self.tel = link, tel
        self.snapshot, self.frames = snapshot, frames
        self.running = True
        self.fullscreen = False
        self.cmd_text, self.cmd_focus = "", False
        self.ov = {}  # 剛送出的設定值暫時覆蓋顯示，避免等遙測回來前連點時用到舊值
        self.test_theta, self.test_phi = 5.0, 0.0

    # --- 設定值 ---
    def val(self, key, stat):
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

    def send_target(self, stat, tx, ty):
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
            elif e.type == pygame.MOUSEWHEEL:
                ui.wheel += e.y
            elif e.type == pygame.KEYDOWN:
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
                    continue
                k = e.key
                if k in (pygame.K_ESCAPE, pygame.K_c):
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
                    pygame.display.set_mode((0, 0) if self.fullscreen else self.size, flags, vsync=1)
        ui.mouse = pygame.mouse.get_pos()

    # --- 繪製 ---
    def box(self, x, y, w, h, title, accent=ACCENT):
        r, s = self.r, self.s
        r.rect(x, y, w, h, BOX)
        r.rect(x, y, 3 * s, h, accent)
        r.frame(x, y, w, h, BORDER, 1)
        r.text(title, x + 12 * s, y + 6 * s, 13 * s, TEXT, "tl", bold=True)
        return y + 28 * s

    def kv(self, x, y, w, label, value, col=TEXT):
        r, s = self.r, self.s
        r.text(label, x, y, 12.5 * s, DIM, "tl")
        r.text(value, x + w, y, 12.5 * s, col, "tr", bold=True)
        return y + 19 * s

    def down_arrow(self, cx, y, h):
        r = self.r
        r._tri([(cx - h * 0.8, y), (cx + h * 0.8, y), (cx, y + h)], rgba(ACCENT, 0.8))

    def left_panel(self, d, x, w):
        r, s = self.r, self.s
        stat = d["stat"]
        pad = 12 * s
        y = 10 * s
        r.text("資料流  Data Flow", x + pad, y, 15 * s, ACCENT, "tl", bold=True)
        y += 28 * s
        iw = w - 2 * pad
        bx = x + pad

        def kvs(y0, rows):
            yy = y0
            for lab, val, *c in rows:
                yy = self.kv(bx + 12 * s, yy, iw - 24 * s, lab, val, c[0] if c else TEXT)
            return yy

        # ① 感測器
        h1 = 28 * s + 4 * 19 * s + 8 * s
        yy = self.box(bx, y, iw, h1, "① 感測器 VL53L7CX")
        n = d["n"]
        fps_col = GOOD if d["fps"] > 5 else WARN
        kvs(yy, [("解析度", f"{n}×{n}" if n else "—"), ("更新率", f"{d['fps']:.1f} Hz", fps_col),
                 ("有效區域", f"{int(stat['zones'])} / {n * n}" if stat and n else "—"),
                 ("球附近平均距離", f"{stat['dist']:.0f} mm" if stat else "—")])
        y += h1 + 4 * s
        self.down_arrow(bx + iw / 2, y, 8 * s)
        y += 12 * s

        # ② 估測
        h2 = 28 * s + 4 * 19 * s + 8 * s
        yy = self.box(bx, y, iw, h2, "② 球位置估測")
        present = bool(stat and stat["present"])
        rows = [("偵測", "● 有球" if present else "○ 無球", GOOD if present else DIM)]
        if d["cx"] >= 0 and present:
            rows.append(("格點座標", f"({d['cx']:.2f}, {d['cy']:.2f})"))
        else:
            rows.append(("格點座標", "—", DIM))
        if stat and present:
            rows.append(("位置 x, y (mm)", f"({stat['x']:+.1f}, {stat['y']:+.1f})"))
            rows.append(("離中心", f"{math.hypot(stat['x'], stat['y']):.1f} mm"))
        else:
            rows += [("位置 x, y (mm)", "—", DIM), ("離中心", "—", DIM)]
        kvs(yy, rows)
        y += h2 + 4 * s
        self.down_arrow(bx + iw / 2, y, 8 * s)
        y += 12 * s

        # ③ PID
        h3 = 28 * s + 6 * 19 * s + 30 * s
        yy = self.box(bx, y, iw, h3, "③ PID  (mm → 度)")
        if stat:
            tx, ty = self.val("tx", stat), self.val("ty", stat)
            ex, ey = stat["x"] - tx, stat["y"] - ty
            rows = [("目標 (mm)", f"({tx:+.1f}, {ty:+.1f})"),
                    ("誤差 (mm)", f"({ex:+.1f}, {ey:+.1f})" if present else "—", TEXT if present else DIM),
                    ("積分 (mm·s)", f"({stat['int_x']:+.1f}, {stat['int_y']:+.1f})"),
                    ("增益 Kp/Ki/Kd", f"{stat['kp']:.3f}/{stat['ki']:.3f}/{stat['kd']:.3f}"),
                    ("輸出 傾角 θ", f"{stat['theta']:.2f}°", WARN),
                    ("輸出 方向 φ", f"{stat['phi']:.1f}°", WARN)]
        else:
            rows = [(a, "—", DIM) for a in ("目標 (mm)", "誤差 (mm)", "積分 (mm·s)", "增益 Kp/Ki/Kd", "輸出 傾角 θ", "輸出 方向 φ")]
        yy = kvs(yy, rows)
        if stat:  # 傾角佔最大傾角的比例
            frac = min(stat["theta"] / max(stat["max_tilt"], 1e-3), 1.0)
            r.rect(bx + 12 * s, yy + 2 * s, iw - 24 * s, 7 * s, BG)
            r.rect(bx + 12 * s, yy + 2 * s, (iw - 24 * s) * frac, 7 * s, BAD if frac > 0.9 else WARN)
            r.text(f"最大傾角 {stat['max_tilt']:.1f}°", bx + iw - 12 * s, yy + 12 * s, 10.5 * s, DIM, "tr")
        y += h3 + 4 * s
        self.down_arrow(bx + iw / 2, y, 8 * s)
        y += 12 * s

        # ④ 伺服
        h4 = 28 * s + 3 * 27 * s + 6 * s
        yy = self.box(bx, y, iw, h4, "④ 逆向運動學 → 伺服角度")
        for i in range(3):
            ang = stat[f"s{i + 1}"] if stat else None
            r.text(f"腳{i + 1} CH{SERVO_CHANNELS[i]}", bx + 12 * s, yy, 12.5 * s, DIM, "tl")
            r.text(f"{ang:.1f}°" if ang is not None else "—", bx + iw - 12 * s, yy, 12.5 * s, TEXT, "tr", bold=True)
            r.rect(bx + 12 * s, yy + 17 * s, iw - 24 * s, 5 * s, BG)
            if ang is not None:
                f = (ang - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG)
                near = f < 0.04 or f > 0.96
                r.rect(bx + 12 * s, yy + 17 * s, (iw - 24 * s) * min(max(f, 0), 1), 5 * s, BAD if near else GOOD)
            yy += 27 * s
        y += h4 + 10 * s

        # 底部：模式 / 連線
        mode = MODE_NAMES[int(stat["mode"])] if stat and 0 <= int(stat["mode"]) < 3 else "—"
        r.text(f"模式  {mode}", bx, y, 13 * s, WARN if stat and stat["mode"] != 0 else GOOD, "tl", bold=True)
        y += 20 * s
        col = GOOD if d["connected"] else BAD
        r.circle(bx + 5 * s, y + 8 * s, 4 * s, col)
        age = d["stat_age"]
        extra = f"  遙測 {age:.1f}s 前" if age is not None and age > 1.0 else ""
        r.text(d["status"] + extra, bx + 14 * s, y, 12 * s, col, "tl")

    def center(self, d, x0, x1, h):
        r, s = self.r, self.s
        n, stat = d["n"], d["stat"]
        top = 10 * s
        r.text("VL53L7CX 距離圖 (mm)", x0, top, 15 * s, ACCENT, "tl", bold=True)
        log_h = 112 * s
        legend_h = 30 * s
        y0 = top + 28 * s
        y1 = h - log_h - legend_h - 14 * s
        side = max(min(x1 - x0, y1 - y0), 40)
        gx0, gy0 = x0 + (x1 - x0 - side) / 2, y0
        if n:
            cell = side / n
            for yy in range(n):
                for xx in range(n):
                    dv = d["dist"][yy, xx]
                    px, py = gx0 + xx * cell, gy0 + yy * cell
                    r.rect(px + 1, py + 1, cell - 2, cell - 2, dist_color(dv))
                    if cell >= 30 * s * 0.8:
                        r.text(f"{int(dv)}" if dv > 0 else "×", px + 5 * s, py + 4 * s,
                               min(cell * 0.2, 16 * s), (255, 255, 255) if dv > 0 else DIM, "tl")
            cxp, cyp = gx0 + side / 2, gy0 + side / 2
            for a in (0.0, 1.0):  # 盤面中心十字
                r.line(cxp - 8 * s, cyp, cxp + 8 * s, cyp, 1.2, TEXT, 0.5)
                r.line(cxp, cyp - 8 * s, cxp, cyp + 8 * s, 1.2, TEXT, 0.5)
            if stat and stat["theta"] > 0.05:  # 傾斜方向（下傾側）
                L = side * 0.38 * min(stat["theta"] / max(stat["max_tilt"], 1e-3), 1.0) + 10 * s
                ph = math.radians(stat["phi"])
                r.arrow(cxp, cyp, cxp + L * math.cos(ph), cyp + L * math.sin(ph), 3 * s, WARN, 12 * s, 0.9)
            if d["cx"] >= 0:
                bx = gx0 + (d["cx"] + 0.5) * cell
                by = gy0 + (d["cy"] + 0.5) * cell
                rr = max(cell * 0.18, 6 * s)
                r.ring(bx, by, rr, 3 * s, GOOD)
                r.circle(bx, by, 2.5 * s, GOOD)
        else:
            r.rect(gx0, gy0, side, side, PANEL)
            r.text("等待 DATA 資料…", gx0 + side / 2, gy0 + side / 2, 15 * s, DIM, "mc")
        r.frame(gx0, gy0, side, side, BORDER, 1)

        # 色階
        ly = gy0 + side + 8 * s
        steps = 50
        for i in range(steps):
            r.rect(gx0 + side * i / steps, ly, side / steps + 1, 8 * s, dist_color(MIN_DISTANCE + (MAX_DISTANCE - MIN_DISTANCE) * i / (steps - 1)))
        r.text(f"{MIN_DISTANCE} mm", gx0, ly + 11 * s, 11 * s, DIM, "tl")
        r.text(f"{MAX_DISTANCE} mm", gx0 + side, ly + 11 * s, 11 * s, DIM, "tr")
        r.text("綠圈：球　橘箭頭：平台下傾方向　×：無效", gx0 + side / 2, ly + 11 * s, 11 * s, DIM, "tc")

        # 韌體訊息
        ly0 = h - log_h - 6 * s
        r.rect(x0, ly0, x1 - x0, log_h, PANEL)
        r.frame(x0, ly0, x1 - x0, log_h, BORDER, 1)
        r.text("韌體訊息", x0 + 8 * s, ly0 + 4 * s, 11 * s, DIM, "tl", bold=True)
        lh = 15 * s
        rows = int((log_h - 24 * s) // lh)
        for i, ln in enumerate(d["log"][-rows:] if rows > 0 else []):
            col = ACCENT if ln.startswith(">>") else DIM if ln.startswith(("→", "[")) else TEXT
            r.text(ln[:140], x0 + 8 * s, ly0 + 20 * s + i * lh, 11.5 * s, col, "tl")

    def right_panel(self, d, x, w, h):
        r, s, ui = self.r, self.s, self.ui
        stat = d["stat"]
        ok = stat is not None
        pad = 12 * s
        bx, iw = x + pad, w - 2 * pad
        y = 10 * s
        r.text("控制  Controls", bx, y, 15 * s, ACCENT, "tl", bold=True)
        y += 28 * s
        rh = 24 * s
        n = d["n"]

        # 解析度 / 模式
        yy = self.box(bx, y, iw, 28 * s + 2 * (30 * s) + 6 * s, "感測器與模式", ACCENT)
        half = (iw - 24 * s - 6 * s) / 2
        if ui.button("res4", (bx + 12 * s, yy, half, 26 * s), "4×4  60Hz", 12.5 * s, HINTS["res4"], active=(n == 4)):
            self.link.send("4")
        if ui.button("res8", (bx + 18 * s + half, yy, half, 26 * s), "8×8  15Hz", 12.5 * s, HINTS["res8"], active=(n == 8)):
            self.link.send("8")
        yy += 30 * s
        if ui.button("auto", (bx + 12 * s, yy, half, 26 * s), "自動平衡 (R)", 12.5 * s, HINTS["auto"],
                     active=bool(ok and stat["mode"] == 0)):
            self.link.send("r")
        if ui.button("level", (bx + 18 * s + half, yy, half, 26 * s), "回水平 (L)", 12.5 * s, HINTS["level"]):
            self.link.send("l")
        y += 28 * s + 2 * 30 * s + 6 * s + 8 * s

        # PID
        hp = 28 * s + 3 * (rh + 6 * s) + 4 * s
        yy = self.box(bx, y, iw, hp, "PID 增益", WARN)
        for key, label, step, lo in (("kp", "Kp  度/mm", 0.01, 0.0), ("ki", "Ki  度/(mm·s)", 0.002, 0.0),
                                     ("kd", "Kd  度/(mm/s)", 0.01, 0.0)):
            v = self.val(key, stat)
            dl = ui.stepper("pid" + key, (bx + 12 * s, yy, iw - 24 * s, rh), label,
                            f"{v:.3f}" if ok else "—", 12.5 * s, HINTS[key], enabled=ok)
            if dl and ok:
                self.send_pid(stat, **{key: round(max(lo, v + dl * step), 4)})
            yy += rh + 6 * s
        y += hp + 8 * s

        # 目標點 / 高度
        ht = 28 * s + 3 * (rh + 6 * s) + 4 * s
        yy = self.box(bx, y, iw, ht, "目標點與高度", GOOD)
        tx, ty = self.val("tx", stat), self.val("ty", stat)
        dl = ui.stepper("tx", (bx + 12 * s, yy, iw - 24 * s, rh), "目標 X (mm)", f"{tx:+.1f}" if ok else "—", 12.5 * s, HINTS["tx"], enabled=ok)
        if dl and ok:
            self.send_target(stat, tx + dl, ty)
        yy += rh + 6 * s
        dl = ui.stepper("ty", (bx + 12 * s, yy, iw - 24 * s, rh), "目標 Y (mm)", f"{ty:+.1f}" if ok else "—", 12.5 * s, HINTS["ty"], enabled=ok)
        if dl and ok:
            self.send_target(stat, tx, ty + dl)
        yy += rh + 6 * s
        hv = self.val("height", stat)
        dl = ui.stepper("h", (bx + 12 * s, yy, iw - 24 * s, rh), "平台高度 (mm)", f"{hv:.1f}" if ok else "—", 12.5 * s, HINTS["height"], enabled=ok)
        if dl and ok:
            nh = round(hv + dl * 1.0, 1)
            self.set_ov("height", nh)
            self.link.send(f"h {nh:.1f}")
        # 置中按鈕放在標題列右側
        if ui.button("center", (bx + iw - 64 * s, y + 3 * s, 56 * s, 20 * s), "置中", 11.5 * s, HINTS["center"], enabled=ok):
            self.send_target(stat, 0.0, 0.0)
        y += ht + 8 * s

        # 傾斜測試
        hq = 28 * s + 2 * (rh + 6 * s) + 32 * s
        yy = self.box(bx, y, iw, hq, "傾斜測試（手動）", ACCENT)
        mt = stat["max_tilt"] if ok else 15.0
        dl = ui.stepper("th", (bx + 12 * s, yy, iw - 24 * s, rh), "傾角 θ (°)", f"{self.test_theta:.1f}", 12.5 * s, HINTS["theta"])
        self.test_theta = min(max(self.test_theta + dl * 0.5, 0.0), mt)
        yy += rh + 6 * s
        dl = ui.stepper("ph", (bx + 12 * s, yy, iw - 24 * s, rh), "方向 φ (°)", f"{self.test_phi:.0f}", 12.5 * s, HINTS["phi"])
        self.test_phi = (self.test_phi + dl * 15.0) % 360.0
        yy += rh + 6 * s
        if ui.button("tilt", (bx + 12 * s, yy, iw - 24 * s, 24 * s), "送出傾斜", 12.5 * s, HINTS["tilt"]):
            self.link.send(f"t {self.test_theta:.1f} {self.test_phi:.0f}")
        y += hq + 8 * s

        # 指令輸入
        cy = y
        r.text("序列埠指令", bx, cy, 12 * s, DIM, "tl", bold=True)
        cy += 18 * s
        box = (bx, cy, iw, 26 * s)
        if ui.inside(box):
            ui.hint = HINTS["cmd"]
        for c in ui.clicks:
            self.cmd_focus = ui.inside(box, c)
        r.rect(*box, BG)
        r.frame(*box, ACCENT if self.cmd_focus else BORDER, 1)
        if self.cmd_text or self.cmd_focus:
            caret = "|" if self.cmd_focus and int(time.time() * 2) % 2 == 0 else ""
            r.text(self.cmd_text + caret, bx + 8 * s, cy + 13 * s, 12.5 * s, TEXT, "ml")
        else:
            r.text("按 / 或點此輸入，Enter 送出", bx + 8 * s, cy + 13 * s, 12 * s, DIM, "ml")
        y = cy + 26 * s + 10 * s

        # 提示
        hh = h - y - 10 * s
        if hh > 40 * s:
            r.rect(bx, y, iw, hh, PANEL)
            r.frame(bx, y, iw, hh, BORDER, 1)
            r.text("提示", bx + 10 * s, y + 5 * s, 12 * s, ACCENT, "tl", bold=True)
            text = ui.hint or DEFAULT_HINT
            ly = y + 24 * s
            for ln in wrap_text(r, text, 12 * s, iw - 20 * s):
                if ly + 15 * s > y + hh:
                    break
                r.text(ln, bx + 10 * s, ly, 12 * s, TEXT if ui.hint else DIM, "tl")
                ly += 16 * s

    def frame(self):
        w, h = pygame.display.get_window_size()
        self.size = (w, h) if not self.fullscreen else self.size
        self.s = min(max(min(w / 1280.0, h / 760.0), 0.55), 3.0)
        s = self.s
        r, ui = self.r, self.ui
        r.begin(w, h)
        ui.hint = None
        d = self.tel.snapshot()
        lw, rw = 318 * s, 330 * s
        if w < lw + rw + 220:  # 視窗太窄時壓縮兩側
            lw, rw = (w - 220) * 0.49, (w - 220) * 0.51
            lw, rw = max(lw, 200), max(rw, 200)
        r.rect(0, 0, lw, h, PANEL)
        r.rect(w - rw, 0, rw, h, PANEL)
        r.rect(lw - 1, 0, 1, h, BORDER)
        r.rect(w - rw, 0, 1, h, BORDER)
        self.left_panel(d, 0, lw)
        self.center(d, lw + 12 * s, w - rw - 12 * s, h)
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
        img = pygame.transform.flip(pygame.image.frombuffer(data, (w, h), "RGB"), False, True)
        pygame.image.save(img, self.snapshot)
        print(f"已儲存畫面 {self.snapshot} ({w}x{h})")


def main():
    ap = argparse.ArgumentParser(description="平衡球平台監控介面")
    ap.add_argument("--port", help="序列埠（預設自動偵測）")
    ap.add_argument("--baud", type=int, default=BAUD_RATE)
    ap.add_argument("--demo", action="store_true", help="使用模擬資料，不連接硬體")
    ap.add_argument("--size", default="1280x760", help="初始視窗大小，例 1600x900")
    ap.add_argument("--snapshot", help="執行數秒後把畫面存成 PNG 並結束（除錯用）")
    ap.add_argument("--frames", type=int, default=90, help="--snapshot 前先渲染的幀數")
    ap.add_argument("--list-ports", action="store_true", help="列出序列埠後結束")
    args = ap.parse_args()

    if args.list_ports:
        for p in serial.tools.list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return
    try:
        size = tuple(int(v) for v in args.size.lower().split("x"))
        assert len(size) == 2
    except (ValueError, AssertionError):
        sys.exit("--size 格式應為 寬x高，例如 1280x760")

    tel = Telemetry()
    link = DemoLink(tel) if args.demo else SerialLink(tel, args.port, args.baud)
    link.start()
    try:
        App(link, tel, size, args.snapshot, args.frames).run()
    finally:
        link.stop()


if __name__ == "__main__":
    main()
