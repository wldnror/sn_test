import os
import csv
import math
import time
import threading
from datetime import datetime
from collections import deque

import spidev
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# =========================================================
# FONT
# =========================================================
FONT_PATHS = [
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
]

for path in FONT_PATHS:
    if os.path.exists(path):
        fm.fontManager.addfont(path)
        matplotlib.rcParams["font.family"] = fm.FontProperties(fname=path).get_name()
        break

matplotlib.rcParams["axes.unicode_minus"] = False
matplotlib.rcParams["toolbar"] = "None"


# =========================================================
# FILES
# =========================================================
DATA_DIR = "bme688_data"
os.makedirs(DATA_DIR, exist_ok=True)

BASELINE_CSV = os.path.join(DATA_DIR, "baseline_auto.csv")
SAMPLES_CSV = os.path.join(DATA_DIR, "samples.csv")
RAW_CSV = os.path.join(DATA_DIR, "raw_log.csv")


# =========================================================
# SPI / BME688
# =========================================================
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0

REG_CHIP_ID = 0xD0
REG_RESET = 0xE0
REG_STATUS = 0x73
REG_CTRL_GAS_0 = 0x70
REG_CTRL_GAS_1 = 0x71
REG_CTRL_HUM = 0x72
REG_CTRL_MEAS = 0x74
REG_CONFIG = 0x75
REG_RES_HEAT_0 = 0x5A
REG_GAS_WAIT_0 = 0x64
REG_FIELD0 = 0x1D
REG_RANGE_SW_ERR = 0x04

GAS_LOOKUP_1 = [
    2147483647, 2147483647, 2147483647, 2147483647,
    2147483647, 2126008810, 2147483647, 2130303777,
    2147483647, 2147483647, 2143188679, 2136746228,
    2147483647, 2126008810, 2147483647, 2147483647
]

GAS_LOOKUP_2 = [
    4096000000, 2048000000, 1024000000, 512000000,
    255744255, 127110228, 64000000, 32258064,
    16016016, 8000000, 4000000, 2000000,
    1000000, 500000, 250000, 125000
]

cal = {}
t_fine = 0.0


def read_reg_raw(reg):
    return spi.xfer2([reg | 0x80, 0x00])[1]


def write_reg_raw(reg, value):
    spi.xfer2([reg & 0x7F, value & 0xFF])


def set_mem_page(reg):
    status = read_reg_raw(REG_STATUS)

    if reg < 0x80:
        status |= 0x10
    else:
        status &= ~0x10

    write_reg_raw(REG_STATUS, status)
    time.sleep(0.001)


def read_reg(reg):
    set_mem_page(reg)
    return read_reg_raw(reg)


def write_reg(reg, value):
    set_mem_page(reg)
    write_reg_raw(reg, value)


def read_regs(reg, length):
    set_mem_page(reg)
    return spi.xfer2([reg | 0x80] + [0x00] * length)[1:]


def u16(lsb, msb):
    return (msb << 8) | lsb


def s16(lsb, msb):
    v = u16(lsb, msb)
    return v - 65536 if v & 0x8000 else v


def s8(v):
    return v - 256 if v & 0x80 else v


def read_calibration():
    b1 = read_regs(0x89, 25)
    b2 = read_regs(0xE1, 16)

    cal["par_t1"] = u16(b2[8], b2[9])
    cal["par_t2"] = s16(b1[1], b1[2])
    cal["par_t3"] = s8(b1[3])

    cal["par_p1"] = u16(b1[5], b1[6])
    cal["par_p2"] = s16(b1[7], b1[8])
    cal["par_p3"] = s8(b1[9])
    cal["par_p4"] = s16(b1[11], b1[12])
    cal["par_p5"] = s16(b1[13], b1[14])
    cal["par_p6"] = s8(b1[16])
    cal["par_p7"] = s8(b1[15])
    cal["par_p8"] = s16(b1[19], b1[20])
    cal["par_p9"] = s16(b1[21], b1[22])
    cal["par_p10"] = b1[23]

    cal["par_h1"] = (b2[2] << 4) | (b2[1] & 0x0F)
    cal["par_h2"] = (b2[0] << 4) | (b2[1] >> 4)
    cal["par_h3"] = s8(b2[3])
    cal["par_h4"] = s8(b2[4])
    cal["par_h5"] = s8(b2[5])
    cal["par_h6"] = b2[6]
    cal["par_h7"] = s8(b2[7])

    cal["range_sw_err"] = (read_reg(REG_RANGE_SW_ERR) & 0xF0) >> 4


def compensate_temp(temp_adc):
    global t_fine

    var1 = ((temp_adc / 16384.0) - (cal["par_t1"] / 1024.0)) * cal["par_t2"]
    var2 = (((temp_adc / 131072.0) - (cal["par_t1"] / 8192.0)) ** 2) * (cal["par_t3"] * 16.0)
    t_fine = var1 + var2

    return t_fine / 5120.0


def compensate_pressure(press_adc):
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * cal["par_p6"] / 131072.0
    var2 += var1 * cal["par_p5"] * 2.0
    var2 = (var2 / 4.0) + (cal["par_p4"] * 65536.0)

    var1 = ((cal["par_p3"] * var1 * var1 / 16384.0) + (cal["par_p2"] * var1)) / 524288.0
    var1 = (1.0 + (var1 / 32768.0)) * cal["par_p1"]

    if var1 == 0:
        return 0.0

    pressure = 1048576.0 - press_adc
    pressure = ((pressure - (var2 / 4096.0)) * 6250.0) / var1

    var1 = cal["par_p9"] * pressure * pressure / 2147483648.0
    var2 = pressure * cal["par_p8"] / 32768.0
    var3 = (pressure / 256.0) ** 3 * (cal["par_p10"] / 131072.0)

    pressure += (var1 + var2 + var3 + (cal["par_p7"] * 128.0)) / 16.0

    return pressure / 100.0


def compensate_humidity(hum_adc, temp_c):
    var1 = hum_adc - ((cal["par_h1"] * 16.0) + ((cal["par_h3"] / 2.0) * temp_c))
    var2 = var1 * (
        (cal["par_h2"] / 262144.0)
        * (1.0 + (cal["par_h4"] / 16384.0) * temp_c + (cal["par_h5"] / 1048576.0) * temp_c * temp_c)
    )
    var3 = cal["par_h6"] / 16384.0
    var4 = cal["par_h7"] / 2097152.0

    humidity = var2 + ((var3 + (var4 * temp_c)) * var2 * var2)

    return max(0.0, min(100.0, humidity))


def calc_gas_resistance(gas_adc, gas_range):
    if gas_adc == 0:
        return 0.0

    var1 = ((1340 + (5 * cal["range_sw_err"])) * GAS_LOOKUP_1[gas_range]) / 65536.0
    var2 = ((gas_adc * 32768.0) - 16777216.0) + var1
    var3 = (GAS_LOOKUP_2[gas_range] * var1) / 512.0

    if var2 == 0:
        return 0.0

    return var3 / var2


def sensor_init():
    chip_id = read_reg(REG_CHIP_ID)
    print("Chip ID:", hex(chip_id))

    if chip_id != 0x61:
        raise RuntimeError("BME688/BME680 chip ID error")

    write_reg(REG_RESET, 0xB6)
    time.sleep(0.2)

    read_calibration()

    write_reg(REG_CONFIG, 0x08)
    write_reg(REG_CTRL_HUM, 0x01)
    write_reg(REG_CTRL_GAS_0, 0x00)
    write_reg(REG_RES_HEAT_0, 0x73)
    write_reg(REG_GAS_WAIT_0, 0x59)
    write_reg(REG_CTRL_GAS_1, 0x20)


def trigger_measurement():
    ctrl_meas = (0b010 << 5) | (0b101 << 2) | 0b01
    write_reg(REG_CTRL_MEAS, ctrl_meas)


def get_season(month):
    if month in [3, 4, 5]:
        return "봄"
    if month in [6, 7, 8]:
        return "여름"
    if month in [9, 10, 11]:
        return "가을"
    return "겨울"


def get_period(hour):
    if 0 <= hour < 6:
        return "새벽"
    if 6 <= hour < 12:
        return "오전"
    if 12 <= hour < 18:
        return "오후"
    return "야간"


def read_sensor():
    trigger_measurement()
    time.sleep(0.8)

    data = read_regs(REG_FIELD0, 17)

    pressure_adc = (data[2] << 12) | (data[3] << 4) | (data[4] >> 4)
    temp_adc = (data[5] << 12) | (data[6] << 4) | (data[7] >> 4)
    hum_adc = (data[8] << 8) | data[9]

    gas_msb = data[15]
    gas_lsb = data[16]

    gas_adc = (gas_msb << 2) | (gas_lsb >> 6)
    gas_range = gas_lsb & 0x0F
    gas_valid = bool(gas_lsb & 0x20)
    heat_stable = bool(gas_lsb & 0x10)

    temp_c = compensate_temp(temp_adc)
    press_hpa = compensate_pressure(pressure_adc)
    hum_pct = compensate_humidity(hum_adc, temp_c)
    gas_ohm = calc_gas_resistance(gas_adc, gas_range)

    now = datetime.now()

    return {
        "timestamp": now.isoformat(timespec="seconds"),
        "epoch": time.time(),
        "season": get_season(now.month),
        "period": get_period(now.hour),
        "hour": now.hour,
        "temp_c": temp_c,
        "hum_pct": hum_pct,
        "press_hpa": press_hpa,
        "gas_ohm": gas_ohm,
        "gas_adc": gas_adc,
        "gas_range": gas_range,
        "gas_valid": gas_valid,
        "heat_stable": heat_stable,
    }


# =========================================================
# DATA / CLASSIFIER
# =========================================================
def mean(values):
    return sum(values) / len(values) if values else 0.0


def stdev(values):
    if len(values) < 2:
        return 0.0

    m = mean(values)
    return math.sqrt(sum((x - m) ** 2 for x in values) / (len(values) - 1))


def extract_features(rows, label, baseline_gas=None):
    gas = [r["gas_ohm"] for r in rows if r["gas_ohm"] > 0]
    temp = [r["temp_c"] for r in rows]
    hum = [r["hum_pct"] for r in rows]
    press = [r["press_hpa"] for r in rows]

    if not gas:
        return None

    gas_start = mean(gas[:max(3, min(10, len(gas)))])
    gas_end = mean(gas[-max(3, min(10, len(gas))):])
    gas_min = min(gas)
    gas_max = max(gas)
    gas_avg = mean(gas)

    ref = baseline_gas if baseline_gas and baseline_gas > 0 else gas_start
    change_pct = ((gas_min - ref) / ref) * 100.0 if ref else 0.0

    now = datetime.now()

    return {
        "id": now.strftime("%Y%m%d_%H%M%S"),
        "timestamp": now.isoformat(timespec="seconds"),
        "label": label,
        "season": get_season(now.month),
        "period": get_period(now.hour),
        "hour": now.hour,
        "count": len(rows),
        "duration_sec": rows[-1]["epoch"] - rows[0]["epoch"] if len(rows) >= 2 else 0,
        "gas_avg": gas_avg,
        "gas_min": gas_min,
        "gas_max": gas_max,
        "gas_start": gas_start,
        "gas_end": gas_end,
        "gas_stdev": stdev(gas),
        "gas_change_pct": change_pct,
        "temp_avg": mean(temp),
        "hum_avg": mean(hum),
        "press_avg": mean(press),
    }


def save_dict_csv(path, row):
    exists = os.path.exists(path)

    with open(path, "a", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))

        if not exists:
            writer.writeheader()

        writer.writerow(row)


def load_csv(path):
    if not os.path.exists(path):
        return []

    with open(path, "r", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    if not rows:
        if os.path.exists(path):
            os.remove(path)
        return

    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def count_labels():
    rows = load_csv(SAMPLES_CSV)
    counts = {"AIR": 0, "ETHANOL": 0, "IPA": 0}

    for r in rows:
        label = r.get("label", "")

        if label in counts:
            counts[label] += 1

    return counts


def feature_distance(a, b):
    keys = [
        ("gas_change_pct", 2.5),
        ("gas_min", 0.0008),
        ("gas_avg", 0.0006),
        ("gas_stdev", 0.001),
        ("temp_avg", 0.6),
        ("hum_avg", 0.25),
        ("press_avg", 0.05),
    ]

    total = 0.0

    for key, scale in keys:
        av = float(a.get(key, 0) or 0)
        bv = float(b.get(key, 0) or 0)
        total += abs(av - bv) * scale

    return total


def classify(feature):
    samples = load_csv(SAMPLES_CSV)
    classes = ["AIR", "ETHANOL", "IPA"]
    scores = {c: 0.0 for c in classes}

    usable = [s for s in samples if s.get("label") in classes]

    if not usable:
        return {"AIR": 0, "ETHANOL": 0, "IPA": 0}, "학습 데이터 없음"

    for s in usable:
        d = feature_distance(feature, s)
        weight = 1.0 / (1.0 + d)

        if s.get("season") == feature.get("season"):
            weight *= 1.15

        if s.get("period") == feature.get("period"):
            weight *= 1.15

        scores[s["label"]] += weight

    total = sum(scores.values())

    if total <= 0:
        return {"AIR": 0, "ETHANOL": 0, "IPA": 0}, "판별 불가"

    pct = {k: round(v / total * 100, 1) for k, v in scores.items()}
    winner = max(pct, key=pct.get)

    return pct, winner


# =========================================================
# APP
# =========================================================
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("BME688 학습/판별 대시보드")
        self.root.attributes("-fullscreen", True)
        self.fullscreen = True

        self.running = True

        self.auto_baseline = True
        self.realtime_detect = False

        self.current_rows = deque(maxlen=300)
        self.detect_rows = deque(maxlen=45)
        self.baseline_buffer = []

        self.latest_air_gas = None
        self.baseline_save_count = len(load_csv(BASELINE_CSV))

        self.sample_mode = None
        self.pending_label = None
        self.sample_rows = []
        self.sample_phase = None
        self.phase_until = 0

        self.detect_history = deque(maxlen=3)

        self.bg = "#101820"
        self.card = "#182632"
        self.text = "#EAF2F8"
        self.muted = "#94A9B8"

        self.root.configure(bg=self.bg)

        self.build_ui()

        self.sensor_thread = threading.Thread(target=self.loop, daemon=True)
        self.sensor_thread.start()

        self.update_ui()

    # -----------------------------------------------------
    # UI BUILD
    # -----------------------------------------------------
    def build_ui(self):
        top = tk.Frame(self.root, bg=self.bg)
        top.pack(fill="x", padx=12, pady=8)

        self.status_label = tk.Label(
            top,
            text="상태: 준비중",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 15, "bold"),
        )
        self.status_label.pack(side="left", padx=8)

        buttons = [
            ("화면모드", self.toggle_fullscreen),
            ("기준기록", self.toggle_auto),
            ("학습", self.open_learn_menu),
            ("판별", self.open_detect_menu),
            ("데이터관리", self.show_data_manager),
            ("종료", self.close),
        ]

        for txt, cmd in buttons:
            tk.Button(
                top,
                text=txt,
                command=cmd,
                font=("NanumGothic", 12, "bold"),
                bg="#263847",
                fg=self.text,
                activebackground="#365369",
                activeforeground="white",
                relief="flat",
                padx=16,
                pady=8,
            ).pack(side="right", padx=4)

        info = tk.Frame(self.root, bg=self.bg)
        info.pack(fill="x", padx=12)

        self.cards = {}

        for name in ["현재상태", "가스저항", "온도", "습도", "기압", "판별확률"]:
            f = tk.Frame(info, bg=self.card, padx=14, pady=10)
            f.pack(side="left", fill="x", expand=True, padx=5, pady=5)

            title = tk.Label(f, text=name, bg=self.card, fg=self.muted, font=("NanumGothic", 11))
            title.pack(anchor="w")

            value = tk.Label(f, text="-", bg=self.card, fg=self.text, font=("NanumGothic", 18, "bold"))
            value.pack(anchor="w")

            self.cards[name] = value

        self.fig = plt.Figure(figsize=(14, 7), facecolor=self.bg)
        self.ax_gas = self.fig.add_subplot(2, 1, 1)
        self.ax_env = self.fig.add_subplot(2, 1, 2)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=12, pady=8)

    # -----------------------------------------------------
    # SIMPLE POPUP MENUS
    # -----------------------------------------------------
    def open_learn_menu(self):
        win = tk.Toplevel(self.root)
        win.title("학습 선택")
        win.geometry("360x260")
        win.configure(bg=self.bg)

        tk.Label(
            win,
            text="학습할 대상을 선택하세요",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 16, "bold"),
        ).pack(pady=18)

        items = [
            ("정상공기 학습", "AIR"),
            ("에탄올 학습", "ETHANOL"),
            ("IPA 학습", "IPA"),
        ]

        for title, label in items:
            tk.Button(
                win,
                text=title,
                command=lambda l=label, w=win: (w.destroy(), self.start_sample(l)),
                font=("NanumGothic", 13, "bold"),
                bg="#263847",
                fg=self.text,
                relief="flat",
                padx=20,
                pady=10,
            ).pack(fill="x", padx=28, pady=6)

    def open_detect_menu(self):
        win = tk.Toplevel(self.root)
        win.title("판별 선택")
        win.geometry("390x260")
        win.configure(bg=self.bg)

        tk.Label(
            win,
            text="판별 방식을 선택하세요",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 16, "bold"),
        ).pack(pady=18)

        rt_text = "실시간 판별 끄기" if self.realtime_detect else "실시간 판별 켜기"

        tk.Button(
            win,
            text=rt_text,
            command=lambda w=win: (w.destroy(), self.toggle_realtime()),
            font=("NanumGothic", 13, "bold"),
            bg="#263847",
            fg=self.text,
            relief="flat",
            padx=20,
            pady=10,
        ).pack(fill="x", padx=28, pady=8)

        tk.Button(
            win,
            text="미지시료 수동 판별",
            command=lambda w=win: (w.destroy(), self.start_unknown()),
            font=("NanumGothic", 13, "bold"),
            bg="#263847",
            fg=self.text,
            relief="flat",
            padx=20,
            pady=10,
        ).pack(fill="x", padx=28, pady=8)

    # -----------------------------------------------------
    # STATUS / MODE
    # -----------------------------------------------------
    def update_status(self, text=None):
        counts = count_labels()

        detect_state = "실시간판별 ON" if self.realtime_detect else "실시간판별 OFF"
        base_state = "AIR기준기록 ON" if self.auto_baseline else "AIR기준기록 OFF"
        buffer_state = f"버퍼 {len(self.baseline_buffer)}/60"
        save_state = f"AIR기준 {self.baseline_save_count}회"
        learn_state = f"AIR {counts['AIR']} / ETH {counts['ETHANOL']} / IPA {counts['IPA']}"

        if text:
            msg = f"상태: {text} | {detect_state} | {base_state} | {buffer_state} | {save_state} | {learn_state}"
        else:
            msg = f"상태: {detect_state} | {base_state} | {buffer_state} | {save_state} | {learn_state}"

        self.status_label.config(text=msg)

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def toggle_auto(self):
        if self.realtime_detect:
            messagebox.showinfo(
                "안내",
                "실시간 판별 중에는 AIR 기준 기록을 켤 수 없습니다.\n"
                "먼저 실시간 판별을 끄세요."
            )
            self.auto_baseline = False
        else:
            self.auto_baseline = not self.auto_baseline

        self.update_status()

    def toggle_realtime(self):
        self.realtime_detect = not self.realtime_detect

        if self.realtime_detect:
            self.auto_baseline = False
            self.baseline_buffer = []
            self.cards["현재상태"].config(text="실시간 판별중", fg="#00E5FF")
            self.update_status("실시간 판별 ON / AIR 기준 기록 자동 OFF")
        else:
            self.detect_history.clear()
            self.cards["현재상태"].config(text="판별 대기", fg=self.text)
            self.cards["판별확률"].config(text="-")
            self.update_status("실시간 판별 OFF")

    # -----------------------------------------------------
    # SAMPLING
    # -----------------------------------------------------
    def start_sample(self, label):
        self.realtime_detect = False
        self.auto_baseline = False
        self.baseline_buffer = []

        self.sample_mode = "LEARN"
        self.pending_label = label
        self.sample_rows = []
        self.sample_phase = "READY"
        self.phase_until = time.time() + 5

        self.update_status(f"{label} 학습 준비 5초 / AIR 기준 기록 일시정지")

    def start_unknown(self):
        self.realtime_detect = False
        self.auto_baseline = False
        self.baseline_buffer = []

        self.sample_mode = "UNKNOWN"
        self.pending_label = "UNKNOWN"
        self.sample_rows = []
        self.sample_phase = "READY"
        self.phase_until = time.time() + 5

        self.update_status("미지시료 수동 판별 준비 5초 / AIR 기준 기록 일시정지")

    def finish_sample(self):
        feature = extract_features(self.sample_rows, self.pending_label, self.latest_air_gas)

        if not feature:
            self.update_status("샘플 실패")
            self.sample_mode = None
            return

        if self.sample_mode == "LEARN":
            save_dict_csv(SAMPLES_CSV, feature)
            self.cards["현재상태"].config(text=f"{self.pending_label} 학습완료", fg="#4DFF88")
            self.update_status(f"{self.pending_label} 학습 저장 완료")
        else:
            pct, winner = classify(feature)
            self.cards["현재상태"].config(text=f"{winner}", fg="#FFD166")
            self.cards["판별확률"].config(
                text=f"AIR {pct['AIR']}% / ETH {pct['ETHANOL']}% / IPA {pct['IPA']}%"
            )
            self.update_status("수동 판별 완료")

        self.sample_mode = None
        self.pending_label = None
        self.sample_rows = []
        self.sample_phase = None

    # -----------------------------------------------------
    # REALTIME CLASSIFY
    # -----------------------------------------------------
    def run_realtime_detect(self):
        if not self.realtime_detect:
            return

        if self.sample_mode is not None:
            return

        if len(self.detect_rows) < 25:
            self.cards["현재상태"].config(text="데이터 수집중", fg=self.text)
            self.cards["판별확률"].config(text="-")
            return

        feature = extract_features(list(self.detect_rows), "REALTIME", self.latest_air_gas)

        if not feature:
            return

        pct, winner = classify(feature)

        if winner in ["학습 데이터 없음", "판별 불가"]:
            self.cards["현재상태"].config(text=winner, fg=self.text)
            self.cards["판별확률"].config(text="학습 필요")
            return

        best_pct = pct[winner]
        self.detect_history.append(winner)

        stable = len(self.detect_history) == 3 and len(set(self.detect_history)) == 1
        confident = best_pct >= 60.0

        if stable and confident:
            if winner == "AIR":
                state = "정상공기"
                color = "#4DFF88"
            elif winner == "ETHANOL":
                state = "에탄올 감지"
                color = "#FF6B6B"
            elif winner == "IPA":
                state = "IPA 감지"
                color = "#FFD166"
            else:
                state = winner
                color = self.text
        else:
            state = f"{winner} 의심"
            color = "#00E5FF"

        self.cards["현재상태"].config(text=f"{state} {best_pct:.1f}%", fg=color)
        self.cards["판별확률"].config(
            text=f"AIR {pct['AIR']}% / ETH {pct['ETHANOL']}% / IPA {pct['IPA']}%"
        )

    # -----------------------------------------------------
    # DATA MANAGER
    # -----------------------------------------------------
    def show_data_manager(self):
        win = tk.Toplevel(self.root)
        win.title("데이터 관리")
        win.geometry("1320x780")
        win.configure(bg=self.bg)

        title = tk.Label(
            win,
            text="데이터 관리",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 18, "bold"),
        )
        title.pack(pady=8)

        summary = tk.Label(
            win,
            text=f"저장폴더: {os.path.abspath(DATA_DIR)}",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 11),
        )
        summary.pack(pady=4)

        notebook = ttk.Notebook(win)
        notebook.pack(fill="both", expand=True, padx=8, pady=8)

        tab_samples = tk.Frame(notebook, bg=self.bg)
        tab_baseline = tk.Frame(notebook, bg=self.bg)

        notebook.add(tab_samples, text="학습 데이터")
        notebook.add(tab_baseline, text="자동 AIR 기준")

        self.build_table_tab(tab_samples, SAMPLES_CSV, "학습 데이터")
        self.build_table_tab(tab_baseline, BASELINE_CSV, "자동 AIR 기준")

        tk.Button(
            win,
            text="닫기",
            command=win.destroy,
            font=("NanumGothic", 12, "bold"),
            padx=18,
            pady=7,
        ).pack(side="right", padx=14, pady=8)

    def build_table_tab(self, parent, csv_path, title):
        columns = (
            "id", "timestamp", "label", "season", "period",
            "gas_change_pct", "gas_avg", "gas_min", "gas_max",
            "temp_avg", "hum_avg", "press_avg", "count"
        )

        frame = tk.Frame(parent, bg=self.bg)
        frame.pack(fill="both", expand=True, padx=8, pady=8)

        tree = ttk.Treeview(frame, columns=columns, show="headings", height=17)
        tree.pack(side="left", fill="both", expand=True)

        scroll = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        scroll.pack(side="right", fill="y")
        tree.configure(yscrollcommand=scroll.set)

        headings = {
            "id": "ID",
            "timestamp": "시간",
            "label": "종류",
            "season": "계절",
            "period": "시간대",
            "gas_change_pct": "변화율%",
            "gas_avg": "평균Ω",
            "gas_min": "최저Ω",
            "gas_max": "최고Ω",
            "temp_avg": "온도",
            "hum_avg": "습도",
            "press_avg": "기압",
            "count": "개수",
        }

        widths = {
            "id": 120,
            "timestamp": 150,
            "label": 115,
            "season": 55,
            "period": 60,
            "gas_change_pct": 80,
            "gas_avg": 90,
            "gas_min": 90,
            "gas_max": 90,
            "temp_avg": 70,
            "hum_avg": 70,
            "press_avg": 85,
            "count": 55,
        }

        for c in columns:
            tree.heading(c, text=headings[c])
            tree.column(c, width=widths[c], anchor="center")

        detail = tk.Text(
            parent,
            height=8,
            bg="#0B1218",
            fg=self.text,
            insertbackground=self.text,
            font=("NanumGothic", 11),
        )
        detail.pack(fill="x", padx=8, pady=8)

        def load_table():
            for item in tree.get_children():
                tree.delete(item)

            rows = load_csv(csv_path)

            for idx, r in enumerate(rows):
                values = (
                    r.get("id", idx),
                    r.get("timestamp", ""),
                    r.get("label", ""),
                    r.get("season", ""),
                    r.get("period", ""),
                    f"{float(r.get('gas_change_pct', 0)):.1f}",
                    f"{float(r.get('gas_avg', 0)):,.0f}",
                    f"{float(r.get('gas_min', 0)):,.0f}",
                    f"{float(r.get('gas_max', 0)):,.0f}",
                    f"{float(r.get('temp_avg', 0)):.1f}",
                    f"{float(r.get('hum_avg', 0)):.1f}",
                    f"{float(r.get('press_avg', 0)):.1f}",
                    r.get("count", ""),
                )
                tree.insert("", "end", iid=str(idx), values=values)

        def show_detail(_event=None):
            selected = tree.selection()
            detail.delete("1.0", "end")

            if not selected:
                return

            idx = int(selected[0])
            rows = load_csv(csv_path)

            if idx >= len(rows):
                return

            r = rows[idx]

            for k, v in r.items():
                detail.insert("end", f"{k}: {v}\n")

        def delete_selected():
            selected = tree.selection()

            if not selected:
                messagebox.showinfo("알림", "삭제할 데이터를 선택하세요.")
                return

            if not messagebox.askyesno("삭제 확인", f"{title} 선택 항목을 삭제할까요?"):
                return

            rows = load_csv(csv_path)
            indexes = sorted([int(i) for i in selected], reverse=True)

            for idx in indexes:
                if 0 <= idx < len(rows):
                    del rows[idx]

            write_csv(csv_path, rows)
            load_table()
            detail.delete("1.0", "end")
            self.refresh_counts_after_data_change()

        def delete_last():
            rows = load_csv(csv_path)

            if not rows:
                return

            if not messagebox.askyesno("삭제 확인", f"{title} 최근 1개를 삭제할까요?"):
                return

            rows.pop()
            write_csv(csv_path, rows)
            load_table()
            detail.delete("1.0", "end")
            self.refresh_counts_after_data_change()

        def clear_all():
            if not messagebox.askyesno("전체 초기화", f"{title} 전체를 삭제할까요?"):
                return

            write_csv(csv_path, [])
            load_table()
            detail.delete("1.0", "end")
            self.refresh_counts_after_data_change()

        tree.bind("<<TreeviewSelect>>", show_detail)

        btns = tk.Frame(parent, bg=self.bg)
        btns.pack(fill="x", padx=8, pady=8)

        tk.Button(btns, text="새로고침", command=load_table, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)
        tk.Button(btns, text="선택 삭제", command=delete_selected, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)
        tk.Button(btns, text="최근 1개 삭제", command=delete_last, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)
        tk.Button(btns, text="전체 초기화", command=clear_all, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)

        load_table()

    def refresh_counts_after_data_change(self):
        self.baseline_save_count = len(load_csv(BASELINE_CSV))
        self.update_status("데이터 변경 완료")

    # -----------------------------------------------------
    # LOOP
    # -----------------------------------------------------
    def loop(self):
        sensor_init()
        self.update_status("센서 시작 완료")

        while self.running:
            try:
                row = read_sensor()

                self.current_rows.append(row)
                self.detect_rows.append(row)

                save_dict_csv(RAW_CSV, row)

                can_record_air_baseline = (
                    self.auto_baseline
                    and not self.realtime_detect
                    and self.sample_mode is None
                    and row["gas_valid"]
                    and row["heat_stable"]
                )

                if can_record_air_baseline:
                    self.baseline_buffer.append(row)

                    if len(self.baseline_buffer) >= 60:
                        feature = extract_features(self.baseline_buffer, "AUTO_AIR_BASELINE")

                        if feature:
                            save_dict_csv(BASELINE_CSV, feature)
                            self.latest_air_gas = feature["gas_avg"]
                            self.baseline_save_count += 1

                        self.baseline_buffer = []

                if self.sample_mode:
                    now = time.time()

                    if self.sample_phase == "READY":
                        remain = int(self.phase_until - now)

                        if remain <= 0:
                            self.sample_phase = "EXPOSE"
                            self.phase_until = now + 20
                            self.update_status(f"{self.pending_label} 노출 기록 20초")
                        else:
                            self.update_status(f"{self.pending_label} 준비 {remain}초")

                    elif self.sample_phase == "EXPOSE":
                        self.sample_rows.append(row)
                        remain = int(self.phase_until - now)

                        if remain <= 0:
                            self.sample_phase = "RECOVER"
                            self.phase_until = now + 40
                            self.update_status(f"{self.pending_label} 회복 기록 40초")
                        else:
                            self.update_status(f"{self.pending_label} 노출중 {remain}초")

                    elif self.sample_phase == "RECOVER":
                        self.sample_rows.append(row)
                        remain = int(self.phase_until - now)

                        if remain <= 0:
                            self.finish_sample()
                        else:
                            self.update_status(f"{self.pending_label} 회복중 {remain}초")

                else:
                    if self.realtime_detect:
                        self.run_realtime_detect()

                    self.update_status()

            except Exception as e:
                self.update_status(f"오류: {e}")

            time.sleep(0.4)

    # -----------------------------------------------------
    # DRAW
    # -----------------------------------------------------
    def style_axis(self, ax):
        ax.set_facecolor(self.card)
        ax.tick_params(colors=self.muted)
        ax.xaxis.label.set_color(self.muted)
        ax.yaxis.label.set_color(self.muted)
        ax.title.set_color(self.text)
        ax.grid(True, color="#314452", alpha=0.45)

        for spine in ax.spines.values():
            spine.set_color("#314452")

    def update_ui(self):
        rows = list(self.current_rows)

        if rows:
            latest = rows[-1]

            self.cards["가스저항"].config(text=f"{latest['gas_ohm']:,.0f} Ω")
            self.cards["온도"].config(text=f"{latest['temp_c']:.2f} ℃")
            self.cards["습도"].config(text=f"{latest['hum_pct']:.1f} %")
            self.cards["기압"].config(text=f"{latest['press_hpa']:.2f} hPa")

            t0 = rows[0]["epoch"]
            xs = [r["epoch"] - t0 for r in rows]
            gas = [r["gas_ohm"] for r in rows]
            temp = [r["temp_c"] for r in rows]
            hum = [r["hum_pct"] for r in rows]
            press = [r["press_hpa"] for r in rows]

            self.ax_gas.clear()
            self.style_axis(self.ax_gas)
            self.ax_gas.plot(xs, gas, color="#00E5FF", linewidth=2.5)
            self.ax_gas.fill_between(xs, gas, color="#00E5FF", alpha=0.13)
            self.ax_gas.set_title("가스 저항 변화")
            self.ax_gas.set_xlabel("시간 (초)")
            self.ax_gas.set_ylabel("가스 저항 (Ω)")

            self.ax_env.clear()
            self.style_axis(self.ax_env)
            self.ax_env.plot(xs, temp, color="#FF6B6B", linewidth=2.0, label="온도 ℃")
            self.ax_env.plot(xs, hum, color="#4DFF88", linewidth=2.0, label="습도 %")
            self.ax_env.plot(xs, [p / 10 for p in press], color="#FFD166", linewidth=2.0, label="기압 hPa/10")
            self.ax_env.set_title("환경 데이터")
            self.ax_env.set_xlabel("시간 (초)")
            self.ax_env.legend(facecolor=self.card, edgecolor="#314452", labelcolor=self.text)

            self.fig.suptitle(
                "BME688 에탄올 / IPA 학습 판별 시스템",
                color=self.text,
                fontsize=16,
                fontweight="bold",
            )

            self.fig.tight_layout()
            self.canvas.draw_idle()

        if self.running:
            self.root.after(1000, self.update_ui)

    # -----------------------------------------------------
    # CLOSE
    # -----------------------------------------------------
    def close(self):
        self.running = False

        try:
            spi.close()
        except Exception:
            pass

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
