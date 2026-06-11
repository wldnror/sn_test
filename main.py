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
# 기본 설정
# =========================================================
DATA_DIR = "bme688_data"
os.makedirs(DATA_DIR, exist_ok=True)

BASELINE_CSV = os.path.join(DATA_DIR, "baseline_auto.csv")
SAMPLES_CSV = os.path.join(DATA_DIR, "samples.csv")
RAW_CSV = os.path.join(DATA_DIR, "raw_log.csv")

STARTUP_WARMUP_SEC = 600       # 시작 후 10분 안정화
COOLDOWN_SEC = 600             # 시료 후 10분 쿨타임
RECOVERY_TOL = 0.15            # AIR 기준 ±15% 회복 판단
CONFIDENCE_LIMIT = 70.0

READY_SEC = 30                 # 학습 전 준비 시간
RECOVER_SEC = 300              # 시료 제거 후 회복 안내 시간

AUTO_BASELINE_SEC = 1800       # 자동 AIR 30분
AUTO_BASELINE_MIN_VALID_ROWS = 300
AUTO_AIR_TO_SAMPLES = True

TRAIN_SEC = 1800               # 각 학습 30분


# =========================================================
# 단순 학습 메뉴
# =========================================================
TRAIN_PRESETS = [
    {
        "button": "정상공기 30분",
        "label": "AIR",
        "level": "AIR",
        "amount_ml": "0",
        "guide": "시료를 넣지 말고 정상공기 상태를 유지하세요.",
    },
    {
        "button": "IPA 약함 0.1mL",
        "label": "IPA",
        "level": "LOW",
        "amount_ml": "0.1",
        "guide": "20cc 약병에 IPA 0.1mL를 넣고 주입하세요.",
    },
    {
        "button": "에탄올 약함 0.1mL",
        "label": "ETHANOL",
        "level": "LOW",
        "amount_ml": "0.1",
        "guide": "20cc 약병에 에탄올 0.1mL를 넣고 주입하세요.",
    },
    {
        "button": "IPA 강함 0.4mL",
        "label": "IPA",
        "level": "HIGH",
        "amount_ml": "0.4",
        "guide": "20cc 약병에 IPA 0.4mL를 넣고 주입하세요.",
    },
    {
        "button": "에탄올 강함 0.4mL",
        "label": "ETHANOL",
        "level": "HIGH",
        "amount_ml": "0.4",
        "guide": "20cc 약병에 에탄올 0.4mL를 넣고 주입하세요.",
    },
]


# =========================================================
# 히터 5단계
# 이름, res_heat 값, 단계 유지시간, 안정화 버림시간
# =========================================================
HEATER_STEPS = [
    ("H1_LOW",  0x45, 15, 4),
    ("H2_MID1", 0x55, 15, 4),
    ("H3_MID2", 0x65, 15, 4),
    ("H4_HIGH", 0x73, 15, 4),
    ("H5_MAX",  0x85, 15, 4),
]


# =========================================================
# 한글 폰트
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


def set_heater(res_heat):
    write_reg(REG_RES_HEAT_0, res_heat)
    write_reg(REG_GAS_WAIT_0, 0x59)
    write_reg(REG_CTRL_GAS_1, 0x20)


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

    set_heater(HEATER_STEPS[0][1])


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


def read_sensor(heater_name, heater_value, heater_elapsed, heater_recordable):
    set_heater(heater_value)
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
        "heater": heater_name,
        "heater_value": heater_value,
        "heater_elapsed": heater_elapsed,
        "heater_recordable_time": heater_recordable,
        "temp_c": temp_c,
        "hum_pct": hum_pct,
        "press_hpa": press_hpa,
        "gas_ohm": gas_ohm,
        "gas_adc": gas_adc,
        "gas_range": gas_range,
        "gas_valid": gas_valid,
        "heat_stable": heat_stable,
        "recordable": bool(heater_recordable and gas_valid and heat_stable and gas_ohm > 0),
    }


# =========================================================
# 데이터 / 판별
# =========================================================
def mean(values):
    return sum(values) / len(values) if values else 0.0


def stdev(values):
    if len(values) < 2:
        return 0.0

    m = mean(values)
    return math.sqrt(sum((x - m) ** 2 for x in values) / (len(values) - 1))


def load_csv(path):
    if not os.path.exists(path):
        return []

    try:
        with open(path, "r", encoding="utf-8-sig") as f:
            return list(csv.DictReader(f))
    except Exception:
        return []


def write_csv(path, rows):
    if not rows:
        if os.path.exists(path):
            os.remove(path)
        return

    fieldnames = []

    for r in rows:
        for k in r.keys():
            if k not in fieldnames:
                fieldnames.append(k)

    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def save_dict_csv(path, row):
    exists = os.path.exists(path)

    if not exists:
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()), extrasaction="ignore")
            writer.writeheader()
            writer.writerow(row)
        return

    with open(path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        old_fieldnames = reader.fieldnames or []

    row_keys = list(row.keys())

    same_header = all(k in old_fieldnames for k in row_keys)

    if same_header:
        with open(path, "a", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=old_fieldnames, extrasaction="ignore")
            writer.writerow(row)
    else:
        rows = load_csv(path)
        rows.append(row)
        write_csv(path, rows)


def count_labels():
    rows = load_csv(SAMPLES_CSV)
    counts = {"AIR": 0, "ETHANOL": 0, "IPA": 0}

    for r in rows:
        label = r.get("label", "")
        if label in counts:
            counts[label] += 1

    return counts


def label_korean(label):
    if label == "AIR":
        return "정상공기"
    if label == "ETHANOL":
        return "에탄올"
    if label == "IPA":
        return "IPA"
    if label == "UNKNOWN":
        return "미지시료"
    if label == "AUTO_AIR_BASELINE":
        return "자동 AIR 기준"
    if label == "REALTIME":
        return "실시간"
    return label


def extract_features(rows, label, level="NONE", amount_ml="0", baseline_gas=None):
    valid_rows = [
        r for r in rows
        if r.get("gas_ohm", 0) > 0
        and r.get("gas_valid", True)
        and r.get("heat_stable", True)
        and r.get("recordable", True)
    ]

    gas = [r["gas_ohm"] for r in valid_rows]
    temp = [r["temp_c"] for r in valid_rows]
    hum = [r["hum_pct"] for r in valid_rows]
    press = [r["press_hpa"] for r in valid_rows]

    if len(gas) < 5:
        return None

    gas_start = mean(gas[:max(3, min(10, len(gas)))])
    gas_end = mean(gas[-max(3, min(10, len(gas))):])
    gas_min = min(gas)
    gas_max = max(gas)
    gas_avg = mean(gas)

    ref = baseline_gas if baseline_gas and baseline_gas > 0 else gas_start
    change_pct = ((gas_min - ref) / ref) * 100.0 if ref else 0.0

    now = datetime.now()

    feature = {
        "id": now.strftime("%Y%m%d_%H%M%S"),
        "timestamp": now.isoformat(timespec="seconds"),
        "label": label,
        "level": level,
        "amount_ml": amount_ml,
        "season": get_season(now.month),
        "period": get_period(now.hour),
        "hour": now.hour,
        "count": len(valid_rows),
        "duration_sec": valid_rows[-1]["epoch"] - valid_rows[0]["epoch"],
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

    for heater_name, _, _, _ in HEATER_STEPS:
        hrows = [r for r in valid_rows if r.get("heater") == heater_name]
        hgas = [r["gas_ohm"] for r in hrows if r.get("gas_ohm", 0) > 0]

        prefix = heater_name.lower()

        if len(hgas) >= 3:
            h_avg = mean(hgas)
            h_min = min(hgas)
            h_max = max(hgas)
            h_start = mean(hgas[:min(5, len(hgas))])
            h_end = mean(hgas[-min(5, len(hgas)):])
            h_ref = baseline_gas if baseline_gas and baseline_gas > 0 else h_start
            h_change = ((h_min - h_ref) / h_ref) * 100.0 if h_ref else 0.0

            feature[f"{prefix}_avg"] = h_avg
            feature[f"{prefix}_min"] = h_min
            feature[f"{prefix}_max"] = h_max
            feature[f"{prefix}_start"] = h_start
            feature[f"{prefix}_end"] = h_end
            feature[f"{prefix}_stdev"] = stdev(hgas)
            feature[f"{prefix}_change_pct"] = h_change
            feature[f"{prefix}_count"] = len(hgas)
        else:
            feature[f"{prefix}_avg"] = 0
            feature[f"{prefix}_min"] = 0
            feature[f"{prefix}_max"] = 0
            feature[f"{prefix}_start"] = 0
            feature[f"{prefix}_end"] = 0
            feature[f"{prefix}_stdev"] = 0
            feature[f"{prefix}_change_pct"] = 0
            feature[f"{prefix}_count"] = len(hgas)

    return feature


def feature_distance(a, b):
    keys = [
        ("gas_change_pct", 2.5),
        ("gas_min", 0.0008),
        ("gas_avg", 0.0006),
        ("gas_stdev", 0.001),
        ("temp_avg", 0.5),
        ("hum_avg", 0.25),
        ("press_avg", 0.05),
    ]

    for heater_name, _, _, _ in HEATER_STEPS:
        p = heater_name.lower()
        keys.extend([
            (f"{p}_change_pct", 3.0),
            (f"{p}_avg", 0.0008),
            (f"{p}_min", 0.0010),
            (f"{p}_stdev", 0.0012),
        ])

    total = 0.0

    for key, scale in keys:
        av = float(a.get(key, 0) or 0)
        bv = float(b.get(key, 0) or 0)

        if av == 0 and bv == 0:
            continue

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
        self.root.title("BME688 에탄올 / IPA 단순 학습 시스템")
        self.root.attributes("-fullscreen", True)
        self.fullscreen = True

        self.running = True

        self.auto_baseline = True
        self.realtime_detect = False

        self.startup_warmup_active = True
        self.startup_warmup_until = time.time() + STARTUP_WARMUP_SEC

        self.cooldown_active = False
        self.cooldown_until = 0
        self.cooldown_reason = ""

        self.current_rows = deque(maxlen=700)
        self.detect_rows = deque(maxlen=300)
        self.baseline_buffer = []
        self.baseline_started_at = None

        self.latest_air_gas = None
        self.baseline_save_count = len(load_csv(BASELINE_CSV))

        self.sample_mode = None
        self.pending_label = None
        self.pending_level = "NONE"
        self.pending_amount_ml = "0"
        self.pending_guide = ""
        self.sample_rows = []
        self.sample_phase = None
        self.phase_until = 0
        self.expose_until = 0
        self.train_duration = 0

        self.current_heater_index = 0
        self.heater_switch_at = 0
        self.heater_step_started_at = time.time()
        self.training_cycle_count = 0

        self.detect_history = deque(maxlen=3)

        self.bg = "#101820"
        self.card = "#182632"
        self.text = "#EAF2F8"
        self.muted = "#94A9B8"

        self.notice_blue = "#0066CC"
        self.notice_orange = "#D35400"
        self.notice_green = "#008855"
        self.notice_yellow = "#B8860B"
        self.notice_red = "#B00020"

        self.root.configure(bg=self.bg)

        self.build_ui()

        self.sensor_thread = threading.Thread(target=self.loop, daemon=True)
        self.sensor_thread.start()

        self.update_ui()

    # -----------------------------------------------------
    # UI
    # -----------------------------------------------------
    def build_ui(self):
        top = tk.Frame(self.root, bg=self.bg)
        top.pack(fill="x", padx=12, pady=8)

        self.status_label = tk.Label(
            top,
            text="상태: 준비중",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 14, "bold"),
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

        for name in ["현재상태", "가스저항", "히터단계", "온도", "습도", "판별확률"]:
            f = tk.Frame(info, bg=self.card, padx=14, pady=10)
            f.pack(side="left", fill="x", expand=True, padx=5, pady=5)

            title = tk.Label(f, text=name, bg=self.card, fg=self.muted, font=("NanumGothic", 11))
            title.pack(anchor="w")

            value = tk.Label(f, text="-", bg=self.card, fg=self.text, font=("NanumGothic", 17, "bold"))
            value.pack(anchor="w")

            self.cards[name] = value

        self.plot_area = tk.Frame(self.root, bg=self.bg)
        self.plot_area.pack(fill="both", expand=True, padx=12, pady=8)

        self.fig = plt.Figure(figsize=(14, 7), facecolor=self.bg)
        self.ax_gas = self.fig.add_subplot(2, 1, 1)
        self.ax_env = self.fig.add_subplot(2, 1, 2)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_area)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.build_notice_overlay()

    def build_notice_overlay(self):
        self.notice_frame = tk.Frame(
            self.root,
            bg=self.notice_blue,
            bd=6,
            relief="ridge",
        )

        self.notice_title = tk.Label(
            self.notice_frame,
            text="",
            bg=self.notice_blue,
            fg="white",
            font=("NanumGothic", 34, "bold"),
            padx=40,
            pady=14,
        )
        self.notice_title.pack(fill="x")

        self.notice_message = tk.Label(
            self.notice_frame,
            text="",
            bg=self.notice_blue,
            fg="white",
            font=("NanumGothic", 23, "bold"),
            padx=40,
            pady=10,
            justify="center",
        )
        self.notice_message.pack(fill="x")

        self.notice_timer = tk.Label(
            self.notice_frame,
            text="",
            bg=self.notice_blue,
            fg="white",
            font=("NanumGothic", 36, "bold"),
            padx=40,
            pady=12,
        )
        self.notice_timer.pack(fill="x")

        self.notice_sub = tk.Label(
            self.notice_frame,
            text="",
            bg=self.notice_blue,
            fg="white",
            font=("NanumGothic", 16, "bold"),
            padx=40,
            pady=10,
        )
        self.notice_sub.pack(fill="x")

        self.hide_notice()

    def show_notice(self, title, message, remain=None, color=None, sub=""):
        if color is None:
            color = self.notice_blue

        self.notice_frame.configure(bg=color)
        self.notice_title.configure(text=title, bg=color)
        self.notice_message.configure(text=message, bg=color)
        self.notice_sub.configure(text=sub, bg=color)

        if remain is None:
            self.notice_timer.configure(text="", bg=color)
        else:
            if remain >= 60:
                self.notice_timer.configure(text=f"남은 시간: {remain // 60:02d}:{remain % 60:02d}", bg=color)
            else:
                self.notice_timer.configure(text=f"남은 시간: {remain}초", bg=color)

        self.notice_frame.place(relx=0.5, rely=0.48, anchor="center")

    def hide_notice(self):
        self.notice_frame.place_forget()

    # -----------------------------------------------------
    # 메뉴
    # -----------------------------------------------------
    def open_learn_menu(self):
        if self.is_blocked():
            return

        win = tk.Toplevel(self.root)
        win.title("단순 학습 선택")
        win.geometry("480x520")
        win.configure(bg=self.bg)

        tk.Label(
            win,
            text="단순 학습 메뉴",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 18, "bold"),
        ).pack(pady=14)

        tk.Label(
            win,
            text="각 항목은 30분 동안 히터 5단계를 반복하며 기록합니다.",
            bg=self.bg,
            fg=self.muted,
            font=("NanumGothic", 11, "bold"),
        ).pack(pady=4)

        for preset in TRAIN_PRESETS:
            tk.Button(
                win,
                text=preset["button"],
                command=lambda p=preset, w=win: (w.destroy(), self.start_training_preset(p)),
                font=("NanumGothic", 14, "bold"),
                bg="#263847",
                fg=self.text,
                activebackground="#365369",
                activeforeground="white",
                relief="flat",
                padx=20,
                pady=12,
            ).pack(fill="x", padx=32, pady=7)

    def open_detect_menu(self):
        if self.is_blocked():
            return

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
            text="미지시료 수동 판별 5분",
            command=lambda w=win: (w.destroy(), self.start_unknown(300)),
            font=("NanumGothic", 13, "bold"),
            bg="#263847",
            fg=self.text,
            relief="flat",
            padx=20,
            pady=10,
        ).pack(fill="x", padx=28, pady=8)

    # -----------------------------------------------------
    # 상태 / 모드
    # -----------------------------------------------------
    def update_status(self, text=None):
        counts = count_labels()

        detect_state = "실시간판별 ON" if self.realtime_detect else "실시간판별 OFF"
        base_state = "AIR기준기록 ON" if self.auto_baseline else "AIR기준기록 OFF"

        if self.baseline_started_at:
            elapsed = int(time.time() - self.baseline_started_at)
            remain = max(0, AUTO_BASELINE_SEC - elapsed)
            buffer_state = f"AIR자동 {elapsed // 60:02d}:{elapsed % 60:02d}/{AUTO_BASELINE_SEC // 60}분"
            buffer_state += f" 남음 {remain // 60:02d}:{remain % 60:02d}"
        else:
            buffer_state = "AIR자동 대기"

        save_state = f"AIR기준 {self.baseline_save_count}회"
        learn_state = f"AIR {counts['AIR']} / ETH {counts['ETHANOL']} / IPA {counts['IPA']}"

        extra = ""

        if self.startup_warmup_active:
            remain = max(0, int(self.startup_warmup_until - time.time()))
            extra += f" | 초기안정화 {remain // 60:02d}:{remain % 60:02d}"

        if self.cooldown_active:
            remain = max(0, int(self.cooldown_until - time.time()))
            extra += f" | 쿨타임 {remain // 60:02d}:{remain % 60:02d}"

        if text:
            msg = f"상태: {text}{extra} | {detect_state} | {base_state} | {buffer_state} | {save_state} | {learn_state}"
        else:
            msg = f"상태: {detect_state}{extra} | {base_state} | {buffer_state} | {save_state} | {learn_state}"

        self.status_label.config(text=msg)

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def toggle_auto(self):
        if self.startup_warmup_active:
            messagebox.showinfo("초기 안정화 중", "초기 안정화 중에는 AIR 기준 기록을 시작할 수 없습니다.")
            self.auto_baseline = False
            self.update_status()
            return

        if self.cooldown_active:
            messagebox.showinfo("쿨타임 중", "시료 잔류 제거 쿨타임 중에는 AIR 기준 기록을 켤 수 없습니다.")
            self.auto_baseline = False
            self.update_status()
            return

        if self.realtime_detect:
            messagebox.showinfo("안내", "실시간 판별 중에는 AIR 기준 기록을 켤 수 없습니다.")
            self.auto_baseline = False
        else:
            self.auto_baseline = not self.auto_baseline
            if not self.auto_baseline:
                self.baseline_buffer = []
                self.baseline_started_at = None

        self.update_status()

    def toggle_realtime(self):
        if self.is_blocked():
            return

        self.realtime_detect = not self.realtime_detect

        if self.realtime_detect:
            self.auto_baseline = False
            self.baseline_buffer = []
            self.baseline_started_at = None
            self.hide_notice()
            self.cards["현재상태"].config(text="실시간 판별중", fg="#00E5FF")
            self.update_status("실시간 판별 ON / AIR 기준 기록 자동 OFF")
        else:
            self.detect_history.clear()
            self.cards["현재상태"].config(text="판별 대기", fg=self.text)
            self.cards["판별확률"].config(text="-")
            self.update_status("실시간 판별 OFF")

    def is_blocked(self):
        if self.startup_warmup_active:
            remain = max(0, int(self.startup_warmup_until - time.time()))
            messagebox.showinfo(
                "초기 안정화 중",
                f"센서 초기 안정화 중입니다.\n남은 시간: {remain // 60}분 {remain % 60}초\n\n"
                "정확도 우선 모드라서 안정화 전에는 학습/판별을 막습니다."
            )
            return True

        if self.cooldown_active:
            remain = max(0, int(self.cooldown_until - time.time()))

            if remain > 0:
                messagebox.showinfo(
                    "쿨타임 중",
                    f"시료 잔류 제거 대기중입니다.\n남은 시간: {remain // 60}분 {remain % 60}초"
                )
                return True

            if not self.is_air_recovered():
                messagebox.showinfo(
                    "회복 대기",
                    "쿨타임은 끝났지만 아직 AIR 기준 범위로 회복되지 않았습니다.\n환기 후 다시 시도하세요."
                )
                return True

            self.cooldown_active = False
            self.auto_baseline = True

        return False

    # -----------------------------------------------------
    # 히터 단계
    # -----------------------------------------------------
    def update_heater_step(self):
        now = time.time()
        name, value, step_sec, settle_sec = HEATER_STEPS[self.current_heater_index]

        if now >= self.heater_switch_at:
            self.current_heater_index = (self.current_heater_index + 1) % len(HEATER_STEPS)

            if self.current_heater_index == 0:
                self.training_cycle_count += 1

            name, value, step_sec, settle_sec = HEATER_STEPS[self.current_heater_index]
            self.heater_switch_at = now + step_sec
            self.heater_step_started_at = now

        elapsed = now - self.heater_step_started_at
        recordable_time = elapsed >= settle_sec

        return name, value, elapsed, recordable_time

    # -----------------------------------------------------
    # 초기 안정화 / 쿨타임
    # -----------------------------------------------------
    def check_startup_warmup(self):
        if not self.startup_warmup_active:
            return

        remain = int(self.startup_warmup_until - time.time())

        if remain > 0:
            self.auto_baseline = False
            self.baseline_started_at = None
            self.show_notice(
                "초기 안정화 중",
                "센서 예열 및 주변 공기 안정화 대기",
                remain=remain,
                color=self.notice_blue,
                sub="이 시간 동안 학습 / 판별 / 자동 AIR 기준 저장은 차단됩니다.",
            )
            self.update_status("초기 안정화 대기중")
            return

        self.startup_warmup_active = False
        self.auto_baseline = True
        self.baseline_buffer = []
        self.baseline_started_at = None

        self.show_notice(
            "초기 안정화 완료",
            "자동 AIR 기준 30분 기록을 시작합니다",
            remain=None,
            color=self.notice_green,
            sub="정상공기 상태에서 히터 5단계 정밀 AIR 데이터가 쌓입니다.",
        )
        self.root.after(3000, self.hide_notice)
        self.update_status("초기 안정화 완료 / AIR 기준 기록 시작")

    def start_cooldown(self, reason):
        self.cooldown_active = True
        self.cooldown_until = time.time() + COOLDOWN_SEC
        self.cooldown_reason = reason

        self.auto_baseline = False
        self.realtime_detect = False
        self.baseline_buffer = []
        self.baseline_started_at = None
        self.detect_history.clear()

        self.cards["현재상태"].config(text="시료 잔류 제거 대기", fg="#FFD166")
        self.cards["판별확률"].config(text="-")

        self.show_notice(
            "시료 잔류 제거 대기",
            "시료를 치우고 충분히 환기하세요",
            remain=COOLDOWN_SEC,
            color=self.notice_yellow,
            sub="쿨타임 중에는 학습 / 판별 / 자동 AIR 기준 기록이 잠깁니다.",
        )

        self.update_status(f"{reason} 후 쿨타임 시작")

    def is_air_recovered(self):
        if not self.latest_air_gas:
            baselines = load_csv(BASELINE_CSV)
            if baselines:
                try:
                    self.latest_air_gas = float(baselines[-1].get("gas_avg", 0))
                except Exception:
                    self.latest_air_gas = None

        if not self.latest_air_gas:
            return False

        rows = [r for r in list(self.current_rows)[-120:] if r.get("recordable")]
        gas = [r["gas_ohm"] for r in rows if r["gas_ohm"] > 0]

        if len(gas) < 30:
            return False

        avg_gas = mean(gas)
        diff_ratio = abs(avg_gas - self.latest_air_gas) / self.latest_air_gas

        return diff_ratio <= RECOVERY_TOL

    def check_cooldown_release(self):
        if not self.cooldown_active:
            return

        remain = int(self.cooldown_until - time.time())

        if remain > 0:
            self.show_notice(
                "시료 잔류 제거 대기",
                "시료를 제거하고 환기하세요",
                remain=remain,
                color=self.notice_yellow,
                sub="쿨타임 중에는 학습 / 판별 / 자동 AIR 기준 기록이 차단됩니다.",
            )
            self.update_status("시료 잔류 제거 대기중")
            return

        if self.is_air_recovered():
            self.cooldown_active = False
            self.cooldown_reason = ""
            self.auto_baseline = True
            self.baseline_buffer = []
            self.baseline_started_at = None

            self.show_notice(
                "회복 완료",
                "정상 공기 범위로 돌아왔습니다",
                remain=None,
                color=self.notice_green,
                sub="자동 AIR 기준 30분 기록을 다시 시작합니다.",
            )

            self.cards["현재상태"].config(text="회복 완료 / AIR 기록 가능", fg="#4DFF88")
            self.update_status("회복 완료 / 자동 AIR 기준 기록 재개")
            self.root.after(3000, self.hide_notice)
        else:
            self.auto_baseline = False
            self.baseline_started_at = None
            self.show_notice(
                "쿨타임 종료 / 아직 미회복",
                "AIR 기준 범위로 돌아오지 않았습니다\n계속 환기하세요",
                remain=None,
                color=self.notice_orange,
                sub="회복 전까지 학습 / 판별 / 자동 AIR 기준 기록이 차단됩니다.",
            )
            self.cards["현재상태"].config(text="쿨타임 종료 / 아직 미회복", fg="#FFD166")
            self.update_status("쿨타임 종료 / AIR 기준 범위 미회복")

    # -----------------------------------------------------
    # 학습 / 판별
    # -----------------------------------------------------
    def start_training_preset(self, preset):
        self.start_training(
            label=preset["label"],
            level=preset["level"],
            amount_ml=preset["amount_ml"],
            guide=preset["guide"],
            duration_sec=TRAIN_SEC,
        )

    def start_training(self, label, level, amount_ml, guide, duration_sec):
        if self.is_blocked():
            return

        self.realtime_detect = False
        self.auto_baseline = False
        self.baseline_buffer = []
        self.baseline_started_at = None

        self.sample_mode = "LEARN"
        self.pending_label = label
        self.pending_level = level
        self.pending_amount_ml = amount_ml
        self.pending_guide = guide

        self.sample_rows = []
        self.sample_phase = "READY"
        self.phase_until = time.time() + READY_SEC
        self.expose_until = 0
        self.train_duration = duration_sec

        self.current_heater_index = 0
        self.heater_step_started_at = time.time()
        self.heater_switch_at = time.time() + HEATER_STEPS[0][2]
        self.training_cycle_count = 0

        if label == "AIR":
            msg = "정상공기 상태를 그대로 유지하세요"
            sub = "시료를 넣지 않습니다."
        else:
            msg = "아직 시료를 주입하지 마세요"
            sub = f"준비 후 안내가 뜨면 {guide}"

        self.show_notice(
            f"{label_korean(label)} {level} 학습 준비",
            msg,
            remain=READY_SEC,
            color=self.notice_blue,
            sub=sub,
        )

        self.update_status(f"{label} {level} 학습 준비")

    def start_unknown(self, duration_sec):
        if self.is_blocked():
            return

        self.realtime_detect = False
        self.auto_baseline = False
        self.baseline_buffer = []
        self.baseline_started_at = None

        self.sample_mode = "UNKNOWN"
        self.pending_label = "UNKNOWN"
        self.pending_level = "UNKNOWN"
        self.pending_amount_ml = "UNKNOWN"
        self.pending_guide = "미지시료를 주입하세요."

        self.sample_rows = []
        self.sample_phase = "READY"
        self.phase_until = time.time() + READY_SEC
        self.expose_until = 0
        self.train_duration = duration_sec

        self.current_heater_index = 0
        self.heater_step_started_at = time.time()
        self.heater_switch_at = time.time() + HEATER_STEPS[0][2]
        self.training_cycle_count = 0

        self.show_notice(
            "미지시료 판별 준비",
            "아직 시료를 주입하지 마세요",
            remain=READY_SEC,
            color=self.notice_blue,
            sub="준비가 끝나면 미지시료를 일정하게 주입하세요.",
        )

        self.update_status("미지시료 판별 준비")

    def finish_sample(self):
        feature = extract_features(
            self.sample_rows,
            self.pending_label,
            self.pending_level,
            self.pending_amount_ml,
            self.latest_air_gas,
        )

        if not feature:
            self.update_status("샘플 실패")
            self.sample_mode = None
            self.show_notice(
                "샘플 실패",
                "유효한 데이터가 부족합니다",
                remain=None,
                color=self.notice_red,
                sub="히터 안정화 이후 유효 데이터가 부족합니다.",
            )
            self.root.after(3000, self.hide_notice)
            return

        finished_label = self.pending_label
        finished_level = self.pending_level
        finished_mode = self.sample_mode

        if finished_mode == "LEARN":
            save_dict_csv(SAMPLES_CSV, feature)
            self.cards["현재상태"].config(
                text=f"{label_korean(finished_label)} {finished_level} 학습완료",
                fg="#4DFF88",
            )

            sub = "시료를 제거하고 회복 단계로 전환됩니다." if finished_label in ["ETHANOL", "IPA"] else "정상공기 학습 완료"
            self.show_notice(
                f"{label_korean(finished_label)} {finished_level} 학습 완료",
                "노출 구간 데이터만 저장되었습니다",
                remain=None,
                color=self.notice_green,
                sub=sub,
            )
            self.update_status(f"{finished_label} {finished_level} 학습 저장 완료")

        else:
            pct, winner = classify(feature)
            self.cards["현재상태"].config(text=f"{label_korean(winner)}", fg="#FFD166")
            self.cards["판별확률"].config(
                text=f"AIR {pct['AIR']}% / ETH {pct['ETHANOL']}% / IPA {pct['IPA']}%"
            )

            self.show_notice(
                "수동 판별 완료",
                f"결과: {label_korean(winner)}",
                remain=None,
                color=self.notice_green,
                sub=f"AIR {pct['AIR']}% / ETH {pct['ETHANOL']}% / IPA {pct['IPA']}%",
            )
            self.update_status("수동 판별 완료")

        self.sample_mode = None
        self.pending_label = None
        self.pending_level = "NONE"
        self.pending_amount_ml = "0"
        self.pending_guide = ""
        self.sample_rows = []
        self.sample_phase = None

        if finished_label in ["ETHANOL", "IPA", "UNKNOWN"]:
            self.root.after(1500, lambda: self.start_cooldown(finished_label))
        elif finished_label == "AIR":
            self.auto_baseline = True
            self.baseline_started_at = None
            self.root.after(3000, self.hide_notice)

    # -----------------------------------------------------
    # 실시간 판별
    # -----------------------------------------------------
    def run_realtime_detect(self):
        if not self.realtime_detect or self.sample_mode is not None:
            return

        valid_detect = [r for r in self.detect_rows if r.get("recordable")]

        if len(valid_detect) < 80:
            self.cards["현재상태"].config(text="데이터 수집중", fg=self.text)
            self.cards["판별확률"].config(text="-")
            return

        feature = extract_features(valid_detect, "REALTIME", "REALTIME", "UNKNOWN", self.latest_air_gas)

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
        confident = best_pct >= CONFIDENCE_LIMIT

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
            state = f"{label_korean(winner)} 의심"
            color = "#00E5FF"

        self.cards["현재상태"].config(text=f"{state} {best_pct:.1f}%", fg=color)
        self.cards["판별확률"].config(
            text=f"AIR {pct['AIR']}% / ETH {pct['ETHANOL']}% / IPA {pct['IPA']}%"
        )

    # -----------------------------------------------------
    # 데이터 관리
    # -----------------------------------------------------
    def show_data_manager(self):
        win = tk.Toplevel(self.root)
        win.title("데이터 관리")
        win.geometry("1420x800")
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

        notebook.add(tab_samples, text="판별용 학습 데이터")
        notebook.add(tab_baseline, text="자동 AIR 기준 원본")

        self.build_table_tab(tab_samples, SAMPLES_CSV, "판별용 학습 데이터")
        self.build_table_tab(tab_baseline, BASELINE_CSV, "자동 AIR 기준 원본")

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
            "id", "timestamp", "label", "level", "amount_ml", "duration_sec", "count",
            "gas_change_pct", "gas_avg", "gas_min", "gas_max",
            "h1_low_change_pct", "h2_mid1_change_pct", "h3_mid2_change_pct",
            "h4_high_change_pct", "h5_max_change_pct",
            "temp_avg", "hum_avg", "press_avg"
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
            "level": "강도",
            "amount_ml": "mL",
            "duration_sec": "시간초",
            "count": "개수",
            "gas_change_pct": "전체변화%",
            "gas_avg": "평균Ω",
            "gas_min": "최저Ω",
            "gas_max": "최고Ω",
            "h1_low_change_pct": "H1%",
            "h2_mid1_change_pct": "H2%",
            "h3_mid2_change_pct": "H3%",
            "h4_high_change_pct": "H4%",
            "h5_max_change_pct": "H5%",
            "temp_avg": "온도",
            "hum_avg": "습도",
            "press_avg": "기압",
        }

        for c in columns:
            tree.heading(c, text=headings[c])
            tree.column(c, width=78, anchor="center")

        detail = tk.Text(
            parent,
            height=9,
            bg="#0B1218",
            fg=self.text,
            insertbackground=self.text,
            font=("NanumGothic", 10),
        )
        detail.pack(fill="x", padx=8, pady=8)

        def safe_float(v):
            try:
                return float(v)
            except Exception:
                return 0.0

        def load_table():
            for item in tree.get_children():
                tree.delete(item)

            rows = load_csv(csv_path)

            for idx, r in enumerate(rows):
                values = []
                for c in columns:
                    if c in [
                        "gas_change_pct", "h1_low_change_pct", "h2_mid1_change_pct",
                        "h3_mid2_change_pct", "h4_high_change_pct", "h5_max_change_pct",
                        "temp_avg", "hum_avg", "press_avg"
                    ]:
                        values.append(f"{safe_float(r.get(c, 0)):.1f}")
                    elif c in ["gas_avg", "gas_min", "gas_max"]:
                        values.append(f"{safe_float(r.get(c, 0)):,.0f}")
                    elif c == "duration_sec":
                        values.append(f"{safe_float(r.get(c, 0)):.0f}")
                    else:
                        values.append(r.get(c, ""))
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
    # 메인 루프
    # -----------------------------------------------------
    def loop(self):
        sensor_init()
        self.update_status("센서 시작 완료")

        self.current_heater_index = 0
        self.heater_step_started_at = time.time()
        self.heater_switch_at = time.time() + HEATER_STEPS[0][2]

        while self.running:
            try:
                self.check_startup_warmup()
                self.check_cooldown_release()

                heater_name, heater_value, heater_elapsed, heater_recordable = self.update_heater_step()
                row = read_sensor(heater_name, heater_value, heater_elapsed, heater_recordable)

                self.current_rows.append(row)
                save_dict_csv(RAW_CSV, row)

                if row.get("recordable"):
                    self.detect_rows.append(row)

                self.cards["히터단계"].config(
                    text=f"{heater_name} {'기록' if row.get('recordable') else '안정화'}"
                )

                can_record_air_baseline = (
                    self.auto_baseline
                    and not self.startup_warmup_active
                    and not self.cooldown_active
                    and not self.realtime_detect
                    and self.sample_mode is None
                    and row.get("recordable")
                )

                if can_record_air_baseline:
                    if self.baseline_started_at is None:
                        self.baseline_started_at = time.time()
                        self.baseline_buffer = []

                    self.baseline_buffer.append(row)

                    elapsed = time.time() - self.baseline_started_at

                    if elapsed >= AUTO_BASELINE_SEC:
                        feature = extract_features(
                            self.baseline_buffer,
                            "AUTO_AIR_BASELINE",
                            "AUTO",
                            "0",
                            None,
                        )

                        if feature and int(feature.get("count", 0)) >= AUTO_BASELINE_MIN_VALID_ROWS:
                            save_dict_csv(BASELINE_CSV, feature)
                            self.latest_air_gas = feature["gas_avg"]
                            self.baseline_save_count += 1

                            if AUTO_AIR_TO_SAMPLES:
                                air_feature = dict(feature)
                                air_feature["id"] = datetime.now().strftime("%Y%m%d_%H%M%S") + "_AUTOAIR"
                                air_feature["timestamp"] = datetime.now().isoformat(timespec="seconds")
                                air_feature["label"] = "AIR"
                                air_feature["level"] = "AUTO"
                                air_feature["amount_ml"] = "0"
                                save_dict_csv(SAMPLES_CSV, air_feature)

                            self.show_notice(
                                "자동 AIR 기준 저장 완료",
                                "30분 AIR 기준 데이터가 저장되었습니다",
                                remain=None,
                                color=self.notice_green,
                                sub="판별용 AIR 학습 데이터에도 반영되었습니다.",
                            )
                            self.root.after(3000, self.hide_notice)
                            self.update_status("자동 AIR 기준 30분 저장 완료")
                        else:
                            self.update_status("자동 AIR 기준 저장 실패 / 유효 데이터 부족")

                        self.baseline_buffer = []
                        self.baseline_started_at = None

                if self.sample_mode:
                    now = time.time()

                    if self.sample_phase == "READY":
                        remain = max(0, int(self.phase_until - now))

                        if remain <= 0:
                            self.sample_phase = "EXPOSE"
                            self.expose_until = now + self.train_duration

                            if self.pending_label == "AIR":
                                message = "정상공기 상태를 그대로 유지하세요"
                                sub = "시료를 넣지 않습니다. 히터 5단계가 반복됩니다."
                            elif self.pending_label == "UNKNOWN":
                                message = "미지시료를 센서 챔버로\n일정하게 주입하세요"
                                sub = "히터 5단계가 반복되며 판별용 데이터를 기록합니다."
                            else:
                                message = self.pending_guide
                                sub = "중간에 조건을 바꾸지 말고 일정하게 주입하세요."

                            self.show_notice(
                                f"{label_korean(self.pending_label)} {self.pending_level} 기록 중",
                                message,
                                remain=int(self.train_duration),
                                color=self.notice_orange,
                                sub=sub,
                            )

                            self.update_status(f"{self.pending_label} {self.pending_level} 기록 시작")
                        else:
                            msg = "정상공기 상태 유지" if self.pending_label == "AIR" else "아직 시료를 주입하지 마세요"
                            self.show_notice(
                                f"{label_korean(self.pending_label)} {self.pending_level} 준비",
                                msg,
                                remain=remain,
                                color=self.notice_blue,
                                sub="준비 시간이 끝나면 정밀기록이 시작됩니다.",
                            )
                            self.update_status(f"{self.pending_label} 준비 {remain}초")

                    elif self.sample_phase == "EXPOSE":
                        if row.get("recordable"):
                            self.sample_rows.append(row)

                        remain = max(0, int(self.expose_until - now))

                        if remain <= 0:
                            if self.pending_label in ["ETHANOL", "IPA", "UNKNOWN"]:
                                self.sample_phase = "RECOVER"
                                self.phase_until = now + RECOVER_SEC

                                self.show_notice(
                                    "회복 단계",
                                    "시료를 제거하고\n충분히 환기하세요",
                                    remain=RECOVER_SEC,
                                    color=self.notice_green,
                                    sub="회복 구간 데이터는 학습에 저장하지 않습니다.",
                                )

                                self.update_status(f"{self.pending_label} 회복 단계 시작")
                            else:
                                self.finish_sample()
                        else:
                            if self.pending_label == "AIR":
                                message = "정상공기 상태를 그대로 유지하세요"
                            elif self.pending_label == "UNKNOWN":
                                message = "미지시료를 일정하게 주입하세요"
                            else:
                                message = self.pending_guide

                            step_status = "기록중" if row.get("recordable") else "히터 안정화중"
                            cycle_text = f"히터: {heater_name} / {step_status} / 사이클: {self.training_cycle_count}"
                            self.show_notice(
                                f"{label_korean(self.pending_label)} {self.pending_level} 기록 중",
                                message,
                                remain=remain,
                                color=self.notice_orange,
                                sub=cycle_text,
                            )
                            self.update_status(f"{self.pending_label} 기록중 {remain // 60:02d}:{remain % 60:02d}")

                    elif self.sample_phase == "RECOVER":
                        # 중요:
                        # 회복 구간은 학습 데이터에 섞지 않는다.
                        # sample_rows에는 노출 구간 데이터만 들어간다.
                        remain = max(0, int(self.phase_until - now))

                        if remain <= 0:
                            self.finish_sample()
                        else:
                            self.show_notice(
                                "회복 단계",
                                "시료를 제거하고\n충분히 환기하세요",
                                remain=remain,
                                color=self.notice_green,
                                sub="회복 데이터는 학습에 저장하지 않습니다.",
                            )
                            self.update_status(f"{self.pending_label} 회복중 {remain // 60:02d}:{remain % 60:02d}")

                else:
                    if self.realtime_detect and not self.cooldown_active and not self.startup_warmup_active:
                        self.run_realtime_detect()

                    self.update_status()

            except Exception as e:
                self.update_status(f"오류: {e}")
                self.show_notice(
                    "오류 발생",
                    str(e),
                    remain=None,
                    color=self.notice_red,
                    sub="센서 연결 또는 코드 상태를 확인하세요.",
                )

            time.sleep(0.4)

    # -----------------------------------------------------
    # 그래프
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
                "BME688 단순 학습: AIR / IPA 0.1mL / ETH 0.1mL / IPA 0.4mL / ETH 0.4mL",
                color=self.text,
                fontsize=15,
                fontweight="bold",
            )

            self.fig.tight_layout()
            self.canvas.draw_idle()

        if self.running:
            self.root.after(1000, self.update_ui)

    # -----------------------------------------------------
    # 종료
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
