import os
import sys
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


DATA_DIR = "bme688_data"
os.makedirs(DATA_DIR, exist_ok=True)

SAMPLES_CSV = os.path.join(DATA_DIR, "samples_live.csv")
RAW_CSV = os.path.join(DATA_DIR, "raw_log_live.csv")
AIR_MEMORY_CSV = os.path.join(DATA_DIR, "air_memory.csv")
AIR_HISTORY_CSV = os.path.join(DATA_DIR, "air_history.csv")

MAX_RAW_ROWS = 100000

MEASURE_SLEEP_SEC = 0.20
LOOP_SLEEP_SEC = 0.005
UI_UPDATE_MS = 200
STATUS_UPDATE_SEC = 1.0

HEATER_NAME = "H1_LOW_FIXED"
HEATER_VALUE = 0x45

ROLLING_WINDOW_SEC = 10.0
MIN_CLASSIFY_ROWS = 4

ACTIVE_GAS_DROP_PCT = -3.0
ACTIVE_HUM_RISE = 0.7

AIR_STABLE_GAS_PCT = 2.0
AIR_STABLE_HUM_DELTA = 0.5
AIR_SLOW_ALPHA = 0.0005

AIR_HISTORY_INTERVAL_SEC = 30
AIR_HISTORY_LOCK_AFTER_REACTION_SEC = 3 * 60 * 60
MAX_AIR_HISTORY_ROWS = 1000

DETECT_CONFIRM_COUNT = 2

DECISION_DELAY_SEC = 5.0

RETURN_GAS_PCT = 6.0
RETURN_HUM_DELTA = 2.0
NORMAL_RETURN_COUNT = 3

TRAIN_READY_SEC = 3
TRAIN_RECORD_SEC = 30
TRAIN_FEATURE_INTERVAL_SEC = 0.5
MIN_TRAIN_ROWS = 10

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

GROUP_LABELS = {
    "IPA": "IPA",
    "IPA_HIGH": "IPA",
    "IPA_LOW": "IPA",
    "IPA_1000": "IPA",
    "IPA_10000": "IPA",
    "IPA_WATER": "IPA_WATER",
    "ETHANOL": "ETHANOL",
    "ETHANOL_HIGH": "ETHANOL",
    "ETHANOL_LOW": "ETHANOL",
}


VALID_CLASSIFY_LABELS = {
    "IPA",
    "IPA_HIGH",
    "IPA_LOW",
    "IPA_1000",
    "IPA_10000",
    "ETHANOL",
    "ETHANOL_HIGH",
    "ETHANOL_LOW",
}

TRAIN_BUTTONS = [
    ("에탄올 원액", "ETHANOL_HIGH"),
    ("에탄올 0.05", "ETHANOL_LOW"),
    ("IPA 원액", "IPA_HIGH"),
    ("IPA 0.05", "IPA_LOW"),
    ("IPA 1000ppm", "IPA_1000"),
    ("IPA 10000ppm", "IPA_10000"),
    ("IPA + 물", "IPA_WATER"),
]


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


def trigger_measurement():
    ctrl_meas = (0b010 << 5) | (0b101 << 2) | 0b01
    write_reg(REG_CTRL_MEAS, ctrl_meas)


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
    set_heater(HEATER_VALUE)


def read_sensor():
    set_heater(HEATER_VALUE)
    trigger_measurement()
    time.sleep(MEASURE_SLEEP_SEC)

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
        "timestamp": now.isoformat(timespec="milliseconds"),
        "epoch": time.time(),
        "heater": HEATER_NAME,
        "heater_value": HEATER_VALUE,
        "temp_c": temp_c,
        "hum_pct": hum_pct,
        "press_hpa": press_hpa,
        "gas_ohm": gas_ohm,
        "gas_adc": gas_adc,
        "gas_range": gas_range,
        "gas_valid": gas_valid,
        "heat_stable": heat_stable,
        "recordable": bool(gas_valid and heat_stable and gas_ohm > 0),
    }


def to_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default


def to_bool(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.lower() in ["true", "1", "yes", "y"]
    return bool(v)


def mean(values):
    return sum(values) / len(values) if values else 0.0


def median(values):
    values = sorted(values)
    n = len(values)
    if n == 0:
        return 0.0
    if n % 2:
        return values[n // 2]
    return (values[n // 2 - 1] + values[n // 2]) / 2.0


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


def append_dict_csv(path, row):
    file_exists = os.path.exists(path) and os.path.getsize(path) > 0
    fieldnames = list(row.keys())

    with open(path, "a", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


def save_dict_csv(path, row):
    rows = load_csv(path)
    rows.append(row)
    write_csv(path, rows)


def trim_csv(path, max_rows):
    rows = load_csv(path)
    if len(rows) > max_rows:
        write_csv(path, rows[-max_rows:])


def label_group(label):
    return GROUP_LABELS.get(label, label)


def label_korean(label):
    g = label_group(label)
    if g == "ETHANOL":
        return "에탄올"
    if g == "IPA":
        return "IPA"
    if g == "NORMAL":
        return "정상범위"
    return label


def label_detail_korean(label):
    names = {
        "ETHANOL": "에탄올",
        "ETHANOL_HIGH": "에탄올 원액",
        "ETHANOL_LOW": "에탄올 0.05",
        "IPA": "IPA",
        "IPA_HIGH": "IPA 원액",
        "IPA_LOW": "IPA 0.05",
        "IPA_1000": "IPA 1000ppm",
        "IPA_10000": "IPA 10000ppm",
        "IPA_WATER": "IPA + 물",
    }
    return names.get(label, label)


def count_labels():
    rows = load_csv(SAMPLES_CSV)
    counts = {"IPA": 0, "ETHANOL": 0}

    for r in rows:
        g = label_group(r.get("label", ""))
        if g in counts:
            counts[g] += 1

    return counts


def clean_rows(rows):
    out = []

    for r in rows:
        gas = to_float(r.get("gas_ohm", 0))

        if gas <= 0:
            continue

        if not to_bool(r.get("gas_valid", True)):
            continue

        if not to_bool(r.get("heat_stable", True)):
            continue

        if not to_bool(r.get("recordable", True)):
            continue

        rr = dict(r)
        rr["epoch"] = to_float(rr.get("epoch", 0))
        rr["gas_ohm"] = gas
        rr["hum_pct"] = to_float(rr.get("hum_pct", 0))
        rr["temp_c"] = to_float(rr.get("temp_c", 0))
        rr["press_hpa"] = to_float(rr.get("press_hpa", 0))
        rr["gas_adc"] = to_float(rr.get("gas_adc", 0))
        rr["gas_range"] = to_float(rr.get("gas_range", 0))
        out.append(rr)

    out.sort(key=lambda x: x["epoch"])
    return out


def load_air_memory():
    rows = load_csv(AIR_MEMORY_CSV)

    if not rows:
        return None

    r = rows[-1]

    gas = to_float(r.get("gas_ohm", 0))
    hum = to_float(r.get("hum_pct", 0))
    temp = to_float(r.get("temp_c", 0))

    if gas <= 0:
        return None

    return {
        "gas": gas,
        "hum": hum,
        "temp": temp,
        "timestamp": r.get("timestamp", ""),
    }


def save_air_memory(gas, hum, temp):
    row = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "gas_ohm": gas,
        "hum_pct": hum,
        "temp_c": temp,
    }
    save_dict_csv(AIR_MEMORY_CSV, row)


def calc_shape(rows):
    rows = clean_rows(rows)
    gas = [r["gas_ohm"] for r in rows]

    if len(gas) < 4:
        return {
            "smooth": 0.0,
            "slope_avg": 0.0,
            "slope_stdev": 0.0,
            "direction_changes": 0,
        }

    diffs = [gas[i] - gas[i - 1] for i in range(1, len(gas))]
    abs_diffs = [abs(x) for x in diffs]
    gas_avg = mean(gas)

    smooth = mean(abs_diffs) / gas_avg if gas_avg else 0.0

    direction_changes = 0
    prev = 0

    for d in diffs:
        cur = 1 if d > 0 else -1 if d < 0 else 0
        if prev != 0 and cur != 0 and cur != prev:
            direction_changes += 1
        if cur != 0:
            prev = cur

    return {
        "smooth": smooth,
        "slope_avg": mean(diffs),
        "slope_stdev": stdev(diffs),
        "direction_changes": direction_changes,
    }


def extract_live_feature(rows, air_gas, air_hum, label="LIVE"):
    rows = clean_rows(rows)

    if len(rows) < MIN_CLASSIFY_ROWS:
        return None

    gas = [r["gas_ohm"] for r in rows]
    hum = [r["hum_pct"] for r in rows]
    temp = [r["temp_c"] for r in rows]
    press = [r["press_hpa"] for r in rows]

    start_epoch = rows[0]["epoch"]
    end_epoch = rows[-1]["epoch"]
    duration = max(0.001, end_epoch - start_epoch)

    gas_ref = air_gas if air_gas and air_gas > 0 else gas[0]
    hum_ref = air_hum if air_hum is not None else hum[0]

    gas_now = gas[-1]
    hum_now = hum[-1]

    gas_min = min(gas)
    gas_max = max(gas)
    gas_avg = mean(gas)
    gas_end = mean(gas[-min(5, len(gas)):])

    hum_min = min(hum)
    hum_max = max(hum)
    hum_avg = mean(hum)
    hum_end = mean(hum[-min(5, len(hum)):])

    gas_now_pct = ((gas_now - gas_ref) / gas_ref) * 100.0 if gas_ref else 0.0
    gas_min_pct = ((gas_min - gas_ref) / gas_ref) * 100.0 if gas_ref else 0.0
    gas_avg_pct = ((gas_avg - gas_ref) / gas_ref) * 100.0 if gas_ref else 0.0
    gas_end_pct = ((gas_end - gas_ref) / gas_ref) * 100.0 if gas_ref else 0.0

    hum_now_delta = hum_now - hum_ref
    hum_max_delta = hum_max - hum_ref
    hum_avg_delta = hum_avg - hum_ref
    hum_end_delta = hum_end - hum_ref

    hum_gas_ratio = hum_max_delta / max(1.0, abs(gas_min_pct))

    early_n = max(2, min(len(rows), len(rows) // 3))
    late_n = max(2, min(len(rows), len(rows) // 3))

    early_gas = mean(gas[:early_n])
    late_gas = mean(gas[-late_n:])
    early_hum = mean(hum[:early_n])
    late_hum = mean(hum[-late_n:])

    shape = calc_shape(rows)

    return {
        "id": datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3],
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "label": label,
        "group": label_group(label),
        "detail": label,
        "duration_sec": duration,
        "count": len(rows),
        "gas_ref": gas_ref,
        "gas_now": gas_now,
        "gas_avg": gas_avg,
        "gas_min": gas_min,
        "gas_max": gas_max,
        "gas_end": gas_end,
        "gas_now_pct": gas_now_pct,
        "gas_min_pct": gas_min_pct,
        "gas_avg_pct": gas_avg_pct,
        "gas_end_pct": gas_end_pct,
        "gas_slope": (gas_end - early_gas) / duration,
        "late_vs_early_pct": ((late_gas - early_gas) / early_gas) * 100.0 if early_gas else 0.0,
        "hum_ref": hum_ref,
        "hum_now": hum_now,
        "hum_avg": hum_avg,
        "hum_min": hum_min,
        "hum_max": hum_max,
        "hum_end": hum_end,
        "hum_now_delta": hum_now_delta,
        "hum_max_delta": hum_max_delta,
        "hum_avg_delta": hum_avg_delta,
        "hum_end_delta": hum_end_delta,
        "hum_rise_speed": hum_max_delta / duration,
        "hum_gas_ratio": hum_gas_ratio,
        "early_hum_avg": early_hum,
        "late_hum_avg": late_hum,
        "late_hum_vs_early": late_hum - early_hum,
        "temp_avg": mean(temp),
        "press_avg": mean(press),
        "gas_smooth_score": shape["smooth"],
        "gas_slope_avg": shape["slope_avg"],
        "gas_slope_stdev": shape["slope_stdev"],
        "gas_direction_changes": shape["direction_changes"],
    }


META_KEYS = {"id", "timestamp", "label", "group", "detail", "train_type"}


def numeric_keys(row):
    keys = []

    for k, v in row.items():
        if k in META_KEYS:
            continue

        try:
            float(v)
            keys.append(k)
        except Exception:
            pass

    return keys


def key_scale(key):
    if key == "hum_gas_ratio":
        return 8.0
    if key == "duration_sec":
        return 0.7
    if key == "count":
        return 0.01
    if key in ["hum_now_delta", "hum_max_delta", "hum_avg_delta", "hum_end_delta"]:
        return 10.0
    if key == "hum_rise_speed":
        return 16.0
    if key == "late_hum_vs_early":
        return 8.0
    if key in ["hum_avg", "hum_max", "hum_now", "hum_end"]:
        return 2.2
    if key in ["gas_now_pct", "gas_min_pct", "gas_avg_pct", "gas_end_pct", "late_vs_early_pct"]:
        return 6.0
    if key == "gas_slope":
        return 0.008
    if key in ["gas_now", "gas_avg", "gas_min", "gas_max", "gas_end"]:
        return 0.0010
    if key == "gas_smooth_score":
        return 160.0
    if key == "gas_slope_stdev":
        return 0.004
    if key == "gas_direction_changes":
        return 0.12
    if key == "temp_avg":
        return 0.15
    if key == "press_avg":
        return 0.01
    return 0.001


def time_weight(current_duration, sample_duration):
    diff = abs(current_duration - sample_duration)
    if diff <= 0.5:
        return 1.7
    if diff <= 1.0:
        return 1.4
    if diff <= 2.0:
        return 1.15
    if diff <= 4.0:
        return 0.85
    return 0.55


def feature_distance(a, b):
    keys = sorted(set(numeric_keys(a)) & set(numeric_keys(b)))
    total = 0.0
    used = 0

    for k in keys:
        av = to_float(a.get(k, 0))
        bv = to_float(b.get(k, 0))

        if av == 0 and bv == 0:
            continue

        total += abs(av - bv) * key_scale(k)
        used += 1

    if used <= 0:
        return 999999.0

    return total / used


def classify_ipa_ethanol(feature):
    samples = load_csv(SAMPLES_CSV)
    usable = []

    for s in samples:
        label = s.get("label", "")
        if label in VALID_CLASSIFY_LABELS:
            usable.append(s)

    scores = {"IPA": 0.0, "ETHANOL": 0.0}
    detail_scores = {}

    if not usable:
        return {"IPA": 0.0, "ETHANOL": 0.0}, "학습 데이터 없음", {}

    cur_duration = to_float(feature.get("duration_sec", 0))

    for s in usable:
        label = s.get("label", "")
        group = label_group(label)
        d = feature_distance(feature, s)
        w = 1.0 / (1.0 + d)
        sample_duration = to_float(s.get("duration_sec", 0))
        w *= time_weight(cur_duration, sample_duration)

        if group in scores:
            scores[group] += w
            detail_scores[label] = detail_scores.get(label, 0.0) + w

    total = scores["IPA"] + scores["ETHANOL"]

    if total <= 0:
        return {"IPA": 0.0, "ETHANOL": 0.0}, "판별 불가", {}

    pct = {
        "IPA": round(scores["IPA"] / total * 100.0, 1),
        "ETHANOL": round(scores["ETHANOL"] / total * 100.0, 1),
    }

    winner = "IPA" if pct["IPA"] >= pct["ETHANOL"] else "ETHANOL"
    return pct, winner, detail_scores


def adjust_for_water_mixed_ipa(feature, pct):
    ipa = float(pct.get("IPA", 0.0))
    eth = float(pct.get("ETHANOL", 0.0))

    hum_max_delta = to_float(feature.get("hum_max_delta", 0))
    gas_min_pct = to_float(feature.get("gas_min_pct", 0))
    gas_avg_pct = to_float(feature.get("gas_avg_pct", 0))
    hum_gas_ratio = to_float(feature.get("hum_gas_ratio", 0))

    ethanol_strong = (
        hum_max_delta >= 7.0
        and abs(gas_min_pct) >= 14.0
        and gas_avg_pct <= -7.0
    )

    ethanol_humidity_pattern = (
        hum_max_delta >= 8.0
        and hum_gas_ratio >= 0.55
    )

    ipa_like = (
        hum_max_delta <= 5.5
        or hum_gas_ratio <= 0.42
    )

    if ethanol_strong:
        eth += 10.0

    if ethanol_humidity_pattern and eth >= 45.0:
        eth += 6.0

    if ipa_like:
        ipa += 6.0

    ipa = max(0.0, ipa)
    eth = max(0.0, eth)

    total = ipa + eth
    if total <= 0:
        return {"IPA": 50.0, "ETHANOL": 50.0}, "IPA"

    new_pct = {
        "IPA": round(ipa / total * 100.0, 1),
        "ETHANOL": round(eth / total * 100.0, 1),
    }

    winner = "IPA" if new_pct["IPA"] >= new_pct["ETHANOL"] else "ETHANOL"
    return new_pct, winner

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("BME688 IPA / 에탄올 판별")
        self.root.attributes("-fullscreen", True)
        self.fullscreen = True

        self.running = True
        self.sensor_ready = False

        air = load_air_memory()

        if air:
            self.mem_air_gas = air["gas"]
            self.mem_air_hum = air["hum"]
            self.mem_air_temp = air["temp"]
        else:
            self.mem_air_gas = None
            self.mem_air_hum = None
            self.mem_air_temp = None

        self.current_rows = deque(maxlen=2000)
        self.rolling_rows = deque(maxlen=300)

        self.live_state = "NORMAL"
        self.detect_count = 0
        self.normal_count = 0
        self.detect_start_time = None
        self.result_winner = None
        self.result_pct = {"IPA": 0.0, "ETHANOL": 0.0}
        self.detail_scores = {}

        self.last_state_text = "시작중"
        self.last_status_update_epoch = 0

        self.train_mode = None
        self.train_label = None
        self.train_rows = []
        self.train_ready_until = 0
        self.train_record_until = 0
        self.train_phase = None
        self.last_train_feature_time = 0
        self.train_saved_count = 0

        self.last_reaction_time = 0
        self.last_air_history_time = 0

        self.bg = "#101820"
        self.card = "#182632"
        self.text = "#EAF2F8"
        self.muted = "#94A9B8"
        self.blue = "#00E5FF"
        self.green = "#4DFF88"
        self.yellow = "#FFD166"
        self.orange = "#FF9F1C"
        self.red = "#FF4D4D"

        self.root.configure(bg=self.bg)
        self.build_ui()

        self.sensor_thread = threading.Thread(target=self.loop, daemon=True)
        self.sensor_thread.start()

        self.update_ui()

    def safe_ui(self, func, *args, **kwargs):
        try:
            self.root.after(0, lambda: func(*args, **kwargs))
        except Exception:
            pass

    def build_ui(self):
        top = tk.Frame(self.root, bg=self.bg)
        top.pack(fill="x", padx=12, pady=8)

        title_box = tk.Frame(top, bg=self.bg)
        title_box.pack(side="left", fill="x", expand=True)

        self.status_main = tk.Label(
            title_box,
            text="상태: 시작중",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 17, "bold"),
            anchor="w",
        )
        self.status_main.pack(anchor="w")

        self.status_sub = tk.Label(
            title_box,
            text="-",
            bg=self.bg,
            fg=self.muted,
            font=("NanumGothic", 11, "bold"),
            anchor="w",
        )
        self.status_sub.pack(anchor="w")

        btns = tk.Frame(top, bg=self.bg)
        btns.pack(side="right")

        buttons = [
            ("종료", self.close),
            ("재시작", self.restart_app),
            ("화면모드", self.toggle_fullscreen),
            ("데이터관리", self.show_data_manager),
            ("학습", self.show_train_menu),
            ("AIR기준저장", self.save_current_air_memory),
            ("초기화", self.reset_live_state),
        ]

        for txt, cmd in buttons:
            tk.Button(
                btns,
                text=txt,
                command=cmd,
                font=("NanumGothic", 10, "bold"),
                bg="#263847",
                fg=self.text,
                activebackground="#365369",
                activeforeground="white",
                relief="flat",
                padx=9,
                pady=8,
            ).pack(side="right", padx=3)

        info = tk.Frame(self.root, bg=self.bg)
        info.pack(fill="x", padx=12)

        self.cards = {}

        for name in ["현재상태", "가스저항", "Gas변화", "습도변화", "IPA", "에탄올"]:
            f = tk.Frame(info, bg=self.card, padx=14, pady=10)
            f.pack(side="left", fill="x", expand=True, padx=5, pady=5)

            tk.Label(
                f,
                text=name,
                bg=self.card,
                fg=self.muted,
                font=("NanumGothic", 11, "bold"),
            ).pack(anchor="w")

            value = tk.Label(
                f,
                text="-",
                bg=self.card,
                fg=self.text,
                font=("NanumGothic", 19, "bold"),
            )
            value.pack(anchor="w")

            self.cards[name] = value

        self.plot_area = tk.Frame(self.root, bg=self.bg)
        self.plot_area.pack(fill="both", expand=True, padx=12, pady=8)

        self.fig = plt.Figure(figsize=(14, 7), facecolor=self.bg)
        self.ax_gas = self.fig.add_subplot(2, 1, 1)
        self.ax_hum = self.fig.add_subplot(2, 1, 2)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_area)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def style_axis(self, ax):
        ax.set_facecolor(self.card)
        ax.tick_params(colors=self.muted)
        ax.xaxis.label.set_color(self.muted)
        ax.yaxis.label.set_color(self.muted)
        ax.title.set_color(self.text)
        ax.grid(True, color="#314452", alpha=0.45)

        for spine in ax.spines.values():
            spine.set_color("#314452")

    def update_status(self, msg=None):
        counts = count_labels()
        air_txt = "없음"

        if self.mem_air_gas and self.mem_air_hum is not None:
            air_txt = f"{self.mem_air_gas:,.0f}Ω / {self.mem_air_hum:.1f}%"

        state = msg if msg else self.last_state_text

        lock_txt = ""
        now = time.time()

        if self.last_reaction_time > 0:
            remain = AIR_HISTORY_LOCK_AFTER_REACTION_SEC - (now - self.last_reaction_time)
            if remain > 0:
                lock_txt = f" | AIR자동기록잠금 {int(remain // 60)}분"

        self.status_main.config(text=f"상태: {state}")
        self.status_sub.config(
            text=f"AIR기준 {air_txt} | IPA학습 {counts['IPA']} / ETH학습 {counts['ETHANOL']} | 판정대기 {DECISION_DELAY_SEC:.1f}s | 측정 {MEASURE_SLEEP_SEC:.2f}s{lock_txt}"
        )

    def restart_app(self):
        self.running = False
        try:
            spi.close()
        except Exception:
            pass
        python = sys.executable
        os.execv(python, [python] + sys.argv)

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def show_train_menu(self):
        if self.train_mode:
            messagebox.showinfo("안내", "이미 학습 중입니다.")
            return

        win = tk.Toplevel(self.root)
        win.title("학습 선택")
        win.geometry("520x500")
        win.configure(bg=self.bg)
        win.attributes("-topmost", True)

        tk.Label(
            win,
            text="학습 종류 선택",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 18, "bold"),
        ).pack(pady=18)

        box = tk.Frame(win, bg=self.bg)
        box.pack(fill="both", expand=True, padx=20, pady=5)

        def start_and_close(label):
            win.destroy()
            self.start_train(label)

        for txt, label in TRAIN_BUTTONS:
            tk.Button(
                box,
                text=txt,
                command=lambda x=label: start_and_close(x),
                font=("NanumGothic", 14, "bold"),
                bg="#30485A",
                fg=self.text,
                activebackground="#3F5F76",
                activeforeground="white",
                relief="flat",
                padx=10,
                pady=10,
            ).pack(fill="x", pady=4)

        tk.Button(
            win,
            text="닫기",
            command=win.destroy,
            font=("NanumGothic", 13, "bold"),
            bg="#263847",
            fg=self.text,
            activebackground="#365369",
            activeforeground="white",
            relief="flat",
            padx=10,
            pady=10,
        ).pack(fill="x", padx=20, pady=12)

    def reset_live_state(self):
        self.live_state = "NORMAL"
        self.detect_count = 0
        self.normal_count = 0
        self.detect_start_time = None
        self.result_winner = None
        self.result_pct = {"IPA": 0.0, "ETHANOL": 0.0}
        self.detail_scores = {}
        self.rolling_rows.clear()
        self.cards["현재상태"].config(text="정상범위", fg=self.blue)
        self.cards["IPA"].config(text="-")
        self.cards["에탄올"].config(text="-")
        self.last_state_text = "정상범위"

    def save_current_air_memory(self):
        rows = [
            r for r in list(self.current_rows)[-5:]
            if r.get("recordable") and r.get("gas_ohm", 0) > 0
        ]

        if len(rows) < 5:
            messagebox.showinfo("안내", "AIR 기준으로 저장할 유효 데이터가 부족합니다.")
            return

        gas = median([r["gas_ohm"] for r in rows])
        hum = median([r["hum_pct"] for r in rows])
        temp = median([r["temp_c"] for r in rows])

        self.mem_air_gas = gas
        self.mem_air_hum = hum
        self.mem_air_temp = temp

        save_air_memory(gas, hum, temp)
        self.reset_live_state()

        messagebox.showinfo("AIR 기준 저장", f"저장 완료\nGas {gas:,.0f}Ω\n습도 {hum:.1f}%")
        self.update_status("AIR 기준 저장 완료")

    def save_air_history_row(self, row, gas_delta_pct, hum_delta):
        now = row["epoch"]

        if self.last_reaction_time > 0:
            if now - self.last_reaction_time < AIR_HISTORY_LOCK_AFTER_REACTION_SEC:
                return

        if now - self.last_air_history_time < AIR_HISTORY_INTERVAL_SEC:
            return

        hist = {
            "timestamp": row["timestamp"],
            "epoch": row["epoch"],
            "gas_ohm": row["gas_ohm"],
            "hum_pct": row["hum_pct"],
            "temp_c": row["temp_c"],
            "press_hpa": row["press_hpa"],
            "gas_delta_pct": gas_delta_pct,
            "hum_delta": hum_delta,
            "heater_value": row["heater_value"],
        }

        append_dict_csv(AIR_HISTORY_CSV, hist)
        trim_csv(AIR_HISTORY_CSV, MAX_AIR_HISTORY_ROWS)
        self.last_air_history_time = now

    def start_train(self, label):
        if self.train_mode:
            messagebox.showinfo("안내", "이미 학습 중입니다.")
            return

        if not self.mem_air_gas:
            messagebox.showinfo("안내", "먼저 깨끗한 공기 상태에서 AIR기준저장을 눌러주세요.")
            return

        self.reset_live_state()

        self.train_mode = "TRAIN"
        self.train_label = label
        self.train_rows = []
        self.train_phase = "READY"
        self.train_ready_until = time.time() + TRAIN_READY_SEC
        self.train_record_until = 0
        self.last_train_feature_time = 0
        self.train_saved_count = 0

        self.cards["현재상태"].config(text=f"{label_detail_korean(label)} 학습 준비", fg=self.yellow)
        self.update_status(f"{label_detail_korean(label)} 학습 준비")

    def process_train(self, row):
        if not self.train_mode:
            return

        now = time.time()

        if self.train_phase == "READY":
            remain = int(self.train_ready_until - now)

            if remain > 0:
                self.safe_ui(
                    self.cards["현재상태"].config,
                    text=f"{label_detail_korean(self.train_label)} 학습 준비 {remain}초",
                    fg=self.yellow,
                )
                return

            self.train_phase = "RECORD"
            self.train_record_until = now + TRAIN_RECORD_SEC
            self.train_rows = []
            self.last_train_feature_time = 0
            self.train_saved_count = 0
            self.last_reaction_time = now

            self.safe_ui(
                self.cards["현재상태"].config,
                text=f"{label_detail_korean(self.train_label)} 주입 / 학습중",
                fg=self.orange,
            )
            return

        if self.train_phase == "RECORD":
            if row.get("recordable"):
                self.train_rows.append(row)

            remain = int(self.train_record_until - now)

            self.safe_ui(
                self.cards["현재상태"].config,
                text=f"{label_detail_korean(self.train_label)} 학습중 {remain}초 / 저장 {self.train_saved_count}",
                fg=self.orange,
            )

            clean = clean_rows(self.train_rows)

            if len(clean) >= MIN_TRAIN_ROWS:
                elapsed = clean[-1]["epoch"] - clean[0]["epoch"]

                if elapsed - self.last_train_feature_time >= TRAIN_FEATURE_INTERVAL_SEC:
                    feature = extract_live_feature(
                        clean,
                        self.mem_air_gas,
                        self.mem_air_hum,
                        label=self.train_label,
                    )

                    if feature:
                        feature["train_type"] = "GROUPED_INTERNAL_LABEL"
                        save_dict_csv(SAMPLES_CSV, feature)
                        self.train_saved_count += 1
                        self.last_train_feature_time = elapsed

            if remain <= 0:
                self.safe_ui(self.finish_train)

    def finish_train(self):
        label = self.train_label
        saved = self.train_saved_count

        self.train_mode = None
        self.train_label = None
        self.train_rows = []
        self.train_phase = None

        self.reset_live_state()

        messagebox.showinfo("학습 완료", f"{label_detail_korean(label)} 학습 완료\n누적 feature {saved}개 저장")
        self.cards["현재상태"].config(text="LIVE 재시작", fg=self.blue)
        self.update_status("학습 완료 / LIVE 재시작")

    def process_live(self, row):
        if self.train_mode:
            return

        if not row.get("recordable"):
            return

        if self.mem_air_gas is None or self.mem_air_gas <= 0:
            self.mem_air_gas = row["gas_ohm"]
            self.mem_air_hum = row["hum_pct"]
            self.mem_air_temp = row["temp_c"]
            save_air_memory(self.mem_air_gas, self.mem_air_hum, self.mem_air_temp)

        air_gas = self.mem_air_gas
        air_hum = self.mem_air_hum

        gas_delta_pct = ((row["gas_ohm"] - air_gas) / air_gas) * 100.0 if air_gas else 0.0
        hum_delta = row["hum_pct"] - air_hum if air_hum is not None else 0.0

        self.safe_ui(self.cards["Gas변화"].config, text=f"{gas_delta_pct:+.1f}%")
        self.safe_ui(self.cards["습도변화"].config, text=f"{hum_delta:+.2f}%p")

        now = row["epoch"]

        self.rolling_rows.append(row)

        while self.rolling_rows and now - self.rolling_rows[0]["epoch"] > ROLLING_WINDOW_SEC:
            self.rolling_rows.popleft()

        is_stable_air = (
            abs(gas_delta_pct) <= AIR_STABLE_GAS_PCT
            and abs(hum_delta) <= AIR_STABLE_HUM_DELTA
        )

        is_active = (
            gas_delta_pct <= ACTIVE_GAS_DROP_PCT
            or (hum_delta >= ACTIVE_HUM_RISE and gas_delta_pct <= -1.0)
        )

        is_return_normal = (
            abs(gas_delta_pct) <= RETURN_GAS_PCT
            and abs(hum_delta) <= RETURN_HUM_DELTA
        )

        if is_active:
            self.last_reaction_time = now

        if self.live_state == "NORMAL":
            self.result_winner = None
            self.result_pct = {"IPA": 0.0, "ETHANOL": 0.0}
            self.detail_scores = {}

            if is_stable_air:
                self.save_air_history_row(row, gas_delta_pct, hum_delta)
                self.mem_air_gas = (self.mem_air_gas * (1.0 - AIR_SLOW_ALPHA)) + (row["gas_ohm"] * AIR_SLOW_ALPHA)
                self.mem_air_hum = (self.mem_air_hum * (1.0 - AIR_SLOW_ALPHA)) + (row["hum_pct"] * AIR_SLOW_ALPHA)
                self.mem_air_temp = (self.mem_air_temp * (1.0 - AIR_SLOW_ALPHA)) + (row["temp_c"] * AIR_SLOW_ALPHA)

            if is_active:
                self.detect_count += 1
            else:
                self.detect_count = 0

            if self.detect_count >= DETECT_CONFIRM_COUNT:
                self.live_state = "ANALYZE"
                self.detect_start_time = now
                self.last_reaction_time = now
                self.normal_count = 0
                self.rolling_rows.clear()
                self.rolling_rows.append(row)

                self.safe_ui(self.cards["현재상태"].config, text="분석중", fg=self.orange)
                self.safe_ui(self.cards["IPA"].config, text="-")
                self.safe_ui(self.cards["에탄올"].config, text="-")
                self.last_state_text = "분석중"
                return

            self.safe_ui(self.cards["현재상태"].config, text="정상범위", fg=self.blue)
            self.safe_ui(self.cards["IPA"].config, text="-")
            self.safe_ui(self.cards["에탄올"].config, text="-")
            self.last_state_text = "정상범위"
            return

        if self.live_state == "ANALYZE":
            elapsed = now - self.detect_start_time if self.detect_start_time else 0.0

            self.safe_ui(self.cards["현재상태"].config, text=f"분석중 {elapsed:.1f}/{DECISION_DELAY_SEC:.1f}s", fg=self.orange)
            self.safe_ui(self.cards["IPA"].config, text="-")
            self.safe_ui(self.cards["에탄올"].config, text="-")
            self.last_state_text = "분석중"

            if elapsed < DECISION_DELAY_SEC:
                return

            clean = clean_rows(self.rolling_rows)

            if len(clean) < MIN_CLASSIFY_ROWS:
                return

            feature = extract_live_feature(
                clean,
                self.mem_air_gas,
                self.mem_air_hum,
                label="LIVE",
            )

            if not feature:
                return

            pct, winner, detail_scores = classify_ipa_ethanol(feature)

            if winner in ["학습 데이터 없음", "판별 불가"]:
                self.safe_ui(self.cards["현재상태"].config, text=winner, fg=self.red)
                self.safe_ui(self.cards["IPA"].config, text="-")
                self.safe_ui(self.cards["에탄올"].config, text="-")
                self.last_state_text = winner
                return

            pct, winner = adjust_for_water_mixed_ipa(feature, pct)

            self.result_winner = winner
            self.result_pct = pct
            self.detail_scores = detail_scores
            self.live_state = "RESULT"
            self.normal_count = 0

            self.safe_ui(self.cards["IPA"].config, text=f"{pct['IPA']:.1f}%")
            self.safe_ui(self.cards["에탄올"].config, text=f"{pct['ETHANOL']:.1f}%")

            color = self.yellow if winner == "IPA" else self.green
            self.last_state_text = f"{label_korean(winner)} {pct[winner]:.1f}%"
            self.safe_ui(self.cards["현재상태"].config, text=self.last_state_text, fg=color)
            return

        if self.live_state == "RESULT":
            winner = self.result_winner
            pct = self.result_pct

            if winner:
                self.safe_ui(self.cards["IPA"].config, text=f"{pct['IPA']:.1f}%")
                self.safe_ui(self.cards["에탄올"].config, text=f"{pct['ETHANOL']:.1f}%")

                color = self.yellow if winner == "IPA" else self.green
                self.last_state_text = f"{label_korean(winner)} {pct[winner]:.1f}%"
                self.safe_ui(self.cards["현재상태"].config, text=self.last_state_text, fg=color)

            if is_return_normal:
                self.normal_count += 1
            else:
                self.normal_count = 0

            if self.normal_count >= NORMAL_RETURN_COUNT:
                self.safe_ui(self.reset_live_state)
                return

    def show_data_manager(self):
        win = tk.Toplevel(self.root)
        win.title("데이터 관리")
        win.geometry("1520x760")
        win.configure(bg=self.bg)

        tk.Label(
            win,
            text=f"학습 데이터: {os.path.abspath(SAMPLES_CSV)}",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 13, "bold"),
        ).pack(pady=8)

        columns = (
            "id", "timestamp", "label", "group", "duration_sec", "count",
            "gas_now_pct", "gas_min_pct", "gas_avg_pct", "gas_end_pct",
            "hum_now_delta", "hum_max_delta", "hum_avg_delta", "hum_rise_speed", "hum_gas_ratio",
            "gas_slope", "gas_smooth_score", "temp_avg", "press_avg"
        )

        frame = tk.Frame(win, bg=self.bg)
        frame.pack(fill="both", expand=True, padx=10, pady=8)

        tree = ttk.Treeview(frame, columns=columns, show="headings")
        tree.pack(side="left", fill="both", expand=True)

        scroll = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        scroll.pack(side="right", fill="y")
        tree.configure(yscrollcommand=scroll.set)

        for c in columns:
            tree.heading(c, text=c)
            tree.column(c, width=105, anchor="center")

        def fmt(v):
            try:
                return f"{float(v):.2f}"
            except Exception:
                return v

        def load_table():
            for item in tree.get_children():
                tree.delete(item)

            rows = load_csv(SAMPLES_CSV)

            for i, r in enumerate(rows):
                rr = dict(r)
                rr["group"] = label_group(rr.get("label", ""))
                values = [fmt(rr.get(c, "")) for c in columns]
                tree.insert("", "end", iid=str(i), values=values)

        def delete_selected():
            sel = tree.selection()

            if not sel:
                return

            if not messagebox.askyesno("삭제", "선택 데이터를 삭제할까요?"):
                return

            rows = load_csv(SAMPLES_CSV)
            idxs = sorted([int(x) for x in sel], reverse=True)

            for idx in idxs:
                if 0 <= idx < len(rows):
                    del rows[idx]

            write_csv(SAMPLES_CSV, rows)
            load_table()
            self.update_status("데이터 삭제 완료")

        def clear_all():
            if not messagebox.askyesno("전체 삭제", "학습 데이터를 전부 삭제할까요?"):
                return

            write_csv(SAMPLES_CSV, [])
            load_table()
            self.update_status("학습 데이터 전체 삭제")

        btns = tk.Frame(win, bg=self.bg)
        btns.pack(fill="x", padx=10, pady=8)

        tk.Button(btns, text="새로고침", command=load_table, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)
        tk.Button(btns, text="선택삭제", command=delete_selected, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)
        tk.Button(btns, text="전체삭제", command=clear_all, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)
        tk.Button(btns, text="닫기", command=win.destroy, font=("NanumGothic", 11, "bold")).pack(side="right", padx=5)

        load_table()

    def loop(self):
        try:
            sensor_init()
            self.sensor_ready = True
            self.safe_ui(self.update_status, "센서 시작 완료")
        except Exception as e:
            self.safe_ui(self.update_status, f"센서 오류: {e}")
            return

        raw_trim_counter = 0

        while self.running:
            try:
                row = read_sensor()
                self.current_rows.append(row)

                append_dict_csv(RAW_CSV, row)

                raw_trim_counter += 1

                if raw_trim_counter >= 1000:
                    raw_trim_counter = 0
                    trim_csv(RAW_CSV, MAX_RAW_ROWS)

                if self.train_mode:
                    self.process_train(row)
                else:
                    self.process_live(row)

                self.safe_ui(self.cards["가스저항"].config, text=f"{row['gas_ohm']:,.0f} Ω")

                now_t = time.time()
                if now_t - self.last_status_update_epoch >= STATUS_UPDATE_SEC:
                    self.last_status_update_epoch = now_t
                    self.safe_ui(self.update_status)

            except Exception as e:
                self.safe_ui(self.status_main.config, text=f"오류: {e}")
                self.safe_ui(self.cards["현재상태"].config, text="오류", fg=self.red)

            time.sleep(LOOP_SLEEP_SEC)

    def update_ui(self):
        rows = list(self.current_rows)

        if rows:
            self.ax_gas.clear()
            self.ax_hum.clear()
            self.style_axis(self.ax_gas)
            self.style_axis(self.ax_hum)

            now = time.time()
            plot_rows = [r for r in rows if now - r["epoch"] <= ROLLING_WINDOW_SEC]

            if plot_rows and self.mem_air_gas and self.mem_air_hum is not None:
                t0 = plot_rows[0]["epoch"]
                xs = [r["epoch"] - t0 for r in plot_rows]

                gas_pct = [
                    ((r["gas_ohm"] - self.mem_air_gas) / self.mem_air_gas) * 100.0
                    for r in plot_rows
                ]

                hum_delta = [
                    r["hum_pct"] - self.mem_air_hum
                    for r in plot_rows
                ]

                self.ax_gas.plot(xs, gas_pct, linewidth=2.5)
                self.ax_gas.axhline(0, linestyle="--", linewidth=1.0)
                self.ax_gas.axhline(ACTIVE_GAS_DROP_PCT, linestyle="--", linewidth=1.0)
                self.ax_gas.set_title("AIR 기준 Gas 변화율")
                self.ax_gas.set_xlabel("최근 시간 (초)")
                self.ax_gas.set_ylabel("Gas 변화율 (%)")

                self.ax_hum.plot(xs, hum_delta, linewidth=2.5)
                self.ax_hum.axhline(0, linestyle="--", linewidth=1.0)
                self.ax_hum.axhline(ACTIVE_HUM_RISE, linestyle="--", linewidth=1.0)
                self.ax_hum.set_title("AIR 기준 습도 변화량")
                self.ax_hum.set_xlabel("최근 시간 (초)")
                self.ax_hum.set_ylabel("습도 변화량 (%p)")

            else:
                t0 = rows[0]["epoch"]
                xs = [r["epoch"] - t0 for r in rows[-100:]]
                gas = [r["gas_ohm"] for r in rows[-100:]]
                hum = [r["hum_pct"] for r in rows[-100:]]

                self.ax_gas.plot(xs, gas, linewidth=2.0)
                self.ax_gas.set_title("Gas 원본")
                self.ax_gas.set_xlabel("시간 (초)")
                self.ax_gas.set_ylabel("Gas 저항 (Ω)")

                self.ax_hum.plot(xs, hum, linewidth=2.0)
                self.ax_hum.set_title("습도 원본")
                self.ax_hum.set_xlabel("시간 (초)")
                self.ax_hum.set_ylabel("습도 (%)")

            self.fig.suptitle(
                "BME688 IPA / 에탄올 판별",
                color=self.text,
                fontsize=15,
                fontweight="bold",
            )

            self.fig.tight_layout()
            self.canvas.draw_idle()

        if self.running:
            self.root.after(UI_UPDATE_MS, self.update_ui)

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
