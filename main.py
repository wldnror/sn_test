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


DATA_DIR = "bme688_data"
os.makedirs(DATA_DIR, exist_ok=True)

SAMPLES_CSV = os.path.join(DATA_DIR, "samples.csv")
FAST_SAMPLES_CSV = os.path.join(DATA_DIR, "fast_samples.csv")
BASELINE_CSV = os.path.join(DATA_DIR, "baseline_auto.csv")
RAW_CSV = os.path.join(DATA_DIR, "raw_log.csv")

STARTUP_WARMUP_SEC = 600
AUTO_BASELINE_SEC = 1800
AIR_TRAIN_SEC = 1800
GAS_TRAIN_SEC = 300

READY_SEC = 30
COOLDOWN_SEC = 600
RECOVERY_TOL = 0.15

MAX_RAW_ROWS = 100000
MAX_BASELINE_ROWS = 100
MAX_AUTO_AIR_SAMPLES = 20

FAST_SCAN_SEC = 30
FAST_WINDOWS = [5, 10, 20, 30]

FAST_MEASURE_SLEEP = 0.30
NORMAL_MEASURE_SLEEP = 0.80
FAST_LOOP_SLEEP = 0.02
NORMAL_LOOP_SLEEP = 0.40

FAST_TRIGGER_MIN_AIR_SEC = 8.0
FAST_TRIGGER_LOOKBACK_SEC = 3.0
FAST_TRIGGER_GAS_DROP_PCT = -2.5
FAST_TRIGGER_HUM_RISE = 0.45
FAST_TRIGGER_GAS_SLOPE_PCT_PER_SEC = -0.7
FAST_TRIGGER_HUM_SLOPE_PER_SEC = 0.12

FAST_CONF_5 = 55.0
FAST_CONF_10 = 62.0
FAST_CONF_20 = 70.0
FAST_CONF_30 = 76.0

HEATER_STEPS = [
    ("H1_LOW",  0x45, 15, 4),
    ("H2_LOW2", 0x49, 15, 4),
    ("H3_LOW3", 0x4D, 15, 4),
    ("H4_LOW4", 0x51, 15, 4),
    ("H5_LOW5", 0x55, 15, 4),
]

FAST_HEATER = ("F1_FAST_LOW", 0x45, 30, 0.8)

TRAIN_PRESETS = [
    {"button": "정상공기 30분", "label": "AIR", "level": "AIR", "amount_ml": "0", "duration_sec": AIR_TRAIN_SEC},
    {"button": "IPA 0.05mL 5분", "label": "IPA", "level": "LOW", "amount_ml": "0.05", "duration_sec": GAS_TRAIN_SEC},
    {"button": "에탄올 0.05mL 5분", "label": "ETHANOL", "level": "LOW", "amount_ml": "0.05", "duration_sec": GAS_TRAIN_SEC},
    {"button": "IPA HIGH 5분", "label": "IPA", "level": "HIGH", "amount_ml": "HIGH", "duration_sec": GAS_TRAIN_SEC},
    {"button": "에탄올 HIGH 5분", "label": "ETHANOL", "level": "HIGH", "amount_ml": "HIGH", "duration_sec": GAS_TRAIN_SEC},
]

FAST_TRAIN_PRESETS = [
    {"button": "빠른학습 IPA 0.05", "label": "IPA", "level": "FAST_LOW", "amount_ml": "0.05"},
    {"button": "빠른학습 에탄올 0.05", "label": "ETHANOL", "level": "FAST_LOW", "amount_ml": "0.05"},
    {"button": "빠른학습 IPA HIGH", "label": "IPA", "level": "FAST_HIGH", "amount_ml": "HIGH"},
    {"button": "빠른학습 에탄올 HIGH", "label": "ETHANOL", "level": "FAST_HIGH", "amount_ml": "HIGH"},
]

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
    var2 = var1 * ((cal["par_h2"] / 262144.0) * (1.0 + (cal["par_h4"] / 16384.0) * temp_c + (cal["par_h5"] / 1048576.0) * temp_c * temp_c))
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


def read_sensor(heater_name, heater_value, heater_elapsed, heater_recordable, measure_sleep):
    set_heater(heater_value)
    trigger_measurement()
    time.sleep(measure_sleep)

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

    rows = load_csv(path)
    rows.append(row)
    write_csv(path, rows)


def trim_csv(path, max_rows):
    rows = load_csv(path)

    if len(rows) > max_rows:
        write_csv(path, rows[-max_rows:])


def clean_valid_rows(rows):
    valid = []

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
        rr["temp_c"] = to_float(rr.get("temp_c", 0))
        rr["hum_pct"] = to_float(rr.get("hum_pct", 0))
        rr["press_hpa"] = to_float(rr.get("press_hpa", 0))
        rr["gas_adc"] = to_float(rr.get("gas_adc", 0))
        rr["gas_range"] = to_float(rr.get("gas_range", 0))

        valid.append(rr)

    valid.sort(key=lambda x: x["epoch"])
    return valid


def rows_after(rows, start_epoch, until_sec=None):
    result = []

    for r in rows:
        ep = to_float(r.get("epoch", 0))

        if ep < start_epoch:
            continue

        if until_sec is not None and ep > start_epoch + until_sec:
            continue

        result.append(r)

    return result


def calc_shape(rows):
    valid = clean_valid_rows(rows)
    gas = [r["gas_ohm"] for r in valid if r["gas_ohm"] > 0]

    if len(gas) < 3:
        return {
            "gas_smooth_score": 0,
            "gas_step_score": 0,
            "gas_slope_avg": 0,
            "gas_slope_stdev": 0,
            "gas_direction_changes": 0,
        }

    diffs = [gas[i] - gas[i - 1] for i in range(1, len(gas))]
    abs_diffs = [abs(x) for x in diffs]
    gas_avg = mean(gas)

    direction_changes = 0
    prev = 0

    for d in diffs:
        cur = 1 if d > 0 else -1 if d < 0 else 0

        if prev != 0 and cur != 0 and cur != prev:
            direction_changes += 1

        if cur != 0:
            prev = cur

    heater_avgs = []

    for h, _, _, _ in HEATER_STEPS:
        vals = [r["gas_ohm"] for r in valid if r.get("heater") == h]

        if vals:
            heater_avgs.append(mean(vals))

    return {
        "gas_smooth_score": mean(abs_diffs) / gas_avg if gas_avg else 0,
        "gas_step_score": stdev(heater_avgs) / mean(heater_avgs) if len(heater_avgs) >= 2 and mean(heater_avgs) else 0,
        "gas_slope_avg": mean(diffs),
        "gas_slope_stdev": stdev(diffs),
        "gas_direction_changes": direction_changes,
    }


def make_full_feature(rows, label, level, amount_ml):
    valid = clean_valid_rows(rows)

    if len(valid) < 5:
        return None

    gas = [r["gas_ohm"] for r in valid]
    hum = [r["hum_pct"] for r in valid]
    temp = [r["temp_c"] for r in valid]
    press = [r["press_hpa"] for r in valid]

    gas_start = mean(gas[:min(10, len(gas))])
    gas_end = mean(gas[-min(10, len(gas)):])
    gas_min = min(gas)
    gas_max = max(gas)
    duration = max(0.001, valid[-1]["epoch"] - valid[0]["epoch"])

    hum_start = mean(hum[:min(10, len(hum))])
    hum_end = mean(hum[-min(10, len(hum)):])
    hum_min = min(hum)
    hum_max = max(hum)

    n = len(valid)
    one = max(1, n // 3)

    early = valid[:one]
    mid = valid[one:one * 2]
    late = valid[one * 2:]

    early_gas = [r["gas_ohm"] for r in early]
    mid_gas = [r["gas_ohm"] for r in mid]
    late_gas = [r["gas_ohm"] for r in late]

    early_hum = [r["hum_pct"] for r in early]
    mid_hum = [r["hum_pct"] for r in mid]
    late_hum = [r["hum_pct"] for r in late]

    shape = calc_shape(valid)
    now = datetime.now()

    feature = {
        "id": now.strftime("%Y%m%d_%H%M%S"),
        "timestamp": now.isoformat(timespec="seconds"),
        "label": label,
        "level": level,
        "amount_ml": amount_ml,
        "count": len(valid),
        "duration_sec": duration,
        "gas_avg": mean(gas),
        "gas_min": gas_min,
        "gas_max": gas_max,
        "gas_start": gas_start,
        "gas_end": gas_end,
        "gas_stdev": stdev(gas),
        "gas_end_vs_start_pct": ((gas_end - gas_start) / gas_start) * 100 if gas_start else 0,
        "gas_drop_pct": ((gas_min - gas_start) / gas_start) * 100 if gas_start else 0,
        "temp_avg": mean(temp),
        "temp_stdev": stdev(temp),
        "hum_avg": mean(hum),
        "hum_min": hum_min,
        "hum_max": hum_max,
        "hum_start": hum_start,
        "hum_end": hum_end,
        "hum_rise_abs": hum_max - hum_start,
        "hum_end_vs_start": hum_end - hum_start,
        "press_avg": mean(press),
        "press_stdev": stdev(press),
        "early_gas_avg": mean(early_gas),
        "mid_gas_avg": mean(mid_gas),
        "late_gas_avg": mean(late_gas),
        "early_hum_avg": mean(early_hum),
        "mid_hum_avg": mean(mid_hum),
        "late_hum_avg": mean(late_hum),
        "mid_vs_early_gas_pct": ((mean(mid_gas) - mean(early_gas)) / mean(early_gas)) * 100 if mean(early_gas) else 0,
        "late_vs_early_gas_pct": ((mean(late_gas) - mean(early_gas)) / mean(early_gas)) * 100 if mean(early_gas) else 0,
        "mid_vs_early_hum": mean(mid_hum) - mean(early_hum),
        "late_vs_early_hum": mean(late_hum) - mean(early_hum),
        "gas_smooth_score": shape["gas_smooth_score"],
        "gas_step_score": shape["gas_step_score"],
        "gas_slope_avg": shape["gas_slope_avg"],
        "gas_slope_stdev": shape["gas_slope_stdev"],
        "gas_direction_changes": shape["gas_direction_changes"],
    }

    for h, _, _, _ in HEATER_STEPS:
        hrows = [r for r in valid if r.get("heater") == h]
        hg = [r["gas_ohm"] for r in hrows]
        hh = [r["hum_pct"] for r in hrows]
        key = h.lower()

        if hg:
            feature[f"{key}_gas_avg"] = mean(hg)
            feature[f"{key}_gas_min"] = min(hg)
            feature[f"{key}_gas_max"] = max(hg)
            feature[f"{key}_gas_stdev"] = stdev(hg)
            feature[f"{key}_count"] = len(hg)
        else:
            feature[f"{key}_gas_avg"] = 0
            feature[f"{key}_gas_min"] = 0
            feature[f"{key}_gas_max"] = 0
            feature[f"{key}_gas_stdev"] = 0
            feature[f"{key}_count"] = 0

        if hh:
            feature[f"{key}_hum_avg"] = mean(hh)
            feature[f"{key}_hum_min"] = min(hh)
            feature[f"{key}_hum_max"] = max(hh)
        else:
            feature[f"{key}_hum_avg"] = 0
            feature[f"{key}_hum_min"] = 0
            feature[f"{key}_hum_max"] = 0

    return feature


def make_fast_feature(rows, label, level, amount_ml, trigger_epoch, trigger_gas, trigger_hum, window_sec):
    valid = clean_valid_rows(rows_after(rows, trigger_epoch, window_sec))

    if len(valid) < 3:
        return None

    gas = [r["gas_ohm"] for r in valid]
    hum = [r["hum_pct"] for r in valid]
    temp = [r["temp_c"] for r in valid]
    press = [r["press_hpa"] for r in valid]

    first_gas = gas[0]
    last_gas = gas[-1]
    min_gas = min(gas)
    max_gas = max(gas)

    first_hum = hum[0]
    last_hum = hum[-1]
    min_hum = min(hum)
    max_hum = max(hum)

    duration = max(0.001, valid[-1]["epoch"] - valid[0]["epoch"])

    gas_drop_from_trigger_pct = ((min_gas - trigger_gas) / trigger_gas) * 100 if trigger_gas else 0
    gas_end_from_trigger_pct = ((last_gas - trigger_gas) / trigger_gas) * 100 if trigger_gas else 0
    hum_rise_from_trigger = max_hum - trigger_hum
    hum_end_from_trigger = last_hum - trigger_hum

    gas_slope_pct_per_sec = gas_end_from_trigger_pct / duration
    hum_slope_per_sec = hum_end_from_trigger / duration

    shape = calc_shape(valid)
    now = datetime.now()

    f = {
        "id": now.strftime("%Y%m%d_%H%M%S"),
        "timestamp": now.isoformat(timespec="seconds"),
        "label": label,
        "level": level,
        "amount_ml": amount_ml,
        "window_sec": window_sec,
        "count": len(valid),
        "duration_sec": duration,
        "trigger_gas": trigger_gas,
        "trigger_hum": trigger_hum,
        "first_gas": first_gas,
        "last_gas": last_gas,
        "gas_avg": mean(gas),
        "gas_min": min_gas,
        "gas_max": max_gas,
        "gas_stdev": stdev(gas),
        "gas_drop_from_trigger_pct": gas_drop_from_trigger_pct,
        "gas_end_from_trigger_pct": gas_end_from_trigger_pct,
        "gas_local_end_vs_start_pct": ((last_gas - first_gas) / first_gas) * 100 if first_gas else 0,
        "gas_slope_pct_per_sec": gas_slope_pct_per_sec,
        "first_hum": first_hum,
        "last_hum": last_hum,
        "hum_avg": mean(hum),
        "hum_min": min_hum,
        "hum_max": max_hum,
        "hum_rise_from_trigger": hum_rise_from_trigger,
        "hum_end_from_trigger": hum_end_from_trigger,
        "hum_local_end_vs_start": last_hum - first_hum,
        "hum_slope_per_sec": hum_slope_per_sec,
        "temp_avg": mean(temp),
        "temp_stdev": stdev(temp),
        "press_avg": mean(press),
        "press_stdev": stdev(press),
        "gas_smooth_score": shape["gas_smooth_score"],
        "gas_step_score": shape["gas_step_score"],
        "gas_slope_avg_raw": shape["gas_slope_avg"],
        "gas_slope_stdev_raw": shape["gas_slope_stdev"],
        "gas_direction_changes": shape["gas_direction_changes"],
    }

    return f


META_KEYS = {
    "id", "timestamp", "label", "level", "amount_ml",
}


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


def fast_key_scale(key):
    if key == "window_sec":
        return 0

    if key in ["count", "duration_sec"]:
        return 0.001

    if key in ["trigger_gas", "first_gas", "last_gas", "gas_avg", "gas_min", "gas_max"]:
        return 0.00045

    if key in ["gas_drop_from_trigger_pct", "gas_end_from_trigger_pct", "gas_local_end_vs_start_pct"]:
        return 5.5

    if key in ["gas_slope_pct_per_sec"]:
        return 8.0

    if key in ["trigger_hum", "first_hum", "last_hum", "hum_avg", "hum_min", "hum_max"]:
        return 1.2

    if key in ["hum_rise_from_trigger", "hum_end_from_trigger", "hum_local_end_vs_start"]:
        return 6.5

    if key in ["hum_slope_per_sec"]:
        return 18.0

    if key in ["temp_avg"]:
        return 0.15

    if key in ["press_avg"]:
        return 0.02

    if key in ["gas_smooth_score"]:
        return 160.0

    if key in ["gas_step_score"]:
        return 90.0

    if key in ["gas_slope_stdev_raw"]:
        return 0.002

    if key in ["gas_direction_changes"]:
        return 0.05

    return 0.001


def full_key_scale(key):
    if key in ["count", "duration_sec"]:
        return 0.001

    if key in ["gas_avg", "gas_min", "gas_max", "gas_start", "gas_end"]:
        return 0.0007

    if key in ["gas_end_vs_start_pct", "gas_drop_pct", "mid_vs_early_gas_pct", "late_vs_early_gas_pct"]:
        return 2.8

    if key in ["hum_avg", "hum_min", "hum_max", "hum_start", "hum_end"]:
        return 2.0

    if key in ["hum_rise_abs", "hum_end_vs_start", "mid_vs_early_hum", "late_vs_early_hum"]:
        return 4.0

    if key in ["temp_avg"]:
        return 0.15

    if key in ["press_avg"]:
        return 0.02

    if key in ["gas_smooth_score"]:
        return 120.0

    if key in ["gas_step_score"]:
        return 80.0

    if "h1_low" in key or "h2_low2" in key or "h3_low3" in key or "h4_low4" in key or "h5_low5" in key:
        if "hum" in key:
            return 1.6
        return 0.0007

    return 0.001


def distance(a, b, mode="fast"):
    keys = sorted(set(numeric_keys(a)) & set(numeric_keys(b)))
    total = 0.0

    for k in keys:
        av = to_float(a.get(k, 0))
        bv = to_float(b.get(k, 0))

        if av == 0 and bv == 0:
            continue

        scale = fast_key_scale(k) if mode == "fast" else full_key_scale(k)
        total += abs(av - bv) * scale

    return total


def env_weight(feature, sample):
    ft = to_float(feature.get("temp_avg", 0))
    st = to_float(sample.get("temp_avg", 0))
    fh = to_float(feature.get("hum_avg", 0))
    sh = to_float(sample.get("hum_avg", 0))
    fp = to_float(feature.get("press_avg", 0))
    sp = to_float(sample.get("press_avg", 0))

    w = 1.0

    if abs(ft - st) <= 2:
        w *= 1.20
    elif abs(ft - st) >= 8:
        w *= 0.70

    if abs(fh - sh) <= 5:
        w *= 1.30
    elif abs(fh - sh) >= 18:
        w *= 0.60

    if abs(fp - sp) <= 4:
        w *= 1.10

    return w


def classify_fast(feature, window_sec):
    samples = load_csv(FAST_SAMPLES_CSV)
    usable = [s for s in samples if s.get("label") in ["IPA", "ETHANOL"] and int(to_float(s.get("window_sec", 0))) == int(window_sec)]

    scores = {"IPA": 0.0, "ETHANOL": 0.0}

    if not usable:
        return {"IPA": 0, "ETHANOL": 0}, "학습 데이터 없음"

    for s in usable:
        d = distance(feature, s, mode="fast")
        w = 1.0 / (1.0 + d)
        w *= env_weight(feature, s)
        scores[s["label"]] += w

    total = scores["IPA"] + scores["ETHANOL"]

    if total <= 0:
        return {"IPA": 0, "ETHANOL": 0}, "판별 불가"

    pct = {
        "IPA": round(scores["IPA"] / total * 100, 1),
        "ETHANOL": round(scores["ETHANOL"] / total * 100, 1),
    }

    winner = "IPA" if pct["IPA"] >= pct["ETHANOL"] else "ETHANOL"
    return pct, winner


def classify_full(feature):
    samples = load_csv(SAMPLES_CSV)
    usable = [s for s in samples if s.get("label") in ["AIR", "IPA", "ETHANOL"]]

    scores = {"AIR": 0.0, "IPA": 0.0, "ETHANOL": 0.0}

    if not usable:
        return {"AIR": 0, "IPA": 0, "ETHANOL": 0}, "학습 데이터 없음"

    for s in usable:
        d = distance(feature, s, mode="full")
        w = 1.0 / (1.0 + d)
        w *= env_weight(feature, s)
        scores[s["label"]] += w

    total = sum(scores.values())

    if total <= 0:
        return {"AIR": 0, "IPA": 0, "ETHANOL": 0}, "판별 불가"

    pct = {k: round(v / total * 100, 1) for k, v in scores.items()}
    winner = max(pct, key=pct.get)

    return pct, winner


def label_kr(label):
    if label == "AIR":
        return "정상공기"
    if label == "IPA":
        return "IPA"
    if label == "ETHANOL":
        return "에탄올"
    if label == "UNKNOWN":
        return "미지시료"
    return label


def confidence_limit(window_sec):
    if window_sec <= 5:
        return FAST_CONF_5
    if window_sec <= 10:
        return FAST_CONF_10
    if window_sec <= 20:
        return FAST_CONF_20
    return FAST_CONF_30


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("BME688 IPA / ETHANOL 빠른 판별")
        self.root.attributes("-fullscreen", True)

        self.running = True
        self.fullscreen = True

        self.bg = "#101820"
        self.card = "#182632"
        self.text = "#EAF2F8"
        self.muted = "#94A9B8"
        self.blue = "#0066CC"
        self.green = "#008855"
        self.orange = "#D35400"
        self.yellow = "#B8860B"
        self.red = "#B00020"

        self.current_rows = deque(maxlen=3000)
        self.raw_trim_counter = 0

        self.startup_active = True
        self.startup_until = time.time() + STARTUP_WARMUP_SEC

        self.cooldown_active = False
        self.cooldown_until = 0

        self.auto_baseline = False
        self.baseline_rows = []
        self.baseline_started_at = None
        self.latest_air_gas = None

        self.mode = "IDLE"
        self.sample_rows = []
        self.pending_label = None
        self.pending_level = None
        self.pending_amount_ml = None
        self.sample_phase = None
        self.phase_until = 0
        self.expose_until = 0
        self.duration_sec = 0

        self.heater_index = 0
        self.heater_started_at = time.time()
        self.heater_switch_at = time.time() + HEATER_STEPS[0][2]
        self.cycle_count = 0

        self.fast_state = "IDLE"
        self.fast_label = None
        self.fast_level = None
        self.fast_amount_ml = None
        self.fast_rows = []
        self.fast_air_rows = deque(maxlen=200)
        self.fast_trigger_epoch = None
        self.fast_trigger_gas = None
        self.fast_trigger_hum = None
        self.fast_reported = set()
        self.fast_started_at = None
        self.fast_last_result = "-"

        self.build_ui()

        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()

        self.update_ui()

    def build_ui(self):
        self.root.configure(bg=self.bg)

        top = tk.Frame(self.root, bg=self.bg)
        top.pack(fill="x", padx=12, pady=8)

        left = tk.Frame(top, bg=self.bg)
        left.pack(side="left", fill="x", expand=True)

        self.status_main = tk.Label(left, text="상태: 시작", bg=self.bg, fg=self.text, font=("NanumGothic", 15, "bold"), anchor="w")
        self.status_main.pack(anchor="w")

        self.status_sub = tk.Label(left, text="-", bg=self.bg, fg=self.muted, font=("NanumGothic", 10, "bold"), anchor="w")
        self.status_sub.pack(anchor="w")

        btns = tk.Frame(top, bg=self.bg)
        btns.pack(side="right")

        buttons = [
            ("종료", self.close),
            ("데이터관리", self.show_data_manager),
            ("빠른판별", self.start_fast_detect),
            ("빠른학습", self.open_fast_learn_menu),
            ("정밀학습", self.open_learn_menu),
            ("AIR기준", self.toggle_air_baseline),
            ("안정화건너뛰기", self.skip_startup),
            ("화면모드", self.toggle_fullscreen),
        ]

        for text, cmd in buttons:
            tk.Button(
                btns,
                text=text,
                command=cmd,
                font=("NanumGothic", 11, "bold"),
                bg="#263847",
                fg=self.text,
                activebackground="#365369",
                activeforeground="white",
                relief="flat",
                padx=10,
                pady=8,
                width=12,
            ).pack(side="right", padx=3)

        cards = tk.Frame(self.root, bg=self.bg)
        cards.pack(fill="x", padx=12)

        self.cards = {}

        for name in ["상태", "가스Ω", "Gas변화%", "습도변화", "히터", "판별결과"]:
            f = tk.Frame(cards, bg=self.card, padx=12, pady=10)
            f.pack(side="left", fill="x", expand=True, padx=5, pady=5)

            tk.Label(f, text=name, bg=self.card, fg=self.muted, font=("NanumGothic", 11)).pack(anchor="w")

            v = tk.Label(f, text="-", bg=self.card, fg=self.text, font=("NanumGothic", 17, "bold"))
            v.pack(anchor="w")

            self.cards[name] = v

        plot_frame = tk.Frame(self.root, bg=self.bg)
        plot_frame.pack(fill="both", expand=True, padx=12, pady=8)

        self.fig = plt.Figure(figsize=(14, 7), facecolor=self.bg)
        self.ax1 = self.fig.add_subplot(3, 1, 1)
        self.ax2 = self.fig.add_subplot(3, 1, 2)
        self.ax3 = self.fig.add_subplot(3, 1, 3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.build_notice()

    def build_notice(self):
        self.notice = tk.Frame(self.root, bg=self.blue, bd=6, relief="ridge")

        self.notice_title = tk.Label(self.notice, text="", bg=self.blue, fg="white", font=("NanumGothic", 34, "bold"), padx=40, pady=12)
        self.notice_title.pack(fill="x")

        self.notice_msg = tk.Label(self.notice, text="", bg=self.blue, fg="white", font=("NanumGothic", 23, "bold"), padx=40, pady=10, justify="center")
        self.notice_msg.pack(fill="x")

        self.notice_timer = tk.Label(self.notice, text="", bg=self.blue, fg="white", font=("NanumGothic", 32, "bold"), padx=40, pady=10)
        self.notice_timer.pack(fill="x")

        self.notice_sub = tk.Label(self.notice, text="", bg=self.blue, fg="white", font=("NanumGothic", 15, "bold"), padx=40, pady=10)
        self.notice_sub.pack(fill="x")

        self.hide_notice()

    def show_notice(self, title, msg, remain=None, color=None, sub=""):
        if color is None:
            color = self.blue

        for w in [self.notice, self.notice_title, self.notice_msg, self.notice_timer, self.notice_sub]:
            w.configure(bg=color)

        self.notice_title.configure(text=title)
        self.notice_msg.configure(text=msg)
        self.notice_sub.configure(text=sub)

        if remain is None:
            self.notice_timer.configure(text="")
        else:
            if remain >= 60:
                self.notice_timer.configure(text=f"남은 시간 {remain // 60:02d}:{remain % 60:02d}")
            else:
                self.notice_timer.configure(text=f"남은 시간 {remain}초")

        self.notice.place(relx=0.5, rely=0.48, anchor="center")

    def hide_notice(self):
        self.notice.place_forget()

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def skip_startup(self):
        self.startup_active = False
        self.auto_baseline = True
        self.baseline_rows = []
        self.baseline_started_at = None
        self.hide_notice()
        self.set_status("초기 안정화 건너뜀 / AIR 기준 기록 시작")

    def toggle_air_baseline(self):
        if self.startup_active:
            messagebox.showinfo("안내", "초기 안정화 중에는 AIR 기준 기록을 켤 수 없습니다.")
            return

        if self.mode != "IDLE" or self.fast_state != "IDLE":
            messagebox.showinfo("안내", "학습/판별 중에는 AIR 기준 기록을 켤 수 없습니다.")
            return

        self.auto_baseline = not self.auto_baseline
        self.baseline_rows = []
        self.baseline_started_at = None

        self.set_status("AIR 기준 기록 ON" if self.auto_baseline else "AIR 기준 기록 OFF")

    def set_status(self, text):
        self.status_main.configure(text=f"상태: {text}")

        samples = load_csv(SAMPLES_CSV)
        fast = load_csv(FAST_SAMPLES_CSV)
        air = len([s for s in samples if s.get("label") == "AIR"])
        ipa = len([s for s in samples if s.get("label") == "IPA"])
        eth = len([s for s in samples if s.get("label") == "ETHANOL"])
        fipa = len([s for s in fast if s.get("label") == "IPA"])
        feth = len([s for s in fast if s.get("label") == "ETHANOL"])

        self.status_sub.configure(
            text=f"정밀 AIR {air} / IPA {ipa} / ETH {eth} | 빠른 IPA {fipa} / 빠른 ETH {feth} | AIR기준 {'ON' if self.auto_baseline else 'OFF'}"
        )

    def is_blocked(self):
        if self.startup_active:
            remain = max(0, int(self.startup_until - time.time()))
            messagebox.showinfo("초기 안정화", f"초기 안정화 중입니다.\n남은 시간 {remain // 60}분 {remain % 60}초")
            return True

        if self.cooldown_active:
            remain = max(0, int(self.cooldown_until - time.time()))

            if remain > 0:
                messagebox.showinfo("쿨타임", f"시료 잔류 제거 대기중입니다.\n남은 시간 {remain // 60}분 {remain % 60}초")
                return True

        return False

    def open_learn_menu(self):
        if self.is_blocked():
            return

        win = tk.Toplevel(self.root)
        win.title("정밀 학습")
        win.geometry("500x560")
        win.configure(bg=self.bg)

        tk.Label(win, text="정밀 학습", bg=self.bg, fg=self.text, font=("NanumGothic", 18, "bold")).pack(pady=16)

        for p in TRAIN_PRESETS:
            tk.Button(
                win,
                text=p["button"],
                command=lambda x=p, w=win: (w.destroy(), self.start_full_training(x)),
                font=("NanumGothic", 14, "bold"),
                bg="#263847",
                fg=self.text,
                relief="flat",
                padx=20,
                pady=12,
            ).pack(fill="x", padx=34, pady=7)

    def open_fast_learn_menu(self):
        if self.is_blocked():
            return

        win = tk.Toplevel(self.root)
        win.title("빠른 학습")
        win.geometry("500x430")
        win.configure(bg=self.bg)

        tk.Label(win, text="빠른 판별용 학습", bg=self.bg, fg=self.text, font=("NanumGothic", 18, "bold")).pack(pady=16)
        tk.Label(win, text="자동 trigger 감지 후 5/10/20/30초 feature를 저장합니다.", bg=self.bg, fg=self.muted, font=("NanumGothic", 11, "bold")).pack(pady=4)

        for p in FAST_TRAIN_PRESETS:
            tk.Button(
                win,
                text=p["button"],
                command=lambda x=p, w=win: (w.destroy(), self.start_fast_train(x)),
                font=("NanumGothic", 14, "bold"),
                bg="#263847",
                fg=self.text,
                relief="flat",
                padx=20,
                pady=12,
            ).pack(fill="x", padx=34, pady=7)

    def start_full_training(self, preset):
        self.auto_baseline = False
        self.mode = "FULL_LEARN"
        self.sample_rows = []
        self.pending_label = preset["label"]
        self.pending_level = preset["level"]
        self.pending_amount_ml = preset["amount_ml"]
        self.duration_sec = preset["duration_sec"]

        self.sample_phase = "READY"
        self.phase_until = time.time() + READY_SEC
        self.expose_until = 0

        self.heater_index = 0
        self.heater_started_at = time.time()
        self.heater_switch_at = time.time() + HEATER_STEPS[0][2]
        self.cycle_count = 0

        self.show_notice(
            f"{label_kr(self.pending_label)} 정밀학습 준비",
            "아직 시료를 주입하지 마세요" if self.pending_label != "AIR" else "정상공기 상태 유지",
            READY_SEC,
            self.blue,
            "준비가 끝나면 기록을 시작합니다.",
        )
        self.set_status("정밀 학습 준비")

    def start_fast_train(self, preset):
        self.start_fast_common("FAST_LEARN", preset["label"], preset["level"], preset["amount_ml"])

    def start_fast_detect(self):
        self.start_fast_common("FAST_DETECT", "UNKNOWN", "FAST_UNKNOWN", "UNKNOWN")

    def start_fast_common(self, state, label, level, amount_ml):
        if self.is_blocked():
            return

        self.auto_baseline = False
        self.mode = "FAST"
        self.fast_state = state
        self.fast_label = label
        self.fast_level = level
        self.fast_amount_ml = amount_ml
        self.fast_rows = []
        self.fast_air_rows.clear()
        self.fast_trigger_epoch = None
        self.fast_trigger_gas = None
        self.fast_trigger_hum = None
        self.fast_reported.clear()
        self.fast_started_at = time.time()
        self.fast_last_result = "-"

        self.show_notice(
            "빠른 판별 준비",
            "정상공기 상태로 잠시 유지하세요",
            None,
            self.blue,
            "변화가 감지되면 자동으로 t=0을 잡습니다.",
        )
        self.set_status("빠른 모드 AIR 기준 대기")

    def start_cooldown(self):
        self.cooldown_active = True
        self.cooldown_until = time.time() + COOLDOWN_SEC
        self.mode = "IDLE"
        self.fast_state = "IDLE"
        self.auto_baseline = False

        self.show_notice("쿨타임", "시료를 제거하고 환기하세요", COOLDOWN_SEC, self.yellow, "쿨타임 중 학습/판별 차단")
        self.set_status("시료 잔류 제거 대기")

    def update_heater(self):
        if self.mode == "FAST":
            h, v, _, settle = FAST_HEATER
            elapsed = time.time() - self.fast_started_at if self.fast_started_at else 0
            return h, v, elapsed, elapsed >= settle

        now = time.time()
        h, v, step_sec, settle = HEATER_STEPS[self.heater_index]

        if now >= self.heater_switch_at:
            self.heater_index = (self.heater_index + 1) % len(HEATER_STEPS)

            if self.heater_index == 0:
                self.cycle_count += 1

            h, v, step_sec, settle = HEATER_STEPS[self.heater_index]
            self.heater_started_at = now
            self.heater_switch_at = now + step_sec

        elapsed = now - self.heater_started_at
        return h, v, elapsed, elapsed >= settle

    def check_startup(self):
        if not self.startup_active:
            return

        remain = int(self.startup_until - time.time())

        if remain > 0:
            self.auto_baseline = False
            self.show_notice("초기 안정화", "센서 예열 및 안정화 중", remain, self.blue, "학습/판별 차단")
            self.set_status("초기 안정화")
            return

        self.startup_active = False
        self.auto_baseline = True
        self.baseline_rows = []
        self.baseline_started_at = None
        self.show_notice("초기 안정화 완료", "AIR 기준 기록을 시작합니다", None, self.green, "")
        self.root.after(3000, self.hide_notice)
        self.set_status("초기 안정화 완료")

    def check_cooldown(self):
        if not self.cooldown_active:
            return

        remain = int(self.cooldown_until - time.time())

        if remain > 0:
            self.show_notice("쿨타임", "시료를 제거하고 환기하세요", remain, self.yellow, "")
            self.set_status("쿨타임")
            return

        self.cooldown_active = False
        self.auto_baseline = True
        self.show_notice("쿨타임 완료", "AIR 기준 기록을 재개합니다", None, self.green, "")
        self.root.after(3000, self.hide_notice)
        self.set_status("쿨타임 완료")

    def handle_auto_baseline(self, row):
        if not self.auto_baseline:
            return

        if self.mode != "IDLE" or self.fast_state != "IDLE" or self.cooldown_active or self.startup_active:
            return

        if not row.get("recordable"):
            return

        if self.baseline_started_at is None:
            self.baseline_started_at = time.time()
            self.baseline_rows = []

        self.baseline_rows.append(row)
        elapsed = time.time() - self.baseline_started_at

        if elapsed >= AUTO_BASELINE_SEC:
            feature = make_full_feature(self.baseline_rows, "AIR", "AUTO", "0")

            if feature:
                save_dict_csv(BASELINE_CSV, feature)
                trim_csv(BASELINE_CSV, MAX_BASELINE_ROWS)
                save_dict_csv(SAMPLES_CSV, feature)

                rows = load_csv(SAMPLES_CSV)
                auto_air = [r for r in rows if r.get("label") == "AIR" and r.get("level") == "AUTO"]
                other = [r for r in rows if not (r.get("label") == "AIR" and r.get("level") == "AUTO")]

                if len(auto_air) > MAX_AUTO_AIR_SAMPLES:
                    auto_air = auto_air[-MAX_AUTO_AIR_SAMPLES:]

                write_csv(SAMPLES_CSV, other + auto_air)

                self.latest_air_gas = feature["gas_avg"]
                self.show_notice("AIR 기준 저장", "자동 AIR 기준이 저장되었습니다", None, self.green, "")
                self.root.after(2500, self.hide_notice)

            self.baseline_rows = []
            self.baseline_started_at = None

    def update_fast_air_reference(self, row):
        if not row.get("recordable"):
            return

        if self.fast_trigger_epoch is not None:
            return

        self.fast_air_rows.append(row)

        valid = clean_valid_rows(self.fast_air_rows)

        if len(valid) < 8:
            return

        duration = valid[-1]["epoch"] - valid[0]["epoch"]

        if duration < FAST_TRIGGER_MIN_AIR_SEC:
            return

        gas = [r["gas_ohm"] for r in valid[-20:]]
        hum = [r["hum_pct"] for r in valid[-20:]]

        self.fast_trigger_gas = mean(gas)
        self.fast_trigger_hum = mean(hum)

    def detect_fast_trigger(self, row):
        if not row.get("recordable"):
            return False

        if self.fast_trigger_epoch is not None:
            return False

        if self.fast_trigger_gas is None or self.fast_trigger_hum is None:
            return False

        now = row["epoch"]

        recent = [r for r in clean_valid_rows(self.fast_air_rows) if now - r["epoch"] <= FAST_TRIGGER_LOOKBACK_SEC]

        if len(recent) < 3:
            return False

        current_gas = row["gas_ohm"]
        current_hum = row["hum_pct"]

        gas_drop_pct = ((current_gas - self.fast_trigger_gas) / self.fast_trigger_gas) * 100 if self.fast_trigger_gas else 0
        hum_rise = current_hum - self.fast_trigger_hum

        first = recent[0]
        dt = max(0.001, row["epoch"] - first["epoch"])

        gas_slope_pct = ((current_gas - first["gas_ohm"]) / first["gas_ohm"]) * 100 / dt if first["gas_ohm"] else 0
        hum_slope = (current_hum - first["hum_pct"]) / dt

        gas_condition = gas_drop_pct <= FAST_TRIGGER_GAS_DROP_PCT or gas_slope_pct <= FAST_TRIGGER_GAS_SLOPE_PCT_PER_SEC
        hum_condition = hum_rise >= FAST_TRIGGER_HUM_RISE or hum_slope >= FAST_TRIGGER_HUM_SLOPE_PER_SEC

        if gas_condition or hum_condition:
            self.fast_trigger_epoch = row["epoch"]
            self.fast_trigger_gas = mean([r["gas_ohm"] for r in recent[:-1]]) if len(recent) > 2 else self.fast_trigger_gas
            self.fast_trigger_hum = mean([r["hum_pct"] for r in recent[:-1]]) if len(recent) > 2 else self.fast_trigger_hum
            self.fast_rows = [row]
            self.fast_reported.clear()

            self.show_notice(
                "시료 반응 감지",
                "자동 t=0 설정됨",
                None,
                self.orange,
                f"기준 gas {self.fast_trigger_gas:.0f}Ω / 기준 습도 {self.fast_trigger_hum:.1f}%",
            )
            self.set_status("빠른 trigger 감지")
            return True

        return False

    def handle_fast_mode(self, row):
        if self.mode != "FAST":
            return

        if row.get("recordable"):
            self.fast_rows.append(row)

        if self.fast_trigger_epoch is None:
            self.update_fast_air_reference(row)
            self.detect_fast_trigger(row)

            if self.fast_trigger_epoch is None:
                if self.fast_trigger_gas:
                    self.cards["Gas변화%"].configure(text="0.0%")
                    self.cards["습도변화"].configure(text="0.0%p")
                    self.cards["상태"].configure(text="AIR 기준 감시")
                    self.cards["판별결과"].configure(text="변화 대기")
                    self.set_status("빠른 모드 변화 감지 대기")
                else:
                    self.cards["상태"].configure(text="AIR 기준 수집")
                    self.cards["판별결과"].configure(text="대기")
                    self.set_status("빠른 모드 AIR 기준 수집")
            return

        elapsed = row["epoch"] - self.fast_trigger_epoch

        if self.fast_trigger_gas and row.get("gas_ohm", 0) > 0:
            gas_pct = ((row["gas_ohm"] - self.fast_trigger_gas) / self.fast_trigger_gas) * 100
        else:
            gas_pct = 0

        hum_delta = row["hum_pct"] - self.fast_trigger_hum if self.fast_trigger_hum is not None else 0

        self.cards["Gas변화%"].configure(text=f"{gas_pct:+.1f}%")
        self.cards["습도변화"].configure(text=f"{hum_delta:+.1f}%p")

        for sec in FAST_WINDOWS:
            if elapsed >= sec and sec not in self.fast_reported:
                self.fast_reported.add(sec)
                self.make_fast_result(sec)

        if elapsed >= FAST_SCAN_SEC:
            if self.fast_state == "FAST_LEARN":
                self.save_fast_learning()
            else:
                self.finish_fast_detect()

    def make_fast_result(self, sec):
        f = make_fast_feature(
            self.fast_rows,
            self.fast_label if self.fast_state == "FAST_LEARN" else "UNKNOWN",
            self.fast_level,
            self.fast_amount_ml,
            self.fast_trigger_epoch,
            self.fast_trigger_gas,
            self.fast_trigger_hum,
            sec,
        )

        if not f:
            return

        if self.fast_state == "FAST_DETECT":
            pct, winner = classify_fast(f, sec)

            if winner in ["학습 데이터 없음", "판별 불가"]:
                self.fast_last_result = winner
                self.cards["판별결과"].configure(text=winner)
                return

            best = pct[winner]
            limit = confidence_limit(sec)

            if sec <= 5:
                grade = "의심"
            elif sec <= 10:
                grade = "1차"
            elif best >= limit:
                grade = "확정"
            else:
                grade = "판별"

            self.fast_last_result = f"{sec}초 {grade}: {label_kr(winner)} {best:.1f}%"
            self.cards["판별결과"].configure(text=self.fast_last_result)
            self.cards["상태"].configure(text=f"{sec}초 {grade}")

            color = self.green if best >= limit else self.orange

            self.show_notice(
                f"{sec}초 {grade}",
                f"{label_kr(winner)} {best:.1f}%",
                None,
                color,
                f"IPA {pct['IPA']}% / ETH {pct['ETHANOL']}%",
            )
            self.root.after(1200, self.hide_notice)

        else:
            self.cards["판별결과"].configure(text=f"{sec}초 feature 수집")
            self.cards["상태"].configure(text="빠른 학습 중")

    def save_fast_learning(self):
        saved = 0

        for sec in FAST_WINDOWS:
            f = make_fast_feature(
                self.fast_rows,
                self.fast_label,
                self.fast_level,
                self.fast_amount_ml,
                self.fast_trigger_epoch,
                self.fast_trigger_gas,
                self.fast_trigger_hum,
                sec,
            )

            if f:
                save_dict_csv(FAST_SAMPLES_CSV, f)
                saved += 1

        self.show_notice(
            "빠른 학습 완료",
            f"{label_kr(self.fast_label)} {self.fast_level}",
            None,
            self.green,
            f"5/10/20/30초 feature {saved}개 저장",
        )

        self.mode = "IDLE"
        self.fast_state = "IDLE"
        self.auto_baseline = False
        self.root.after(1800, self.start_cooldown)

    def finish_fast_detect(self):
        self.show_notice(
            "빠른 판별 종료",
            self.fast_last_result,
            None,
            self.green,
            "시료 제거 후 쿨타임으로 전환합니다.",
        )

        self.mode = "IDLE"
        self.fast_state = "IDLE"
        self.auto_baseline = False
        self.root.after(1800, self.start_cooldown)

    def handle_full_sample(self, row):
        if self.mode != "FULL_LEARN":
            return

        now = time.time()

        if self.sample_phase == "READY":
            remain = max(0, int(self.phase_until - now))

            if remain <= 0:
                self.sample_phase = "EXPOSE"
                self.expose_until = now + self.duration_sec
                self.sample_rows = []

                self.show_notice(
                    f"{label_kr(self.pending_label)} 기록 중",
                    "지금 시료를 주입하세요" if self.pending_label != "AIR" else "정상공기 상태 유지",
                    self.duration_sec,
                    self.orange,
                    "정밀 학습 데이터 기록 중",
                )
                self.set_status("정밀 학습 기록 시작")
            else:
                self.show_notice(
                    f"{label_kr(self.pending_label)} 준비",
                    "아직 시료를 주입하지 마세요" if self.pending_label != "AIR" else "정상공기 상태 유지",
                    remain,
                    self.blue,
                    "",
                )
            return

        if self.sample_phase == "EXPOSE":
            if row.get("recordable"):
                self.sample_rows.append(row)

            remain = max(0, int(self.expose_until - now))

            if remain <= 0:
                f = make_full_feature(self.sample_rows, self.pending_label, self.pending_level, self.pending_amount_ml)

                if f:
                    save_dict_csv(SAMPLES_CSV, f)

                    self.show_notice(
                        "정밀 학습 완료",
                        f"{label_kr(self.pending_label)} 저장 완료",
                        None,
                        self.green,
                        "",
                    )
                else:
                    self.show_notice("학습 실패", "유효 데이터 부족", None, self.red, "")

                label = self.pending_label
                self.mode = "IDLE"
                self.sample_phase = None
                self.sample_rows = []

                if label in ["IPA", "ETHANOL"]:
                    self.root.after(1500, self.start_cooldown)
                else:
                    self.root.after(3000, self.hide_notice)
                    self.auto_baseline = True
            else:
                self.show_notice(
                    f"{label_kr(self.pending_label)} 기록 중",
                    "시료 반응 기록 중" if self.pending_label != "AIR" else "정상공기 기록 중",
                    remain,
                    self.orange,
                    f"히터 {self.cards['히터'].cget('text')}",
                )

    def loop(self):
        sensor_init()
        self.set_status("센서 시작")

        while self.running:
            try:
                self.check_startup()
                self.check_cooldown()

                h, v, elapsed, recordable = self.update_heater()
                measure_sleep = FAST_MEASURE_SLEEP if self.mode == "FAST" else NORMAL_MEASURE_SLEEP

                row = read_sensor(h, v, elapsed, recordable, measure_sleep)

                self.current_rows.append(row)
                save_dict_csv(RAW_CSV, row)

                self.raw_trim_counter += 1
                if self.raw_trim_counter >= 1000:
                    self.raw_trim_counter = 0
                    trim_csv(RAW_CSV, MAX_RAW_ROWS)

                self.cards["가스Ω"].configure(text=f"{row['gas_ohm']:,.0f}")
                self.cards["히터"].configure(text=f"{h} {'기록' if row.get('recordable') else '안정화'}")

                self.handle_auto_baseline(row)
                self.handle_full_sample(row)
                self.handle_fast_mode(row)

                if self.mode == "IDLE" and self.fast_state == "IDLE" and not self.startup_active and not self.cooldown_active:
                    self.cards["상태"].configure(text="대기")
                    self.set_status("대기")

            except Exception as e:
                self.show_notice("오류", str(e), None, self.red, "센서 연결 또는 코드 확인")
                self.set_status(f"오류: {e}")

            time.sleep(FAST_LOOP_SLEEP if self.mode == "FAST" else NORMAL_LOOP_SLEEP)

    def style_axis(self, ax):
        ax.set_facecolor(self.card)
        ax.tick_params(colors=self.muted)
        ax.xaxis.label.set_color(self.muted)
        ax.yaxis.label.set_color(self.muted)
        ax.title.set_color(self.text)
        ax.grid(True, color="#314452", alpha=0.45)

        for s in ax.spines.values():
            s.set_color("#314452")

    def update_ui(self):
        rows = list(self.current_rows)

        if rows:
            latest = rows[-1]

            if self.mode == "FAST" and self.fast_trigger_epoch:
                plot_rows = [r for r in rows if r["epoch"] >= self.fast_trigger_epoch - 5 and r["epoch"] <= self.fast_trigger_epoch + 35]
                xs = [r["epoch"] - self.fast_trigger_epoch for r in plot_rows]

                gas_pct = []
                hum_delta = []

                for r in plot_rows:
                    if self.fast_trigger_gas:
                        gas_pct.append(((r["gas_ohm"] - self.fast_trigger_gas) / self.fast_trigger_gas) * 100)
                    else:
                        gas_pct.append(0)

                    if self.fast_trigger_hum is not None:
                        hum_delta.append(r["hum_pct"] - self.fast_trigger_hum)
                    else:
                        hum_delta.append(0)

                self.ax1.clear()
                self.style_axis(self.ax1)
                self.ax1.plot(xs, gas_pct, color="#00E5FF", linewidth=2.2)
                self.ax1.axvline(0, color="#FF6B6B", linewidth=2)
                self.ax1.axvline(5, color="#FFD166", linestyle="--")
                self.ax1.axvline(10, color="#FFD166", linestyle="--")
                self.ax1.set_title("빠른 판별: Gas 변화율 %")
                self.ax1.set_ylabel("%")

                self.ax2.clear()
                self.style_axis(self.ax2)
                self.ax2.plot(xs, hum_delta, color="#4DFF88", linewidth=2.2)
                self.ax2.axvline(0, color="#FF6B6B", linewidth=2)
                self.ax2.axvline(5, color="#FFD166", linestyle="--")
                self.ax2.axvline(10, color="#FFD166", linestyle="--")
                self.ax2.set_title("빠른 판별: 습도 변화량 %p")
                self.ax2.set_ylabel("%p")

                self.ax3.clear()
                self.style_axis(self.ax3)
                self.ax3.plot(xs, [r["gas_ohm"] for r in plot_rows], color="#FFD166", linewidth=1.8, label="Gas Ω")
                self.ax3.axvline(0, color="#FF6B6B", linewidth=2)
                self.ax3.set_title("원본 Gas 저항")
                self.ax3.set_xlabel("trigger 기준 시간 초")
                self.ax3.legend(facecolor=self.card, edgecolor="#314452", labelcolor=self.text)

            else:
                plot_rows = rows[-300:]
                t0 = plot_rows[0]["epoch"]
                xs = [r["epoch"] - t0 for r in plot_rows]

                self.ax1.clear()
                self.style_axis(self.ax1)
                self.ax1.plot(xs, [r["gas_ohm"] for r in plot_rows], color="#00E5FF", linewidth=2)
                self.ax1.set_title("Gas 저항")
                self.ax1.set_ylabel("Ω")

                self.ax2.clear()
                self.style_axis(self.ax2)
                self.ax2.plot(xs, [r["hum_pct"] for r in plot_rows], color="#4DFF88", linewidth=2)
                self.ax2.set_title("습도")
                self.ax2.set_ylabel("%")

                self.ax3.clear()
                self.style_axis(self.ax3)
                self.ax3.plot(xs, [r["temp_c"] for r in plot_rows], color="#FF6B6B", linewidth=2, label="온도")
                self.ax3.plot(xs, [r["press_hpa"] / 10 for r in plot_rows], color="#FFD166", linewidth=2, label="기압/10")
                self.ax3.set_title("온도 / 기압")
                self.ax3.set_xlabel("시간 초")
                self.ax3.legend(facecolor=self.card, edgecolor="#314452", labelcolor=self.text)

            self.fig.tight_layout()
            self.canvas.draw_idle()

        if self.running:
            self.root.after(200, self.update_ui)

    def show_data_manager(self):
        win = tk.Toplevel(self.root)
        win.title("데이터 관리")
        win.geometry("1100x760")
        win.configure(bg=self.bg)

        tk.Label(win, text="데이터 관리", bg=self.bg, fg=self.text, font=("NanumGothic", 18, "bold")).pack(pady=10)

        nb = ttk.Notebook(win)
        nb.pack(fill="both", expand=True, padx=10, pady=10)

        for title, path in [
            ("정밀 학습", SAMPLES_CSV),
            ("빠른 학습", FAST_SAMPLES_CSV),
            ("AIR 기준", BASELINE_CSV),
        ]:
            tab = tk.Frame(nb, bg=self.bg)
            nb.add(tab, text=title)
            self.build_table(tab, path)

    def build_table(self, parent, path):
        rows = load_csv(path)

        frame = tk.Frame(parent, bg=self.bg)
        frame.pack(fill="both", expand=True, padx=8, pady=8)

        cols = ["id", "timestamp", "label", "level", "amount_ml", "window_sec", "count", "gas_drop_from_trigger_pct", "hum_rise_from_trigger", "gas_avg", "hum_avg"]

        tree = ttk.Treeview(frame, columns=cols, show="headings")
        tree.pack(side="left", fill="both", expand=True)

        sb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        sb.pack(side="right", fill="y")
        tree.configure(yscrollcommand=sb.set)

        for c in cols:
            tree.heading(c, text=c)
            tree.column(c, width=110, anchor="center")

        for i, r in enumerate(rows):
            vals = []

            for c in cols:
                v = r.get(c, "")

                try:
                    fv = float(v)

                    if "gas" in c or "hum" in c:
                        v = f"{fv:.2f}"
                    elif c in ["count", "window_sec"]:
                        v = f"{fv:.0f}"
                except Exception:
                    pass

                vals.append(v)

            tree.insert("", "end", iid=str(i), values=vals)

        btns = tk.Frame(parent, bg=self.bg)
        btns.pack(fill="x", padx=8, pady=8)

        def delete_selected():
            selected = tree.selection()

            if not selected:
                return

            if not messagebox.askyesno("삭제", "선택 데이터를 삭제할까요?"):
                return

            all_rows = load_csv(path)

            for idx in sorted([int(x) for x in selected], reverse=True):
                if 0 <= idx < len(all_rows):
                    del all_rows[idx]

            write_csv(path, all_rows)
            parent.destroy()

        def clear_all():
            if messagebox.askyesno("전체삭제", "전체 삭제할까요?"):
                write_csv(path, [])
                parent.destroy()

        tk.Button(btns, text="선택삭제", command=delete_selected, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)
        tk.Button(btns, text="전체삭제", command=clear_all, font=("NanumGothic", 11, "bold")).pack(side="left", padx=5)

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
