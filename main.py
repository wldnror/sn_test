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


# ============================================================
# 기본 설정
# ============================================================

DATA_DIR = "bme688_data"
os.makedirs(DATA_DIR, exist_ok=True)

SAMPLES_CSV = os.path.join(DATA_DIR, "samples.csv")
RAW_CSV = os.path.join(DATA_DIR, "raw_log.csv")

MAX_RAW_ROWS = 100000

# 실시간 빠른 판별용
REALTIME_DEFAULT_ON = True

# 센서 측정 대기시간
# 0.30부터 테스트 추천. 안정적이면 0.20까지 낮춰볼 수 있음.
MEASURE_SLEEP_SEC = 0.30
LOOP_SLEEP_SEC = 0.02

# 그래프/UI 갱신
UI_UPDATE_MS = 200

# AIR 기준 추적
AIR_TRACK_MAX_ROWS = 120
AIR_TRACK_MIN_ROWS = 20

# 변화 감지 조건
TRIGGER_GAS_DROP_PCT = -3.0      # AIR 기준 대비 gas -3% 이하
TRIGGER_HUM_RISE = 0.7           # AIR 기준 대비 습도 +0.7%p 이상
TRIGGER_CONFIRM_COUNT = 2        # 2회 연속 변화 감지 시 시작점 확정

# 반응 후 판별 시간
FAST_WINDOWS = [5, 10, 20, 30]

# 가스 반응이 끝난 뒤 다시 AIR 기준으로 돌아가기 위한 조건
RECOVER_WAIT_SEC = 60
RECOVER_GAS_TOL_PCT = 15.0
RECOVER_HUM_TOL = 3.0

# 빠른 모드에서는 히터 하나 고정
FAST_HEATER_NAME = "F1_LOW"
FAST_HEATER_VALUE = 0x45

# 학습용 기본 기록 시간
TRAIN_READY_SEC = 5
TRAIN_RECORD_SEC = 30

# 학습용 라벨
LABELS = ["IPA", "ETHANOL"]

# 폰트
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


# ============================================================
# SPI / BME688 레지스터
# ============================================================

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


# ============================================================
# BME688 저수준 함수
# ============================================================

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
    set_heater(FAST_HEATER_VALUE)


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
    set_heater(FAST_HEATER_VALUE)
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
        "season": get_season(now.month),
        "period": get_period(now.hour),
        "hour": now.hour,
        "heater": FAST_HEATER_NAME,
        "heater_value": FAST_HEATER_VALUE,
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


# ============================================================
# CSV / 계산 유틸
# ============================================================

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


def count_labels():
    rows = load_csv(SAMPLES_CSV)
    counts = {"IPA": 0, "ETHANOL": 0}
    for r in rows:
        label = r.get("label")
        if label in counts:
            counts[label] += 1
    return counts


def label_korean(label):
    if label == "ETHANOL":
        return "에탄올"
    if label == "IPA":
        return "IPA"
    if label == "AIR":
        return "정상공기"
    return label


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


def rows_until_sec(rows, sec):
    rows = clean_rows(rows)
    if not rows:
        return []
    start = rows[0]["epoch"]
    return [r for r in rows if r["epoch"] - start <= sec]


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


def extract_realtime_features(rows, sec, trigger_gas, trigger_hum, label="UNKNOWN"):
    sub = rows_until_sec(rows, sec)

    if len(sub) < 3:
        return None

    gas = [r["gas_ohm"] for r in sub]
    hum = [r["hum_pct"] for r in sub]
    temp = [r["temp_c"] for r in sub]
    press = [r["press_hpa"] for r in sub]

    start_epoch = sub[0]["epoch"]
    duration = max(0.001, sub[-1]["epoch"] - start_epoch)

    gas_start = trigger_gas if trigger_gas and trigger_gas > 0 else mean(gas[:3])
    hum_start = trigger_hum if trigger_hum is not None else mean(hum[:3])

    gas_end = mean(gas[-min(5, len(gas)):])
    gas_min = min(gas)
    gas_max = max(gas)

    hum_end = mean(hum[-min(5, len(hum)):])
    hum_min = min(hum)
    hum_max = max(hum)

    shape = calc_shape(sub)

    feature = {
        "id": datetime.now().strftime("%Y%m%d_%H%M%S"),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "label": label,
        "window_sec": sec,
        "count": len(sub),
        "duration_sec": duration,

        "gas_start_ref": gas_start,
        "gas_avg": mean(gas),
        "gas_min": gas_min,
        "gas_max": gas_max,
        "gas_end": gas_end,
        "gas_drop_pct": ((gas_min - gas_start) / gas_start) * 100.0 if gas_start else 0,
        "gas_end_vs_start_pct": ((gas_end - gas_start) / gas_start) * 100.0 if gas_start else 0,
        "gas_slope": (gas_end - gas_start) / duration if duration else 0,

        "hum_start_ref": hum_start,
        "hum_avg": mean(hum),
        "hum_min": hum_min,
        "hum_max": hum_max,
        "hum_end": hum_end,
        "hum_rise_abs": hum_max - hum_start,
        "hum_end_vs_start": hum_end - hum_start,
        "hum_rise_speed": (hum_max - hum_start) / duration if duration else 0,

        "temp_avg": mean(temp),
        "press_avg": mean(press),

        "gas_smooth_score": shape["smooth"],
        "gas_slope_avg": shape["slope_avg"],
        "gas_slope_stdev": shape["slope_stdev"],
        "gas_direction_changes": shape["direction_changes"],
    }

    # 기존 samples.csv의 fast10_xxx 형태와도 비교 가능하게 복사
    p = f"fast{sec}_"
    feature[p + "count"] = feature["count"]
    feature[p + "duration_sec"] = feature["duration_sec"]
    feature[p + "gas_avg"] = feature["gas_avg"]
    feature[p + "gas_min"] = feature["gas_min"]
    feature[p + "gas_max"] = feature["gas_max"]
    feature[p + "gas_end"] = feature["gas_end"]
    feature[p + "gas_change_pct"] = feature["gas_drop_pct"]
    feature[p + "gas_end_vs_start_pct"] = feature["gas_end_vs_start_pct"]
    feature[p + "gas_slope"] = feature["gas_slope"]
    feature[p + "hum_avg"] = feature["hum_avg"]
    feature[p + "hum_min"] = feature["hum_min"]
    feature[p + "hum_max"] = feature["hum_max"]
    feature[p + "hum_rise_abs"] = feature["hum_rise_abs"]
    feature[p + "hum_end_vs_start"] = feature["hum_end_vs_start"]
    feature[p + "hum_rise_speed"] = feature["hum_rise_speed"]
    feature[p + "gas_smooth_score"] = feature["gas_smooth_score"]
    feature[p + "gas_slope_stdev"] = feature["gas_slope_stdev"]
    feature[p + "gas_direction_changes"] = feature["gas_direction_changes"]

    return feature


# ============================================================
# 분류기
# ============================================================

META_KEYS = {
    "id", "timestamp", "label", "level", "amount_ml", "season", "period", "hour"
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


def feature_key_scale(key):
    # 습도는 IPA/ETH 분리에서 중요
    if "hum_rise_abs" in key:
        return 6.0
    if "hum_end_vs_start" in key:
        return 5.0
    if "hum_rise_speed" in key:
        return 7.0
    if key.endswith("hum_avg") or key.endswith("hum_max") or key.endswith("hum_end"):
        return 2.5

    # 가스 변화율 중요
    if "gas_drop_pct" in key:
        return 5.0
    if "gas_change_pct" in key:
        return 5.0
    if "gas_end_vs_start_pct" in key:
        return 5.0
    if "gas_slope" in key:
        return 0.006

    # gas 절대값은 환경 영향이 있어서 낮게
    if key.endswith("gas_avg") or key.endswith("gas_min") or key.endswith("gas_max") or key.endswith("gas_end"):
        return 0.0015

    # 패턴
    if "smooth" in key:
        return 180.0
    if "direction_changes" in key:
        return 0.15
    if "slope_stdev" in key:
        return 0.004

    # 환경값
    if "temp" in key:
        return 0.2
    if "press" in key:
        return 0.03

    if "count" in key or "duration" in key or "window_sec" in key:
        return 0.001

    return 0.001


def allowed_keys_for_sec(feature, sample, sec):
    prefix = f"fast{sec}_"

    preferred = [
        "gas_avg", "gas_min", "gas_max", "gas_end",
        "gas_drop_pct", "gas_end_vs_start_pct", "gas_slope",
        "hum_avg", "hum_max", "hum_end",
        "hum_rise_abs", "hum_end_vs_start", "hum_rise_speed",
        "temp_avg", "press_avg",
        "gas_smooth_score", "gas_slope_stdev", "gas_direction_changes",

        prefix + "gas_avg",
        prefix + "gas_min",
        prefix + "gas_max",
        prefix + "gas_end",
        prefix + "gas_change_pct",
        prefix + "gas_end_vs_start_pct",
        prefix + "gas_slope",
        prefix + "hum_avg",
        prefix + "hum_max",
        prefix + "hum_rise_abs",
        prefix + "hum_end_vs_start",
        prefix + "hum_rise_speed",
        prefix + "gas_smooth_score",
        prefix + "gas_slope_stdev",
        prefix + "gas_direction_changes",
    ]

    keys = []
    for k in preferred:
        if k in feature and k in sample:
            keys.append(k)

    # 위 키가 거의 없으면 공통 숫자키로 fallback
    if len(keys) < 4:
        keys = sorted(set(numeric_keys(feature)) & set(numeric_keys(sample)))

    return keys


def feature_distance(feature, sample, sec):
    keys = allowed_keys_for_sec(feature, sample, sec)

    total = 0.0
    used = 0

    for k in keys:
        av = to_float(feature.get(k, 0))
        bv = to_float(sample.get(k, 0))

        if av == 0 and bv == 0:
            continue

        total += abs(av - bv) * feature_key_scale(k)
        used += 1

    if used == 0:
        return 999999.0

    return total / max(1, used)


def classify_ipa_ethanol(feature, sec):
    samples = load_csv(SAMPLES_CSV)
    usable = [s for s in samples if s.get("label") in ["IPA", "ETHANOL"]]

    scores = {"IPA": 0.0, "ETHANOL": 0.0}

    if not usable:
        return {"IPA": 0.0, "ETHANOL": 0.0}, "학습 데이터 없음"

    for s in usable:
        label = s.get("label")
        d = feature_distance(feature, s, sec)

        # 거리 작을수록 가중치 큼
        weight = 1.0 / (1.0 + d)

        # 같은 window feature가 있는 샘플 우대
        if f"fast{sec}_gas_end_vs_start_pct" in s or f"fast{sec}_hum_rise_abs" in s:
            weight *= 1.25

        scores[label] += weight

    total = scores["IPA"] + scores["ETHANOL"]

    if total <= 0:
        return {"IPA": 0.0, "ETHANOL": 0.0}, "판별 불가"

    pct = {
        "IPA": round(scores["IPA"] / total * 100.0, 1),
        "ETHANOL": round(scores["ETHANOL"] / total * 100.0, 1),
    }

    winner = "IPA" if pct["IPA"] >= pct["ETHANOL"] else "ETHANOL"
    return pct, winner


# ============================================================
# UI 앱
# ============================================================

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("BME688 실시간 IPA / 에탄올 판별")
        self.root.attributes("-fullscreen", True)
        self.fullscreen = True

        self.running = True
        self.sensor_ready = False

        # 데이터
        self.current_rows = deque(maxlen=1200)
        self.air_track_rows = deque(maxlen=AIR_TRACK_MAX_ROWS)

        # 실시간 감지 상태
        self.realtime_detect = REALTIME_DEFAULT_ON
        self.live_state = "AIR_TRACK"
        self.trigger_count = 0
        self.trigger_time = None
        self.trigger_gas = None
        self.trigger_hum = None
        self.event_rows = []
        self.reported_windows = set()
        self.last_pct = {"IPA": 0.0, "ETHANOL": 0.0}
        self.last_winner = "-"
        self.recover_started_at = None

        # 학습 상태
        self.train_mode = None
        self.train_label = None
        self.train_rows = []
        self.train_ready_until = 0
        self.train_record_until = 0
        self.train_phase = None

        # 색상
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

    # ------------------------------------------------------------
    # UI 생성
    # ------------------------------------------------------------

    def build_ui(self):
        top = tk.Frame(self.root, bg=self.bg)
        top.pack(fill="x", padx=12, pady=8)

        title_box = tk.Frame(top, bg=self.bg)
        title_box.pack(side="left", fill="x", expand=True)

        self.status_main = tk.Label(
            title_box,
            text="상태: 시작 중",
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
            ("화면모드", self.toggle_fullscreen),
            ("데이터관리", self.show_data_manager),
            ("에탄올학습", lambda: self.start_train("ETHANOL")),
            ("IPA학습", lambda: self.start_train("IPA")),
            ("실시간ON/OFF", self.toggle_realtime),
            ("리셋", self.reset_live),
        ]

        for txt, cmd in buttons:
            tk.Button(
                btns,
                text=txt,
                command=cmd,
                font=("NanumGothic", 11, "bold"),
                bg="#263847",
                fg=self.text,
                activebackground="#365369",
                activeforeground="white",
                relief="flat",
                padx=12,
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
                font=("NanumGothic", 18, "bold"),
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

    # ------------------------------------------------------------
    # 상태 표시
    # ------------------------------------------------------------

    def update_status(self, msg=None):
        counts = count_labels()

        rt = "ON" if self.realtime_detect else "OFF"
        state = msg if msg else self.live_state

        self.status_main.config(text=f"상태: {state}")
        self.status_sub.config(
            text=f"실시간 {rt} | 학습 IPA {counts['IPA']}개 / ETH {counts['ETHANOL']}개 | "
                 f"트리거 gas {TRIGGER_GAS_DROP_PCT}% / hum +{TRIGGER_HUM_RISE}%p | "
                 f"측정대기 {MEASURE_SLEEP_SEC:.2f}s"
        )

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def toggle_realtime(self):
        self.realtime_detect = not self.realtime_detect
        self.reset_live()
        self.update_status("실시간 ON" if self.realtime_detect else "실시간 OFF")

    def reset_live(self):
        self.live_state = "AIR_TRACK"
        self.trigger_count = 0
        self.trigger_time = None
        self.trigger_gas = None
        self.trigger_hum = None
        self.event_rows = []
        self.reported_windows = set()
        self.last_pct = {"IPA": 0.0, "ETHANOL": 0.0}
        self.last_winner = "-"
        self.recover_started_at = None
        self.air_track_rows.clear()

        self.cards["현재상태"].config(text="AIR 기준 추적", fg=self.blue)
        self.cards["Gas변화"].config(text="-")
        self.cards["습도변화"].config(text="-")
        self.cards["IPA"].config(text="-")
        self.cards["에탄올"].config(text="-")

    # ------------------------------------------------------------
    # 학습
    # ------------------------------------------------------------

    def start_train(self, label):
        if self.train_mode:
            messagebox.showinfo("안내", "이미 학습 중입니다.")
            return

        self.realtime_detect = False
        self.train_mode = "TRAIN"
        self.train_label = label
        self.train_rows = []
        self.train_phase = "READY"
        self.train_ready_until = time.time() + TRAIN_READY_SEC
        self.train_record_until = 0

        self.update_status(f"{label_korean(label)} 학습 준비")
        self.cards["현재상태"].config(text=f"{label_korean(label)} 학습 준비", fg=self.yellow)

    def process_train(self, row):
        if not self.train_mode:
            return

        now = time.time()

        if self.train_phase == "READY":
            remain = int(self.train_ready_until - now)

            if remain > 0:
                self.cards["현재상태"].config(
                    text=f"{label_korean(self.train_label)} 학습 준비 {remain}초",
                    fg=self.yellow,
                )
                return

            self.train_phase = "RECORD"
            self.train_record_until = now + TRAIN_RECORD_SEC
            self.train_rows = []
            self.cards["현재상태"].config(
                text=f"{label_korean(self.train_label)} 주입 / 기록 시작",
                fg=self.orange,
            )
            return

        if self.train_phase == "RECORD":
            if row.get("recordable"):
                self.train_rows.append(row)

            remain = int(self.train_record_until - now)
            self.cards["현재상태"].config(
                text=f"{label_korean(self.train_label)} 학습중 {remain}초",
                fg=self.orange,
            )

            if remain <= 0:
                self.finish_train()

    def finish_train(self):
        if not self.train_rows:
            messagebox.showerror("학습 실패", "유효 데이터가 없습니다.")
            self.train_mode = None
            self.realtime_detect = True
            return

        # 학습 기록 시작값은 첫 데이터 기준
        clean = clean_rows(self.train_rows)
        if len(clean) < 5:
            messagebox.showerror("학습 실패", "유효 데이터가 너무 적습니다.")
            self.train_mode = None
            self.realtime_detect = True
            return

        trigger_gas = clean[0]["gas_ohm"]
        trigger_hum = clean[0]["hum_pct"]

        saved_count = 0

        for sec in FAST_WINDOWS:
            f = extract_realtime_features(clean, sec, trigger_gas, trigger_hum, label=self.train_label)
            if f:
                f["level"] = "REALTIME_FAST"
                f["amount_ml"] = "UNKNOWN"
                f["train_sec"] = sec
                save_dict_csv(SAMPLES_CSV, f)
                saved_count += 1

        self.train_mode = None
        self.train_label = None
        self.train_rows = []
        self.train_phase = None
        self.realtime_detect = True
        self.reset_live()

        messagebox.showinfo("학습 완료", f"{saved_count}개 빠른 판별 feature 저장 완료")
        self.update_status("학습 완료 / 실시간 재시작")

    # ------------------------------------------------------------
    # 실시간 감지 핵심
    # ------------------------------------------------------------

    def process_realtime(self, row):
        if not self.realtime_detect:
            return

        if self.train_mode:
            return

        if not row.get("recordable"):
            return

        # --------------------------------------------------------
        # 1) AIR 기준 추적 상태
        # --------------------------------------------------------
        if self.live_state == "AIR_TRACK":
            self.air_track_rows.append(row)

            if len(self.air_track_rows) < AIR_TRACK_MIN_ROWS:
                self.cards["현재상태"].config(text=f"AIR 기준 수집 {len(self.air_track_rows)}/{AIR_TRACK_MIN_ROWS}", fg=self.blue)
                self.cards["Gas변화"].config(text="-")
                self.cards["습도변화"].config(text="-")
                return

            air_gas = mean([r["gas_ohm"] for r in self.air_track_rows])
            air_hum = mean([r["hum_pct"] for r in self.air_track_rows])

            gas_drop_pct = ((row["gas_ohm"] - air_gas) / air_gas) * 100.0 if air_gas else 0.0
            hum_rise = row["hum_pct"] - air_hum

            self.cards["현재상태"].config(text="AIR 기준 추적중", fg=self.blue)
            self.cards["Gas변화"].config(text=f"{gas_drop_pct:+.1f}%")
            self.cards["습도변화"].config(text=f"{hum_rise:+.2f}%p")

            changed = gas_drop_pct <= TRIGGER_GAS_DROP_PCT or hum_rise >= TRIGGER_HUM_RISE

            if changed:
                self.trigger_count += 1
            else:
                self.trigger_count = 0

            if self.trigger_count >= TRIGGER_CONFIRM_COUNT:
                # 시작점 확정
                self.live_state = "GAS_DETECTED"
                self.trigger_time = row["epoch"]

                # 기준은 현재 row가 아니라 직전 AIR 평균
                self.trigger_gas = air_gas
                self.trigger_hum = air_hum

                self.event_rows = [row]
                self.reported_windows = set()
                self.last_pct = {"IPA": 0.0, "ETHANOL": 0.0}

                self.cards["현재상태"].config(text="가스 반응 감지됨 t=0", fg=self.orange)
                return

        # --------------------------------------------------------
        # 2) 가스 감지 후 판별 상태
        # --------------------------------------------------------
        elif self.live_state == "GAS_DETECTED":
            self.event_rows.append(row)

            elapsed = row["epoch"] - self.trigger_time

            gas_delta_pct = ((row["gas_ohm"] - self.trigger_gas) / self.trigger_gas) * 100.0 if self.trigger_gas else 0.0
            hum_delta = row["hum_pct"] - self.trigger_hum if self.trigger_hum is not None else 0.0

            self.cards["Gas변화"].config(text=f"{gas_delta_pct:+.1f}%")
            self.cards["습도변화"].config(text=f"{hum_delta:+.2f}%p")

            # 현재 가능한 가장 가까운 윈도우로 계속 판별
            active_sec = None
            for sec in FAST_WINDOWS:
                if elapsed >= sec:
                    active_sec = sec

            if active_sec:
                feature = extract_realtime_features(
                    self.event_rows,
                    active_sec,
                    self.trigger_gas,
                    self.trigger_hum,
                    label="REALTIME",
                )

                if feature:
                    pct, winner = classify_ipa_ethanol(feature, active_sec)
                    self.last_pct = pct
                    self.last_winner = winner

                    self.cards["IPA"].config(text=f"{pct['IPA']:.1f}%")
                    self.cards["에탄올"].config(text=f"{pct['ETHANOL']:.1f}%")

                    if winner == "IPA":
                        color = self.yellow
                    elif winner == "ETHANOL":
                        color = self.green
                    else:
                        color = self.text

                    self.cards["현재상태"].config(
                        text=f"{active_sec}초 {label_korean(winner)} {pct.get(winner, 0):.1f}%",
                        fg=color,
                    )

            else:
                self.cards["현재상태"].config(
                    text=f"가스 감지 / {elapsed:.1f}초",
                    fg=self.orange,
                )

            # 30초 이후에는 회복 대기 상태로 전환
            if elapsed >= max(FAST_WINDOWS):
                self.live_state = "RECOVER_WAIT"
                self.recover_started_at = time.time()
                self.cards["현재상태"].config(text="판별 완료 / 회복 대기", fg=self.yellow)

        # --------------------------------------------------------
        # 3) 회복 대기
        # --------------------------------------------------------
        elif self.live_state == "RECOVER_WAIT":
            if self.trigger_gas is None or self.trigger_hum is None:
                self.reset_live()
                return

            gas_diff = abs(row["gas_ohm"] - self.trigger_gas) / self.trigger_gas * 100.0
            hum_diff = abs(row["hum_pct"] - self.trigger_hum)

            self.cards["Gas변화"].config(text=f"{((row['gas_ohm'] - self.trigger_gas) / self.trigger_gas) * 100.0:+.1f}%")
            self.cards["습도변화"].config(text=f"{row['hum_pct'] - self.trigger_hum:+.2f}%p")

            elapsed_recover = time.time() - self.recover_started_at if self.recover_started_at else 0

            recovered = (
                elapsed_recover >= RECOVER_WAIT_SEC
                and gas_diff <= RECOVER_GAS_TOL_PCT
                and hum_diff <= RECOVER_HUM_TOL
            )

            if recovered:
                self.reset_live()
                self.cards["현재상태"].config(text="회복 완료 / AIR 기준 재추적", fg=self.green)
            else:
                self.cards["현재상태"].config(
                    text=f"회복 대기 {int(elapsed_recover)}초",
                    fg=self.yellow,
                )

    # ------------------------------------------------------------
    # 데이터 관리
    # ------------------------------------------------------------

    def show_data_manager(self):
        win = tk.Toplevel(self.root)
        win.title("데이터 관리")
        win.geometry("1200x720")
        win.configure(bg=self.bg)

        tk.Label(
            win,
            text=f"학습 데이터: {os.path.abspath(SAMPLES_CSV)}",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 13, "bold"),
        ).pack(pady=8)

        columns = (
            "id", "timestamp", "label", "window_sec", "count",
            "gas_drop_pct", "gas_end_vs_start_pct",
            "hum_rise_abs", "hum_end_vs_start",
            "gas_avg", "gas_min", "hum_avg", "hum_max"
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
            tree.column(c, width=110, anchor="center")

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
                values = [fmt(r.get(c, "")) for c in columns]
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

    # ------------------------------------------------------------
    # 센서 루프
    # ------------------------------------------------------------

    def loop(self):
        try:
            sensor_init()
            self.sensor_ready = True
            self.update_status("센서 시작 완료")
        except Exception as e:
            self.update_status(f"센서 오류: {e}")
            return

        raw_trim_counter = 0

        while self.running:
            try:
                row = read_sensor()
                self.current_rows.append(row)

                save_dict_csv(RAW_CSV, row)
                raw_trim_counter += 1

                if raw_trim_counter >= 1000:
                    raw_trim_counter = 0
                    trim_csv(RAW_CSV, MAX_RAW_ROWS)

                if self.train_mode:
                    self.process_train(row)
                else:
                    self.process_realtime(row)

                self.update_status()

            except Exception as e:
                self.status_main.config(text=f"오류: {e}")
                self.cards["현재상태"].config(text="오류", fg=self.red)

            time.sleep(LOOP_SLEEP_SEC)

    # ------------------------------------------------------------
    # 그래프/UI 갱신
    # ------------------------------------------------------------

    def update_ui(self):
        rows = list(self.current_rows)

        if rows:
            latest = rows[-1]

            self.cards["가스저항"].config(text=f"{latest['gas_ohm']:,.0f} Ω")

            self.ax_gas.clear()
            self.ax_hum.clear()
            self.style_axis(self.ax_gas)
            self.style_axis(self.ax_hum)

            if self.live_state in ["GAS_DETECTED", "RECOVER_WAIT"] and self.trigger_time and self.trigger_gas:
                plot_rows = [r for r in rows if r["epoch"] >= self.trigger_time - 1.0]

                xs = [r["epoch"] - self.trigger_time for r in plot_rows]
                gas_pct = [((r["gas_ohm"] - self.trigger_gas) / self.trigger_gas) * 100.0 for r in plot_rows]
                hum_delta = [r["hum_pct"] - self.trigger_hum for r in plot_rows]

                self.ax_gas.plot(xs, gas_pct, linewidth=2.5)
                self.ax_gas.axvline(0, linestyle="--", linewidth=1.5)
                self.ax_gas.axhline(0, linestyle="--", linewidth=1.0)
                self.ax_gas.set_title("Trigger 기준 Gas 변화율")
                self.ax_gas.set_xlabel("t=0 이후 시간 (초)")
                self.ax_gas.set_ylabel("Gas 변화율 (%)")

                self.ax_hum.plot(xs, hum_delta, linewidth=2.5)
                self.ax_hum.axvline(0, linestyle="--", linewidth=1.5)
                self.ax_hum.axhline(0, linestyle="--", linewidth=1.0)
                self.ax_hum.set_title("Trigger 기준 습도 변화량")
                self.ax_hum.set_xlabel("t=0 이후 시간 (초)")
                self.ax_hum.set_ylabel("습도 변화량 (%p)")

            else:
                # AIR 추적 중에는 최근 60초 원본 표시
                now = time.time()
                plot_rows = [r for r in rows if now - r["epoch"] <= 60]

                if plot_rows:
                    t0 = plot_rows[0]["epoch"]
                    xs = [r["epoch"] - t0 for r in plot_rows]
                    gas = [r["gas_ohm"] for r in plot_rows]
                    hum = [r["hum_pct"] for r in plot_rows]

                    self.ax_gas.plot(xs, gas, linewidth=2.0)
                    self.ax_gas.set_title("최근 60초 Gas 원본")
                    self.ax_gas.set_xlabel("시간 (초)")
                    self.ax_gas.set_ylabel("Gas 저항 (Ω)")

                    self.ax_hum.plot(xs, hum, linewidth=2.0)
                    self.ax_hum.set_title("최근 60초 습도")
                    self.ax_hum.set_xlabel("시간 (초)")
                    self.ax_hum.set_ylabel("습도 (%)")

            self.fig.suptitle(
                "BME688 실시간 IPA / 에탄올 판별 - 변화 감지 자동 t=0",
                color=self.text,
                fontsize=15,
                fontweight="bold",
            )
            self.fig.tight_layout()
            self.canvas.draw_idle()

        if self.running:
            self.root.after(UI_UPDATE_MS, self.update_ui)

    # ------------------------------------------------------------
    # 종료
    # ------------------------------------------------------------

    def close(self):
        self.running = False
        try:
            spi.close()
        except Exception:
            pass
        self.root.destroy()


# ============================================================
# 실행
# ============================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
