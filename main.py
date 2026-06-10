import os
import csv
import math
import time
import threading
from datetime import datetime
from collections import deque

import spidev
import tkinter as tk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# =========================
# FONT
# =========================
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


# =========================
# FILES
# =========================
DATA_DIR = "bme688_data"
os.makedirs(DATA_DIR, exist_ok=True)

BASELINE_CSV = os.path.join(DATA_DIR, "baseline_auto.csv")
SAMPLES_CSV = os.path.join(DATA_DIR, "samples.csv")
RAW_CSV = os.path.join(DATA_DIR, "raw_log.csv")


# =========================
# SPI / BME688
# =========================
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


# =========================
# DATA / CLASSIFIER
# =========================
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


# =========================
# UI
# =========================
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("BME688 학습/판별 대시보드")
        self.root.attributes("-fullscreen", True)
        self.fullscreen = True

        self.running = True
        self.auto_baseline = True
        self.current_rows = deque(maxlen=300)
        self.baseline_buffer = []
        self.latest_air_gas = None
        self.baseline_save_count = len(load_csv(BASELINE_CSV))

        self.sample_mode = None
        self.pending_label = None
        self.sample_until = 0
        self.sample_rows = []

        self.bg = "#101820"
        self.card = "#182632"
        self.text = "#EAF2F8"
        self.muted = "#94A9B8"

        self.root.configure(bg=self.bg)

        self.build_ui()

        self.sensor_thread = threading.Thread(target=self.loop, daemon=True)
        self.sensor_thread.start()

        self.update_ui()

    def build_ui(self):
        top = tk.Frame(self.root, bg=self.bg)
        top.pack(fill="x", padx=12, pady=8)

        self.status_label = tk.Label(
            top,
            text="상태: 자동 기준 기록 준비중",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 16, "bold"),
        )
        self.status_label.pack(side="left", padx=8)

        buttons = [
            ("전체화면/창모드", self.toggle_fullscreen),
            ("자동 기준 ON/OFF", self.toggle_auto),
            ("정상공기 학습", lambda: self.start_sample("AIR")),
            ("에탄올 학습", lambda: self.start_sample("ETHANOL")),
            ("IPA 학습", lambda: self.start_sample("IPA")),
            ("미지시료 판별", self.start_unknown),
            ("학습 데이터 보기", self.show_training_data),
            ("종료", self.close),
        ]

        for txt, cmd in buttons:
            tk.Button(
                top,
                text=txt,
                command=cmd,
                font=("NanumGothic", 11, "bold"),
                bg="#263847",
                fg=self.text,
                activebackground="#365369",
                activeforeground="white",
                relief="flat",
                padx=10,
                pady=7,
            ).pack(side="right", padx=3)

        info = tk.Frame(self.root, bg=self.bg)
        info.pack(fill="x", padx=12)

        self.cards = {}

        for name in ["가스저항", "온도", "습도", "기압", "판별결과"]:
            f = tk.Frame(info, bg=self.card, padx=14, pady=10)
            f.pack(side="left", fill="x", expand=True, padx=5, pady=5)

            title = tk.Label(f, text=name, bg=self.card, fg=self.muted, font=("NanumGothic", 12))
            title.pack(anchor="w")

            value = tk.Label(f, text="-", bg=self.card, fg=self.text, font=("NanumGothic", 21, "bold"))
            value.pack(anchor="w")

            self.cards[name] = value

        self.fig = plt.Figure(figsize=(14, 7), facecolor=self.bg)
        self.ax_gas = self.fig.add_subplot(2, 1, 1)
        self.ax_env = self.fig.add_subplot(2, 1, 2)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=12, pady=8)

    def update_status(self, text=None):
        counts = count_labels()
        base_state = "자동기준 기록중" if self.auto_baseline else "자동기준 OFF"
        buffer_state = f"버퍼 {len(self.baseline_buffer)}/60"
        save_state = f"기준저장 {self.baseline_save_count}회"
        learn_state = f"AIR {counts['AIR']} / ETH {counts['ETHANOL']} / IPA {counts['IPA']}"

        if text:
            msg = f"상태: {text} | {base_state} | {buffer_state} | {save_state} | {learn_state}"
        else:
            msg = f"상태: {base_state} | {buffer_state} | {save_state} | {learn_state}"

        self.status_label.config(text=msg)

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)

    def toggle_auto(self):
        self.auto_baseline = not self.auto_baseline
        self.update_status()

    def start_sample(self, label):
        self.sample_mode = "LEARN"
        self.pending_label = label
        self.sample_rows = []
        self.sample_until = time.time() + 60
        self.update_status(f"{label} 학습중")

    def start_unknown(self):
        self.sample_mode = "UNKNOWN"
        self.pending_label = "UNKNOWN"
        self.sample_rows = []
        self.sample_until = time.time() + 60
        self.update_status("미지시료 판별중")

    def show_training_data(self):
        win = tk.Toplevel(self.root)
        win.title("학습 데이터 보기")
        win.geometry("1150x680")
        win.configure(bg=self.bg)

        title = tk.Label(
            win,
            text="학습 데이터 요약",
            bg=self.bg,
            fg=self.text,
            font=("NanumGothic", 18, "bold"),
        )
        title.pack(pady=10)

        text = tk.Text(
            win,
            bg="#0B1218",
            fg=self.text,
            insertbackground=self.text,
            font=("NanumGothic", 11),
            wrap="none",
        )
        text.pack(fill="both", expand=True, padx=10, pady=10)

        rows = load_csv(SAMPLES_CSV)
        baselines = load_csv(BASELINE_CSV)

        counts = count_labels()

        text.insert("end", f"저장 폴더: {os.path.abspath(DATA_DIR)}\n\n")
        text.insert("end", f"자동 기준 기록 수: {len(baselines)}개\n")
        text.insert("end", f"AIR 학습 수: {counts['AIR']}개\n")
        text.insert("end", f"ETHANOL 학습 수: {counts['ETHANOL']}개\n")
        text.insert("end", f"IPA 학습 수: {counts['IPA']}개\n\n")

        if not rows:
            text.insert("end", "아직 학습 데이터가 없습니다.\n")
            return

        text.insert("end", "최근 학습 데이터 40개\n")
        text.insert("end", "-" * 140 + "\n")

        for r in rows[-40:]:
            line = (
                f"{r.get('timestamp','')} | "
                f"{r.get('label',''):8s} | "
                f"{r.get('season','')} | "
                f"{r.get('period','')} | "
                f"gas_avg={float(r.get('gas_avg',0)):,.0f}Ω | "
                f"min={float(r.get('gas_min',0)):,.0f}Ω | "
                f"change={float(r.get('gas_change_pct',0)):.1f}% | "
                f"T={float(r.get('temp_avg',0)):.1f}℃ | "
                f"H={float(r.get('hum_avg',0)):.1f}% | "
                f"P={float(r.get('press_avg',0)):.1f}hPa\n"
            )
            text.insert("end", line)

    def close(self):
        self.running = False
        try:
            spi.close()
        except Exception:
            pass
        self.root.destroy()

    def loop(self):
        sensor_init()
        self.update_status("센서 시작 완료")

        while self.running:
            try:
                row = read_sensor()
                self.current_rows.append(row)
                save_dict_csv(RAW_CSV, row)

                if self.auto_baseline and row["gas_valid"] and row["heat_stable"]:
                    self.baseline_buffer.append(row)

                    if len(self.baseline_buffer) >= 60:
                        feature = extract_features(self.baseline_buffer, "AUTO_BASELINE")

                        if feature:
                            save_dict_csv(BASELINE_CSV, feature)
                            self.latest_air_gas = feature["gas_avg"]
                            self.baseline_save_count += 1

                        self.baseline_buffer = []

                if self.sample_mode:
                    self.sample_rows.append(row)
                    remain = int(self.sample_until - time.time())

                    if remain <= 0:
                        self.finish_sample()
                    else:
                        self.update_status(f"{self.pending_label} 측정중 {remain}초")

                else:
                    self.update_status()

            except Exception as e:
                self.update_status(f"오류: {e}")

            time.sleep(0.4)

    def finish_sample(self):
        feature = extract_features(self.sample_rows, self.pending_label, self.latest_air_gas)

        if not feature:
            self.update_status("샘플 실패")
            self.sample_mode = None
            return

        if self.sample_mode == "LEARN":
            save_dict_csv(SAMPLES_CSV, feature)
            self.update_status(f"{self.pending_label} 학습 저장 완료")

        else:
            pct, winner = classify(feature)
            self.cards["판별결과"].config(
                text=f"{winner} | AIR {pct['AIR']}% / ETH {pct['ETHANOL']}% / IPA {pct['IPA']}%"
            )
            self.update_status("판별 완료")

        self.sample_mode = None
        self.pending_label = None
        self.sample_rows = []

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
                "BME688 자동 기준 학습 / 에탄올 / IPA 판별 시스템",
                color=self.text,
                fontsize=16,
                fontweight="bold",
            )

            self.fig.tight_layout()
            self.canvas.draw_idle()

        if self.running:
            self.root.after(1000, self.update_ui)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
