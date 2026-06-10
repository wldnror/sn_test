import os
import time
from collections import deque

import spidev
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


os.environ.setdefault("QT_LOGGING_RULES", "*.debug=false;qt.qpa.*=false")

# Korean font
matplotlib.rcParams["font.family"] = "NanumGothic"
matplotlib.rcParams["axes.unicode_minus"] = False


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


BG = "#101820"
CARD = "#182632"
GRID = "#314452"
TEXT = "#EAF2F8"
MUTED = "#94A9B8"

C_GAS = "#00E5FF"
C_TEMP = "#FF6B6B"
C_HUM = "#4DFF88"
C_PRESS = "#FFD166"


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


cal = {}
t_fine = 0.0

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


def read_calibration():
    global cal

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
    var2 = var2 + (var1 * cal["par_p5"] * 2.0)
    var2 = (var2 / 4.0) + (cal["par_p4"] * 65536.0)

    var1 = ((cal["par_p3"] * var1 * var1 / 16384.0) + (cal["par_p2"] * var1)) / 524288.0
    var1 = (1.0 + (var1 / 32768.0)) * cal["par_p1"]

    if var1 == 0:
        return 0

    pressure = 1048576.0 - press_adc
    pressure = ((pressure - (var2 / 4096.0)) * 6250.0) / var1

    var1 = cal["par_p9"] * pressure * pressure / 2147483648.0
    var2 = pressure * cal["par_p8"] / 32768.0
    var3 = (pressure / 256.0) ** 3 * (cal["par_p10"] / 131072.0)

    pressure = pressure + (var1 + var2 + var3 + (cal["par_p7"] * 128.0)) / 16.0
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
        return 0

    var1 = ((1340 + (5 * cal["range_sw_err"])) * GAS_LOOKUP_1[gas_range]) / 65536.0
    var2 = ((gas_adc * 32768.0) - 16777216.0) + var1
    var3 = (GAS_LOOKUP_2[gas_range] * var1) / 512.0

    if var2 == 0:
        return 0

    return var3 / var2


def sensor_reset():
    write_reg(REG_RESET, 0xB6)
    time.sleep(0.2)


def sensor_init():
    chip_id = read_reg(REG_CHIP_ID)
    print("Chip ID:", hex(chip_id))

    if chip_id != 0x61:
        raise RuntimeError("BME688/BME680 chip ID error")

    sensor_reset()
    print("Chip ID after reset:", hex(read_reg(REG_CHIP_ID)))

    read_calibration()

    write_reg(REG_CONFIG, 0x08)
    write_reg(REG_CTRL_HUM, 0x01)

    write_reg(REG_CTRL_GAS_0, 0x00)
    write_reg(REG_RES_HEAT_0, 0x73)
    write_reg(REG_GAS_WAIT_0, 0x59)
    write_reg(REG_CTRL_GAS_1, 0x20)

    print("Sensor init done")


def trigger_forced_measurement():
    ctrl_meas = (0b010 << 5) | (0b101 << 2) | 0b01
    write_reg(REG_CTRL_MEAS, ctrl_meas)


def read_raw_data():
    data = read_regs(REG_FIELD0, 17)

    status = data[0]
    new_data = bool(status & 0x80)

    pressure_adc = (data[2] << 12) | (data[3] << 4) | (data[4] >> 4)
    temp_adc = (data[5] << 12) | (data[6] << 4) | (data[7] >> 4)
    hum_adc = (data[8] << 8) | data[9]

    gas_msb = data[15]
    gas_lsb = data[16]

    gas_adc = (gas_msb << 2) | (gas_lsb >> 6)
    gas_range = gas_lsb & 0x0F
    gas_valid = bool(gas_lsb & 0x20)
    heat_stable = bool(gas_lsb & 0x10)

    return new_data, temp_adc, pressure_adc, hum_adc, gas_adc, gas_range, gas_valid, heat_stable


def style_axis(ax):
    ax.set_facecolor(CARD)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.xaxis.label.set_color(MUTED)
    ax.yaxis.label.set_color(MUTED)
    ax.title.set_color(TEXT)
    ax.grid(True, color=GRID, alpha=0.45)
    for spine in ax.spines.values():
        spine.set_color(GRID)


def draw_card(ax, title, value, unit, color, sub=""):
    ax.clear()
    ax.set_facecolor(CARD)
    ax.set_xticks([])
    ax.set_yticks([])

    for spine in ax.spines.values():
        spine.set_color(GRID)

    ax.text(0.05, 0.78, title, color=MUTED, fontsize=11, transform=ax.transAxes)
    ax.text(0.05, 0.35, value, color=color, fontsize=23, fontweight="bold", transform=ax.transAxes)
    ax.text(0.05, 0.12, unit, color=MUTED, fontsize=10, transform=ax.transAxes)

    if sub:
        ax.text(0.95, 0.12, sub, color=MUTED, fontsize=9, ha="right", transform=ax.transAxes)


sensor_init()

times = deque(maxlen=300)
gas_values = deque(maxlen=300)
temp_values = deque(maxlen=300)
hum_values = deque(maxlen=300)
press_values = deque(maxlen=300)

plt.ion()
fig = plt.figure(figsize=(15, 8.5), facecolor=BG)
fig.canvas.manager.set_window_title("BME688 실시간 대시보드")

gs = gridspec.GridSpec(
    3,
    4,
    figure=fig,
    height_ratios=[1.05, 2.3, 1.7],
    hspace=0.55,
    wspace=0.35,
)

ax_card_gas = fig.add_subplot(gs[0, 0])
ax_card_temp = fig.add_subplot(gs[0, 1])
ax_card_hum = fig.add_subplot(gs[0, 2])
ax_card_press = fig.add_subplot(gs[0, 3])

ax_gas = fig.add_subplot(gs[1, :])
ax_temp = fig.add_subplot(gs[2, 0])
ax_hum = fig.add_subplot(gs[2, 1])
ax_press = fig.add_subplot(gs[2, 2:4])

start_time = time.time()

print("BME688 live dashboard start")
print("Wait 1-3 minutes, then test with alcohol/perfume/sanitizer.")
print("Press Ctrl+C to stop.")

try:
    while True:
        trigger_forced_measurement()
        time.sleep(0.8)

        new_data, temp_adc, pressure_adc, hum_adc, gas_adc, gas_range, gas_valid, heat_stable = read_raw_data()

        temp_c = compensate_temp(temp_adc)
        press_hpa = compensate_pressure(pressure_adc)
        hum_pct = compensate_humidity(hum_adc, temp_c)
        gas_ohm = calc_gas_resistance(gas_adc, gas_range)

        now = time.time() - start_time

        times.append(now)
        gas_values.append(gas_ohm)
        temp_values.append(temp_c)
        hum_values.append(hum_pct)
        press_values.append(press_hpa)

        status = "정상" if gas_valid and heat_stable else "예열중"

        print(
            f"{now:7.1f}s | "
            f"T={temp_c:6.2f} C | "
            f"H={hum_pct:6.2f} % | "
            f"P={press_hpa:8.2f} hPa | "
            f"Gas={gas_ohm:10.0f} ohm | "
            f"ADC={gas_adc:4d} | RANGE={gas_range:2d} | {status}"
        )

        draw_card(ax_card_gas, "가스 저항", f"{gas_ohm:,.0f}", "Ω", C_GAS, status)
        draw_card(ax_card_temp, "온도", f"{temp_c:.2f}", "°C", C_TEMP)
        draw_card(ax_card_hum, "습도", f"{hum_pct:.1f}", "% RH", C_HUM)
        draw_card(ax_card_press, "기압", f"{press_hpa:.2f}", "hPa", C_PRESS)

        ax_gas.clear()
        style_axis(ax_gas)
        ax_gas.plot(times, gas_values, color=C_GAS, linewidth=2.6)
        ax_gas.fill_between(times, gas_values, color=C_GAS, alpha=0.14)
        ax_gas.set_title("가스 반응 그래프")
        ax_gas.set_xlabel("시간 (초)")
        ax_gas.set_ylabel("가스 저항 (Ω)")

        ax_temp.clear()
        style_axis(ax_temp)
        ax_temp.plot(times, temp_values, color=C_TEMP, linewidth=2.2)
        ax_temp.set_title("온도")
        ax_temp.set_xlabel("시간 (초)")
        ax_temp.set_ylabel("°C")

        ax_hum.clear()
        style_axis(ax_hum)
        ax_hum.plot(times, hum_values, color=C_HUM, linewidth=2.2)
        ax_hum.set_title("습도")
        ax_hum.set_xlabel("시간 (초)")
        ax_hum.set_ylabel("% RH")

        ax_press.clear()
        style_axis(ax_press)
        ax_press.plot(times, press_values, color=C_PRESS, linewidth=2.2)
        ax_press.set_title("기압")
        ax_press.set_xlabel("시간 (초)")
        ax_press.set_ylabel("hPa")

        fig.suptitle("BME688 센서 실시간 모니터", color=TEXT, fontsize=18, fontweight="bold")
        plt.pause(0.01)

        time.sleep(0.7)

except KeyboardInterrupt:
    print("")
    print("Test stopped.")
    spi.close()
