import time
from collections import deque

import spidev
import matplotlib.pyplot as plt


# =========================
# SPI SETUP
# =========================
spi = spidev.SpiDev()
spi.open(0, 0)          # SPI0 CE0
spi.max_speed_hz = 50000
spi.mode = 0


# =========================
# REGISTERS
# =========================
REG_CHIP_ID = 0xD0
REG_RESET = 0xE0

REG_CTRL_GAS_1 = 0x71
REG_CTRL_HUM = 0x72
REG_CTRL_MEAS = 0x74
REG_CONFIG = 0x75

REG_RES_HEAT_0 = 0x5A
REG_GAS_WAIT_0 = 0x64

REG_FIELD0 = 0x1D


# =========================
# SPI FUNCTIONS
# =========================
def read_reg(reg):
    resp = spi.xfer2([reg | 0x80, 0x00])
    return resp[1]


def read_regs(reg, length):
    resp = spi.xfer2([reg | 0x80] + [0x00] * length)
    return resp[1:]


def write_reg(reg, value):
    spi.xfer2([reg & 0x7F, value & 0xFF])


# =========================
# SENSOR INIT
# =========================
def sensor_reset():
    write_reg(REG_RESET, 0xB6)
    time.sleep(0.2)


def sensor_init():
    chip_id = read_reg(REG_CHIP_ID)
    print("Chip ID:", hex(chip_id))

    if chip_id != 0x61:
        raise RuntimeError("BME688/BME680 chip ID error")

    sensor_reset()

    chip_id = read_reg(REG_CHIP_ID)
    print("Chip ID after reset:", hex(chip_id))

    # IIR filter coefficient
    write_reg(REG_CONFIG, 0x08)

    # Humidity oversampling x1
    write_reg(REG_CTRL_HUM, 0x01)

    # Gas heater resistance test value
    write_reg(REG_RES_HEAT_0, 0x73)

    # Heater duration about 100 ms
    write_reg(REG_GAS_WAIT_0, 0x59)

    # IMPORTANT:
    # run_gas = bit 5 = 0x20
    # heater profile index = 0
    write_reg(REG_CTRL_GAS_1, 0x20)

    print("Sensor init done")


# =========================
# MEASUREMENT
# =========================
def trigger_forced_measurement():
    # osrs_t x2 = 010
    # osrs_p x16 = 101
    # forced mode = 01
    ctrl_meas = (0b010 << 5) | (0b101 << 2) | 0b01
    write_reg(REG_CTRL_MEAS, ctrl_meas)


def read_raw_data():
    data = read_regs(REG_FIELD0, 17)

    status = data[0]
    new_data = bool(status & 0x80)

    press_msb = data[0x1F - REG_FIELD0]
    press_lsb = data[0x20 - REG_FIELD0]
    press_xlsb = data[0x21 - REG_FIELD0]

    temp_msb = data[0x22 - REG_FIELD0]
    temp_lsb = data[0x23 - REG_FIELD0]
    temp_xlsb = data[0x24 - REG_FIELD0]

    hum_msb = data[0x25 - REG_FIELD0]
    hum_lsb = data[0x26 - REG_FIELD0]

    pressure_adc = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4)
    temp_adc = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4)
    hum_adc = (hum_msb << 8) | hum_lsb

    gas_msb = data[0x2A - REG_FIELD0]
    gas_lsb = data[0x2B - REG_FIELD0]

    gas_adc = (gas_msb << 2) | (gas_lsb >> 6)
    gas_range = gas_lsb & 0x0F

    gas_valid = bool(gas_lsb & 0x20)
    heat_stable = bool(gas_lsb & 0x10)

    return {
        "new_data": new_data,
        "pressure_adc": pressure_adc,
        "temp_adc": temp_adc,
        "hum_adc": hum_adc,
        "gas_adc": gas_adc,
        "gas_range": gas_range,
        "gas_valid": gas_valid,
        "heat_stable": heat_stable,
    }


# =========================
# MAIN
# =========================
sensor_init()

times = deque(maxlen=300)
gas_values = deque(maxlen=300)

plt.ion()
fig, ax = plt.subplots()

start_time = time.time()

print("BME688 raw gas test start")
print("Wait 1-3 minutes, then test with alcohol/perfume/sanitizer.")
print("Press Ctrl+C to stop.")

try:
    while True:
        trigger_forced_measurement()

        time.sleep(0.35)

        d = read_raw_data()
        now = time.time() - start_time

        gas_value = d["gas_adc"]

        print(
            f"{now:7.1f}s | "
            f"NEW={d['new_data']} | "
            f"T_ADC={d['temp_adc']:7d} | "
            f"P_ADC={d['pressure_adc']:7d} | "
            f"H_ADC={d['hum_adc']:5d} | "
            f"GAS_ADC={d['gas_adc']:5d} | "
            f"RANGE={d['gas_range']:2d} | "
            f"VALID={d['gas_valid']} | "
            f"HEAT={d['heat_stable']}"
        )

        times.append(now)
        gas_values.append(gas_value)

        ax.clear()
        ax.plot(times, gas_values, label="Gas ADC raw")
        ax.set_xlabel("Time sec")
        ax.set_ylabel("Gas ADC raw")
        ax.set_title("BME688 Gas Response Raw")
        ax.grid(True)
        ax.legend()

        plt.pause(0.01)

        time.sleep(1)

except KeyboardInterrupt:
    print("")
    print("Test stopped.")
    spi.close()
