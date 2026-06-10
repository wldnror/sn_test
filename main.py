import time
from collections import deque

import board
import digitalio
import adafruit_bme680
import matplotlib.pyplot as plt


# Raspberry Pi SPI0
# SCK  = GPIO11
# MOSI = GPIO10
# MISO = GPIO9
# CS   = GPIO8 / CE0
spi = board.SPI()

cs = digitalio.DigitalInOut(board.CE0)

sensor = adafruit_bme680.Adafruit_BME680_SPI(spi, cs)

# 해수면 기압값, 일단 기본값
sensor.sea_level_pressure = 1013.25

times = deque(maxlen=300)
gas_values = deque(maxlen=300)
temp_values = deque(maxlen=300)
hum_values = deque(maxlen=300)
press_values = deque(maxlen=300)

plt.ion()
fig, ax = plt.subplots()

start = time.time()

print("BME688 SPI test start")
print("처음 2~5분 안정화 후 알코올/향수/손소독제를 가까이 대보세요.")
print("Ctrl+C 종료")

try:
    while True:
        now = time.time() - start

        temp = sensor.temperature
        hum = sensor.relative_humidity
        pressure = sensor.pressure
        gas = sensor.gas

        print(
            f"{now:7.1f}s | "
            f"T={temp:6.2f} C | "
            f"H={hum:6.2f} % | "
            f"P={pressure:8.2f} hPa | "
            f"Gas={gas:10.0f} ohm"
        )

        times.append(now)
        gas_values.append(gas)

        ax.clear()
        ax.plot(times, gas_values, label="Gas resistance")
        ax.set_xlabel("Time (sec)")
        ax.set_ylabel("Gas resistance (ohm)")
        ax.set_title("BME688 Gas Resistance")
        ax.grid(True)
        ax.legend()

        plt.pause(0.01)

        time.sleep(1)

except KeyboardInterrupt:
    print("\n테스트 종료")
