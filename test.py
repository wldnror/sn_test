import time
from collections import deque

import bme680
import matplotlib.pyplot as plt


# SPI CE0 사용: /dev/spidev0.0
sensor = bme680.BME680(
    spi_device=0,
    spi_channel=0
)

# 센서 설정
sensor.set_humidity_oversample(bme680.OS_2X)
sensor.set_pressure_oversample(bme680.OS_4X)
sensor.set_temperature_oversample(bme680.OS_8X)
sensor.set_filter(bme680.FILTER_SIZE_3)

# 가스 측정 히터 설정
sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
sensor.set_gas_heater_temperature(320)   # 히터 온도 °C
sensor.set_gas_heater_duration(150)      # 히터 시간 ms
sensor.select_gas_heater_profile(0)

times = deque(maxlen=300)
gas_values = deque(maxlen=300)

plt.ion()
fig, ax = plt.subplots()

start = time.time()

print("BME688 SPI gas test start")
print("처음 2~5분 정도 안정화 후 알코올/향수/손소독제를 가까이 대보세요.")
print("Ctrl+C 로 종료")

try:
    while True:
        if sensor.get_sensor_data():
            now = time.time() - start

            temp = sensor.data.temperature
            hum = sensor.data.humidity
            pressure = sensor.data.pressure

            if sensor.data.heat_stable:
                gas = sensor.data.gas_resistance
            else:
                gas = None

            print(
                f"{now:7.1f}s | "
                f"T={temp:6.2f} C | "
                f"H={hum:6.2f} % | "
                f"P={pressure:8.2f} hPa | "
                f"Gas={gas}"
            )

            times.append(now)
            gas_values.append(gas if gas is not None else 0)

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
