import time
import board
import digitalio
import adafruit_bme680

spi = board.SPI()
cs = digitalio.DigitalInOut(board.CE0)

sensor = adafruit_bme680.Adafruit_BME680_SPI(
    spi,
    cs,
    baudrate=50000
)

while True:
    print("Temp:", sensor.temperature)
    print("Humidity:", sensor.relative_humidity)
    print("Pressure:", sensor.pressure)
    print("Gas:", sensor.gas)
    print("----------------")
    time.sleep(1)
