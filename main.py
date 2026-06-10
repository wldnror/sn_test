import board
import digitalio
import adafruit_bme680

spi = board.SPI()

for ce in [board.CE0, board.CE1]:
    try:
        cs = digitalio.DigitalInOut(ce)
        sensor = adafruit_bme680.Adafruit_BME680_SPI(spi, cs)
        print("OK:", ce)
        print("Temp:", sensor.temperature)
        break
    except Exception as e:
        print("Fail:", ce, e)
