import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)   # CE0
spi.max_speed_hz = 50000
spi.mode = 0

def read_reg(reg):
    result = spi.xfer2([reg, 0x00])
    return result[1]

while True:
    chip_id = read_reg(0xD0)
    print("Chip ID:", hex(chip_id))
    time.sleep(1)
