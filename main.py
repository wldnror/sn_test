import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0

print(spi.xfer2([0x00]))
print(spi.xfer2([0xD0, 0x00]))  # chip id 읽기 시도
