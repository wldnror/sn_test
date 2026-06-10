import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 50000
spi.mode = 0

def read_reg(reg):
    return spi.xfer2([reg | 0x80, 0x00])[1]

def write_reg(reg, val):
    spi.xfer2([reg & 0x7F, val & 0xFF])

write_reg(0xE0, 0xB6)
time.sleep(0.2)

write_reg(0x75, 0x08)
write_reg(0x72, 0x01)
write_reg(0x5A, 0x73)
write_reg(0x64, 0x59)
write_reg(0x71, 0x20)
write_reg(0x74, (0b010 << 5) | (0b101 << 2) | 0b01)

time.sleep(1.0)

for r in [0xD0, 0x75, 0x72, 0x5A, 0x64, 0x71, 0x74]:
    print(hex(r), "=", hex(read_reg(r)))

data = spi.xfer2([0x1D | 0x80] + [0x00] * 17)[1:]
print("FIELD0:", [hex(x) for x in data])
