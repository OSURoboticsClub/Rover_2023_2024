import smbus2
import bme280
import math
port = 1
address = 0x77
bus = smbus2.SMBus(port)

bme280.load_calibration_params(bus, address)
data = bme280.sample(bus, address)
altitude = 44330 * (1.0-(data.pressure/1013.25)**(1/5.255))
print(f"{altitude:.2f}")
