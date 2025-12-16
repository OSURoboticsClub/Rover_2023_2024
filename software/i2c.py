import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
while not i2c.try_lock():
    pass

print("I2C addresses found:", [hex(device_address) for device_address in i2c.scan()])
i2c.unlock()
