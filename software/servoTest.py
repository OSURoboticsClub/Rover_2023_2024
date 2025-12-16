import serial
import sys
input = sys.argv[1]

arduino = serial.Serial(port="/dev/ttyACM0",baudrate=9600, timeout=0.1)
arduino.write(bytes(input,'utf-8'))
