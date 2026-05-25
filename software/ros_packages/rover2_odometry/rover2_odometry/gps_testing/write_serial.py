import serial
import time
import sys
import os

# Configure the serial port
# Replace 'COMx' with your actual serial port (e.g., 'COM1' on Windows, '/dev/ttyUSB0' on Linux)
# Set the baudrate to match your device
ser = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1) # timeout in seconds
file_name = sys.argv[1]

cur_dir = os.getcwd()

config_dir = cur_dir + "/" + file_name


try:
    print(f"Opening configuration file: {config_dir}")
    config_file = open(config_dir, 'r')
    all_lines = config_file.readlines()
    print("Sending configuration commands...")
    
    for line in all_lines:
        command_str = line[3:]
        binary_data = bytes.fromhex(command_str)
        ser.write(binary_data)
    

    print(f"Opening serial port {ser.port} at {ser.baudrate} baud...")
    
    # Read data in a loop
    while True:
        if ser.in_waiting > 0:  # Check if there's data in the input buffer
            line = ser.readline().strip() # Read a line and decode it
            if line:
                print(f"Received: {line}")
        time.sleep(0.1) # Small delay to prevent busy-waiting

except serial.SerialException as e:
    print(f"Serial port error: {e}")
except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    if ser.is_open:
        ser.close()
        print("Serial port closed.")
