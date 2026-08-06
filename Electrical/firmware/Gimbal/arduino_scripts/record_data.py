import serial
import csv
import time
import threading
import queue
from datetime import datetime

SERIAL_PORT = "/dev/ttyUSB0"   # change as needed
BAUD_RATE = 2000000

q = queue.Queue(maxsize=100000)
stop_event = threading.Event()

def serial_reader(ser):
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                q.put(line, timeout=1)
        except Exception as e:
            print("Serial read error:", e)
            stop_event.set()

input("Press ENTER to start recording...")

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"imu_log_{timestamp}.csv"

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(2)
ser.reset_input_buffer()

print(f"Recording to {filename}")
print("Press CTRL+C to stop.")

reader_thread = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
reader_thread.start()

lines_written = 0

try:
    with open(filename, "w", newline="", buffering=1024 * 1024) as file:
        writer = csv.writer(file)

        while True:
            try:
                line = q.get(timeout=1)
            except queue.Empty:
                continue

            writer.writerow(line.split(","))
            lines_written += 1

            if lines_written % 1000 == 0:
                file.flush()
                print(f"Saved {lines_written} lines...")

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    stop_event.set()
    reader_thread.join(timeout=1)

    while not q.empty():
        line = q.get()
        with open(filename, "a", newline="") as file:
            csv.writer(file).writerow(line.split(","))

    ser.close()
    print(f"Done. Saved {lines_written} lines to {filename}")