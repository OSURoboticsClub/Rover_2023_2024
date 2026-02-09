import serial
import time
import struct

# Configure the serial port
ser = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)

def is_rtcm(buf):
    # RTCM3 always starts with D3, then 6-bit length
    return len(buf) >= 2 and buf[0] == 0xD3

def parse_rtcm(data):
    print(f"RTCM3: {len(data)}")

def parse_nmea(line):
    parts = line.split(",")
    if parts[0] in ["$GNGGA", "$GPGGA"]:
        try:
            if len(parts) > 9 and parts[2] and parts[4]:
                lat_raw = parts[2]
                lon_raw = parts[4]

                lat_deg = int(float(lat_raw) / 100)
                lat_min = float(lat_raw) - (lat_deg * 100)
                lat = lat_deg + (lat_min / 60.0)

                lon_deg = int(float(lon_raw) / 100)
                lon_min = float(lon_raw) - (lon_deg * 100)
                lon = lon_deg + (lon_min / 60.0)

                if parts[3] == 'S':
                    lat = -lat
                if parts[5] == 'W':
                    lon = -lon

                fix_quality = parts[6]
                num_sats = parts[7]
                altitude = parts[9] if len(parts) > 9 else "N/A"

                print(f"NMEA {parts[0]}: lat={lat:.6f}, lon={lon:.6f}, "
                      f"fix={fix_quality}, sats={num_sats}, alt={altitude}m")
        except (ValueError, IndexError):
            pass

def parse_ubx_svin(payload: bytes):
    """
    Parse a UBX-NAV-SVIN payload (40 bytes) into readable fields.
    """
    # 
    payload = payload[6:-2]
    if len(payload) != 40:
        print(f"Invalid payload length: {len(payload)} (expected 40)")
        return

    # Format string based on payload layout:
    # <     : little-endian
    # B 3x  : version (U1) + 3 reserved bytes
    # I I   : iTOW (U4), dur (U4)
    # i i i : meanX, meanY, meanZ (I4 signed)
    # b b b : meanXHP, meanYHP, meanZHP (I1 signed)
    # B     : reserved2
    # I     : meanAcc (U4)
    # I     : obs (U4)
    # B B   : valid, active
    # 2x    : reserved3
    fmt = '<B3x I I i i i b b b B I I B B 2x'

    unpacked = struct.unpack(fmt, payload)

    version = unpacked[0]
    iTOW = unpacked[1]
    dur = unpacked[2]
    meanX = unpacked[3]
    meanY = unpacked[4]
    meanZ = unpacked[5]
    meanXHP = unpacked[6]
    meanYHP = unpacked[7]
    meanZHP = unpacked[8]
    reserved2 = unpacked[9]
    meanAcc = unpacked[10]
    obs = unpacked[11]
    valid = unpacked[12]
    active = unpacked[13]

    # Compute high-precision ECEF coordinates in cm
    x = meanX + meanXHP * 0.01
    y = meanY + meanYHP * 0.01
    z = meanZ + meanZHP * 0.01

    print(f"iTOW: {iTOW} ms, duration: {dur}s")
    print(f"ECEF X: {x:.2f} cm, Y: {y:.2f} cm, Z: {z:.2f} cm")
    print(f"Accuracy: {meanAcc*0.1:.2f} mm, observations: {obs}")
    print(f"Valid: {valid}, Active: {active}")



try:
    print(f"Opening serial port {ser.port} at {ser.baudrate} baud...")
    buffer = bytearray()

    while True:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            buffer.extend(chunk)

            while buffer:
                # RTCM3 detection
                if len(buffer) >= 3 and is_rtcm(buffer):
                    msg_len = ((buffer[1] & 0x03) << 8) | buffer[2]
                    total_len = 3 + msg_len + 3
                    if len(buffer) >= total_len:
                        msg = buffer[:total_len]
                        parse_rtcm(msg)
                        buffer = buffer[total_len:]
                        continue
                    break  # wait for more bytes

                # NMEA sentence detection
                elif buffer[0] == ord('$'):
                    newline_idx = buffer.find(b'\n')
                    if newline_idx != -1:
                        line = buffer[:newline_idx].decode('ascii', errors='ignore').strip()
                        parse_nmea(line)
                        buffer = buffer[newline_idx + 1:]
                        continue
                    break  # wait for more bytes

                # UBX detection
                elif len(buffer) >= 6 and buffer[0] == 0xB5 and buffer[1] == 0x62:
                    # UBX header found
                    msg_class = buffer[2]
                    msg_id = buffer[3]
                    length = buffer[4] | (buffer[5] << 8)  # little-endian payload length
                    total_len = 6 + length + 2  # 6-byte header + payload + 2-byte checksum

                    if len(buffer) >= total_len:
                        ubx_msg = buffer[:total_len]
                        parse_ubx_svin(ubx_msg)
                        # TODO: parse UBX message payload here if desired
                        buffer = buffer[total_len:]
                        continue
                    break  # wait for more bytes

                else:
                    # Unknown byte — discard only the first byte
                    buffer = buffer[1:]

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nProgram terminated by user.")

finally:
    if ser.is_open:
        ser.close()
    print("Serial port closed.")
