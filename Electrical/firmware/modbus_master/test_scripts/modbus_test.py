# This is a simple Python script just to test comms with a modbus slave
# The only thing you really need to modify are the 3 variables at the top
# If the script successfully completes all tests then you have a successful modbus communication with your slave
import sys
sys.path.insert(0, '..')
import modbus_master

# Configure depending on your needs
PORT = 'COM6'
SLAVE_ID = 1
BPS = 115200

def run_tests():
    print("Test 1: Connect to Serial Port")
    node = modbus_master.Instrument(PORT, SLAVE_ID, BPS)
    if node.serial:
        print("Success!")
    else:
        print("FAILED!")
        return

    print("\nTest 2: Write/Read Integers")
    node.write_registers(0, [1, 3, 3, 7, -1, 65536])
    res = node.read_registers(0, 6)
    if res == [1, 3, 3, 7, 65535, 0]:
        print("Success!")
    else:
        print("FAILED!")
        return

    print("\nTest 3: Write/Read Floats")
    node.write_registers(256, [3.14, 2.78, 0.45])
    res = [round(r, 2) for r in node.read_registers(256, 3)]
    if res == [3.14, 2.78, 0.45]:
        print("Success!")
    else:
        print("FAILED!")
        return
    
    print("\nTest 4: Write/Read Chars")
    node.write_registers(512, ['a', 'b', 'c'])
    res = node.read_registers(512, 3)
    if res == ['a', 'b', 'c']:
        print("Success!")
    else:
        print("FAILED!")
        return

    print("\nTest 5: Write/Read Bools")
    node.write_registers(768, [True, False, True])
    res = node.read_registers(768, 3)
    if res == [True, False, True]:
        print("Success!")
    else:
        print("FAILED!")
        return

    print("\nTest 6: Write/Read Mixed Types")
    node.write_registers(766, ['a', 'b', True, False])
    res = node.read_registers(766, 4)
    if res == ['a', 'b', True, False]:
        print("Success!")
    else:
        print("FAILED!")
        return
    
    print("\nAll tests successfully passed!")

run_tests()