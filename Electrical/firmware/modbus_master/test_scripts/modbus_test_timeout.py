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

node = modbus_master.Instrument(PORT, SLAVE_ID, BPS)
print(node.read_register(69))