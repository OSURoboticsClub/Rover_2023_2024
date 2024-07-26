import struct
import serial

# Constants
_WRITE_INSTR = 16
_READ_INSTR = 3
_WRITE_RESP_SZ = 8
_READ_RESP_SZ_BASE = 5

_INT_REG_BYTE_SZ = 2
_FLOAT_REG_BYTE_SZ = 4
_CHAR_REG_BYTE_SZ = 1
_BOOL_REG_BYTE_SZ = 1

_INT_REG_OFFSET = 0
_FLOAT_REG_OFFSET = 256
_CHAR_REG_OFFSET = 512
_BOOL_REG_OFFSET = 768
_REG_MAX = 1023

_MAX_DATA_BYTES = 255
_INT_MAX = 65536

_serialports = {}

class Instrument:
    """Represents a slave
    """
    def __init__(
        self,
        port,
        slave_id,
        baudrate=19200
    ):
        """Opens serial comms with a node and assigns it a slave id
        """
        self.slave_id = slave_id
        self.port = port
        self.serial = None

        try:
            if port not in _serialports or not _serialports[port]:
                self.serial = _serialports[port] = serial.Serial(port, baudrate,
                                                                timeout=1)
            else:
                self.serial = _serialports[port]
        except serial.SerialException:
            print(f"Unable to open {port} for serial communications.")

    def write_registers(
        self,
        register_addr,
        values
    ):
        """Write a list of values starting at register_addr
        """
        if not self.serial:
            return

        if not _is_valid_write_data(register_addr, values):
            print("Attempted to write invalid data.")
            return

        num_bytes = _calculate_num_bytes(values)

        if num_bytes > _MAX_DATA_BYTES:
            print(f"Write command would have sent over {_MAX_DATA_BYTES} data bytes.")
            return

        # The beginning of all write packets
        header = [
            self.slave_id,
            _WRITE_INSTR,
            register_addr,
            len(values),  # Length of values is equal to num registsers
            num_bytes
        ]

        # Create the byte string for the write packet
        byte_string = _create_byte_string_int8(header[0:2])
        byte_string += _create_byte_string(header[2:4])
        byte_string += _create_byte_string_int8([header[4]])
        byte_string += _create_byte_string(values)

        packet = bytes(byte_string, encoding='latin1')
        packet = _add_crc(packet)

        # Send the instruction to slave, then read its response
        self.serial.write(packet)
        resp = self.serial.read(_calculate_resp_size(_WRITE_INSTR))

        if not resp:
            print("Write received no response.")
            return

        # Do a CRC check (probably not necessary since this is just discarded)
        crc = resp[-2] << 8 | resp[-1]
        if not _check_crc(resp, crc):
            print("Write response CRC check failed.")

    def read_registers(
        self,
        register_addr,
        num_registers
    ):
        """Read num_registers starting at register_addr
        """
        if not self.serial:
            return []
        
        # The beginning of all read packets
        header = [
            self.slave_id,
            _READ_INSTR,
            register_addr,
            num_registers
        ]

        # Create the byte string for the read packet
        byte_string = _create_byte_string_int8(header[0:2])
        byte_string += _create_byte_string(header[2:4])

        packet = bytes(byte_string, encoding='latin1')
        packet = _add_crc(packet)

        # Find the count of each register in the packet to calculate
        # expected response data bytes
        num_ints, num_floats, num_chars, num_bools = _calc_num_types(
            register_addr,
            num_registers
        )
        num_bytes = num_ints * _INT_REG_BYTE_SZ
        num_bytes += (num_floats * _FLOAT_REG_BYTE_SZ)
        num_bytes += (num_chars * _CHAR_REG_BYTE_SZ)
        num_bytes += (num_bools * _BOOL_REG_BYTE_SZ)

        if num_bytes > _MAX_DATA_BYTES:
            print(f"Read command requested over {_MAX_DATA_BYTES} data bytes.")
            return []

        # Send the instruction to slave, then read its response
        self.serial.write(packet)
        resp = self.serial.read(_calculate_resp_size(_READ_INSTR, num_bytes))

        # Don't continue if no response
        if not resp:
            print("Read received no response.")
            return []

        # Convert the data bytes (including CRC) of the response bytestring
        # into a list of values
        data = _unpack(resp, register_addr, num_registers)

        # Perform a CRC check and if it fails, don't return any values
        crc = data[-1]
        if not _check_crc(resp, crc):
            print("Read response CRC check failed.")
            return []
        return data[0:-1]  # Return data byte values (minus CRC)
    
    def write_register(
        self,
        register_addr,
        value
    ):
        """Write a value to register at register_addr
        """
        self.write_registers(register_addr, [value])

    def read_register(
        self,
        register_addr,
    ):
        """Read register at register_addr
        """
        res = self.read_registers(register_addr, 1)
        return res[0] if res else None

def _create_byte_string(values):
    """Convert a list of values into a byte string
    """
    bstr = ''
    for v in values:
        if isinstance(v, bool):  # Bool must come before int
            bstr += _pack('>?', v)
        elif isinstance(v, int):
            bstr += _pack('>H', v % _INT_MAX)
        elif isinstance(v, float):
            bstr += _pack('>f', v)
        elif isinstance(v, str):  # Would probably act weird if v is >1 char
            bstr += _pack('>c', bytes(v, encoding='latin1'))
    return bstr


def _create_byte_string_int8(values):
    """Converts a list of integers into a uint8 byte string
    """
    bstr = ''
    for v in values:
        bstr += _pack('>B', v)  # Interprets v as a single uint8
    return bstr


def _calculate_num_bytes(values):
    """Calculates the total number of bytes for a list of values
    """
    num_bytes = 0
    for v in values:
        if isinstance(v, bool):  # Bool must come before int
            num_bytes += _BOOL_REG_BYTE_SZ
        elif isinstance(v, int):
            num_bytes += _INT_REG_BYTE_SZ
        elif isinstance(v, float):
            num_bytes += _FLOAT_REG_BYTE_SZ
        elif isinstance(v, str):
            num_bytes += _CHAR_REG_BYTE_SZ
    return num_bytes


def _calculate_resp_size(instr, num_bytes=0):
    """Calculates the expected response size for a command
    """
    if instr == _WRITE_INSTR:
        return _WRITE_RESP_SZ
    else:
        return _READ_RESP_SZ_BASE + num_bytes


def _calculate_crc(packet):
    """Calculates the CRC for a list of values (BLACK MAGIC)
    """
    crc = 0xffff
    for i in range(len(packet)):
        crc ^= packet[i]

        for j in range(8, 0, -1):
            if((crc & 0x001) != 0):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    crc = (crc & 0xff) << 8 | (crc >> 8)
    return crc


def _check_crc(resp, crc):
    """Checks that the CRC of a list of values matches the expected CRC
    """
    return _calculate_crc(resp[:-2]) == crc


def _add_crc(packet):
    """Adds crc to packet"""
    crc = _calculate_crc(packet)
    packet += crc.to_bytes(2, 'big')
    return packet


def _pack(formatstring, value):
    """Packs a value into a bytestring based on format
    """
    return str(struct.pack(formatstring, value), encoding='latin1')


def _unpack(
        resp,
        register_addr,
        num_registers,
        skip_header=True
):
    """Given a response byte string, unpacks it into a list of values
    """
    # Get the number of each register type in the packet
    num_ints, num_floats, num_chars, num_bools = _calc_num_types(
        register_addr,
        num_registers
    )

    values = []
    offset = 3 if skip_header else 0  # Skip slave id, function code, etc

    # Loop through each register type, increasing offset by register size
    # every time to ensure we are reading the correct value from the packet
    while num_ints > 0:
        values.append(struct.unpack_from('>H', resp, offset)[0])
        offset += _INT_REG_BYTE_SZ
        num_ints -= 1

    while num_floats > 0:
        values.append(struct.unpack_from('>f', resp, offset)[0])
        offset += _FLOAT_REG_BYTE_SZ
        num_floats -= 1

    while num_chars > 0:
        values.append(
            str(struct.unpack_from('>c', resp, offset)[0], encoding='latin1')
        )
        offset += _CHAR_REG_BYTE_SZ
        num_chars -= 1

    while num_bools > 0:
        values.append(struct.unpack_from('>?', resp, offset)[0])
        offset += _BOOL_REG_BYTE_SZ
        num_bools -= 1

    # Append the CRC to the value list
    values.append(struct.unpack_from('>H', resp, offset)[0])
    return values


def _calc_num_type(
    register_addr,
    num_registers,
    max_reg
):
    """Returns the number of a register type in a packet
    """
    if num_registers <= 0:
        return 0
    elif register_addr < max_reg:
        if register_addr + num_registers <= max_reg:
            total = num_registers
        else:
            total = max_reg - register_addr
    else:
        total = 0
    return total


def _calc_num_types(
    register_addr,
    num_registers,
):
    """Returns the total number of each register type in a packet
    """
    # To calculate the number of each register type, we increase the register
    # addr by the total count of the previous types and decrease num registers
    # by the same
    num_ints = _calc_num_type(
        register_addr,
        num_registers,
        _FLOAT_REG_OFFSET
    )
    num_floats = _calc_num_type(
        register_addr + num_ints,
        num_registers - num_ints,
        _CHAR_REG_OFFSET
    )
    num_chars = _calc_num_type(
        register_addr + num_ints + num_floats,
        num_registers - num_ints - num_floats,
        _BOOL_REG_OFFSET
    )
    num_bools = _calc_num_type(
        register_addr + num_ints + num_floats + num_chars,
        num_registers - num_ints - num_floats - num_chars,
        _REG_MAX
    )

    return num_ints, num_floats, num_chars, num_bools


def _reg_in_range(addr, start, end):
    """Checks whether a given register address falls within the range provided (end exclusive)
    """
    return addr >= start and addr < end


def _is_valid_write_data (
    reg_addr,
    values
):
    """Checks that the type of the given list of values matches the registers
    """
    if reg_addr < _INT_REG_OFFSET or reg_addr >= _REG_MAX:
        return False

    for v in values:
        if    ((_reg_in_range(reg_addr, _INT_REG_OFFSET, _FLOAT_REG_OFFSET) and (isinstance(v, bool) or not isinstance(v, int)))
            or (_reg_in_range(reg_addr, _FLOAT_REG_OFFSET, _CHAR_REG_OFFSET) and not isinstance(v, float))
            or (_reg_in_range(reg_addr, _CHAR_REG_OFFSET, _BOOL_REG_OFFSET) and (not isinstance(v, str) or len(v) > 1))
            or (_reg_in_range(reg_addr, _BOOL_REG_OFFSET, _REG_MAX) and not isinstance(v, bool))):
            return False
        reg_addr += 1

    return True
