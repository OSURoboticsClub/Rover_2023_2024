from modbus_master import Instrument

inst = Instrument('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D3499JFZ-if00-port0', 15, 115200)

inst.write_registers(1, [1])
