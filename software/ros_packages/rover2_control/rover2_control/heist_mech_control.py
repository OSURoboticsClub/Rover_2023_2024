from enum import Enum
from typing import List

import rclpy
from rclpy.node import Node
import can
from rover2_control_interface.srv import HeistMechanismService

#controller for the heist mechanism(s).
# Needed functionality:
#   - Move heist mechanism to one of three positions: home, morse, xlr
#   - Send morse code message
# 
# Implement a ros service, accecpt opcode, parameter string

MORSE_MAP = {
    # Letters
    'A': '.-', 'B': '-...', 'C': '-.-.', 'D': '-..', 'E': '.', 
    'F': '..-.', 'G': '--.', 'H': '....', 'I': '..', 'J': '.---', 
    'K': '-.-', 'L': '.-..', 'M': '--', 'N': '-.', 'O': '---', 
    'P': '.--.', 'Q': '--.-', 'R': '.-.', 'S': '...', 'T': '-', 
    'U': '..-', 'V': '...-', 'W': '.--', 'X': '-..-', 'Y': '-.--', 
    'Z': '--..',
    
    # Numbers
    '1': '.----', '2': '..---', '3': '...--', '4': '....-', '5': '.....', 
    '6': '-....', '7': '--...', '8': '---..', '9': '----.', '0': '-----',
    
    # Punctuation
    '.': '.-.-.-', ',': '--..--', '?': '..--..', '/': '-..-.',

    # Spaces
    ' ':'/'

}


class MechansimCommand(Enum):
    GET_STATUS         = 4
    MORSE_ADD          = 1
    MORSE_EXEC         = 2
    SERVO_POS          = 3
    TOGGLE_SOLENOID    = 6
    READ_MORSE_BUFFER  = 7
    CLEAR_MORSE_BUFFER = 5


class ServoPosition(Enum):
    HOME  = 2
    MORSE = 1
    XLR   = 0


class ServiceCommand(Enum):
    SEND_MORSE      = 0
    TOGGLE_SOLENOID = 1
    SERVO_HOME      = 2
    SERVO_XLR       = 3
    SERVO_MORSE     = 4


def string_to_morse(string: str) -> List[str]:

    result = list() 

    for i, char in enumerate(string):
        symbol = MORSE_MAP.get(char)
        if symbol is not None: 
            if symbol != '/' and (i != len(string) - 1) and MORSE_MAP.get(string[i + 1]) != '/':
                symbol += ' '

        result.append(symbol)

    return result


def morse_to_string(morse: List[str]):

    result = ""

    for symbol in morse:
        for char, code in MORSE_MAP.items():
            if code == symbol:
                result += char

    return result


class CanFrame():

    node_id: int
    cmd_id: int
    data: bytearray | None

    def __init__(self, node_id: int, cmd_id: int, data: bytearray | None = None):
        if data is not None and len(data) > 8:
            raise(ValueError("CAN Frame data longer than 8 bytes"))

        self.node_id = node_id
        self.cmd_id = cmd_id
        self.data = data

    def get_can_message(self):
        return can.Message(arbitration_id = (self.node_id << 5 | self.cmd_id), data = self.data)

    
        

class HeistMechController(Node):

    def recv_can(self, node_ids: list[int] | None = None) -> List[CanFrame]:
        result = list()

        for msg in self.bus:
            node_id = msg.arbitration_id >> 5

            if node_ids is not None and node_id not in node_ids: continue

            cmd_id = msg.arbitration_id | 0b11111
            result.append(CanFrame(node_id, cmd_id, msg.data))


    def ServiceCallback(self, req: HeistMechanismService.Request, resp: HeistMechanismService.Response) -> HeistMechanismService.Response:

        def try_send_can(cmd: int, data: bytearray | None = None):
            try:
                self.bus.send(CanFrame(self.mech_can_id, cmd, data).get_can_message())
                resp.result_msg = "Command successful"
            except Exception as e:
                resp.result_msg = f"Failed to send CAN with exception: + {e}"


        def send_servo_can(pos: ServoPosition):
            try_send_can(MechansimCommand.SERVO_POS.value, [pos.value])


        resp = HeistMechanismService.Response()
        resp.success = False
        resp.result_msg = "Unknown error"

        morse = string_to_morse(req.message)

        if req.command == ServiceCommand.SEND_MORSE.value:
            try:
                if len(morse) == 0:
                    resp.result_msg("Failed to send morse: message empty")
                else:
                    morse_string = ""
                    for s in morse:
                        morse_string += s

                    #clear buffer
                    self.bus.send(CanFrame(self.mech_can_id, MechansimCommand.CLEAR_MORSE_BUFFER.value).get_can_message())

                    #send morse code
                    morse_substr = ""
                    for c in morse_string:
                        morse_substr += c
                        if len(morse_substr) == 8:
                            self.bus.send(CanFrame(self.mech_can_id, MechansimCommand.MORSE_ADD.value, bytearray(morse_substr, 'ASCII')).get_can_message())
                            morse_substr = ""

                    if len(morse_substr) != 0:
                        self.bus.send(CanFrame(self.mech_can_id, MechansimCommand.MORSE_ADD.value, bytearray(morse_substr, 'ASCII')).get_can_message())

                    #send command to execute buffer
                    self.bus.send(CanFrame(self.mech_can_id, MechansimCommand.MORSE_EXEC.value).get_can_message())

                    resp.result_msg = "Operation succeeded"
                    resp.success = True

            except Exception as e:
                #resp.result_msg = f"Failed to send message with exception: + {e}"
                raise
                return resp


        elif req.command == ServiceCommand.TOGGLE_SOLENOID.value:
            try_send_can(MechansimCommand.TOGGLE_SOLENOID.value)

        elif req.command == ServiceCommand.SERVO_HOME.value:
            send_servo_can(ServoPosition.HOME)

        elif req.command == ServiceCommand.SERVO_MORSE.value:
            send_servo_can(ServoPosition.MORSE)

        elif req.command == ServiceCommand.SERVO_XLR.value:
            send_servo_can(ServoPosition.XLR)

        else:
            resp.result_msg = "Unknown command"

        return resp


    def __init__(self):

        super().__init__("heist_mech_control_node")

        self.can_net = self.declare_parameter('can', 'can_arm').value
        self.service_name = self.declare_parameter('service_name', '/heist_mech').value
        self.mech_can_id = self.declare_parameter('can_id', 0x01).value

        self.bus = can.interface.Bus(channel=self.can_net, bustype='socketcan')
        while not (self.bus.recv(timeout=0) is None): pass

        self.create_service(HeistMechanismService, self.service_name, self.ServiceCallback)




def main(args=None):
    rclpy.init(args=args)

    heist_mech_node = HeistMechController()

    rclpy.spin(heist_mech_node)

    heist_mech_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()