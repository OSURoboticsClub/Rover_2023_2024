import time
import sys
import tty
import termios
import os
from typing import Any, Callable, Tuple
from typing import List
import typing

ESC = '\x1b'

#TODO, proper support for OS command sequences. Currently only designed to support CSI commands.
class EscapeSequence:
    params: str
    command: str
    numberParams: list[int]

    def __init__(self, seq: str):

        self.params = ""
        self.command = ""
        self.numberParams = list()

        paramSubstr = ""
        substrIsNum = False

        c = 0
        if seq == "":
            return
        if seq[0] == ESC:
            c += 1

        while c in range(len(seq)):
            if ord(seq[c]) in range(0x40, 0x7F) and seq[c] != "[":
                self.command = seq[c]
                break
            else:
                self.params += seq[c]

                if seq[c].isdecimal():
                    substrIsNum = True
                    paramSubstr += seq[c]
                else:
                    if substrIsNum:
                        self.numberParams.append(int(paramSubstr))
                        substrIsNum = False
                        paramSubstr = ""

            c += 1

        if substrIsNum:
            self.numberParams.append(int(paramSubstr))


#extracts all the escape sequences from a string
def _get_escape_sequences(string: str) -> list[EscapeSequence]:
    sequences = list()
    start = 0

    for i in range(len(string)):
        if string[i] == ESC:
            start = i
        elif ord(string[i]) in range(0x40, 0x7F) and string[i] != "[":
            sequences.append(EscapeSequence(string[start:i+1]))

    return sequences


class Color():
        r: int
        g: int
        b: int

        def __init__(self, r: int, g: int, b: int):
            if r in range(0, 256) and g in range(0, 256) and b in range(0, 256):
                self.r = r
                self.g = g
                self.b = b

        def __setattr__(self, name: str, value: Any) -> None:
            if type(value) is not int or value not in range(0, 256):
                raise(ValueError("Channel values must be between 0 and 255"))
            
            super().__setattr__(name, value)


class ConsoleUI:

    #TODO More testing
    # Cleanup on exit
    # Fix input text scrolling


    _sizeX: int
    _sizeY: int
    _color: bool
    _charsBuffer: List[List[str]]
    _colorsBuffer: List[List[Tuple[Color, Color]]]
    _exitCallback: typing.Callable
    _collectInput: bool
    _inputBuffer: str
    _lastInput: str
    _keyBinds: dict[int, typing.Callable]
    _fakeCursorDuration: float
    _fakeCursorTime: float
    _prevFrameTime: float
    _updateCallbacks: dict[int, Callable]
    _updateCallbackId: int


    def __init__(self, exitCallback: typing.Callable, automaticSize = True, color: bool = True, sizeX: int = 80, sizeY: int = 24):
        self._sizeX = sizeX
        self._sizeY = sizeY
        self._color = color
        self._exitCallback = exitCallback
        self._collectInput = False
        self._inputBuffer = ""
        self._lastInput = ""
        self._keyBinds = dict()
        self._fakeCursorDuration = 0.25
        self._fakeCursorTime = time.time()
        self._prevFrameTime = time.time()
        self._updateCallbacks = dict()
        self._updateCallbackId = 0

        #initialize console
        #clear console
        self._write_cursor_position(0, 0)
        sys.stdout.write(ESC+"[3J"+ESC+"[2J")
        sys.stdout.write("Initializing ...")
        #disable automatic line wrapping
        sys.stdout.write(ESC+"[=7l")
        #clear stdin
        sys.stdin.flush()

        sys.stdout.flush()

        #set console to raw, non-blocking mode
        self._originalAttributes = termios.tcgetattr(sys.stdin.fileno())
        tty.setraw(sys.stdin)
        os.set_blocking(sys.stdin.fileno(), False)

        if automaticSize:
            self.auto_resize()
        else:
            self.resize(sizeX, sizeY)

    
    def _console_write(self, string: str):

        #This strikes me as a disgusting hack, but seems to be very effective.
        #It seems that either bytes are duplicated, or the console cant interpret
        #broken escape sequences when attempting to write in nonblocking mode using
        #the commented logic at the bottom.

        os.set_blocking(sys.stdin.fileno(), True)

        sys.stdout.write(string)
        sys.stdout.flush()

        os.set_blocking(sys.stdin.fileno(), False)

        return

        #buffer = string.encode()
        #bytes = len(buffer)
        #while bytes > 0:
        #    try:
        #        bytes -= os.write(sys.stdout.fileno(), string.encode())
        #    except BlockingIOError as e:
        #        if e.errno == 11:
        #            continue
        #        else:
        #            raise


    def _set_cursor_visibility(self, visible: bool):
        if visible:
            self._console_write(ESC+"[?25h")
        else:
            self._console_write(ESC+"[?25l")


    def _write_cursor_position(self, x: int, y: int):
        if x >= self._sizeX or y >= self._sizeY:
            raise ValueError("Cursor must be inside console bounds!")
        
        self._console_write(ESC+f"[{y+1};{x+1}H")


    def _color_str(self, foreground: Color, background: Color) -> str:
        if self._color:
            return ESC+f"[38;2;{foreground.r};{foreground.g};{foreground.b}m" + ESC+f"[48;2;{background.r};{background.g};{background.b}m"
        else:
            return ""


    def _set_color(self, foreground: Color, background: Color):
        if self._color:
            self._console_write(self._color_str(foreground, background))


    def get_size(self) -> Tuple[int, int]:
        return (self._sizeX, self._sizeY)


    def auto_resize(self):
        #request console size
        self._console_write(ESC+"[9999;9999H")
        self._console_write(ESC+"[6n")

        #attempt to read console size from stdin
        time.sleep(2)

        input = ""

        try:
            input = sys.stdin.read()
        except Exception:
            pass

        seq = _get_escape_sequences(input)

        for s in seq:
            if s.command == "R":
                if len(s.numberParams) == 2:
                    self._sizeY = s.numberParams[0] - 1
                    self._sizeX = s.numberParams[1] - 1

        self.resize(self._sizeX, self._sizeY)


    #changes the size of the console
    def resize(self, sizeX: int, sizeY: int):

        if sizeX < 0 or sizeY < 0:
            raise ValueError("Size must be positive.")

        self._sizeX = sizeX
        self._sizeY = sizeY
        self._charsBuffer = list()
        self._colorsBuffer = list()
        for x in range(0, sizeX):
            self._charsBuffer.append(list())
            self._colorsBuffer.append(list())
            for y in range(0, sizeY):
                self._charsBuffer[x].append('')
                self._colorsBuffer[x].append((Color(255,255,255), Color(0,0,0)))

    
    #fills the output buffer with the specified color/char
    def fill_buffer(self, char: str, fg: Color, bg: Color):
        if len(char) > 1:
            raise ValueError("Char must be single character.")
        
        for x in range(0, self._sizeX):
            for y in range(0, self._sizeY):
                self._charsBuffer[x][y] = char
                self._colorsBuffer[x][y] = (fg, bg)

    
    #draw a character at a given location
    def draw_char(self, x: int, y: int, char: str, fg: Color, bg: Color):
        if x < 0 or y < 0 or x >= self._sizeX or y >= self._sizeY:
            return
        
        self._charsBuffer[x][y] = char
        self._colorsBuffer[x][y] = (fg, bg)

    
    #draw a string at a given location (coordinate indicates left side)
    def draw_string(self, x: int, y: int, string: str, fg: Color, bg: Color, maxLen: int = sys.maxsize):
        for i in range(len(string) if len(string) < maxLen else maxLen):
            self.draw_char(x + i, y, string[i], fg, bg)


    def draw_table(
            self,
            x: int,
            y: int,
            table: list[list[tuple[str, Color, Color]]],
            title: typing.Optional[tuple[str, Color, Color]],
            colWidth: int = 0,
            fixedColWidth: bool = False,
    ):
        raise NotImplemented()
        pass


    #draw output buffer to console
    def write_buffer(self):
        frame = ""

        self._write_cursor_position(0, 0)
        self._console_write(ESC+"[3J")
        self._set_cursor_visibility(False)

        prevColorStr = None

        for y in range(self._sizeY):
            for x in range(self._sizeX):
                colorStr = self._color_str(self._colorsBuffer[x][y][0], self._colorsBuffer[x][y][1])
                if colorStr != prevColorStr:
                    frame += colorStr
                    prevColorStr = colorStr
                frame += self._charsBuffer[x][y]
                if self._charsBuffer[x][y] == "":
                    frame += " "
            frame += "\r\n"

        self._console_write(frame)
        self._prevFrameTime = time.time()


    #bind callback to a keyboard input. 
    def bind_key(self, key: int, callback: typing.Callable):
        self._keyBinds.update({key: callback})


    #registers a callback that is run when update is called. Returns the callback's unique id
    def register_update_callback(self, callback: typing.Callable) -> int:
        self._updateCallbacks.update({self._updateCallbackId: callback})
        id = self._updateCallbackId
        self._updateCallbackId += 1
        return id


    #removes a callback from the list
    def remove_update_callback(self, id: int):
        self._updateCallbacks.pop(id)
    
    
    #Allows user input at a given location.
    def get_input(self, x: int, y: int, fg: Color, bg: Color, boxLength: typing.Optional[int] = None, showCursor: bool = True):
        self._inputX = x
        self._inputY = y
        if boxLength is None or x + boxLength >= self._sizeX:
            self._inputBoxLength = self._sizeX - x
        else: 
            self._inputBoxLength = boxLength
        self._collectInput = True
        self._inputColor = (fg, bg)
        self._cursorVisibility = showCursor


    def stop_input(self):
        self._collectInput = False
        self._inputBuffer = ""

    
    def collecting_input(self) -> bool:
        return self._collectInput


    def get_last_input(self) -> str:
        return self._lastInput


    def get_live_input_buffer(self) -> str:
        return self._inputBuffer


    #process input in stdin buffer
    def handle_input(self):

        #return

        def run_keybind(key: int):
            func = self._keyBinds.get(key)
            if func is not None:
                func()

        escSeq = False
        
        input = ""

        try:
            input = sys.stdin.read()
        except Exception:
            pass

        for c in input:
            if escSeq:
                if ord(c) in range(0x40, 0x7F) and c != "[":
                    escSeq = False
                    continue
                #TODO handle escape sequences here
            elif c == ESC:
                escSeq = True
            #handle CTRL+C
            elif ord(c) == 3:
                self._exitCallback()
                self.cleanup()
            #handle backspace and delete
            elif ord(c) == 8 or ord(c) == 127:
                #if currently collecting input, remove most last char from input buffer
                if self._collectInput:
                    if self._inputBuffer != "":
                        self._inputBuffer  = self._inputBuffer[0:-1]
            #handle enter
            elif ord(c) == 10 or ord(c) == 13:
                #if collecting input, finish and save buffer
                if self._collectInput:
                    self._collectInput = False
                    self._cursorVisibility = False
                    self._lastInput = self._inputBuffer
                    self._inputBuffer = ""
                #otherwise run a keybind if one exists
                else:
                    run_keybind(ord(c))
            #handle printable characters
            elif ord(c) in range(32, 127):
                if self._collectInput:
                    self._inputBuffer += c
                else:
                    run_keybind(ord(c))
            else:
                run_keybind(ord(c))

        #echo user input if necessary
        if self._collectInput:
            startIndex = 0
            if len(self._inputBuffer) > self._inputBoxLength:
                startIndex = len(self._inputBuffer) - self._inputBoxLength
            
            for i in range(len(self._inputBuffer) - startIndex):
                self.draw_char(self._inputX + i, self._inputY, self._inputBuffer[i + startIndex], self._inputColor[0], self._inputColor[1])

            curTime = time.time()
            if curTime - self._fakeCursorTime > self._fakeCursorDuration * 2:
                self._fakeCursorTime = curTime

            #TODO fix cursor colors
            if curTime - self._fakeCursorTime > self._fakeCursorDuration:
                x = self._inputX + len(self._inputBuffer) #(self._inputBoxLength - 1 if self._inputBoxLength >= len(self._inputBuffer) else len(self._inputBuffer))
                if x > self._inputX + self._inputBoxLength - 1:
                    x = self._inputX + self._inputBoxLength - 1
                self._colorsBuffer[x][self._inputY] = (Color(0, 0, 0), Color(255, 255, 255))


    def update(self, fg: Color = Color(255,255,255), bg: Color = Color(0,0,0), autoResize=True):

        #if autoResize:
        #    self.auto_resize() need to find a new way to handle this

        self.fill_buffer("", fg, bg)

        #handle user input
        self.handle_input()

        #run update callbacks
        for c in self._updateCallbacks.values():
            c()

        self.write_buffer()

    def cleanup(self):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSAFLUSH, self._originalAttributes)


        
        


    



    



    
