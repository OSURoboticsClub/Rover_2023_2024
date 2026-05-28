import camera_gimbal_controller.console_ui as cui
import time
import typing
import sys

class UIElement:

    _console: cui.ConsoleUI
    _id: int

    def __init__(self, console: cui.ConsoleUI) -> None:
        self._console = console
        self._id = console.register_update_callback(self.update)

    def update(self) -> None:
        pass


class StaticStringAnimation(UIElement):

    _frames: list[tuple[str, cui.Color, cui.Color]]
    _frameRate: float
    _currFrame: int
    _locations: list[tuple[int, int]]
    _prevTime: float

    def __init__(self, console: cui.ConsoleUI, frames: list[tuple[str, cui.Color, cui.Color]], frameRate: float) -> None:
        super().__init__(console)
        if len(frames) == 0:
            raise ValueError("StaticStringAnimation requires at least one frame.")
        
        self._frames = frames
        self._frameRate = frameRate
        self._prevTime = time.time()
        self._locations = list()
        self._currFrame = 0


    def add_location(self, loc: tuple[int, int]) -> None:
        self._locations.append(loc)


    def remove_location(self, loc: tuple[int, int]) -> None:
        self._locations.remove(loc)


    def clear_locations(self) -> None:
        self._locations.clear()


    def update(self) -> None:
        super().update()
        
        curTime = time.time()
        if curTime - self._prevTime > 1/self._frameRate:
            self._currFrame += 1
            self._currFrame %= len(self._frames)

        for l in self._locations:
            self._console.draw_string(
                l[0], l[1], 
                self._frames[self._currFrame][0], 
                self._frames[self._currFrame][1], 
                self._frames[self._currFrame][2]
            )


class AutomaticStringField(UIElement):

    _getData: typing.Callable
    _displayLength: int
    _location: tuple[int, int]
    _visible: bool

    def __init__(
            self,
            console: cui.ConsoleUI,
            getData: typing.Callable, #callback which returns a tuple contining in order, string, fg color, bg color
            displayLength: typing.Optional[int],
            locationX: int,
            locationY: int,
            visible: bool = True
    ):
        super().__init__(console)
        self._getData = getData
        self._location = (locationX, locationY)
        self._visible = visible
        if displayLength is not None and displayLength > 0:
            self._displayLength = displayLength
        else:
            self._displayLength = sys.maxsize

    
    def setVisibility(self, visible: bool):
        self._visible = visible

    
    def update(self) -> None:
        super().update()

        if self._visible:
            data = self._getData()
            self._console.draw_string(
                self._location[0],
                self._location[1],
                data[0],
                data[1],
                data[2],
                maxLen=self._displayLength
            )


class StringField(UIElement):

    _visible: bool
    _data: str
    _fg: cui.Color
    _bg: cui.Color
    _x: int
    _y: int
    _maxLen: int

    def set(self, data: str, fg: cui.Color | None = None, bg: cui.Color | None = None):
        self._data = data
        if fg is not None:
            self._fg = fg
        if bg is not None:
            self._bg = bg

    def setVisibility(self, visble: bool):
        self._visible = visble

    def update(self) -> None:
        super().update()
        if self._visible:
            self._console.draw_string (
                self._x,
                self._y,
                self._data,
                self._fg,
                self._bg,
                self._maxLen
            )

    def __init__(
            self, 
            console: cui.ConsoleUI,
            x: int,
            y: int,
            data: str = "",
            fg: cui.Color = cui.Color(255,255,255),
            bg: cui.Color = cui.Color(0,0,0),
            maxLen: int = sys.maxsize,
            visible: bool = True
        ) -> None:
        super().__init__(console)
        self._x = x
        self._y = y
        self._data = data
        self._visible = visible
        self._fg = fg
        self._bg = bg
        self._maxLen = maxLen


class TableField(UIElement):

    _visible: bool
    _data: dict[str, str]
    _title: str
    _fg: cui.Color
    _bg: cui.Color
    _title_fg: cui.Color
    _title_bg: cui.Color
    _x: int
    _y: int
    _col1Width: int
    _col2Width: int
    _autoCol1Width: bool
    _autoCol2Width: bool

    def _compute_widths(self, data: dict[str, str]) -> tuple[int, int]:
        keyWidths = [len(str(key)) for key in data.keys()]
        valueWidths = [len(str(value)) for value in data.values()]

        return (
            max(keyWidths) if keyWidths else 0,
            max(valueWidths) if valueWidths else 0
        )

    def _refresh_auto_widths(self) -> None:
        if self._autoCol1Width or self._autoCol2Width:
            col1Width, col2Width = self._compute_widths(self._data)
            if self._autoCol1Width:
                self._col1Width = col1Width
            if self._autoCol2Width:
                self._col2Width = col2Width

    def _format_cell(self, text: str, width: int) -> str:
        if len(text) > width:
            return text[:width]
        return text.ljust(width)

    def set(self, data: dict[str, str], title: str | None = None, col1Width: int | None = None, col2Width: int | None = None):
        self._data = data
        if title is not None:
            self._title = title
        self._autoCol1Width = col1Width is None
        self._autoCol2Width = col2Width is None
        if col1Width is not None:
            self._col1Width = col1Width
        if col2Width is not None:
            self._col2Width = col2Width
        self._refresh_auto_widths()

    def update_row(self, key: str, value: str) -> None:
        self._data.update({key: value})
        self._refresh_auto_widths()

    def remove_row(self, key: str) -> bool:
        if str(key) not in self._data:
            return False
        self._data.pop(str(key))
        self._refresh_auto_widths()
        return True

    def setVisibility(self, visble: bool):
        self._visible = visble

    def update(self) -> None:
        super().update()
        if not self._visible:
            return

        titleY = self._y
        rowY = self._y + 1

        if self._title != "":
            self._console.draw_string(
                self._x,
                titleY,
                self._title,
                self._title_fg,
                self._title_bg,
                sys.maxsize
            )

        for index, (key, value) in enumerate(self._data.items()):
            line = (
                self._format_cell(str(key), self._col1Width)
                + "  "
                + self._format_cell(str(value), self._col2Width)
            )
            self._console.draw_string(
                self._x,
                rowY + index,
                line,
                self._fg,
                self._bg,
                self._col1Width + 2 + self._col2Width
            )

    def __init__(
            self,
            console: cui.ConsoleUI,
            x: int,
            y: int,
            data: dict[str, str],
            title: str = "",
            col1Width: int | None = None,
            col2Width: int | None = None,
            fg: cui.Color = cui.Color(255, 255, 255),
            bg: cui.Color = cui.Color(0, 0, 0),
            title_fg: cui.Color | None = None,
            title_bg: cui.Color | None = None,
            visible: bool = True
        ) -> None:
        super().__init__(console)
        self._x = x
        self._y = y
        self._data = data
        self._title = title
        self._visible = visible
        self._fg = fg
        self._bg = bg
        self._title_fg = title_fg if title_fg is not None else fg
        self._title_bg = title_bg if title_bg is not None else bg
        self._autoCol1Width = col1Width is None
        self._autoCol2Width = col2Width is None
        if col1Width is not None:
            self._col1Width = col1Width
        else:
            self._col1Width = 0
        if col2Width is not None:
            self._col2Width = col2Width
        else:
            self._col2Width = 0
        self._refresh_auto_widths()


class ListField(UIElement):

    _visible: bool
    _data: list[str]
    _title: str
    _fg: cui.Color
    _bg: cui.Color
    _title_fg: cui.Color
    _title_bg: cui.Color
    _x: int
    _y: int

    def set(self, data: list[str], title: str | None = None):
        self._data = data
        if title is not None:
            self._title = title

    def add_item(self, item: str) -> None:
        self._data.append(item)

    def remove_item(self, item: str) -> bool:
        try:
            self._data.remove(item)
            return True
        except ValueError:
            return False

    def clear(self) -> None:
        self._data.clear()

    def setVisibility(self, visible: bool):
        self._visible = visible

    def update(self) -> None:
        super().update()
        if not self._visible:
            return

        if self._title != "":
            self._console.draw_string(
                self._x,
                self._y,
                self._title,
                self._title_fg,
                self._title_bg,
                sys.maxsize
            )

        for index, item in enumerate(self._data):
            self._console.draw_string(
                self._x,
                self._y + 1 + index,
                item,
                self._fg,
                self._bg,
                sys.maxsize
            )

    def __init__(
            self,
            console: cui.ConsoleUI,
            x: int,
            y: int,
            data: list[str] | None = None,
            title: str = "",
            fg: cui.Color = cui.Color(255, 255, 255),
            bg: cui.Color = cui.Color(0, 0, 0),
            title_fg: cui.Color | None = None,
            title_bg: cui.Color | None = None,
            visible: bool = True
        ) -> None:
        super().__init__(console)
        self._x = x
        self._y = y
        self._data = data if data is not None else []
        self._title = title
        self._visible = visible
        self._fg = fg
        self._bg = bg
        self._title_fg = title_fg if title_fg is not None else fg
        self._title_bg = title_bg if title_bg is not None else bg


class InputField(UIElement):

    _activeBox: int | None = None #stores the id of which input box is currently active. Only one can be active at a time
    _fg: cui.Color
    _bg: cui.Color
    _x: int
    _y: int
    _boxLength: int
    _onEnter: typing.Callable[[str], None] #called when the user presses enter
    _periodic: typing.Callable[[str], None] | None #called every update when active, with the current input buffer as paramater

    #makes this input box active, and clears the input buffer
    def activate(self):
        InputField._activeBox = self._id
        if self._console.collecting_input():
            self._console.stop_input()
        self._console.get_input(self._x, self._y, self._fg, self._bg, self._boxLength)

    #makes this input box inactive, if it is active
    def deactivate(self):
        if InputField._activeBox == self._id:
            InputField._activeBox = None
            self._console.stop_input()

    def update(self) -> None:
        super().update()

        if InputField._activeBox == self._id:
            if self._periodic is not None:
                self._periodic(self._console.get_live_input_buffer())
            if not self._console.collecting_input():
                InputField._activeBox = None
                self._onEnter(self._console.get_last_input())

    def is_active(self) -> bool:
        return InputField._activeBox == self._id

    def __init__(
            self,
            console: cui.ConsoleUI,
            x: int,
            y: int,
            length: int,
            onEnter: typing.Callable[[str], None],
            periodic: typing.Callable[[str], None] | None = None,
            fg: cui.Color = cui.Color(255, 255, 255),
            bg: cui.Color = cui.Color(0, 0, 0),
        ) -> None:
        super().__init__(console)
        self._x = x
        self._y = y
        self._boxLength = length
        self._onEnter = onEnter
        self._periodic = periodic
        self._fg = fg
        self._bg = bg

        

    
    

        