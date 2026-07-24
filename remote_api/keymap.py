"""GLFW key name -> key code map.

This mirrors the table in src/remote/RemoteControlServer.cpp::resolveKeyCode so
that the Python frontend can validate / document key names. The authoritative
resolution happens server-side; this module is provided for client-side
validation and convenience.

Codes follow GLFW 3.3 (https://www.glfw.org/docs/latest/group__keys.html).
Single-character names "A".."Z" and "0".."9" map to their ASCII code points,
matching GLFW. Names here are stored upper-cased; lookups are case-insensitive.
"""

from __future__ import annotations

from typing import Dict, Union

# Printable / named keys. Matches the C++ table.
_NAMED: Dict[str, int] = {
    "SPACE": 32, "APOSTROPHE": 39, "COMMA": 44, "MINUS": 45, "PERIOD": 46,
    "SLASH": 47, "SEMICOLON": 59, "EQUAL": 61, "LEFT_BRACKET": 91,
    "BACKSLASH": 92, "RIGHT_BRACKET": 93, "GRAVE_ACCENT": 96,
    "WORLD_1": 161, "WORLD_2": 162,
    "ESCAPE": 256, "ENTER": 257, "TAB": 258, "BACKSPACE": 259,
    "INSERT": 260, "DELETE": 261, "RIGHT": 262, "LEFT": 263, "DOWN": 264,
    "UP": 265, "PAGE_UP": 266, "PAGE_DOWN": 267, "HOME": 268, "END": 269,
    "CAPS_LOCK": 280, "SCROLL_LOCK": 281, "NUM_LOCK": 282, "PRINT_SCREEN": 283,
    "PAUSE": 284,
    "F1": 290, "F2": 291, "F3": 292, "F4": 293, "F5": 294, "F6": 295,
    "F7": 296, "F8": 297, "F9": 298, "F10": 299, "F11": 300, "F12": 301,
    "F13": 302, "F14": 303, "F15": 304, "F16": 305, "F17": 306, "F18": 307,
    "F19": 308, "F20": 309, "F21": 310, "F22": 311, "F23": 312, "F24": 313,
    "F25": 314,
    "KP_0": 320, "KP_1": 321, "KP_2": 322, "KP_3": 323, "KP_4": 324,
    "KP_5": 325, "KP_6": 326, "KP_7": 327, "KP_8": 328, "KP_9": 329,
    "KP_DECIMAL": 330, "KP_DIVIDE": 331, "KP_MULTIPLY": 332, "KP_SUBTRACT": 333,
    "KP_ADD": 334, "KP_ENTER": 335, "KP_EQUAL": 336,
    "LEFT_SHIFT": 340, "LEFT_CONTROL": 341, "LEFT_ALT": 342, "LEFT_SUPER": 343,
    "RIGHT_SHIFT": 344, "RIGHT_CONTROL": 345, "RIGHT_ALT": 346,
    "RIGHT_SUPER": 347, "MENU": 348,
}


def resolve_key(key: Union[str, int]) -> int:
    """Return the GLFW key code for a name or integer code, or -1 if unknown."""
    if isinstance(key, int):
        return key
    if not isinstance(key, str):
        return -1
    if len(key) == 1:
        c = key[0]
        if "a" <= c <= "z":
            c = chr(ord(c) - 32)
        if ("A" <= c <= "Z") or ("0" <= c <= "9"):
            return ord(c)
    return _NAMED.get(key.upper(), -1)


def is_known(key: Union[str, int]) -> bool:
    return resolve_key(key) >= 0
