'''
action_map.py

Contains enum mapping for robot actions
'''

from enum import Enum


class ActionMap(Enum):
    EMERGENCY_STOP = b"\x01"

    LOCO_STOP = b"\x10"
    LOCO_FORWARD = b"\x20"
    LOCO_BACKWARD = b"\x30"
    LOCO_LEFT = b"\x40"
    LOCO_RIGHT = b"\x50"

    PIVOT_ZERO = b"\x60"
    PIVOT_TO_LOCO = b"\x61"
    PIVOT_TO_CONST = b"\x62"
    PIVOT_TO_EXCA = b"\x63"

    PIVOT_STOP = b"\x64"
    PIVOT_BELT_OUT = b"\x65"
    PIVOT_BELT_IN= b"\x66"

    PIVOT_DEPO_STOP = b"\x67"
    PIVOT_DEPO_UP = b"\x68"
    PIVOT_DEPO_DOWN = b"\x69"

    BELT_COLLECT = b"\x70"
    BELT_DUMP = b"\x71"
    BELT_STOP = b"\x72"

    REQUEST_DATA = b"\x80"
    START_AUTO = b"\x81"

    HINGE_UP = b"\x90"
    HINGE_DOWN = b"\x91"