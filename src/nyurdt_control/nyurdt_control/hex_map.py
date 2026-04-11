from enum import Enum

#Miscellaneous Hex Values
class OtherHex(Enum):
    EMERGENCY_STOP = b"\x01" # xbox main button
    REQUEST_DATA = b"\x80" # exit
    SWITCH_AUTONOMOUS = b"\x81" # window

#Locomotion Hex Values
class LocomotionHex(Enum):
    LOCOMOTION_STOP = b"\x10"
    LOCOMOTION_FORWARD_25 = b"\x20"
    LOCOMOTION_FORWARD_50 = b"\x21"
    LOCOMOTION_FORWARD_75 = b"\x22"
    LOCOMOTION_FORWARD_100 = b"\x23"
    LOCOMOTION_BACKWARD_25 = b"\x30"
    LOCOMOTION_BACKWARD_50 = b"\x31"
    LOCOMOTION_BACKWARD_75 = b"\x32"
    LOCOMOTION_BACKWARD_100 = b"\x33"
    LOCOMOTION_LEFT_25 = b"\x40"
    LOCOMOTION_LEFT_50 = b"\x41"
    LOCOMOTION_LEFT_75 = b"\x42"
    LOCOMOTION_LEFT_100 = b"\x43"
    LOCOMOTION_RIGHT_25 = b"\x50"
    LOCOMOTION_RIGHT_50 = b"\x51"
    LOCOMOTION_RIGHT_75 = b"\x52"
    LOCOMOTION_RIGHT_100 = b"\x53"

#Excavation Hex Values
class ExcavationHex(Enum):
    ZERO_EXCAVATION = b"\x60" # A button
    LOCOMOTION_POSITION = b"\x61" # B button
    EXCAVATION_POSITION = b"\x62" # Y button 
    BELT_STOP = b"\x63" # LB
    BELT_OUTWARD = b"\x64" # RB
    BELT_INWARD = b"\x65" # menu

class DepositionHex(Enum):
    BIN_OPEN = b"\x70" # (dpad x -1)
    BIN_CLOSE= b"\x71" # dpad X +1
    VIBRATE_BIN = b"\x72" # dpad y -1 
    VIBRATE_STOP = b"\x73" # dpad y +1
    
