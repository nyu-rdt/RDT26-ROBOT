"""I2C helpers for talking to the robot Teensy from the Jetson."""

import os
import sys
from typing import Dict

teensy_address = int(os.environ.get("NYURDT_TEENSY_I2C_ADDRESS", "0x08"), 0)
_bus_num = int(os.environ.get("NYURDT_TEENSY_I2C_BUS", "7"))
_backend = None
_i2c = None
_smbus = None

# // Packet layout (18 bytes):
# //   [0-7]   motor_currents[8]  uint8, 0-255 = 0-20A   (CURRENT_SENSE_ENABLED)
# //   [8]     left_encoder       uint8, 0-255 = 0-360°  (ROTARY_ENCODERS_ENABLED)
# //   [9]     right_encoder      uint8, 0-255 = 0-360°  (ROTARY_ENCODERS_ENABLED)
# //   [10-13] load_cells[4]      uint8 each             (LOAD_CELLS_ENABLED)
# //   [14]    string_pot         uint8 (conveyor pos)   (STRING_POT_ENABLED)
# //   [15]    gate_pos           uint8                  (GATE_POS_ENABLED)
# //   [16]    flags              uint8 (bit0=relay, bit1=3s_low, bit2=6s_low)
# //   [17]    fixes_attempted    uint8                  (no driver yet)
def _default_sensor_dict() -> Dict[str, int | bool]:
    return {
        "motor_currents": [0] * 8,
        "left_encoder": 0,
        "right_encoder": 0,
        "load_cells": [0] * 4,
        "string_pot": 0,
        "gate_pos": 0,
        "flags": 0,
        "fixes_attempted": 0,
        
        # 2025 variables
        # "relay_status": 0,
        # "left_speed": 0,
        # "right_speed": 0,
        # "at_locomotion_position": False,
        # "at_excavation_position": False,
        # "at_construction_position": False,
        # "belt_pivot_angle": 0,
    }


def _to_signed_byte(value: int) -> int:
    return value - 256 if value > 127 else value


try:
    from smbus2 import SMBus  # type: ignore

    _smbus = SMBus(_bus_num)
    _backend = "smbus"
except Exception:
    _smbus = None
    _backend = None

if _backend is None:
    try:
        import busio  # type: ignore
        import board  # type: ignore

        try:
            try:
                _i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            except TypeError:
                _i2c = busio.I2C(board.SCL, board.SDA)
            _backend = "busio"
        except Exception:
            _i2c = None
            _backend = None
    except Exception:
        _backend = None


def send_HEX(hexcode: int) -> int:
    try:
        if _backend == "smbus":
            _smbus.write_byte(teensy_address, hexcode & 0xFF)  # type: ignore[union-attr]
            return 0
        if _backend == "busio":
            _i2c.writeto(teensy_address, bytes([hexcode & 0xFF]))
            return 0
        raise RuntimeError("No I2C backend available")
    except Exception as exc:
        print(f"send_HEX failed ({_backend}, bus={_bus_num}, addr={teensy_address:#04x}): {exc}", file=sys.stderr)
        return -1


# Directly from child.cpp on teensy
# // Packet layout (18 bytes):
# //   [0-7]   motor_currents[8]  uint8, 0-255 = 0-20A   (CURRENT_SENSE_ENABLED)
# //   [8]     left_encoder       uint8, 0-255 = 0-360°  (ROTARY_ENCODERS_ENABLED)
# //   [9]     right_encoder      uint8, 0-255 = 0-360°  (ROTARY_ENCODERS_ENABLED)
# //   [10-13] load_cells[4]      uint8 each             (LOAD_CELLS_ENABLED)
# //   [14]    string_pot         uint8 (conveyor pos)   (STRING_POT_ENABLED)
# //   [15]    gate_pos           uint8                  (GATE_POS_ENABLED)
# //   [16]    flags              uint8 (bit0=relay, bit1=3s_low, bit2=6s_low)
# //   [17]    fixes_attempted    uint8                  (no driver yet)
def get_data_dictionary() -> Dict[str, int | bool]:
    result = _default_sensor_dict()

    try:
        if _backend == "smbus":
            arr = _smbus.read_i2c_block_data(teensy_address, 0, 3)  # type: ignore[union-attr]
        elif _backend == "busio":
            data = bytearray(3)
            while not _i2c.try_lock():  # type: ignore[union-attr]
                pass
            try:
                _i2c.readfrom_into(teensy_address, data)  # type: ignore[union-attr]
            finally:
                _i2c.unlock()  # type: ignore[union-attr]
            arr = list(data)
        else:
            raise RuntimeError("No I2C backend available")

        
        result["motor_currents"] = arr[0:8]
        result["left_encoder"] = arr[8]
        result["right_encoder"] = arr[9]
        result["load_cells"] = arr[10:14]
        result["string_pot"] = arr[14]
        result["gate_pos"] = arr[15]
        result["flags"] = arr[16]
        result["fixes_attempted"] = arr[17]
        
        
        # result["relay_status"] = int(arr[0])
        # result["left_speed"] = _to_signed_byte(int(arr[1]))
        # result["right_speed"] = _to_signed_byte(int(arr[2]))
        
        
        
        return result
    except Exception as exc:
        print(f"get_data_dictionary failed ({_backend}, bus={_bus_num}, addr={teensy_address:#04x}): {exc}", file=sys.stderr)
        return result
