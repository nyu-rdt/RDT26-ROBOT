import os
import sys

# cass i2c_comms():
teensy_address = int(os.environ.get("NYURDT_TEENSY_I2C_ADDRESS", "0x08"), 0)
_bus_num = int(os.environ.get("NYURDT_TEENSY_I2C_BUS", "7"))
_backend = None
_i2c = None
_smbus = None

# initialize I2C backend (try smbus2 first, then busio)
def i2c_init():
    global _backend, _smbus, _i2c
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
                _i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            except TypeError:
                _i2c = busio.I2C(board.SCL, board.SDA)
            _backend = "busio"
        except Exception:
            _i2c = None
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