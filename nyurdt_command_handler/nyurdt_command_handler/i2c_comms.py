"""I2C helper with multiple backend fallbacks.

Tries in order:
 - Adafruit Blinka (busio + board)
 - smbus2 (traditional Linux I2C)
 - Mock (no hardware available)

Exports send_HEX(hexcode) and get_data_dictionary().
"""

import time
import sys
from typing import Dict

teensy_address = 0x24
_backend = None
_i2c = None
_smbus = None

# Try Adafruit Blinka (busio + board)
try:
	import busio  # type: ignore
	import board  # type: ignore

	try:
		# frequency param name is different across versions; use keyword if supported
		try:
			_i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
		except TypeError:
			_i2c = busio.I2C(board.SCL, board.SDA)
		_backend = 'busio'
	except Exception:
		# busio imported but board likely doesn't provide SCL/SDA on this platform
		_i2c = None
		_backend = None
except Exception:
	_backend = None

# Try smbus2 as a fallback
if _backend is None:
	try:
		from smbus2 import SMBus  # type: ignore
		# Common I2C bus on RPi is 1, on other systems it may vary
		try:
			_smbus = SMBus(1)
			_backend = 'smbus'
		except Exception:
			_smbus = None
			_backend = None
	except Exception:
		_backend = None

if _backend is None:
	# No hardware backend available — use a mock that raises useful errors
	class _MockI2C:
		def writeto(self, addr, buf):
			raise RuntimeError('No I2C backend available on this system')

		def readfrom_into(self, addr, buf):
			raise RuntimeError('No I2C backend available on this system')

	_i2c = _MockI2C()
	_backend = 'mock'


def send_HEX(hexcode: int) -> int:
	"""Send a single-byte command to the Teensy over I2C.

	Returns 0 on success, -1 on failure.
	"""
	try:
		if _backend == 'busio':
			buff = bytearray(1)
			buff[0] = hexcode & 0xFF
			_i2c.writeto(teensy_address, buff)
			return 0
		elif _backend == 'smbus':
			# SMBus supports write_byte
			_smbus.write_byte(teensy_address, hexcode & 0xFF)  # type: ignore
			return 0
		else:
			raise RuntimeError('No I2C backend available')
	except Exception as e:
		print(f"send_HEX failed ({_backend}): {e}", file=sys.stderr)
		return -1


def get_data_dictionary() -> Dict[str, int]:
	"""Read 3 bytes from Teensy and return a dict with keys Weight, Position, Encoder Angle."""
	try:
		if _backend == 'busio':
			data = bytearray(3)
			_i2c.readfrom_into(teensy_address, data)
			return {"Weight": int(data[0]), "Position": int(data[1]), "Encoder Angle": int(data[2])}
		elif _backend == 'smbus':
			# read_i2c_block_data returns a list of ints
			arr = _smbus.read_i2c_block_data(teensy_address, 0, 3)  # type: ignore
			return {"Weight": int(arr[0]), "Position": int(arr[1]), "Encoder Angle": int(arr[2])}
		else:
			raise RuntimeError('No I2C backend available')
	except Exception as e:
		print(f"get_data_dictionary failed ({_backend}): {e}", file=sys.stderr)
		# Return a safe default rather than crashing
		return {"Weight": 0, "Position": 0, "Encoder Angle": 0}


if __name__ == '__main__':
	print('I2C backend in use:', _backend)
	print(get_data_dictionary())