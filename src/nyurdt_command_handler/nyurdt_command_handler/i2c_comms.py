import busio
import board
import time

# Initialize I2C. 100kHz or 400kHz are standard. 
# 115200 is a Serial baud rate, not a standard I2C clock, but Blinka usually rounds it.
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

teensy_address = 0x08 

def send_HEX(hexcode):
    """Sends a single byte to the Teensy with error protection."""
    if not i2c.try_lock():
        return -1
    try:
        buff = bytearray([hexcode & 0xFF]) # Ensure it's a single byte
        i2c.writeto(teensy_address, buff)
        return 0
    except OSError as e:
        print(f"I2C Write Error: {e}")
        return -1
    finally:
        i2c.unlock()

def get_data_dictionary():
    """Reads two bytes from Teensy and combines them into a 16-bit angle."""
    if not i2c.try_lock():
        return None
        
    try:
        # It's much faster to read both bytes at once than in two separate loops
        read_buffer = bytearray(2) 
        
        # We try to read 2 bytes from the Teensy
        i2c.readfrom_into(teensy_address, read_buffer)
        
        # Combine bytes: read_buffer[0] is High, read_buffer[1] is Low
        # No need for bin() strings, just use bit shifting
        pivot_angle = (read_buffer[0] << 8) | read_buffer[1]
        
        return {"Pivot angle": pivot_angle}

    except OSError as e:
        print(f"I2C Read Error: {e}")
        return None
    finally:
        i2c.unlock()