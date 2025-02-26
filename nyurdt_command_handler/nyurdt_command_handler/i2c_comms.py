import busio
import board
import time

i2c = busio.I2C(board.SCL,board.SDA,100000)
teensy_address = 0x24

def send_HEX(hexcode):
	buff = bytearray(1)
	buff[0] = hexcode
	i2c.writeto(teensy_address, buff)
	return 0


def get_data_dictionary():
	#Data in order of being sent: Weight Position, Encoder Angler
	#Position: 1 is Locomotion, 2 is Excavation
	data = bytearray(3)
	i2c.readfrom_into(teensy_address, data)
	dictionary = {"Weight": data[0], "Position": data[1], "Encoder Angle": data[2]}
	return dictionary

print(get_data_dictionary())