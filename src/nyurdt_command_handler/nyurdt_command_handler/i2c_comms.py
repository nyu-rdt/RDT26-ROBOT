import busio
import board
import time


print(dir(board))
i2c = busio.I2C(board.SCL, board.SDA, 100000)
# busio.I2C.configure(500000);

i2c.scan()

#time.sleep(1)

teensy_address = 0x42 # edit this based off teensy address 

def send_HEX(hexcode):
	buff = bytearray(1)
	buff[0] = hexcode
	i2c.writeto(teensy_address, buff)
	return 0


def get_data_dictionary():
	while not i2c.try_lock():
		pass; # will keep going until the i2c lock has been attained
	
	pivot_lower = bytearray(1)
	pivot_higher = bytearray(1)
	while pivot_higher == bytearray(1):
		i2c.readfrom_into(teensy_address, pivot_higher)
		print("Looking for high")
		time.sleep(1)
		
	print(pivot_higher)
	while pivot_lower == bytearray(1):
		i2c.readfrom_into(teensy_address, pivot_lower)
		print("Looking for low")
		time.sleep(1)

	print("out of loop")
	pivot_angle = (int(bin(pivot_higher[0]), 2) << 8) | int(bin(pivot_lower[0]), 2)
	Dict = {"Pivot angle" : pivot_angle};
	
	i2c.unlock() # safely unlocks i2c
	return Dict
