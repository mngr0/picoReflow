import config
import time
import board
import digitalio
import busio
import adafruit_max31855
from busio import SPI
from adafruit_bus_device.spi_device import SPIDevice

cs1=digitalio.DigitalInOut(config.gpio_sensor_cs1)
cs1.direction = digitalio.Direction.OUTPUT
cs1.value = True

cs2=digitalio.DigitalInOut(config.gpio_sensor_cs2)
cs2.direction = digitalio.Direction.OUTPUT
cs2.value = True


spi = board.SPI()


#while not spi.try_lock():
#	pass
print("try")

#spi.configure(baudrate=50000, phase=0, polarity=0)



t1 = adafruit_max31855.MAX31855(spi, cs1)

t2 = adafruit_max31855.MAX31855(spi, cs2)
sd = SPIDevice(spi,cs1)

#while 1:
#	cs1.value = False
#	result = bytearray(4)
#	with sd as diocane:
#		diocane.readinto(result)
#	cs1.value = True
#	print(result)
#	time.sleep(1)

print("try2")
result =  bytearray(4)
while 1:
	print("try3")
	#print(t1.temperature)
	print(t2.temperature)
	time.sleep(1)
