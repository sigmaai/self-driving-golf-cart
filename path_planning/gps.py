import gpsd
import os

class GPS():
	
	def __init__(self):
    
		os.system("sudo systemctl stop gpsd.socket")
		os.system("sudo systemctl disable gpsd.socket")
		os.system("sudo gpsd /dev/ttyUSB0 -F /var/run/gpsd.sock")
		
		print("GPS initiated")
		
	
	def query_gps_location(self):
		gpsd.connect()
		packet = gpsd.get_current()
		return packet.position()
