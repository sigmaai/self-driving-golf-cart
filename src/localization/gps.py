import gpsd
import os
import time

class GPS():

    def __init__(self, port):

        os.system("sudo systemctl stop gpsd.socket")
        os.system("sudo systemctl disable gpsd.socket")
        os.system("sudo gpsd /dev/ttyUSB{} -F /var/run/gpsd.sock".format(port))
        time.sleep(3)
        print("GPS initiated")

    def query_gps_location(self):
        gpsd.connect()
        packet = gpsd.get_current()
        return packet.position()
