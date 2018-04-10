# speed controller -- send serial steering commands to Arduino
# By Neil Nie & Michael Meng
# Jan, 2018
# Copyright (c), All rights reserved

import serial


class SC:

    def __init__(self, device_id=0):

        self.ser = serial.Serial('/dev/ttyACM%d' % device_id, 115200)


    def drive(self, angle, precision=3):
        ret =  self.ser.write(('b'+str(round(angle,precision)).zfill(6)+'a').encode())

        if (ret == precision+5):
            pass
        else:
            print('Failed to send speed %f' % angle)
