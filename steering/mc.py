# motor controller -- send serial steering commands to Arduino
# By Neil Nie & Michael Meng
# Jan, 2018
# Copyright (c), All rights reserved

import serial


class MC:

    def __init__(self, device_id=0):

        self.ser = serial.Serial('/dev/ttyACM%d' % device_id, 115200)


    def turn(self, angle, precision=2):
        ret =  self.ser.write(('b'+str(round(angle,precision)).zfill(6)+'e').encode())

        if (ret == 8):
            pass
        else:
            print('Failed to send turning angle %f' % angle)

     
    def pos(self):
        return float(self.ser.readline().strip())
