import serial

class MC:

    def __init__(self,device_id=0):
        try:
	    self.ser = serial.Serial('/dev/ttyUSB%d' % device_id,9600)
	except Exception as e:
	    print(str(e));

    def turn(self,angle):
    	if self.ser.write(str(round(angle,4))) > 0:
	    #print('Successfully send turning angle %f' % angle)
	    return 1;
        else:
            print('Failed to send turning angle %f' % angle)
    	    return 0;


