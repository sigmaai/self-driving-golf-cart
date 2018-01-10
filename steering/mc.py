import serial

class MC:

    def __init__(self,device_id=0):
        try:
	    self.ser = serial.Serial('/dev/ttyUSB%d' % device_id,115200)
	except Exception as e:
	    print(str(e));

    def turn(self,angle,precision=3):
    	ret =  self.ser.write('b'+str(round(angle,precision)).zfill(5)+'e')
	if (ret == precision+4):
	  pass  
          #print('Successfully send turning angle %f' % angle)
        else:
            print('Failed to send turning angle %f' % angle)
    	return ret


