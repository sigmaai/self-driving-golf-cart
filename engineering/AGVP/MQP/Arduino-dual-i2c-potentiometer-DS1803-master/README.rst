DS1803 Arduino Library (*Dual Digital Addressable i2c Potentiometer*)
=====================================================================

::

   /* Example sketch for controlling the DS1803 ic with Arduino
      Written by Federico and Riccardo Galli
      http://www.sideralis.org */
   
   #include <DS1803.h>
   #include <Wire.h>
   
   /* Create an instance of the class
      0x28 is the address of the DS1803
      with three pin grounded. */
   DS1803 pot(0x28);
   
   void setup() {  
     //Begin serial connection at 9600 baud
     Serial.begin(9600) ;
   } 
   
   void loop() {
     //Set the first potentiometer at 200 
     pot.setPot(200,0);
     //Set the second potentiometer at 255
     pot.setPot(255,1);
     
     //Read the values saved on the DS1803
     int8_t *value=pot.getValue();
     Serial.println(value[0],DEC);
     Serial.println(value[1],DEC);
   }
