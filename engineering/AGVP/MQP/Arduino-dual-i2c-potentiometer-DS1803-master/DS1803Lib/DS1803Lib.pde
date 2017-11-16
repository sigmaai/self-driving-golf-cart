/* Example sketch for controlling the DS1803 ic with Arduino
   Tested with arduino-0022
   22/02/2010 Release 0.1

   Copyright and License
   ---------------------
   DS1803 Arduino Library

   Copyright 2010 Federico Emanuele Galli <fede@sideralis.org> [http://www.sideralis.org]
   Copyright 2010 Riccardo Attilio Galli <riccardo@sideralis.org> [http://www.sideralis.org]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

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


