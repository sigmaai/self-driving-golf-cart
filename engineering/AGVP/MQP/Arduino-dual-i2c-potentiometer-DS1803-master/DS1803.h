/*
   Library for controlling the DS1803 ic with Arduino
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


#ifndef DS1803_h
#define DS1803_h
#include "WProgram.h"
#include <Wire.h>

#define WIPER_0 0xA9
#define WIPER_1 0xAA
#define WIPER_01 0xAF


class DS1803
{
  int8_t addr;
  public:
    DS1803(int8_t addr);
    void setPot(int value, int wiper);
    int8_t *getValue();
  //private:
    //;
};

#endif
