// inslude the SPI library:
#include <SPI.h>

// set pin 10 as the slave select for the digital pot:
const int slave_Select_Pin  = 10;
int       level1            = 0;
int       level2            = 0;

void setup() {

  // set the slaveSelectPin as an output:
  pinMode (slave_Select_Pin, OUTPUT);
  digitalWrite(slave_Select_Pin, LOW);
  Serial.begin(9600);

  // initialize SPI:
  SPI.begin();
  MSP42010PotWrite(slave_Select_Pin, B00010001, level1);
  MSP42010PotWrite(slave_Select_Pin, B00010010, level2);
}

void loop() {

  for (int i = 128; i <= 256; i++) {
    MSP42010PotWrite(slave_Select_Pin, B00010001, i);
    MSP42010PotWrite(slave_Select_Pin, B00010010, i);
    delay(50);
  }

  for (int i = 256; i >= 128;  i--) {
    MSP42010PotWrite(slave_Select_Pin, B00010001, i);
    MSP42010PotWrite(slave_Select_Pin, B00010010, i);
    delay(50);
  }
}

void MSP42010PotWrite(int slaveSelectPin, byte address, int value) {
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin, LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin, HIGH);
}


