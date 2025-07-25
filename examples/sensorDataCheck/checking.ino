#include "Arduino.h"
#include "AViC.h"

AViC avic;

void setup(){
  Serial.begin(115200);
  avic.initialize(true);
}

void loop(){
  
  avic.readAcceleData();
  avic.readBaroData();
  avic.printSensorData();
  avic.ledSW(5);

  delay(1000);
}