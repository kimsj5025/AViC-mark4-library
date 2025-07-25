#include<AViC.h>
#include "Arduino.h"

AViC avic;

void setup(){
  avic.initialize(true);
}

void loop(){
  avic.ledSW(50);
  delay(1000);
}