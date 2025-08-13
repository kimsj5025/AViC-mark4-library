#include "Arduino.h"
#include "AViC.h"

AViC avic;

void setup(){

  Serial.begin(115200);
  avic.initialize(true);

}

void loop(){

  avic.readAcceleData(); //가속도 데이터를 읽어옵니다.
  avic.readBaroData(); //기압계 데이터를 읽어옵니다.
  avic.printSensorData(); //시리얼 모니터로 읽어온 데이터를 출력합니다.
  avic.ledSW(5); //LED를 조정합니다. 
  delay(1000);

}