#include "Adafruit_VL53L0X.h"
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // 아두이노의 10번 핀(RX)과 11번 핀(TX)를 HC-06 모듈과 연결

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600); // 블루투스 통신 시작
  
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.println(uint16_t(measure.RangeMilliMeter));
    BTSerial.println(uint16_t(measure.RangeMilliMeter));  // 16진수로 거리 값을 송신
  }

    
  delay(10);
}
