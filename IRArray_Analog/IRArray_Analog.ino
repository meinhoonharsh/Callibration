// Author: RICR - Raj Institute of Coding and Robotics
// https://ricr.in
// contact@ricr.in +918889991736

#include <QTRSensors.h>

QTRSensors qtr;  // Use the QTRSensors class
const uint8_t SensorCount = 5;  // 5 sensors in the array
uint16_t sensorValues[SensorCount];  // Array to hold sensor readings

void setup() {
  Serial.begin(9600);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A3, A4,A5,A6,A7}, SensorCount);

  Serial.println("Starting Calibration...");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  
  Serial.println("Calibration Complete...");
}

void loop() {
  qtr.read(sensorValues);
int position = qtr.readLineWhite(sensorValues);
  Serial.println(position);
  delay(500);  // Wait for half a second between readings
}
