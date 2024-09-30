// Author: RICR - Raj Institute of Coding and Robotics
// https://ricr.in
// contact@ricr.in +918889991736

// Download below two library on your arduino IDE you will get this from google
#include <SparkFun_TB6612.h>  // This libaray is for sparkfun motor driver  

// Initialization of the motors
#define AIN1 5
#define BIN1 6
#define AIN2 4
#define BIN2 7
#define PWMA 3
#define PWMB 11
#define STBY 6
const int offsetA = 1;
const int offsetB = 1;


// Creating the instance of class for motor and senors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{   



}
void loop() {
  forward(motor1, motor2, 30);
  delay(500);
  forward(motor1, motor2, 100);
  delay(500);                              // For graduly increasing speed so that bot dose not get direct high speed at the start
  forward(motor1, motor2, 200);
  delay(500);
  brake(motor1, motor2);
  delay(1000);
}
