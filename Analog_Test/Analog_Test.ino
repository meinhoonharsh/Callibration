// Author: RICR - Raj Institute of Coding and Robotics
// https://ricr.in
// contact@ricr.in +918889991736

void setup() {
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.print(analogRead(A3));
  Serial.print("\t");
  Serial.print(analogRead(A4));
  Serial.print("\t");
  Serial.print(analogRead(A5));
  Serial.print("\t");
  Serial.print(analogRead(A6));
  Serial.print("\t");
  Serial.print(analogRead(A7));
  Serial.print("\t");
  Serial.println();
}
