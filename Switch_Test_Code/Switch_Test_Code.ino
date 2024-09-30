// Author: RICR - Raj Institute of Coding and Robotics
// https://ricr.in
// contact@ricr.in +918889991736

void setup()
{
    pinMode(12, INPUT);
    pinMode(13, INPUT);
    pinMode(A0, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    int a = digitalRead(12);
    int b = digitalRead(13);

    Serial.print(a);
    Serial.print(" - ");
    Serial.println(b);
    digitalWrite(A0, HIGH);
    delay(300);
    digitalWrite(A0, LOW);
    delay(300);
}
