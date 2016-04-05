#include<Servo.h>

Servo esc;
int esc_value = 0;

void setup() 
{
  Serial.begin(9600);
  esc.attach(9);
}

void loop() 
{
  Serial.println(esc_value);
  esc.writeMicroseconds(esc_value);

  if(Serial.available() > 0)
  {
    esc_value = Serial.parseInt();
  }
}
