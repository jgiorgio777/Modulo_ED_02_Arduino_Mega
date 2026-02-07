#include <Arduino.h>

int led1 = 13;
char var1;

void setup()
{
  Serial.begin(9600);
  pinMode(led1, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0)
  {
    var1 = Serial.read();

    if (var1 == 'a')
    {
      digitalWrite(led1, HIGH);
    }
    else
    {
      digitalWrite(led1, LOW);
    }
  }
}
