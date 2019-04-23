#include <Arduino.h>

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(3000);
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("test");
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("test2");
}