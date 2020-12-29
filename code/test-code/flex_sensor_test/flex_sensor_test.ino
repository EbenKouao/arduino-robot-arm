/******************************************************************************
  Author: Smartbuilds.io
  YouTube: https://www.youtube.com/channel/UCGxwyXJWEarxh2XWqvygiIg
  Fork your own version: https://github.com/EbenKouao/arduino-robotic-arm
  Date: 28/12/2020
  Description: Test the Flex Sensor
******************************************************************************/
const int flexPin = A3; // Pin connected to voltage divider output

void setup()
{
  Serial.begin(4800);
  pinMode(flexPin, INPUT);
}

void loop()
{
  // Read the ADC, and calculate voltage and resistance from it
  int flexSensor = analogRead(flexPin);

  Serial.println(flexSensor);

  delay(50);
}
