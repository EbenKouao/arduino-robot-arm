/******************************************************************************
  Author: Smartbuilds.io
  YouTube: https://www.youtube.com/channel/UCGxwyXJWEarxh2XWqvygiIg
  Fork your own version: https://github.com/EbenKouao/arduino-robotic-arm
  Date: 28/12/2020
  Description: Test the Single Servo Motor

  Note: To Avoid excess Arduino Power consumption.
  Power with external power sourcee if you would would like to add more than 2 servos.
******************************************************************************/

#include <Servo.h>

Servo myservo_1;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

void setup() {
  myservo_1.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo_1.write(90);
  delay(5000);
}

void loop() {

  for (pos = 0; pos <= 80; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo_1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 80; pos >= 1; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo_1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }


}
