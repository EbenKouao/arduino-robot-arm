/******************************************************************************
  Author: Smartbuilds.io
  YouTube: https://www.youtube.com/channel/UCGxwyXJWEarxh2XWqvygiIg
  Fork your own version: https://github.com/EbenKouao/arduino-robotic-arm
  Date: 28/12/2020
  Robot Arm
  Version 1.0
  Creator: smartbuilds.io
  Description: Robotic Arm Mark II - Servo Motor

  To use the module connect it to your Arduino as follows:

  PCA9685...........Uno/Nano
  GND...............GND
  OE................N/A
  SCL...............A5
  SDA...............A4
  VCC...............5V

******************************************************************************/

/* Include the HCPCA9685 library */
#include "HCPCA9685.h"

/* I2C slave address for the device/module. For the HCMODU0097 the default I2C address
   is 0x40 */
#define  I2CAdd 0x40

/* Create an instance of the library */
HCPCA9685 HCPCA9685(I2CAdd);

char state = 0; // Changes value from ASCII to char
int response_time = 5;

//initial parking position of the motor
const int servo_joint_L_parking_pos = 60;
const int servo_joint_R_parking_pos = 60;
const int servo_joint_1_parking_pos = 70;
const int servo_joint_2_parking_pos = 47;
const int servo_joint_3_parking_pos = 63;
const int servo_joint_4_parking_pos = 63;

//Degree of robot serovo sensitivity - Intervals
int servo_joint_L_pos_increment = 15;
int servo_joint_R_pos_increment = 15;
int servo_joint_1_pos_increment = 15;
int servo_joint_2_pos_increment = 15;
int servo_joint_3_pos_increment = 15;
int servo_joint_4_pos_increment = 15;


//Keep track of the current value of the motor positons
int servo_joint_L_parking_pos_i = servo_joint_L_parking_pos;
int servo_joint_R_parking_pos_i = servo_joint_R_parking_pos;
int servo_joint_1_parking_pos_i = servo_joint_1_parking_pos;
int servo_joint_2_parking_pos_i = servo_joint_2_parking_pos;
int servo_joint_3_parking_pos_i = servo_joint_3_parking_pos;
int servo_joint_4_parking_pos_i = servo_joint_4_parking_pos;


//Minimum and maximum angle of servo motor
int servo_joint_L_min_pos = 10;
int servo_joint_L_max_pos = 180;

int servo_joint_R_min_pos = 10;
int servo_joint_R_max_pos = 180;

int servo_joint_1_min_pos = 10;
int servo_joint_1_max_pos = 180;

int servo_joint_2_min_pos = 10;
int servo_joint_2_max_pos = 180;

int servo_joint_3_min_pos = 10;
int servo_joint_3_max_pos = 180;

int servo_joint_4_min_pos = 10;
int servo_joint_4_max_pos = 180;

int servo_L_pos = 0;
int servo_R_pos = 0;
int servo_joint_1_pos = 0;
int servo_joint_2_pos = 0;
int servo_joint_3_pos = 0;
int servo_joint_4_pos = 0;

unsigned int pos;

// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 3;
const int stepsPerRevolution = 200;
int stepDelay = 200;

void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  /* Initialise the library and set it to 'servo mode' */
  HCPCA9685.Init(SERVO_MODE);

  /* Wake the device up */
  HCPCA9685.Sleep(false);

  Serial.begin(4800); // Default communication rate of the Bluetooth module

  setupRobot();
  delay(3000);

}


void loop()
{

  if (Serial.available() > 0) { // Checks whether data is comming from the serial port

    state = Serial.read(); // Reads the data from the serial port
    Serial.print(state); // Prints out the value sent


    //Move Base Left Motor
    if (state == 'S') {

      baseRotateLeft();
      //The other stepper motor
      delay(response_time);

    }

    //Move Base Right Motor
    if (state == 'O') {

      baseRotateRight();
      //The other stepper motor
      delay(response_time);

    }

    //Move Claw Motor Downwards
    if (state == 'c') {

      if (servo_joint_L_parking_pos_i < servo_joint_L_max_pos) {
        HCPCA9685.Servo(0, servo_joint_L_parking_pos_i);
        HCPCA9685.Servo(1, (servo_joint_L_max_pos - servo_joint_L_parking_pos_i));

        delay(response_time);
        Serial.println(servo_joint_L_parking_pos_i);

        servo_joint_L_parking_pos_i = servo_joint_L_parking_pos_i + servo_joint_L_pos_increment;
        servo_joint_R_parking_pos_i = servo_joint_L_max_pos - servo_joint_L_parking_pos_i;

      }
    }

    //Move Claw Motor Upwards
    if (state == 'C') {

      if (servo_joint_L_parking_pos_i > servo_joint_L_min_pos) {
        HCPCA9685.Servo(0, servo_joint_L_parking_pos_i);
        HCPCA9685.Servo(1, (servo_joint_L_max_pos - servo_joint_L_parking_pos_i));

        delay(response_time);
        Serial.println(servo_joint_L_parking_pos_i);


        servo_joint_L_parking_pos_i = servo_joint_L_parking_pos_i - servo_joint_L_pos_increment;
        servo_joint_R_parking_pos_i = servo_joint_L_max_pos - servo_joint_L_parking_pos_i;

      }
    }

    //Move Claw Motor Downwards
    if (state == 'p') {

      if (servo_joint_1_parking_pos_i < servo_joint_1_max_pos) {
        HCPCA9685.Servo(2, servo_joint_1_parking_pos_i);
        delay(response_time);
        Serial.println(servo_joint_1_parking_pos_i);

        servo_joint_1_parking_pos_i = servo_joint_1_parking_pos_i + servo_joint_1_pos_increment;

      }
    }

    //Move Claw Motor Upwards
    if (state == 'P') {

      if (servo_joint_1_parking_pos_i > servo_joint_1_min_pos) {
        HCPCA9685.Servo(2, servo_joint_1_parking_pos_i);
        delay(response_time);
        Serial.println(servo_joint_1_parking_pos_i);

        servo_joint_1_parking_pos_i = servo_joint_1_parking_pos_i - servo_joint_1_pos_increment;

      }
    }

    //Move Claw Motor Downwards
    if (state == 'G') {

      if (servo_joint_2_parking_pos_i < servo_joint_2_max_pos) {
        HCPCA9685.Servo(3, servo_joint_2_parking_pos_i);
        delay(response_time);
        Serial.println(servo_joint_2_parking_pos_i);

        servo_joint_2_parking_pos_i = servo_joint_2_parking_pos_i + servo_joint_2_pos_increment;

      }
    }

    //Move Claw Motor Upwards
    if (state == 'U') {

      if (servo_joint_2_parking_pos_i > servo_joint_2_min_pos) {
        HCPCA9685.Servo(3, servo_joint_2_parking_pos_i);
        delay(response_time);
        Serial.println(servo_joint_2_parking_pos_i);

        servo_joint_2_parking_pos_i = servo_joint_2_parking_pos_i - servo_joint_2_pos_increment;

      }
    }


    //Move Claw Motor Downwards
    if (state == 'f') {

      if (servo_joint_3_parking_pos_i < servo_joint_3_max_pos) {
        HCPCA9685.Servo(4, servo_joint_3_parking_pos_i);
        delay(response_time);
        Serial.println(servo_joint_3_parking_pos_i);
        servo_joint_3_parking_pos_i = servo_joint_3_parking_pos_i + servo_joint_3_pos_increment;

      }
    }

    //Move Claw Motor Upwards
    if (state == 'F') {

      if (servo_joint_3_parking_pos_i > servo_joint_3_min_pos) {
        HCPCA9685.Servo(4, servo_joint_3_parking_pos_i);
        delay(response_time);
        Serial.println(servo_joint_3_parking_pos_i);
        servo_joint_3_parking_pos_i = servo_joint_3_parking_pos_i - servo_joint_3_pos_increment;

      }
    }

    //Move Claw Motor Downwards
    if (state == 'L') {

      if (servo_joint_4_parking_pos_i < servo_joint_4_max_pos) {
        HCPCA9685.Servo(5, servo_joint_4_parking_pos_i);
        delay(response_time);
        Serial.println(servo_joint_4_parking_pos_i);
        servo_joint_4_parking_pos_i = servo_joint_4_parking_pos_i + servo_joint_4_pos_increment;

      }
    }

    //Move Claw Motor Upwards
    if (state == 'R') {

      if (servo_joint_4_parking_pos_i > servo_joint_4_min_pos) {
        HCPCA9685.Servo(5, servo_joint_4_parking_pos_i);
        delay(response_time);
        Serial.println(servo_joint_4_parking_pos_i);
        servo_joint_4_parking_pos_i = servo_joint_4_parking_pos_i - servo_joint_4_pos_increment;

      }
    }


  }
}

//create functions -> Place if statements in them

void gripperServo() {

}

void wristServoRotate() {

}

void wristServo() {

}

void elbowServo() {

}

void shoulderServo() {

}

void baseRotate() {

}

void setupRobot() {

  //Set starting position of the servo motors
  //  HCPCA9685.Servo(0, 90);
  //   delay(100);
  //  HCPCA9685.Servo(1, 90);
  //    delay(100);

  //  HCPCA9685.Servo(2, servo_joint_1_parking_pos);
  //  HCPCA9685.Servo(3, 90);

  //  for (pos = 0; pos < 180; pos++)
  //  {
  HCPCA9685.Servo(0, 180);
  delay(100);
  HCPCA9685.Servo(1, 180 - 180);
  delay(100);
  HCPCA9685.Servo(2, 100);
  delay(200);
  HCPCA9685.Servo(6, 10);
  delay(100);
  HCPCA9685.Servo(4, 25);
  delay(400);
  HCPCA9685.Servo(5, 180);
  delay(100);

  //  }


}

void baseRotateLeft() {
  //clockwise
  digitalWrite(dirPin, HIGH);
  // Spin motor
  for (int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(response_time); // Wait a second
}


void baseRotateRight() {

  //counterclockwise
  digitalWrite(dirPin, LOW);
  // Spin motor
  for (int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(response_time); // Wait a second
}
