/*
Author: Benjamin Low (Lthben@gmail.com)
Date: May 2015
Description: Code for controlling 36 stepper motors for project "Neobombe",
presented at Singapore Maker Faire 2015 by Art Makes Us. "Neobombe" stimulates
the Alan Turing machine seen in the 2014 film "The Imitation Game".
*/

/*
Notes:
        - call release() whenever the motors do not need to turn, to dissipate heat
        - avoid address 0x70 since this addresses ALL the motors
        - http://www.dunkels.com/adam/pt/ Protothreads for simultaneous control
        - use SINGLE stepping mode to reduce the current loading and heat buildup
*/
//#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

//USER SETTINGS
const int MOTORSPEED = 20;

Adafruit_MotorShield AFMS_1 = Adafruit_MotorShield(0x60);
Adafruit_StepperMotor *myStepper_1 = AFMS_1.getStepper(200, 1); //NEMA 17 stepper
Adafruit_StepperMotor *myStepper_2 = AFMS_1.getStepper(200, 2); //NEMA 17 stepper

Adafruit_MotorShield AFMS_2 = Adafruit_MotorShield(0x61);
Adafruit_StepperMotor *myStepper_3 = AFMS_2.getStepper(513, 1); //small stepper
Adafruit_StepperMotor *myStepper_4 = AFMS_2.getStepper(513, 2); //small stepper

int ledPin = 5;
int switchPin = 7;

void setup() {
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT);

  AFMS_1.begin();
  AFMS_2.begin();

  TWBR = ((F_CPU / 400000l) - 16) / 2; // Change the i2c clock to 400KHz

  myStepper_1->setSpeed(MOTORSPEED);
  myStepper_2->setSpeed(MOTORSPEED);
  myStepper_3->setSpeed(MOTORSPEED);
  myStepper_4->setSpeed(MOTORSPEED);
  
  Serial.flush();
}


boolean is_processing;
byte incoming_byte;

void loop() {

  if (Serial.available() > 0) {

    incoming_byte = Serial.read();

    if (incoming_byte == 65) {

      digitalWrite(ledPin, HIGH);
      is_processing = true;

    }
    else if (incoming_byte == 66) {
            
      digitalWrite(ledPin, LOW);
      is_processing = false;
      
      myStepper_1->release();
      myStepper_2->release();
      myStepper_3->release();
      myStepper_4->release();
    }
  }

  if (is_processing) myStepper_1->step(8, FORWARD, SINGLE);
  if (is_processing) myStepper_2->step(8, FORWARD, SINGLE);
  if (is_processing) myStepper_3->step(20, FORWARD, SINGLE);
  if (is_processing) myStepper_4->step(20, FORWARD, SINGLE);

}


