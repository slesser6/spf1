#include "motor_driver.h"
#include <softPwm.h>
#include <stdio.h>
#include <wiringPi.h>

/**
 * @brief Initialize motor driver GPIO pins and PWM.
 *
 * Sets pin modes for motor control pins and initializes software PWM on enable
 * pins. Stops the motors initially.
 */
void motorDriverInit() {
  wiringPiSetup();
  pinMode(PIN_L293D_RIGHT_1, OUTPUT);
  pinMode(PIN_L293D_RIGHT_2, OUTPUT);
  pinMode(PIN_L293D_RIGHT_EN, OUTPUT);
  pinMode(PIN_L293D_LEFT_1, OUTPUT);
  pinMode(PIN_L293D_LEFT_2, OUTPUT);
  pinMode(PIN_L293D_LEFT_EN, OUTPUT);

  softPwmCreate(PIN_L293D_RIGHT_EN, 0, 100);
  softPwmCreate(PIN_L293D_LEFT_EN, 0, 100);
  motor_stop();
}

/**
 * @brief Drive motors forward at specified speed.
 *
 * @param speed Speed value from 0 (stop) to 100 (full speed).
 */
void motorDriveForward(int speed) {
  if (speed < 0)
    speed = 0;
  if (speed > 100)
    speed = 100;

  digitalWrite(PIN_L293D_RIGHT_1, LOW);
  digitalWrite(PIN_L293D_RIGHT_2, HIGH);
  softPwmWrite(PIN_L293D_RIGHT_EN, speed);
  digitalWrite(PIN_L293D_LEFT_1, LOW);
  digitalWrite(PIN_L293D_LEFT_2, HIGH);
  softPwmWrite(PIN_L293D_LEFT_EN, speed);
}

/**
 * @brief Drive motors backward at specified speed.
 *
 * @param speed Speed value from 0 (stop) to 100 (full speed).
 */
void motorDriveBackward(int speed) {
  if (speed < 0)
    speed = 0;
  if (speed > 100)
    speed = 100;

  digitalWrite(PIN_L293D_RIGHT_1, HIGH);
  digitalWrite(PIN_L293D_RIGHT_2, LOW);
  softPwmWrite(PIN_L293D_RIGHT_EN, speed);
  digitalWrite(PIN_L293D_LEFT_1, HIGH);
  digitalWrite(PIN_L293D_LEFT_2, LOW);
  softPwmWrite(PIN_L293D_LEFT_EN, speed);
}

/**
 * @brief Turn robot left by running motors in opposite directions.
 *
 * @param speed Speed value from 0 (stop) to 100 (full speed).
 */
void motorTurnLeft(int speed) {
  if (speed < 0)
    speed = 0;
  if (speed > 100)
    speed = 100;

  digitalWrite(PIN_L293D_RIGHT_1, LOW);
  digitalWrite(PIN_L293D_RIGHT_2, HIGH);
  softPwmWrite(PIN_L293D_RIGHT_EN, speed);
  digitalWrite(PIN_L293D_LEFT_1, HIGH);
  digitalWrite(PIN_L293D_LEFT_2, LOW);
  softPwmWrite(PIN_L293D_LEFT_EN, speed);
}

/**
 * @brief Turn robot right by running motors in opposite directions.
 *
 * @param speed Speed value from 0 (stop) to 100 (full speed).
 */
void motorTurnRight(int speed) {
  if (speed < 0)
    speed = 0;
  if (speed > 100)
    speed = 100;

  digitalWrite(PIN_L293D_RIGHT_1, HIGH);
  digitalWrite(PIN_L293D_RIGHT_2, LOW);
  softPwmWrite(PIN_L293D_RIGHT_EN, speed);
  digitalWrite(PIN_L293D_LEFT_1, LOW);
  digitalWrite(PIN_L293D_LEFT_2, HIGH);
  softPwmWrite(PIN_L293D_LEFT_EN, speed);
}

/**
 * @brief Stop both motors immediately.
 */
void motorStop() {
  digitalWrite(PIN_L293D_RIGHT_1, LOW);
  digitalWrite(PIN_L293D_RIGHT_2, LOW);
  softPwmWrite(PIN_L293D_RIGHT_EN, 0);
  digitalWrite(PIN_L293D_LEFT_1, LOW);
  digitalWrite(PIN_L293D_LEFT_2, LOW);
  softPwmWrite(PIN_L293D_LEFT_EN, 0);
}
