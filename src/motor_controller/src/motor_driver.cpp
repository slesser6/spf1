#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include "motor_driver.h"

void motor_driver_init() {
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

void motor_drive_forward(int speed) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    digitalWrite(PIN_L293D_RIGHT_1, LOW);
    digitalWrite(PIN_L293D_RIGHT_2, HIGH);
    softPwmWrite(PIN_L293D_RIGHT_EN, speed);
    digitalWrite(PIN_L293D_LEFT_1, LOW);
    digitalWrite(PIN_L293D_LEFT_2, HIGH);
    softPwmWrite(PIN_L293D_LEFT_EN, speed);
}

void motor_drive_backward(int speed) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    digitalWrite(PIN_L293D_RIGHT_1, HIGH);
    digitalWrite(PIN_L293D_RIGHT_2, LOW);
    softPwmWrite(PIN_L293D_RIGHT_EN, speed);
    digitalWrite(PIN_L293D_LEFT_1, HIGH);
    digitalWrite(PIN_L293D_LEFT_2, LOW);
    softPwmWrite(PIN_L293D_LEFT_EN, speed);
}

void motor_turn_left(int speed) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    digitalWrite(PIN_L293D_RIGHT_1, LOW);
    digitalWrite(PIN_L293D_RIGHT_2, HIGH);
    softPwmWrite(PIN_L293D_RIGHT_EN, speed);
    digitalWrite(PIN_L293D_LEFT_1, HIGH);
    digitalWrite(PIN_L293D_LEFT_2, LOW);
    softPwmWrite(PIN_L293D_LEFT_EN, speed);
}

void motor_turn_right(int speed) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    digitalWrite(PIN_L293D_RIGHT_1, HIGH);
    digitalWrite(PIN_L293D_RIGHT_2, LOW);
    softPwmWrite(PIN_L293D_RIGHT_EN, speed);
    digitalWrite(PIN_L293D_LEFT_1, LOW);
    digitalWrite(PIN_L293D_LEFT_2, HIGH);
    softPwmWrite(PIN_L293D_LEFT_EN, speed);
}

void motor_stop() {
    digitalWrite(PIN_L293D_RIGHT_1, LOW);
    digitalWrite(PIN_L293D_RIGHT_2, LOW);
    softPwmWrite(PIN_L293D_RIGHT_EN, 0);
    digitalWrite(PIN_L293D_LEFT_1, LOW);
    digitalWrite(PIN_L293D_LEFT_2, LOW);
    softPwmWrite(PIN_L293D_LEFT_EN, 0);
}
