#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define PIN_L293D_RIGHT_1 4 // GPIO 23 
#define PIN_L293D_RIGHT_2 5 // GPIO 24
#define PIN_L293D_RIGHT_EN 6 // GPIO 25

#define PIN_L293D_LEFT_1 0 // GPIO 27
#define PIN_L293D_LEFT_2 2 // GPIO 17
#define PIN_L293D_LEFT_EN 3 // GPIO 22

void motor_driver_init();
void motor_drive_forward(int speed);
void motor_drive_backward(int speed);
void motor_turn_left(int speed);
void motor_turn_right(int speed);
void motor_stop();

#endif
