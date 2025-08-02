#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

/**
 * @file motor_driver.h
 * @brief Motor driver interface for controlling motors using L293D motor driver.
 */

/** 
 * @def PIN_L293D_RIGHT_1
 * @brief GPIO pin connected to right motor input 1.
 */
#define PIN_L293D_RIGHT_1 4 // GPIO 23 

/** 
 * @def PIN_L293D_RIGHT_2
 * @brief GPIO pin connected to right motor input 2.
 */
#define PIN_L293D_RIGHT_2 5 // GPIO 24

/** 
 * @def PIN_L293D_RIGHT_EN
 * @brief GPIO pin connected to enable pin for right motor.
 */
#define PIN_L293D_RIGHT_EN 6 // GPIO 25

/** 
 * @def PIN_L293D_LEFT_1
 * @brief GPIO pin connected to left motor input 1.
 */
#define PIN_L293D_LEFT_1 0 // GPIO 27

/** 
 * @def PIN_L293D_LEFT_2
 * @brief GPIO pin connected to left motor input 2.
 */
#define PIN_L293D_LEFT_2 2 // GPIO 17

/** 
 * @def PIN_L293D_LEFT_EN
 * @brief GPIO pin connected to enable pin for left motor.
 */
#define PIN_L293D_LEFT_EN 3 // GPIO 22

/**
 * @brief Initialize the motor driver pins and setup.
 *
 * Configures GPIO pins as outputs and initializes PWM if needed.
 */
void motorDriverInit();

/**
 * @brief Drive the motors forward at the specified speed.
 *
 * @param speed Speed value (typically 0-100).
 */
void motorDriveForward(int speed);

/**
 * @brief Drive the motors backward at the specified speed.
 *
 * @param speed Speed value (typically 0-100).
 */
void motorDriveBackward(int speed);

/**
 * @brief Turn the robot left by controlling motor speeds.
 *
 * @param speed Speed value (typically 0-100).
 */
void motorTurnLeft(int speed);

/**
 * @brief Turn the robot right by controlling motor speeds.
 *
 * @param speed Speed value (typically 0-100).
 */
void motorTurnRight(int speed);

/**
 * @brief Stop all motor movement.
 */
void motorStop();

#endif
