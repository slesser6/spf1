#include <stdio.h>
#include <unistd.h>  // for sleep
#include "motor_driver.h"

/**
 * @file
 * @brief Simple test program for motor driver functions.
 *
 * This program initializes the motor driver, then performs a series of motions:
 * stop, drive forward, stop, drive backward, turn left, turn right, and final stop,
 * each action lasting approximately one second.
 */

/**
 * @brief Main function to test motor driving capabilities.
 *
 * Initializes motor driver and runs a sequence of motor commands with delays.
 *
 * @return int Exit status code.
 */
int main() {
    motor_driver_init();

    motor_stop();
    sleep(1);

    printf("Driving forward...\n");
    motor_drive_forward(100);
    sleep(1);

    printf("Stopping...\n");
    motor_stop();
    sleep(1);

    printf("Driving backward...\n");
    motor_drive_backward(100);
    sleep(1);

    printf("Turning left...\n");
    motor_turn_left(100);
    sleep(1);

    printf("Turning right...\n");
    motor_turn_right(100);
    sleep(1);

    printf("Final stop.\n");
    motor_stop();

    return 0;
}
