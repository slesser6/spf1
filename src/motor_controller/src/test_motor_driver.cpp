#include <stdio.h>
#include <unistd.h>  // for sleep
#include "motor_driver.h"

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
