#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define MAX_SPEED 50

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} PIDController;

float pid_compute(PIDController *pid, float setpoint, float measured, float dt);
int clamp_speed(float speed);

#endif