#include "pid_controller.h"

float pid_compute(PIDController *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

int clamp_speed(float speed)
{
    if (speed > MAX_SPEED) return MAX_SPEED;
    if (speed < 0) return 0;
    return (int)speed;
}