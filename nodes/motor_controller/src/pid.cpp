#include "pid.h"

/**
 * @brief Compute the PID controller output.
 *
 * Calculates the control output based on the PID algorithm using the current error,
 * integral of the error, and derivative of the error.
 *
 * @param pid Pointer to the PIDController struct containing gains and state.
 * @param setpoint The desired target value.
 * @param measured The current measured value.
 * @param dt Time interval since the last call (in seconds).
 * @return float The computed control output.
 */
float pid_compute(PIDController *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

/**
 * @brief Clamp a speed value to within allowable bounds.
 *
 * Ensures the speed is between 0 and MAX_SPEED.
 *
 * @param speed The speed value to clamp.
 * @return int The clamped speed as an integer.
 */
int clamp_speed(float speed)
{
    if (speed > MAX_SPEED) return MAX_SPEED;
    if (speed < 0) return 0;
    return (int)speed;
}
