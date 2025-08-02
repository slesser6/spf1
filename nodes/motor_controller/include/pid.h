#ifndef PID_H
#define PID_H

/**
 * @file pid.h
 * @brief PID controller definitions and utility functions.
 */

/**
 * @def MAX_SPEED
 * @brief Maximum allowed speed value for motor commands.
 */
#define MAX_SPEED 100

/**
 * @struct PIDController
 * @brief Struct holding parameters and state for a PID controller.
 *
 * @var PIDController::kp
 * Proportional gain.
 * @var PIDController::ki
 * Integral gain.
 * @var PIDController::kd
 * Derivative gain.
 * @var PIDController::prev_error
 * Previous error value (for derivative calculation).
 * @var PIDController::integral
 * Accumulated integral of error.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} PIDController;

/**
 * @brief Compute the PID control output.
 *
 * Calculates the control signal based on the setpoint, measured value,
 * PID gains, and elapsed time.
 *
 * @param pid Pointer to PIDController struct.
 * @param setpoint Desired target value.
 * @param measured Current measured value.
 * @param dt Time difference in seconds since last update.
 * @return float Control output value.
 */
float pidCompute(PIDController *pid, float setpoint, float measured, float dt);

/**
 * @brief Clamp a speed value to valid range [0, MAX_SPEED].
 *
 * Ensures speed does not exceed maximum and is not negative.
 *
 * @param speed Input speed value.
 * @return int Clamped speed as integer.
 */
int clampSpeed(float speed);

#endif