/*
 * flight_controller.c
 * =============================================================================
 * Drone Flight Computer -- Embedded C Flight Controller Implementation
 * Software-in-the-Loop (SITL) shared library
 *
 * This file contains ALL flight-critical processing logic, mirroring what
 * would run on actual embedded hardware (STM32F4, ESP32, etc.).
 *
 * Compiled as a shared library and called from Python via ctypes:
 *   Windows : gcc -shared -o flight_controller.dll flight_controller.c -lm
 *   Linux   : gcc -shared -fPIC -o flight_controller.so flight_controller.c -lm
 * =============================================================================
 */

#include "flight_controller.h"
#include <math.h>

/* --------------------------------------------------------------------------
 * Internal (private) controller state
 * -------------------------------------------------------------------------- */

/** Complementary filter alpha weight (gyro trust) */
#define CF_ALPHA       0.96f

/** Base throttle for hover in PWM microseconds */
#define BASE_THROTTLE  1500

/** Proportional gain: attitude angle (deg) -> PWM correction (us) */
#define KP_ATTITUDE    5.0f

/** ESC PWM limits (microseconds) */
#define PWM_MIN        1000
#define PWM_MAX        2000

/* Internal state -- persists between fc_update() calls */
static float g_roll  = 0.0f;   /* Current fused roll  estimate (degrees) */
static float g_pitch = 0.0f;   /* Current fused pitch estimate (degrees) */

/* --------------------------------------------------------------------------
 * Helper: integer clamp
 * -------------------------------------------------------------------------- */
static int clamp_int(int value, int lo, int hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

/* --------------------------------------------------------------------------
 * API Implementation
 * -------------------------------------------------------------------------- */

/**
 * fc_init -- Reset all internal flight controller state.
 * Call once at startup before entering the main loop.
 */
void fc_init(void)
{
    g_roll  = 0.0f;
    g_pitch = 0.0f;
}

/**
 * fc_update -- Fuse raw IMU data into a stable attitude estimate.
 *
 * Algorithm (Complementary Filter):
 *
 *   1. Derive ABSOLUTE roll/pitch from accelerometer using atan2.
 *      These are noisy but drift-free over time.
 *
 *   2. Integrate gyroscope angular velocity over dt.
 *      This is smooth but accumulates drift over time.
 *
 *   3. Blend both sources:
 *         angle = ALPHA * (angle + gyro * dt) + (1 - ALPHA) * accel_angle
 *
 *      ALPHA = 0.96 means:
 *        - 96% trust in gyro  (short-term accuracy)
 *        - 4%  trust in accel (long-term drift correction)
 */
void fc_update(const IMUData* imu, float dt, AttitudeOutput* output)
{
    float ax = imu->accel_x;
    float ay = imu->accel_y;
    float az = imu->accel_z;
    float gx = imu->gyro_x;   /* deg/s */
    float gy = imu->gyro_y;   /* deg/s */

    /* Guard: avoid division by zero or very small dt */
    if (dt < 1e-6f) dt = 1e-6f;

    /* --- Step 1: Accelerometer-derived angles (absolute, noisy) --- */
    /* Roll:  rotation around X-axis, derived from Y and Z components */
    float accel_roll  = (float)(atan2(ay, az) * (180.0 / M_PI));

    /* Pitch: rotation around Y-axis, derived from X and Z components */
    float accel_pitch = (float)(atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / M_PI));

    /* --- Step 2 + 3: Complementary filter --- */
    g_roll  = CF_ALPHA * (g_roll  + gx * dt) + (1.0f - CF_ALPHA) * accel_roll;
    g_pitch = CF_ALPHA * (g_pitch + gy * dt) + (1.0f - CF_ALPHA) * accel_pitch;

    output->roll  = g_roll;
    output->pitch = g_pitch;
}

/**
 * fc_motor_mixing -- Map attitude angles to 4-motor PWM commands.
 *
 * Standard quadcopter (+) mixing matrix (top-down view):
 *
 *       M1(FL,CW)   M2(FR,CCW)
 *           \         /
 *            [  FC  ]
 *           /         \
 *       M3(RL,CCW)  M4(RR,CW)
 *
 * Positive roll  (right lean) -> increase right motors, decrease left motors.
 * Positive pitch (nose up)    -> increase front motors, decrease rear motors.
 *
 * All outputs are clamped to [PWM_MIN, PWM_MAX].
 */
void fc_motor_mixing(float roll, float pitch, MotorOutput* output)
{
    float roll_cmd  = KP_ATTITUDE * roll;
    float pitch_cmd = KP_ATTITUDE * pitch;

    int m1 = (int)(BASE_THROTTLE - roll_cmd + pitch_cmd);  /* Front-Left  */
    int m2 = (int)(BASE_THROTTLE + roll_cmd + pitch_cmd);  /* Front-Right */
    int m3 = (int)(BASE_THROTTLE - roll_cmd - pitch_cmd);  /* Rear-Left   */
    int m4 = (int)(BASE_THROTTLE + roll_cmd - pitch_cmd);  /* Rear-Right  */

    output->m1 = clamp_int(m1, PWM_MIN, PWM_MAX);
    output->m2 = clamp_int(m2, PWM_MIN, PWM_MAX);
    output->m3 = clamp_int(m3, PWM_MIN, PWM_MAX);
    output->m4 = clamp_int(m4, PWM_MIN, PWM_MAX);
}
