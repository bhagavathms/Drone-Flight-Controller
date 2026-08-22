/*
 * flight_controller.h
 * =============================================================================
 * Drone Flight Computer -- Embedded C Flight Controller
 * Software-in-the-Loop (SITL) shared library header
 *
 * Exposes structs and function declarations for use by the Python SITL harness
 * via ctypes. All flight-critical logic (sensor fusion, attitude estimation,
 * motor mixing) lives in this C layer, exactly as it would on real embedded
 * hardware (STM32, ESP32, etc.).
 * =============================================================================
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

/* --------------------------------------------------------------------------
 * Cross-platform DLL export macro
 * -------------------------------------------------------------------------- */
#ifdef _WIN32
    #define FC_EXPORT __declspec(dllexport)
#else
    #define FC_EXPORT __attribute__((visibility("default")))
#endif

/* --------------------------------------------------------------------------
 * Data Structures
 * -------------------------------------------------------------------------- */

/**
 * IMUData -- Raw sensor reading from the MPU-6050 (or virtual equivalent).
 *
 * Accelerometer values in g-force units (1.0 = standard gravity).
 * Gyroscope values in degrees per second.
 */
typedef struct {
    float accel_x;   /* Lateral acceleration      (g) */
    float accel_y;   /* Longitudinal acceleration  (g) */
    float accel_z;   /* Vertical acceleration      (g) -- ~1.0 at rest */
    float gyro_x;    /* Roll  rate  (deg/s) */
    float gyro_y;    /* Pitch rate  (deg/s) */
    float gyro_z;    /* Yaw   rate  (deg/s) */
} IMUData;

/**
 * AttitudeOutput -- Fused attitude angles from the complementary filter.
 */
typedef struct {
    float roll;      /* Fused roll  angle (degrees) */
    float pitch;     /* Fused pitch angle (degrees) */
} AttitudeOutput;

/**
 * MotorOutput -- Quadcopter motor PWM commands.
 *
 * Standard layout (top-down view):
 *   M1 Front-Left  (CW)   M2 Front-Right (CCW)
 *   M3 Rear-Left  (CCW)   M4 Rear-Right   (CW)
 *
 * PWM values in microseconds, clamped to [1000, 2000].
 */
typedef struct {
    int m1;   /* Front-Left  PWM (us) */
    int m2;   /* Front-Right PWM (us) */
    int m3;   /* Rear-Left   PWM (us) */
    int m4;   /* Rear-Right  PWM (us) */
} MotorOutput;

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/**
 * fc_init() -- Reset the flight controller state.
 * Must be called once before the first fc_update().
 */
FC_EXPORT void fc_init(void);

/**
 * fc_update() -- Fuse IMU data and compute attitude estimate.
 *
 * Implements a Complementary Filter:
 *   angle = 0.96 * (angle + gyro * dt) + 0.04 * accel_angle
 *
 * @param imu     Pointer to raw IMU sensor data
 * @param dt      Time delta since last call (seconds)
 * @param output  Pointer to AttitudeOutput struct to fill
 */
FC_EXPORT void fc_update(const IMUData* imu, float dt, AttitudeOutput* output);

/**
 * fc_motor_mixing() -- Convert attitude to 4-motor PWM commands.
 *
 * Uses standard quadcopter (+) mixing matrix:
 *   M1 = throttle - roll + pitch
 *   M2 = throttle + roll + pitch
 *   M3 = throttle - roll - pitch
 *   M4 = throttle + roll - pitch
 *
 * All outputs clamped to [1000, 2000] us.
 *
 * @param roll    Fused roll  angle (degrees)
 * @param pitch   Fused pitch angle (degrees)
 * @param output  Pointer to MotorOutput struct to fill
 */
FC_EXPORT void fc_motor_mixing(float roll, float pitch, MotorOutput* output);

#endif /* FLIGHT_CONTROLLER_H */
