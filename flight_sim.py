import time
import math
import os
import random


class VirtualSensors:
    """
    Software-in-the-Loop (SITL) virtual sensor suite for drone flight computer simulation.
    Simulates IMU (MPU-6050) and battery readings without physical hardware.
    """

    def __init__(self):
        """Initialize the virtual sensor suite and record the simulation start time."""
        self.start_time = time.time()
        print(f"[VirtualSensors] Initialized. Simulation start time: {self.start_time:.2f}")

    def read_mpu6050(self):
        """
        Simulate MPU-6050 IMU sensor readings.

        Uses elapsed time with sin/cos functions to generate slowly oscillating
        fake accelerometer and gyroscope data, mimicking a drone hovering with
        slight vibrations and attitude perturbations.

        Returns:
            dict: A dictionary containing:
                - 'accel_x' (float): Lateral acceleration in g-force units
                - 'accel_y' (float): Longitudinal acceleration in g-force units
                - 'accel_z' (float): Vertical acceleration in g-force units (~1.0 g for gravity)
                - 'gyro_x' (float): Roll rate in degrees/second
                - 'gyro_y' (float): Pitch rate in degrees/second
                - 'gyro_z' (float): Yaw rate in degrees/second
        """
        elapsed = time.time() - self.start_time

        # Slowly oscillating accelerometer values simulating gentle attitude sway
        # Small amplitude oscillations on X and Y axes
        accel_x = 0.05 * math.sin(0.3 * elapsed) + random.uniform(-0.005, 0.005)
        accel_y = 0.05 * math.cos(0.25 * elapsed) + random.uniform(-0.005, 0.005)

        # Z-axis keeps gravity near 1.0 g with minor oscillation
        accel_z = 1.0 + 0.02 * math.sin(0.1 * elapsed) + random.uniform(-0.003, 0.003)

        # Slowly oscillating gyroscope values simulating minor rotational drift
        gyro_x = 1.5 * math.sin(0.2 * elapsed) + random.uniform(-0.1, 0.1)
        gyro_y = 1.5 * math.cos(0.15 * elapsed) + random.uniform(-0.1, 0.1)
        gyro_z = 0.8 * math.sin(0.05 * elapsed) + random.uniform(-0.05, 0.05)

        return {
            "accel_x": round(accel_x, 4),
            "accel_y": round(accel_y, 4),
            "accel_z": round(accel_z, 4),
            "gyro_x":  round(gyro_x, 4),
            "gyro_y":  round(gyro_y, 4),
            "gyro_z":  round(gyro_z, 4),
        }

    def read_battery(self):
        """
        Simulate LiPo battery voltage degradation over time.

        Voltage starts at a fully-charged 12.6V (3S LiPo) and slowly degrades
        based on elapsed simulation time, never dropping below the safe minimum
        of 10.5V (3.5V per cell).

        Returns:
            dict: A dictionary containing:
                - 'voltage'  (float): Current simulated battery voltage in Volts
                - 'elapsed'  (float): Seconds elapsed since simulation start
                - 'percent'  (float): Remaining battery percentage (0–100%)
        """
        MAX_VOLTAGE = 12.6   # Fully charged 3S LiPo (4.2V/cell)
        MIN_VOLTAGE = 10.5   # Safe minimum 3S LiPo (3.5V/cell)
        DISCHARGE_RATE = 0.002  # Volts per second of simulated discharge

        elapsed = time.time() - self.start_time

        # Linear discharge model clamped to minimum safe voltage
        voltage = max(MIN_VOLTAGE, MAX_VOLTAGE - DISCHARGE_RATE * elapsed)

        # Calculate remaining percentage
        voltage_range = MAX_VOLTAGE - MIN_VOLTAGE
        percent = ((voltage - MIN_VOLTAGE) / voltage_range) * 100.0

        return {
            "voltage": round(voltage, 3),
            "elapsed": round(elapsed, 2),
            "percent": round(percent, 1),
        }


# ---------------------------------------------------------------------------
# Quick self-test / demo when run directly
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 55)
    print("  Drone Flight Computer — SITL Sensor Demo")
    print("=" * 55)

    sensors = VirtualSensors()

    for cycle in range(1, 6):
        print(f"\n--- Cycle {cycle} ---")
        imu = sensors.read_mpu6050()
        bat = sensors.read_battery()

        print(f"  Accel  X={imu['accel_x']:+.4f}g  Y={imu['accel_y']:+.4f}g  Z={imu['accel_z']:+.4f}g")
        print(f"  Gyro   X={imu['gyro_x']:+.4f}°/s  Y={imu['gyro_y']:+.4f}°/s  Z={imu['gyro_z']:+.4f}°/s")
        print(f"  Battery {bat['voltage']}V  ({bat['percent']}%)  Elapsed: {bat['elapsed']}s")

        time.sleep(0.5)

    print("\n[SITL] Demo complete.")
