import time
import math
import os
import random
import sys

# Ensure UTF-8 output on Windows terminals (PowerShell / cmd)
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except AttributeError:
        pass


# =============================================================================
# VIRTUAL SENSORS -- Software-in-the-Loop hardware abstraction layer
# =============================================================================

class VirtualSensors:
    """
    Software-in-the-Loop (SITL) virtual sensor suite for drone flight computer simulation.
    Simulates IMU (MPU-6050) and battery readings without physical hardware.
    """

    def __init__(self):
        """Initialize the virtual sensor suite and record the simulation start time."""
        self.start_time = time.time()

    def read_mpu6050(self):
        """
        Simulate MPU-6050 IMU sensor readings.

        Uses elapsed time with sin/cos functions to generate slowly oscillating
        fake accelerometer and gyroscope data, mimicking a drone hovering with
        slight vibrations and attitude perturbations.

        Returns:
            dict: Keys: accel_x, accel_y, accel_z (g), gyro_x, gyro_y, gyro_z (deg/s)
        """
        elapsed = time.time() - self.start_time

        # Slowly oscillating accelerometer values simulating gentle attitude sway
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
            dict: Keys: voltage (V), elapsed (s), percent (%)
        """
        MAX_VOLTAGE    = 12.6    # Fully charged 3S LiPo (4.2V/cell)
        MIN_VOLTAGE    = 10.5    # Safe minimum 3S LiPo  (3.5V/cell)
        DISCHARGE_RATE = 0.002   # Volts per second of simulated discharge

        elapsed = time.time() - self.start_time

        # Linear discharge model clamped to minimum safe voltage
        voltage = max(MIN_VOLTAGE, MAX_VOLTAGE - DISCHARGE_RATE * elapsed)

        voltage_range = MAX_VOLTAGE - MIN_VOLTAGE
        percent = ((voltage - MIN_VOLTAGE) / voltage_range) * 100.0

        return {
            "voltage": round(voltage, 3),
            "elapsed": round(elapsed, 2),
            "percent": round(percent, 1),
        }


# =============================================================================
# FLIGHT CONTROLLER -- Sensor fusion, attitude estimation, motor mixing
# =============================================================================

class FlightController:
    """
    Software-in-the-Loop Flight Controller.

    Implements:
      - Accelerometer-based roll/pitch estimation via atan2
      - Complementary filter for sensor fusion (gyro + accel)
      - Standard quadcopter motor-mixing algorithm
      - PWM output clamped to [1000, 2000] µs (ESC range)

    Motor layout (top-down view):
        M1 (Front-Left)  CW    M2 (Front-Right) CCW
        M3 (Rear-Left)  CCW    M4 (Rear-Right)   CW
    """

    # Complementary filter coefficient (gyro trust weight)
    ALPHA = 0.96

    # ESC PWM limits (µs)
    PWM_MIN = 1000
    PWM_MAX = 2000

    # Base throttle for a hovering quadcopter (mid-range)
    BASE_THROTTLE = 1500

    def __init__(self):
        """Initialise attitude state and timing."""
        self.roll  = 0.0   # Estimated roll  angle (degrees)
        self.pitch = 0.0   # Estimated pitch angle (degrees)
        self.last_time = time.time()

    # ------------------------------------------------------------------
    # Attitude estimation
    # ------------------------------------------------------------------

    def update(self, imu: dict):
        """
        Fuse accelerometer + gyroscope data with a Complementary Filter.

        Step 1 – Accel angles: derive absolute roll/pitch from gravity vector.
        Step 2 – Gyro integration: integrate angular velocity over dt.
        Step 3 – Complementary filter: blend both estimates.

        Args:
            imu (dict): Output of VirtualSensors.read_mpu6050()

        Returns:
            tuple[float, float]: (roll_deg, pitch_deg) fused attitude estimate
        """
        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        # Guard against zero or negative dt on the first call
        if dt <= 0:
            dt = 1e-3

        ax = imu["accel_x"]
        ay = imu["accel_y"]
        az = imu["accel_z"]
        gx = imu["gyro_x"]   # deg/s
        gy = imu["gyro_y"]   # deg/s

        # --- Step 1: Accelerometer-derived angles ---
        # atan2 gives roll/pitch relative to the gravity vector (in radians → degrees)
        accel_roll  = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

        # --- Step 2 & 3: Complementary filter ---
        # Gyro integration provides high-frequency accuracy; accel corrects drift.
        self.roll  = self.ALPHA * (self.roll  + gx * dt) + (1 - self.ALPHA) * accel_roll
        self.pitch = self.ALPHA * (self.pitch + gy * dt) + (1 - self.ALPHA) * accel_pitch

        return round(self.roll, 4), round(self.pitch, 4)

    # ------------------------------------------------------------------
    # Motor mixing
    # ------------------------------------------------------------------

    def motor_mixing(self, roll: float, pitch: float) -> list[int]:
        """
        Convert roll/pitch attitude error into 4-motor PWM outputs.

        Uses a standard quadcopter (+) mixing matrix:
            M1 (FL) = throttle - roll + pitch
            M2 (FR) = throttle + roll + pitch
            M3 (RL) = throttle - roll - pitch
            M4 (RR) = throttle + roll - pitch

        A proportional gain scales the angle (degrees) into PWM µs correction.
        Each output is clamped to [PWM_MIN, PWM_MAX].

        Args:
            roll  (float): Fused roll  angle in degrees
            pitch (float): Fused pitch angle in degrees

        Returns:
            list[int]: [M1, M2, M3, M4] PWM values in µs
        """
        KP = 5.0   # Proportional gain: deg → µs correction

        roll_cmd  = KP * roll
        pitch_cmd = KP * pitch

        m1 = self.BASE_THROTTLE - roll_cmd + pitch_cmd   # Front-Left
        m2 = self.BASE_THROTTLE + roll_cmd + pitch_cmd   # Front-Right
        m3 = self.BASE_THROTTLE - roll_cmd - pitch_cmd   # Rear-Left
        m4 = self.BASE_THROTTLE + roll_cmd - pitch_cmd   # Rear-Right

        # Clamp all outputs to valid ESC range
        pwm = [int(max(self.PWM_MIN, min(self.PWM_MAX, m)))
               for m in (m1, m2, m3, m4)]

        return pwm


# =============================================================================
# HELPERS -- Bar rendering for dashboard
# =============================================================================

def _bar(value: float, min_val: float, max_val: float, width: int = 20, char: str = "#") -> str:
    """Render a simple ASCII progress bar."""
    ratio  = max(0.0, min(1.0, (value - min_val) / (max_val - min_val)))
    filled = int(ratio * width)
    return char * filled + "." * (width - filled)


def _pwm_bar(pwm: int, width: int = 15) -> str:
    return _bar(pwm, 1000, 2000, width, "=")


# =============================================================================
# MAIN LOOP -- Continuous SITL dashboard
# =============================================================================

def main():
    """
    Main Software-in-the-Loop simulation loop.

    Continuously:
      1. Reads virtual IMU and battery sensors
      2. Fuses sensor data through the complementary filter
      3. Computes motor PWM outputs via the mixing algorithm
      4. Renders a formatted live dashboard (refreshed via cls/clear)

    Loop rate: ~10 Hz (0.1 s sleep)
    """
    sensors = VirtualSensors()
    fc      = FlightController()

    cycle = 0

    while True:
        cycle += 1

        # --- Read sensors ---
        imu = sensors.read_mpu6050()
        bat = sensors.read_battery()

        # --- Update flight controller ---
        roll, pitch = fc.update(imu)
        pwm         = fc.motor_mixing(roll, pitch)

        # --- Clear terminal and render dashboard ---
        os.system("cls" if os.name == "nt" else "clear")

        batt_bar = _bar(bat["voltage"], 10.5, 12.6, 20)
        batt_warn = "!! LOW BATTERY !!" if bat["percent"] < 20 else ""
        W = 60  # dashboard width
        div = "+" + "-" * (W - 2) + "+"

        def row(text=""):
            """Left-align text inside a fixed-width bordered row."""
            print(f"| {text:<{W - 3}}|")

        print(div)
        row(" DRONE FLIGHT COMPUTER  --  SITL DASHBOARD")
        print(div)
        row(f" Cycle : {cycle:<6}   Elapsed : {bat['elapsed']:>7.2f}s")
        print(div)
        row(" [ IMU -- MPU-6050 Raw ]")
        row(f"  Accel  X:{imu['accel_x']:+7.4f}g  Y:{imu['accel_y']:+7.4f}g  Z:{imu['accel_z']:+7.4f}g")
        row(f"  Gyro   X:{imu['gyro_x']:+7.4f}d/s Y:{imu['gyro_y']:+7.4f}d/s Z:{imu['gyro_z']:+7.4f}d/s")
        print(div)
        row(" [ Attitude -- Complementary Filter ]")
        row(f"  Roll  : {roll:+8.4f} deg   [{_bar(roll,  -30, 30):20s}]")
        row(f"  Pitch : {pitch:+8.4f} deg   [{_bar(pitch, -30, 30):20s}]")
        print(div)
        row(" [ Motor PWM Outputs (us) ]")
        row(f"  M1 FL: {pwm[0]:4d} [{_pwm_bar(pwm[0])}]   M2 FR: {pwm[1]:4d} [{_pwm_bar(pwm[1])}]")
        row(f"  M3 RL: {pwm[2]:4d} [{_pwm_bar(pwm[2])}]   M4 RR: {pwm[3]:4d} [{_pwm_bar(pwm[3])}]")
        print(div)
        row(" [ Battery -- 3S LiPo ]")
        row(f"  {bat['voltage']:5.3f}V  [{batt_bar}]  {bat['percent']:5.1f}%  {batt_warn}")
        print(div)
        print("  Press Ctrl+C to exit SITL simulation.")

        time.sleep(0.1)


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[SITL] Simulation terminated by user.")
