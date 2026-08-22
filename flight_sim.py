"""
flight_sim.py
=============================================================================
Drone Flight Computer -- Software-in-the-Loop (SITL) Python Harness

Architecture:
    [VirtualSensors]  -->  [SITLBridge / C .dll]  -->  [Dashboard]
      Python layer          Embedded C layer            Python layer

Responsibilities of each layer:
  - VirtualSensors : Generates fake but physically plausible IMU and battery
                     data (accelerometer, gyroscope, voltage). Replaces real
                     MPU-6050 hardware in the SITL loop.
  - SITLBridge     : Loads flight_controller.dll (compiled from flight_controller.c)
                     via ctypes and calls the C functions fc_update() and
                     fc_motor_mixing(). ALL flight logic runs in C.
  - main() / Dashboard : Reads from both layers, prints a live terminal
                         dashboard refreshed at ~10 Hz.

To rebuild the C library:
    Windows : build.bat
    Linux   : gcc -shared -fPIC -o flight_controller.so flight_controller.c -lm
=============================================================================
"""

import time
import math
import os
import random
import sys
import ctypes
import ctypes.util

# =============================================================================
# VIRTUAL SENSORS -- Python SITL hardware abstraction layer
# =============================================================================

class VirtualSensors:
    """
    Software-in-the-Loop virtual sensor suite.
    Simulates MPU-6050 IMU and 3S LiPo battery without physical hardware.
    """

    def __init__(self):
        """Record simulation start time."""
        self.start_time = time.time()

    def read_mpu6050(self):
        """
        Generate slowly oscillating IMU readings using sin/cos on elapsed time.

        Simulates a hovering drone with slight attitude perturbations and sensor
        noise. Z-axis gravity is kept near 1.0 g.

        Returns:
            dict: accel_x/y/z (g), gyro_x/y/z (deg/s)
        """
        elapsed = time.time() - self.start_time

        accel_x = 0.05 * math.sin(0.3 * elapsed)  + random.uniform(-0.005, 0.005)
        accel_y = 0.05 * math.cos(0.25 * elapsed) + random.uniform(-0.005, 0.005)
        accel_z = 1.0  + 0.02 * math.sin(0.1 * elapsed) + random.uniform(-0.003, 0.003)

        gyro_x  = 1.5 * math.sin(0.2 * elapsed)  + random.uniform(-0.1,  0.1)
        gyro_y  = 1.5 * math.cos(0.15 * elapsed) + random.uniform(-0.1,  0.1)
        gyro_z  = 0.8 * math.sin(0.05 * elapsed) + random.uniform(-0.05, 0.05)

        return {
            "accel_x": round(accel_x, 4),
            "accel_y": round(accel_y, 4),
            "accel_z": round(accel_z, 4),
            "gyro_x":  round(gyro_x,  4),
            "gyro_y":  round(gyro_y,  4),
            "gyro_z":  round(gyro_z,  4),
        }

    def read_battery(self):
        """
        Simulate 3S LiPo voltage degradation over time.

        Starts at 12.6V (fully charged), degrades at 0.002 V/s,
        never drops below 10.5V (safe minimum 3.5V/cell).

        Returns:
            dict: voltage (V), elapsed (s), percent (%)
        """
        MAX_V  = 12.6
        MIN_V  = 10.5
        RATE   = 0.002   # V/s

        elapsed = time.time() - self.start_time
        voltage = max(MIN_V, MAX_V - RATE * elapsed)
        percent = ((voltage - MIN_V) / (MAX_V - MIN_V)) * 100.0

        return {
            "voltage": round(voltage, 3),
            "elapsed": round(elapsed, 2),
            "percent": round(percent, 1),
        }


# =============================================================================
# SITL BRIDGE -- ctypes interface to the Embedded C flight controller
# =============================================================================

class _IMUData(ctypes.Structure):
    """
    Mirror of the C IMUData struct (flight_controller.h).
    Field order and types MUST match exactly.
    """
    _fields_ = [
        ("accel_x", ctypes.c_float),
        ("accel_y", ctypes.c_float),
        ("accel_z", ctypes.c_float),
        ("gyro_x",  ctypes.c_float),
        ("gyro_y",  ctypes.c_float),
        ("gyro_z",  ctypes.c_float),
    ]


class _AttitudeOutput(ctypes.Structure):
    """Mirror of the C AttitudeOutput struct."""
    _fields_ = [
        ("roll",  ctypes.c_float),
        ("pitch", ctypes.c_float),
    ]


class _MotorOutput(ctypes.Structure):
    """Mirror of the C MotorOutput struct."""
    _fields_ = [
        ("m1", ctypes.c_int),
        ("m2", ctypes.c_int),
        ("m3", ctypes.c_int),
        ("m4", ctypes.c_int),
    ]


class SITLBridge:
    """
    Python-to-C bridge for the embedded flight controller.

    Loads flight_controller.dll (Windows) or flight_controller.so (Linux/Mac)
    from the same directory as this script and exposes a clean Python API.

    All actual flight math runs inside the compiled C code.
    """

    def __init__(self):
        """Load the shared library and configure function signatures."""
        # Resolve library path relative to this script
        script_dir = os.path.dirname(os.path.abspath(__file__))

        if sys.platform == "win32":
            lib_name = "flight_controller.dll"
        else:
            lib_name = "flight_controller.so"

        lib_path = os.path.join(script_dir, lib_name)

        if not os.path.exists(lib_path):
            raise FileNotFoundError(
                f"[SITLBridge] Shared library not found: {lib_path}\n"
                f"             Run build.bat (Windows) or make (Linux) first."
            )

        self._lib = ctypes.CDLL(lib_path)
        self._configure_signatures()
        self._lib.fc_init()

        self._last_time = time.time()
        print(f"[SITLBridge] Loaded {lib_name} -- C flight controller active.")

    def _configure_signatures(self):
        """Declare C function argument and return types for type safety."""
        lib = self._lib

        # void fc_init(void)
        lib.fc_init.restype  = None
        lib.fc_init.argtypes = []

        # void fc_update(const IMUData*, float dt, AttitudeOutput*)
        lib.fc_update.restype  = None
        lib.fc_update.argtypes = [
            ctypes.POINTER(_IMUData),
            ctypes.c_float,
            ctypes.POINTER(_AttitudeOutput),
        ]

        # void fc_motor_mixing(float roll, float pitch, MotorOutput*)
        lib.fc_motor_mixing.restype  = None
        lib.fc_motor_mixing.argtypes = [
            ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(_MotorOutput),
        ]

    def process(self, imu_dict: dict) -> dict:
        """
        Send raw sensor data into the C flight controller and get back
        fused attitude and motor PWM outputs.

        Args:
            imu_dict (dict): Output from VirtualSensors.read_mpu6050()

        Returns:
            dict:
                - roll    (float): Fused roll  angle (degrees)
                - pitch   (float): Fused pitch angle (degrees)
                - pwm     (list[int]): [M1, M2, M3, M4] PWM values (us)
        """
        # Compute dt
        now = time.time()
        dt  = now - self._last_time
        self._last_time = now
        if dt <= 0:
            dt = 1e-3

        # Populate C struct from Python dict
        imu_c = _IMUData(
            accel_x = imu_dict["accel_x"],
            accel_y = imu_dict["accel_y"],
            accel_z = imu_dict["accel_z"],
            gyro_x  = imu_dict["gyro_x"],
            gyro_y  = imu_dict["gyro_y"],
            gyro_z  = imu_dict["gyro_z"],
        )

        # Call C: sensor fusion -> attitude estimate
        attitude = _AttitudeOutput()
        self._lib.fc_update(
            ctypes.byref(imu_c),
            ctypes.c_float(dt),
            ctypes.byref(attitude),
        )

        # Call C: motor mixing -> PWM outputs
        motors = _MotorOutput()
        self._lib.fc_motor_mixing(
            ctypes.c_float(attitude.roll),
            ctypes.c_float(attitude.pitch),
            ctypes.byref(motors),
        )

        return {
            "roll":  round(attitude.roll,  4),
            "pitch": round(attitude.pitch, 4),
            "pwm":   [motors.m1, motors.m2, motors.m3, motors.m4],
        }


# =============================================================================
# DASHBOARD HELPERS
# =============================================================================

def _bar(value: float, min_val: float, max_val: float, width: int = 20) -> str:
    """Render a plain-ASCII progress bar."""
    ratio  = max(0.0, min(1.0, (value - min_val) / (max_val - min_val)))
    filled = int(ratio * width)
    return "#" * filled + "." * (width - filled)


def _pwm_bar(pwm: int, width: int = 14) -> str:
    return "=" * int((pwm - 1000) / 1000 * width) + "." * (width - int((pwm - 1000) / 1000 * width))


# =============================================================================
# MAIN LOOP -- SITL simulation dashboard
# =============================================================================

def main():
    """
    Main SITL loop.

    1. VirtualSensors  ->  generates fake IMU + battery data  (Python)
    2. SITLBridge      ->  passes IMU to C .dll, gets roll/pitch/PWM back
    3. Dashboard       ->  prints formatted live view, refreshed every 0.1s
    """
    sensors = VirtualSensors()
    bridge  = SITLBridge()

    cycle = 0

    while True:
        cycle += 1

        # --- Layer 1: Read virtual sensors (Python) ---
        imu = sensors.read_mpu6050()
        bat = sensors.read_battery()

        # --- Layer 2: Process in Embedded C via ctypes bridge ---
        fc_out = bridge.process(imu)

        roll  = fc_out["roll"]
        pitch = fc_out["pitch"]
        pwm   = fc_out["pwm"]

        # --- Layer 3: Render dashboard (Python) ---
        os.system("cls" if os.name == "nt" else "clear")

        W   = 62
        div = "+" + "-" * (W - 2) + "+"

        def row(text=""):
            print(f"| {text:<{W - 3}}|")

        batt_warn = "!! LOW BATT !!" if bat["percent"] < 20 else ""

        print(div)
        row(" DRONE FLIGHT COMPUTER  --  SITL DASHBOARD")
        row(f" Cycle: {cycle:<6}  Elapsed: {bat['elapsed']:>7.2f}s   [C firmware active]")
        print(div)
        row(" [ INPUT: Virtual MPU-6050 Sensor Data (Python) ]")
        row(f"  Accel  X:{imu['accel_x']:+7.4f}g   Y:{imu['accel_y']:+7.4f}g   Z:{imu['accel_z']:+7.4f}g")
        row(f"  Gyro   X:{imu['gyro_x']:+7.4f}d/s  Y:{imu['gyro_y']:+7.4f}d/s  Z:{imu['gyro_z']:+7.4f}d/s")
        print(div)
        row(" [ PROCESSING: Embedded C Flight Controller (.dll) ]")
        row("  Complementary Filter (alpha=0.96) + atan2 attitude:")
        row(f"  Roll  : {roll:+8.4f} deg   [{_bar(roll,  -30, 30)}]")
        row(f"  Pitch : {pitch:+8.4f} deg   [{_bar(pitch, -30, 30)}]")
        print(div)
        row(" [ OUTPUT: Motor PWM Commands (from C motor_mixing) ]")
        row(f"  M1 FL: {pwm[0]:4d}us [{_pwm_bar(pwm[0])}]   M2 FR: {pwm[1]:4d}us [{_pwm_bar(pwm[1])}]")
        row(f"  M3 RL: {pwm[2]:4d}us [{_pwm_bar(pwm[2])}]   M4 RR: {pwm[3]:4d}us [{_pwm_bar(pwm[3])}]")
        print(div)
        row(" [ Battery -- 3S LiPo ]")
        row(f"  {bat['voltage']:5.3f}V  [{_bar(bat['voltage'], 10.5, 12.6)}]  {bat['percent']:5.1f}%  {batt_warn}")
        print(div)
        print("  Press Ctrl+C to exit.")

        time.sleep(0.1)


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[SITL] Simulation terminated by user.")
    except FileNotFoundError as e:
        print(f"\n[SITL ERROR] {e}")
        sys.exit(1)
