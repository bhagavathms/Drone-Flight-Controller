# 🚁 Drone Flight Computer (SITL) — Complete Guide & Interview Playbook

Welcome to the complete, beginner-friendly yet interview-grade breakdown of this **Drone Flight Computer Software-in-the-Loop (SITL) Simulation** project.

---

## 📚 Table of Contents
1. [Part 1: The Big Picture (Explained Like You're 10 Years Old)](#part-1-the-big-picture-explained-like-youre-10-years-old)
2. [Part 2: Drone Flight Physics & Math Deep Dive](#part-2-drone-flight-physics--math-deep-dive)
3. [Part 3: Complete Line-by-Line Code Breakdown](#part-3-complete-line-by-line-code-breakdown)
4. [Part 4: Embedded & Flight Control Interview Q&A](#part-4-embedded--flight-control-interview-qa)

---

# Part 1: The Big Picture (Explained Like You're 10 Years Old)

Imagine you are trying to balance a heavy tray with four tiny toy helicopters attached to its corners. If the tray tilts even a tiny bit to the left, you need to instantly speed up the left motors and slow down the right motors to keep it flat.

A **Drone Flight Computer** is the robot brain doing this exact balancing trick **thousands of times every second**.

### What is SITL (Software-in-the-Loop)?
If you make a tiny mistake in your drone code on a real physical drone, it will flip upside down, smash into a wall, and break into a hundred pieces ($500 gone in 1 second!).

**SITL** is like a **video game simulator for your code**:
- **Python** plays the role of the **Environment / Physics Engine**: It creates fake gravity, fake wind, shaking sensor noise, and drains a virtual battery.
- **Embedded C** plays the role of the **Real Drone Brain**: It reads the fake sensor numbers, does math super fast, and decides how fast the 4 motors should spin.
- **ctypes** is the **Walkie-Talkie**: It passes messages between Python and C in shared computer memory.

```
       [ PYTHON LAYER ]                                [ EMBEDDED C LAYER ]
┌──────────────────────────────┐              ┌──────────────────────────────────┐
│  VirtualSensors              │              │  flight_controller.c             │
│  - Fake MPU-6050 (IMU)       │───IMU Data──▶│  - Sensor Fusion (Filter)        │
│  - Fake LiPo Battery         │  (accel/gyro)│  - Attitude Estimation (Angle)   │
│                              │              │  - Quadcopter Motor Mixer        │
│  SITLBridge (ctypes)         │◀──PWM Speed──│  - Safety Limit Clamping         │
│  - Memory Translator         │  (4 Motors)  └──────────────────────────────────┘
│                              │
│  Live Dashboard Display UI   │
└──────────────────────────────┘
```

---

# Part 2: Drone Flight Physics & Math Deep Dive

### 1. Accelerometer vs. Gyroscope (The MPU-6050 IMU)
Inside a drone, the IMU chip contains two main sensors:

| Sensor | Measures | Strengths | Weaknesses |
|---|---|---|---|
| **Gyroscope** | Angular rotation rate ($\text{deg/s}$) | Extremely fast and smooth | **Drifts over time** (1° off every few seconds) |
| **Accelerometer** | Acceleration / Gravity ($g$) | Never drifts (gravity always points DOWN) | **Super noisy** (vibrates wildly from spinning motors) |

### 2. Finding Angles with Accelerometer (`atan2`)
When the drone tilts by Roll ($\theta$) or Pitch ($\phi$), the force of gravity ($1.0g$) splits across the X, Y, and Z axes. Trigonometry brings it back:

$$\text{Roll Angle} = \text{atan2}(a_y, a_z) \times \frac{180}{\pi}$$

$$\text{Pitch Angle} = \text{atan2}(-a_x, \sqrt{a_y^2 + a_z^2}) \times \frac{180}{\pi}$$

> **Why `atan2(y, x)` instead of standard `atan(y / x)`?**
> Standard `atan` fails when $x = 0$ (division by zero error!) and cannot distinguish between opposite quadrants (e.g. $+45^\circ$ vs $-135^\circ$). `atan2` safely handles all 4 quadrants from $-180^\circ$ to $+180^\circ$ without crashing.

### 3. Sensor Fusion: The Complementary Filter
We combine the smooth Gyroscope and stable Accelerometer using a **Complementary Filter**:

$$\text{Angle}_{\text{new}} = \alpha \cdot (\text{Angle}_{\text{old}} + \text{Gyro} \cdot \Delta t) + (1 - \alpha) \cdot \text{Accel\_Angle}$$

With $\alpha = 0.96$:
- **96% weight** goes to integrated Gyroscope (High-Pass Filter — removes slow drift).
- **4% weight** goes to Accelerometer (Low-Pass Filter — smooths out motor vibrations).

### 4. Quadcopter Motor Mixing Matrix
A Quadcopter in **+ (Plus) Configuration** has 4 motors:
- **M1 (Front-Left, CW)**
- **M2 (Front-Right, CCW)**
- **M3 (Rear-Left, CCW)**
- **M4 (Rear-Right, CW)**

To correct Roll and Pitch:
$$\text{M1 (Front-Left)}  = \text{Throttle} - \text{RollCmd} + \text{PitchCmd}$$
$$\text{M2 (Front-Right)} = \text{Throttle} + \text{RollCmd} + \text{PitchCmd}$$
$$\text{M3 (Rear-Left)}   = \text{Throttle} - \text{RollCmd} - \text{PitchCmd}$$
$$\text{M4 (Rear-Right)}  = \text{Throttle} + \text{RollCmd} - \text{PitchCmd}$$

### 5. Pulse Width Modulation (PWM) Signals
Electronic Speed Controllers (ESCs) expect standard RC PWM pulse widths:
- **$1000\,\mu\text{s}$ (1.0 ms)** = Motor Idle / Stopped.
- **$1500\,\mu\text{s}$ (1.5 ms)** = 50% Hover Throttle.
- **$2000\,\mu\text{s}$ (2.0 ms)** = 100% Full Throttle.

---

# Part 3: Complete Line-by-Line Code Breakdown

## 1. `flight_controller.h` (C Header File)

```c
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H
```
> **Include Guards**: Prevents syntax errors if multiple C files `#include "flight_controller.h"`.

```c
#ifdef _WIN32
    #define FC_EXPORT __declspec(dllexport)
#else
    #define FC_EXPORT __attribute__((visibility("default")))
#endif
```
> **DLL Export Macro**: On Windows, functions in a `.dll` are hidden by default unless tagged with `__declspec(dllexport)`. On Linux, gcc uses `__attribute__((visibility("default")))`.

```c
typedef struct {
    float accel_x;   /* (g) */
    float accel_y;   /* (g) */
    float accel_z;   /* (g) */
    float gyro_x;    /* (deg/s) */
    float gyro_y;    /* (deg/s) */
    float gyro_z;    /* (deg/s) */
} IMUData;
```
> **C Struct**: Stores raw sensor data. Contains 6 single-precision floats (4 bytes each = 24 contiguous bytes in RAM).

---

## 2. `flight_controller.c` (Embedded C Engine)

```c
#define CF_ALPHA       0.96f
#define BASE_THROTTLE  1500
#define KP_ATTITUDE    5.0f
#define PWM_MIN        1000
#define PWM_MAX        2000

static float g_roll  = 0.0f;
static float g_pitch = 0.0f;
```
> **`static` Scope Keyword**: Restricts `g_roll` and `g_pitch` to *this file only* (Internal Linkage). Encapsulates state so outside code cannot corrupt memory directly.

```c
void fc_update(const IMUData* imu, float dt, AttitudeOutput* output)
{
    float ax = imu->accel_x;
    float ay = imu->accel_y;
    float az = imu->accel_z;
    float gx = imu->gyro_x;
    float gy = imu->gyro_y;

    if (dt < 1e-6f) dt = 1e-6f; // Prevent division by zero

    float accel_roll  = (float)(atan2(ay, az) * (180.0 / M_PI));
    float accel_pitch = (float)(atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / M_PI));

    g_roll  = CF_ALPHA * (g_roll  + gx * dt) + (1.0f - CF_ALPHA) * accel_roll;
    g_pitch = CF_ALPHA * (g_pitch + gy * dt) + (1.0f - CF_ALPHA) * accel_pitch;

    output->roll  = g_roll;
    output->pitch = g_pitch;
}
```
> Performs sensor fusion math in pure C. Takes pointers (`IMUData*`) for zero-copy memory performance.

---

## 3. `flight_sim.py` (Python SITL Harness & ctypes Bridge)

```python
class _IMUData(ctypes.Structure):
    _fields_ = [
        ("accel_x", ctypes.c_float),
        ("accel_y", ctypes.c_float),
        ("accel_z", ctypes.c_float),
        ("gyro_x",  ctypes.c_float),
        ("gyro_y",  ctypes.c_float),
        ("gyro_z",  ctypes.c_float),
    ]
```
> **ctypes Struct Mirroring**: Python class matching the exact memory layout of the C `IMUData` struct byte-for-byte.

```python
self._lib = ctypes.CDLL(lib_path)

lib.fc_update.argtypes = [
    ctypes.POINTER(_IMUData),
    ctypes.c_float,
    ctypes.POINTER(_AttitudeOutput),
]
lib.fc_update.restype = None
```
> Loads `flight_controller.dll` into Python memory space and declares C parameter types (`argtypes`) to prevent segmentation faults.

---

## 4. `build.bat` (Windows Build Script)

```cmd
SET MSYS2_GCC=C:\msys64\usr\bin\bash.exe
SET BUILD_CMD=export PATH=/mingw64/bin:/usr/bin:$PATH; gcc -shared -o flight_controller.dll flight_controller.c -lm -Wall -O2
```
> Compiles C code using 64-bit GCC into a `.dll`.
> - `-shared`: Produce dynamic link library.
> - `-lm`: Link standard math library (`math.h`).
> - `-O2`: Level 2 compiler optimization for max execution speed.

---

# Part 4: Embedded & Flight Control Interview Q&A

### Q1: Why did you split the architecture into Python for sensors/UI and C for control logic?
**Answer**: Real flight hardware (like STM32 microcontrollers) cannot run Python efficiently due to garbage collection delays, dynamic typing overhead, and high memory footprint. By isolating all sensor fusion and motor mixing in Embedded C, the exact same C logic can be compiled into a `.dll` for SITL testing, or flashed directly onto an ARM Cortex-M microcontroller using GCC ARM.

---

### Q2: What is the difference between a Complementary Filter and a Kalman Filter? Which one is better?
**Answer**:
- **Complementary Filter**: Uses a fixed weighting factor ($\alpha = 0.96$) blending high-pass filtered gyro data and low-pass filtered accel data. Computationally lightweight ($O(1)$ complexity, zero matrix inversions), perfect for 8-bit or 32-bit microcontrollers without FPU.
- **Kalman Filter**: Continuously updates an optimal Kalman Gain $K$ based on dynamic noise covariance matrices ($Q$ and $R$). Higher accuracy under non-stationary noise, but computationally expensive (matrix multiplication & inversion $O(n^3)$).
- *Trade-off*: Complementary filters are preferred on resource-constrained embedded targets when loop execution rate (>400Hz) matters more than marginal noise reduction.

---

### Q3: How does Python's `ctypes` pass memory to C? What can go wrong?
**Answer**:
`ctypes` passes memory by pointer reference (`ctypes.byref()`).
**Key Failure Modes**:
1. **Struct Padding / Memory Alignment**: If C struct members are aligned to 8-byte boundaries and Python assumes 4-byte boundaries, data fields corrupt.
2. **Bitness Mismatch**: Loading a 32-bit `.dll` in a 64-bit Python interpreter causes `WinError 193 (%1 is not a valid Win32 application)`.
3. **Data Type Mismatches**: Passing `c_int` where C expects `c_float` corrupts floating-point register interpretation.

---

### Q4: Why use `atan2(y, x)` instead of `atan(y / x)` in embedded flight control?
**Answer**:
1. `atan(y / x)` throws division-by-zero exceptions when $x = 0$ (e.g., drone pitch at vertical $90^\circ$).
2. `atan2(y, x)` inspects the signs of both inputs to return the true quadrant angle in range $(-\pi, +\pi]$, whereas `atan` is ambiguous across opposite quadrants.

---

### Q5: What is the purpose of the `static` keyword for global state variables in C?
**Answer**:
In C, `static` applied to file-scope variables (`static float g_roll;`) grants **internal linkage**. This prevents other `.c` compilation units from accessing or modifying `g_roll` directly, maintaining modular encapsulation and avoiding linker name collisions.

---

### Q6: How do you prevent PWM motor signal overflow/underflow in firmware?
**Answer**:
Through defensive clamping:
```c
static int clamp_int(int value, int lo, int hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}
```
This guarantees motor speed outputs never violate standard ESC pulse limits ($1000\,\mu\text{s}$ to $2000\,\mu\text{s}$), preventing motor desync or physical hardware strain during severe tilt maneuvers.

---

## 🎯 Summary
You now possess a complete, professional, Software-in-the-Loop drone flight controller implementation with **Embedded C processing**, **Python SITL harness**, **FFI ctypes bridge**, and **interview-grade theoretical understanding**.
