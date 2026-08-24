# 🛸 Drone Flight Controller — SITL Simulation

Software-in-the-Loop (SITL) simulation for a quadcopter drone flight computer. 

This project decouples hardware simulation from firmware processing: **virtual hardware sensors and UI dashboard run in Python**, while all **flight control algorithms (sensor fusion, attitude estimation, motor mixing) execute in high-performance Embedded C** compiled as a native shared library (`flight_controller.dll` / `.so`).

---

## 🏗️ Architecture

```
                    PYTHON LAYER                                EMBEDDED C LAYER
      ┌──────────────────────────────────────┐        ┌───────────────────────────────────┐
      │  VirtualSensors                      │        │  flight_controller.c              │
      │  - Simulated MPU-6050 (Accel/Gyro)   │        │  - atan2 Roll/Pitch Estimation    │
      │  - 3S LiPo Battery Discharge Model   │──IMU──▶│  - Complementary Filter           │
      │                                      │        │  - Quadcopter Motor Mixing Matrix │
      │  SITLBridge (ctypes)                 │◀──PWM──│  - ESC PWM Clamping (1000-2000µs)│
      │  - Bridges C struct memory layouts   │        └───────────────────────────────────┘
      │                                      │
      │  Live Dashboard / Terminal UI        │
      └──────────────────────────────────────┘
```

---

## 📁 Repository Structure

- `flight_controller.c` — Embedded C firmware containing core flight control algorithms.
- `flight_controller.h` — C header defining memory-aligned structs (`IMUData`, `AttitudeOutput`, `MotorOutput`) and API declarations.
- `flight_controller.dll` — 64-bit compiled dynamic link library (Windows).
- `flight_sim.py` — Python SITL harness providing virtual sensor inputs, `ctypes` bindings, and live terminal dashboard.
- `build.bat` — One-click Windows build script to recompile C code with 64-bit MinGW GCC.

---

## ⚡ Key Technical Features

1. **Embedded C Processing Engine (`flight_controller.c`)**:
   - **Attitude Estimation**: Trigonometric roll and pitch calculation using `atan2`.
   - **Sensor Fusion**: Complementary filter blending high-frequency gyroscope integration with low-frequency accelerometer gravity vector (`α = 0.96`).
   - **Motor Mixing**: Standard Quad (+) quadcopter mixing matrix mapping attitude correction to 4 motor outputs.
   - **ESC Protection**: PWM outputs hard-clamped between `1000 µs` (idle) and `2000 µs` (full throttle).

2. **Virtual Hardware Abstraction Layer (`flight_sim.py`)**:
   - Simulated 6-DOF MPU-6050 IMU generating realistic sinusoidal sway and Gaussian sensor noise.
   - Simulated 3S LiPo battery discharge curve (12.6V down to 10.5V minimum).

3. **Ctypes FFI Bridge**:
   - Direct memory-mapped struct passing between Python and compiled C logic without process IPC overhead.

---

## 🚀 How to Run

### Quick Start
Make sure Python 3.x is installed:
```bash
python flight_sim.py
```

### Rebuilding C Shared Library

#### On Windows (via MinGW-w64 / MSYS2):
Double-click `build.bat` or run:
```cmd
build.bat
```

#### On Linux / macOS (via GCC):
```bash
gcc -shared -fPIC -o flight_controller.so flight_controller.c -lm -Wall -O2
```

---

## 🛠️ Work in Progress & Roadmap

- [x] SITL Virtual Sensor Suite (IMU + Battery)
- [x] Embedded C Flight Controller Engine
- [x] Complementary Filter & Motor Mixing Matrix
- [x] Python ctypes C-to-Python SITL Harness
- [ ] Magnetometer-based Yaw Estimation (9-DOF fusion)
- [ ] PID Controller Loop (Roll, Pitch, Yaw rate control)
- [ ] FreeRTOS Task Scheduling & Real-Time Execution
- [ ] Wireless Telemetry Interface
- [ ] Physical Microcontroller Porting (STM32F4)

---

## 🎯 Skills & Concepts Demonstrated

- **Embedded Systems & C**: Pointers, structs, static memory management, sensor math.
- **Flight Control Engineering**: Attitude kinematics, complementary filtering, quadcopter motor dynamics.
- **Software Architecture**: Software-in-the-Loop simulation design, FFI / C Interop via `ctypes`.
- **Real-Time Systems**: Sensor sampling rates, timing deltas, ESC output safety bounds.
