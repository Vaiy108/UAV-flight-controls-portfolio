
# Crazyflie 2.0 – Python Flight Control Experiments (Hardware Integration)

This folder contains Python scripts and notes for Crazyflie 2.0 + Crazyradio:
- connection checks
- basic maneuvers
- telemetry logging

This section documents hands-on flight control experiments performed on a **Crazyflie 2.0 nano quadrotor** using Python and the Bitcraze `cflib`.  
The focus is on **real hardware interaction**, safe setpoint control, and basic maneuver execution.

## Hardware & Setup
- Drone: Crazyflie 2.0
- Radio: Crazyradio PA
- OS: Windows 10
- Control interface: Python + `cflib`
- Command mode: Direct attitude/thrust setpoints

All experiments were performed indoors with conservative thrust and tilt limits.

---

## Scripts Overview

### 01_connect_info.py
- Verifies Crazyradio communication
- Connects to the drone and reads basic firmware and battery parameters
- Safe to run with **propellers off**

### 02_lift_thrust_calibration.py
- Gradually ramps thrust to identify approximate lift-off thrust
- Used to characterize hover thrust for this specific platform
- Emphasizes safe ramping and clean motor stop

### 03_square_maneuver.py
- Executes a small square trajectory using roll/pitch tilt commands
- Demonstrates directional control and repeatable maneuver sequencing
- Uses low tilt angles suitable for indoor testing

### 04_circle_maneuver.py
- Executes a circular trajectory using sinusoidal roll/pitch commands
- Demonstrates smooth continuous control and timing-based trajectory shaping

---

## Running the Scripts

Install dependencies:
```bash
pip install -r requirements.txt

## Run a script (example):
python scripts/03_square_maneuver.py

Ensure the Crazyradio is connected and the drone battery is sufficiently charged before running any flight script.

##Safety Notes

All scripts stream setpoints continuously (≈50 Hz) as required by Crazyflie

Conservative thrust and tilt limits are used by default

Each script includes a clean motor stop on exit

Fly only in a clear indoor space and be ready to abort if needed
