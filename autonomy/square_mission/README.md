# Autonomous Square Waypoint Mission

This project implements an autonomous mission using a finite state machine:
MANUAL → ARMING → TAKEOFF → WAYPOINT → LANDING → DISARMING → MANUAL

## What it does
- Arms and takes off to 3 meters
- Flies a square using 4 waypoints in local NED coordinates
- Lands and disarms automatically

## Demo
![Square mission demo](demo.gif)

## How to run
```bash
python square_waypoint_mission.py --host 127.0.0.1 --port 5760

