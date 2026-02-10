# Autonomous Square Waypoint Mission

This project implements an autonomous mission using a finite state machine:
MANUAL → ARMING → TAKEOFF → WAYPOINT → LANDING → DISARMING → MANUAL

## What it does
- Arms and takes off to 3 meters
- Flies a square using 4 waypoints in local NED coordinates
- Lands and disarms automatically

## Demo
<p align="center">
<img src= "autonomy/square_mission/demo_uav.gif" width="350"/>
</p>


## How to run
```bash
python square_waypoint_mission.py --host 127.0.0.1 --port 5760
```
## Key implementation details

- Event-driven callbacks (LOCAL_POSITION, LOCAL_VELOCITY, STATE)

- Waypoint arrival detection using XY distance threshold

- Waypoints generated relative to current local position

## Future improvements

- smoother cornering / trajectory tracking

- yaw control at each waypoint

- more robust “arrival” logic using velocity + position
