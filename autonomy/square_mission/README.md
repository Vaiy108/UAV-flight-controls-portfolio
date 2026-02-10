# Autonomous Square Waypoint Mission

## Overview
This project implements a waypoint navigation mission using the Udacity drone API.
The drone autonomously arms, takes off, flies a square trajectory, lands, and disarms.

## Skills Demonstrated

- Finite state machine design

- MAVLink / simulator integration

- Local NED coordinate control

- Event-driven callbacks

- Autonomous mission sequencing

This project implements an autonomous mission using a finite state machine:
MANUAL → ARMING → TAKEOFF → WAYPOINT → LANDING → DISARMING → MANUAL

## What it does
- Arms and takes off to 3 meters
- Flies a square using 4 waypoints in local NED coordinates
- Lands and disarms automatically

## Demo
<p align="center">
<img src= "demo_uav.gif" width="400"/>
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
