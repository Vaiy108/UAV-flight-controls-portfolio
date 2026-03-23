import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):
    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """Handle local position updates during takeoff and waypoint flight."""
        if self.flight_state == States.TAKEOFF:
            # Local NED uses negative Z for altitude above the home frame.
            altitude = -self.local_position[2]

            # Once the drone reaches 95% of the commanded altitude,
            # generate the mission waypoints and begin waypoint flight.
            if altitude > 0.95 * self.target_position[2]:
                if not self.all_waypoints:
                    self.all_waypoints = self.calculate_box()
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            # Check horizontal distance to the current waypoint.
            horizontal_distance = np.linalg.norm(
                self.target_position[0:2] - self.local_position[0:2]
            )

            # When close enough to the current target, move to the next waypoint
            # or begin landing if the mission is complete.
            if horizontal_distance < 0.5:
                if self.all_waypoints:
                    self.waypoint_transition()
                else:
                    self.landing_transition()

    def velocity_callback(self):
        """Handle velocity updates during landing."""
        if self.flight_state == States.LANDING:
            # Confirm landing using both global altitude relative to home
            # and local altitude near zero.
            landed_globally = (self.global_position[2] - self.global_home[2]) < 0.1
            landed_locally = abs(self.local_position[2]) < 0.01

            if landed_globally and landed_locally:
                self.disarming_transition()

    def state_callback(self):
        """Handle state transitions based on arming/disarming status."""
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()

        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()

        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def calculate_box(self):
        """Generate a square waypoint mission in local NED coordinates."""
        side_length = 5.0
        altitude = 3.0

        # Start relative to the current local position.
        north0 = self.local_position[0]
        east0 = self.local_position[1]

        waypoints = [
            [north0 + side_length, east0, altitude, 0.0],
            [north0 + side_length, east0 + side_length, altitude, 0.0],
            [north0, east0 + side_length, altitude, 0.0],
            [north0, east0, altitude, 0.0],
        ]
        return waypoints

    def arming_transition(self):
        """Take control, arm the drone, and set the current home position."""
        print("arming transition")

        self.take_control()
        self.arm()

        self.set_home_position(
            self.global_position[0],
            self.global_position[1],
            self.global_position[2],
        )

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """Command takeoff to the mission altitude."""
        print("takeoff transition")

        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)

        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """Command the next waypoint in the mission."""
        print("waypoint transition")

        north, east, altitude, heading = self.all_waypoints.pop(0)
        self.target_position = np.array([north, east, altitude])

        # Command position in local NED coordinates.
        self.cmd_position(north, east, altitude, heading)

        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """Command landing."""
        print("landing transition")

        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """Disarm the drone after landing."""
        print("disarm transition")

        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """Release control, stop logging/connection, and end the mission."""
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """Start logging and begin the drone connection."""
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5760, help="Port number")
    parser.add_argument(
        "--host",
        type=str,
        default="127.0.0.1",
        help="Host address, e.g. '127.0.0.1'",
    )
    args = parser.parse_args()

    conn = MavlinkConnection(
        f"tcp:{args.host}:{args.port}",
        threaded=False,
        PX4=False,
    )
    # conn = WebSocketConnection(f"ws://{args.host}:{args.port}")

    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
