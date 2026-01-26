import time
import cflib.crtp
from cflib.crazyflie import Crazyflie

URI = "radio://0/80/2M"

# SAFETY: props on only in a clear indoor space.
# First run: keep MAX_THRUST low.
START_THRUST = 8000
MAX_THRUST   = 32000
STEP         = 1000

# How long to hold each thrust value (seconds)
STEP_HOLD_S  = 0.25

# Send rate (Hz). Crazyflie expects continuous setpoints.
SEND_HZ = 50
DT = 1.0 / SEND_HZ

cf = None


def send_for(seconds, thrust, roll=0.0, pitch=0.0, yawrate=0.0):
    end = time.time() + seconds
    while time.time() < end:
        cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        time.sleep(DT)


def safe_stop():
    # Stop motors cleanly
    try:
        send_for(0.6, thrust=0)
    except Exception:
        pass


def connected(uri):
    print(f" Connected: {uri}")
    time.sleep(0.5)

    print("\n--- Lift Thrust Calibration ---")
    print("Purpose: find approximate thrust value where the drone first becomes light / lifts off.")
    print("Operator action: WATCH the drone and note the first thrust where lift-off occurs.")
    print("SAFETY: be ready to catch/kill by unplugging USB or stopping script.\n")

    lift_thrust = None

    try:
        # Unlock by streaming 0 thrust
        send_for(0.4, thrust=0)

        thrust = START_THRUST
        while thrust <= MAX_THRUST:
            send_for(STEP_HOLD_S, thrust=thrust)

            print(f"Thrust step: {thrust}  <-- note if lift starts here")
            # Optional: small pause between steps
            send_for(0.05, thrust=0)

            thrust += STEP

        print("\nCalibration finished.")
        print("If you observed lift-off, record that thrust value in your notes/README.")

    finally:
        safe_stop()
        print("Motors stopped")
        cf.close_link()


def connection_failed(uri, msg):
    print(f" Connection failed: {uri} | {msg}")


def disconnected(uri):
    print(f" Disconnected: {uri}")


if __name__ == "__main__":
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie()
    cf.connected.add_callback(connected)
    cf.connection_failed.add_callback(connection_failed)
    cf.disconnected.add_callback(disconnected)

    print(f"Connecting to {URI} ...")
    cf.open_link(URI)

    time.sleep(15)
