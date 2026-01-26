import time
import cflib.crtp
from cflib.crazyflie import Crazyflie


URI = "radio://0/80/2M"   # change to 1M if that's what you actually fly with

# Thrust tuning (conservative starting thrust)
HOVER_THRUST = 28000
MAX_THRUST   = 32000

# Control rate
RATE_HZ = 50
DT = 1.0 / RATE_HZ

# Maneuver tuning 
TILT_DEG   = 4.0      # roll/pitch in degrees (<= 5 indoors)
LEG_TIME_S = 0.8      # seconds per leg
PAUSE_S    = 0.3      # hover pause between legs
SETTLE_S   = 1.0      # settle before/after maneuver

RAMP_START = 0
RAMP_STEP  = 1000
RAMP_HOLD  = 0.2

cf = None

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def send_setpoint(roll=0.0, pitch=0.0, yawrate=0.0, thrust=0):
    thrust = clamp(int(thrust), 0, MAX_THRUST)
    cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

def send_for(seconds, roll=0.0, pitch=0.0, yawrate=0.0, thrust=0):
    end = time.time() + seconds
    while time.time() < end:
        send_setpoint(roll, pitch, yawrate, thrust)
        time.sleep(DT)

def ramp_to_hover():
    # Unlock by streaming 0 thrust briefly
    send_for(0.4, thrust=0)
    for t in range(RAMP_START, HOVER_THRUST + 1, RAMP_STEP):
        send_for(RAMP_HOLD, thrust=t)

def safe_stop():
    try:
        send_for(0.8, thrust=0)
    except Exception:
        pass

def run_square():
    # settle hover
    send_for(SETTLE_S, thrust=HOVER_THRUST)

    # Square: forward, right, back, left
    send_for(LEG_TIME_S, pitch=+TILT_DEG, thrust=HOVER_THRUST)  # forward
    send_for(PAUSE_S, thrust=HOVER_THRUST)

    send_for(LEG_TIME_S, roll=+TILT_DEG, thrust=HOVER_THRUST)   # right
    send_for(PAUSE_S, thrust=HOVER_THRUST)

    send_for(LEG_TIME_S, pitch=-TILT_DEG, thrust=HOVER_THRUST)  # back
    send_for(PAUSE_S, thrust=HOVER_THRUST)

    send_for(LEG_TIME_S, roll=-TILT_DEG, thrust=HOVER_THRUST)   # left
    send_for(SETTLE_S, thrust=HOVER_THRUST)

def connected(uri):
    print(f"Connected: {uri}")
    time.sleep(0.5)

    try:
        ramp_to_hover()
        run_square()
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

    # Keep alive long enough for callbacks to run
    time.sleep(20)
