import time
import math
import cflib.crtp
from cflib.crazyflie import Crazyflie

URI = "radio://0/80/2M"   # can be changed to 1M if needed

HOVER_THRUST = 28000
MAX_THRUST   = 32000

RATE_HZ = 50
DT = 1.0 / RATE_HZ

TILT_DEG      = 4.0   # small circle indoors
CIRCLE_TIME_S = 6.0   # time for one loop
SETTLE_S      = 1.0

RAMP_STEP = 1000
RAMP_HOLD = 0.2

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
    send_for(0.4, thrust=0)
    for t in range(0, HOVER_THRUST + 1, RAMP_STEP):
        send_for(RAMP_HOLD, thrust=t)

def safe_stop():
    try:
        send_for(0.8, thrust=0)
    except Exception:
        pass

def run_circle():
    send_for(SETTLE_S, thrust=HOVER_THRUST)

    start = time.time()
    while (time.time() - start) < CIRCLE_TIME_S:
        phase = 2.0 * math.pi * (time.time() - start) / CIRCLE_TIME_S
        pitch =  TILT_DEG * math.cos(phase)
        roll  =  TILT_DEG * math.sin(phase)
        send_setpoint(roll=roll, pitch=pitch, yawrate=0.0, thrust=HOVER_THRUST)
        time.sleep(DT)

    send_for(SETTLE_S, thrust=HOVER_THRUST)

def connected(uri):
    print(f"Connected: {uri}")
    time.sleep(0.5)

    try:
        ramp_to_hover()
        run_circle()
    finally:
        safe_stop()
        print("Motors stopped")
        cf.close_link()

def connection_failed(uri, msg):
    print(f"Connection failed: {uri} | {msg}")

def disconnected(uri):
    print(f"Disconnected: {uri}")

if __name__ == "__main__":
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie()

    cf.connected.add_callback(connected)
    cf.connection_failed.add_callback(connection_failed)
    cf.disconnected.add_callback(disconnected)

    print(f"Connecting to {URI} ...")
    cf.open_link(URI)
    time.sleep(20)
