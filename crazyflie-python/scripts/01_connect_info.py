import time
import cflib.crtp
from cflib.crazyflie import Crazyflie

# Crazyflie 2.0 working URI (you can change channel if needed)
URI = "radio://0/80/2M"


def connected(uri):
    print(f" Connected: {uri}")
    # Give param table a moment to load
    time.sleep(1.0)

    # Read a few useful parameters (some may vary by firmware)
    params = [
        ("firmware.revision", "Firmware revision"),
        ("stabilizer.controller", "Stabilizer controller"),
        ("pm.vbat", "Battery voltage (V)"),
    ]

    for key, label in params:
        try:
            val = cf.param.get_value(key)
            print(f"{label}: {val}")
        except Exception as e:
            print(f"{label}: not available ({key}) [{e}]")

    print("Done. Closing link in 2 seconds...")
    time.sleep(2.0)
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

    # Keep the script alive long enough for callbacks
    time.sleep(10)
