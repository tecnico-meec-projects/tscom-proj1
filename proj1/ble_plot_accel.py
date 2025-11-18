import asyncio
import threading
import time
from collections import deque

from bleak import BleakClient
from bleak import BleakScanner
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("QtAgg")



DEVICE_MAC = "E6:1B:97:99:AF:26"  #Nano33 BLE MAC

UUIDS = {
    "ax": "19B10002-E8F2-537E-4F6C-D104768A1214",
    "ay": "19B20001-E8F2-537E-4F6C-D104768A1214",
    "az": "19B20002-E8F2-537E-4F6C-D104768A1214",
}

WINDOW_SIZE = 300  # Number of samples (30s at 10 Hz)



t0 = time.time()
times = deque(maxlen=WINDOW_SIZE)
values = {k: deque(maxlen=WINDOW_SIZE) for k in UUIDS}
data_lock = threading.Lock()


def now_rel():
    return time.time() - t0



def make_handler(key):
    def handler(sender, data: bytearray):
        try:
            val = float(data.decode("utf-8").strip())
        except Exception:
            return
        with data_lock:
            if key == "ax":
                times.append(now_rel())
            values[key].append(val)
    return handler




async def ble_task():
    print("Scanning for BLE device for 10 seconds...")
    device = await BleakScanner.find_device_by_address(DEVICE_MAC, timeout=10.0)
    if device is None:
        print(f"Device with MAC {DEVICE_MAC} not found. Make sure it's advertising!")
        return

    print(f"Found: {device.address} ({device.name})")
    async with BleakClient(device) as client:
        print("Connected to BLE device.")
        for key, uuid in UUIDS.items():
            await client.start_notify(uuid, make_handler(key))
        print("Notifications active (Ax, Ay, Az). Receiving data...")
        while True:
            await asyncio.sleep(1)


def run_ble_loop():
    while True:
        try:
            asyncio.run(ble_task())
        except Exception as e:
            print(f"BLE error: {e}, retrying in 5s...")
            time.sleep(5)




ble_thread = threading.Thread(target=run_ble_loop, daemon=True)
ble_thread.start()




plt.ion()
fig, ax = plt.subplots()
line_x, = ax.plot([], [], label="Ax (g)")
line_y, = ax.plot([], [], label="Ay (g)")
line_z, = ax.plot([], [], label="Az (g)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Acceleration (g)")
ax.set_title("IMU Acceleration (3 axes) - Real Time")
ax.legend()
ax.grid(True)



while True:
    with data_lock:

        n = len(times)
        if n == 0 or len(values["ax"]) < n or len(values["ay"]) < n or len(values["az"]) < n:
            continue


        t_list = list(times)
        ax_list = list(values["ax"])[-n:]
        ay_list = list(values["ay"])[-n:]
        az_list = list(values["az"])[-n:]


    line_x.set_data(t_list, ax_list)
    line_y.set_data(t_list, ay_list)
    line_z.set_data(t_list, az_list)
    ax.relim()
    ax.autoscale_view()
    plt.pause(0.05)
