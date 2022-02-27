import time

from vl53l5cx.vl53l5cx import VL53L5CX


driver = VL53L5CX()

alive = driver.is_alive()
if not alive:
    raise IOError("VL53L5CX Device is not alive")

print("Initialising...")
t = time.time()
driver.init()
print(f"Initialised ({time.time() - t:.1f}s)")


# Ranging:
driver.start_ranging()

previous_time = 0
loop = 0
while loop < 10:
    if driver.check_data_ready():
        ranging_data = driver.get_ranging_data()

        # As the sensor is set in 4x4 mode by default, we have a total 
        # of 16 zones to print. For this example, only the data of first zone are 
        # print
        now = time.time()
        if previous_time != 0:
            time_to_get_new_data = now - previous_time
            print(f"Print data no : {driver.streamcount: >3d} ({time_to_get_new_data * 1000:.1f}ms)")
        else:
            print(f"Print data no : {driver.streamcount: >3d}")

        for i in range(16):
            print(f"Zone : {i: >3d}, "
                  f"Status : {ranging_data.target_status[driver.nb_target_per_zone * i]: >3d}, "
                  f"Distance : {ranging_data.distance_mm[driver.nb_target_per_zone * i]: >4.0f} mm")

        print("")

        previous_time = now
        loop += 1

    time.sleep(0.005)
