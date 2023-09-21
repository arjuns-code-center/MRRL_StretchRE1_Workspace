# PyRPLiDAR package testing on RPLiDAR

from rplidar import RPLidar
import time

start = time.time()
lidar = RPLidar('COM4')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

while True:
    t = time.time()
    
    for i, scan in enumerate(lidar.iter_scans(20000)):
        for s in scan:
            print("Angle: {}, Distance: {}".format(s[1], s[2]))

    if t - start > 10:
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()