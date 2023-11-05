from math import floor, pi
from rplidar import RPLidar
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.axes import Axes
from threading import Thread
import time

PORT_NAME = 'COM5'
lidar = None
lidar_iter = None
while True:
    try:
        lidar = RPLidar(PORT_NAME)
        lidar_iter = iter(lidar.iter_scans(max_buf_meas=10000))
        next(lidar_iter)
    except Exception as e:
        lidar.disconnect()
        print(e)
        continue
    break

scan_data = [0]*360

def look(verbose=True):
    global scan_data
    scan = list(reversed(next(lidar_iter)))
    for (_, angle, distance) in scan:
        scan_data[floor(angle)%360] = distance
    print()
    print(scan_data)
    print()
def spin(stop_check, verbose=True):
    while not stop_check():
        look()

stop_spinning=False
t = Thread(target=spin, args=(lambda: stop_spinning, ))
t.start()

fig, axs = plt.subplots(ncols=2, subplot_kw={'projection': 'polar'})
axl: Axes = axs[0]
axl.set_title('Upside_down_lidar')
axl.set_rmax(4000)
scatL = axl.scatter([], [], marker='.')

axr: Axes = axs[1]
axr.set_title('Upside_up_lidar')
axr.set_rmax(4000)
scatR = axr.scatter([], [], marker='.')
def update(_):
    scan_data_rev = list(reversed(scan_data))
    # RMAX=max(scan_data_rev)*1.2
    # axr.set_rmax(RMAX)
    scatR.set_offsets([(2*pi/360*i, scan_data_rev[i]) for i in range(0,360)])

    # axl.set_rmax(RMAX)
    scatL.set_offsets([(2*pi/360*i, scan_data[i]) for i in range(0,360)])
    return [scatR, scatL]

anime = anim.FuncAnimation(
    fig,
    update,
    1,
    interval=1,
    blit=True
)
plt.show()





stop_spinning=True
t.join()
lidar.stop()
lidar.stop_motor()
lidar.disconnect()