from math import floor, pi
from rplidar import RPLidar
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.axes import Axes

PORT_NAME = 'COM5'
lidar = RPLidar(PORT_NAME)
lidar_iter = iter(lidar.iter_scans(max_buf_meas=10000))


def process_data(data):
    # for angle in range(360):
    #     distance = data[angle]
    #     print(distance)
    print()
    print(data)
    print()
    data = [x for x in data if x != 0]
    min_distance = min(data)
    max_distance = max(data)
    print("Minimum distance: ", min_distance)
    print("Maximum distance: ", max_distance)

scan_data = [0]*360

fig, axs = plt.subplots(ncols=2, subplot_kw={'projection': 'polar'})
axl: Axes = axs[0]
axl.set_title('Upside_down_lidar')
axr: Axes = axs[1]
axr.set_title('Upside_up_lidar')
def update(_):
    global lidar, lidar_iter
    try:
        scan = next(lidar_iter) 
    except Exception as e:
        lidar.disconnect()
        lidar = RPLidar(PORT_NAME)
        lidar_iter = iter(lidar.iter_scans(max_buf_meas=10000))
        print(e)
        return
    scan = list(reversed(scan))
    for (_, angle, distance) in scan:
        scan_data[min([359, floor(angle)])] = distance
    # process_data(scan_data)
    axl.clear()
    axl.plot([2*pi/360*i for i in range(0,360)], scan_data, ".")
    scan_data_rev = list(reversed(scan_data))
    axl.set_title('Upside_down_lidar')
    axr.clear()
    axr.plot([2*pi/360*i for i in range(0,360)], scan_data_rev, ".")
    axr.set_title('Upside_up_lidar')
    fig.tight_layout()

    return

anime = anim.FuncAnimation(
    fig,
    update,
    1,
    interval=1,
    # blit=True
)
plt.show()
lidar.stop()
lidar.stop_motor()
lidar.disconnect()