#!/usr/bin/env python3
# credit to https://github.com/SkoltechRobotics/rplidar/blob/master/rplidar.py for the example
"""Animates distances and measurment quality"""
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

PORT_NAME = "/dev/ttyUSB0"
DMAX = 4000
IMIN = 0
IMAX = 50


def update_line(num, iterator, line, localizer):
    scan = next(iterator)
    cam_detections = localizer.getDetections()

    # use the lidar data too
    offsets = [(np.radians(meas[1]), meas[2]) for meas in scan]
    intens = [meas[0] for meas in scan]

    # for testing purposes to see just the camera
    # offsets = [(0, 0)]
    # intens = [0]

    # add cam detections
    for cam_detection in cam_detections:
        positive_val = (
            cam_detection.angle
            if cam_detection.angle > 0
            else cam_detection.angle + 360
        )
        positive_val = int(positive_val)
        distance = cam_detection.distance

        offsets.append((np.radians(positive_val), distance))
        intens.append(20)  # arbitrary intensity

    line.set_offsets(np.array(offsets))
    line.set_array(np.array(intens))

    return (line,)


def run():
    lidar = RPLidar(PORT_NAME)

    fig = plt.figure()
    ax = plt.subplot(111, projection="polar")
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=plt.cm.Greys_r, lw=0)
    print(line)
    ax.set_rmax(DMAX)
    ax.grid(True)

    # fix bc my env bugs when doing this before
    import sys

    sys.path.append("../../")
    from Vision.modules.localization import CameraLocalizer

    localizer = CameraLocalizer(
        "../../Vision/modules/trained_models/rockdetection/best-simplified5s.blob"
    )
    localizer.start(blocking=False)

    # regular stuff
    iterator = lidar.iter_scans()
    ani = animation.FuncAnimation(
        fig, update_line, fargs=(iterator, line, localizer), interval=50
    )
    plt.show()
    lidar.stop()
    localizer.stop()
    lidar.disconnect()


if __name__ == "__main__":
    run()
