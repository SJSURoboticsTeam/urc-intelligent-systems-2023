import sys
sys.path.append( '../../')
from Vision.modules.LiDARModule import LiDARModule

if __name__ == "__main__":
    spatial_visualizer = LiDARModule('/dev/tty.usbserial-0001') # Change this port name to match your device
    spatial_visualizer.start_device()