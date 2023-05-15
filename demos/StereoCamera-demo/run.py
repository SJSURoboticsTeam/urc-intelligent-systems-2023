import sys
sys.path.append( '../../')
from Vision.modules.StereoCamera import StereoCamera

if __name__ == "__main__":
    spatial_visualizer = StereoCamera()
    # Run the camera and iterate over the output values
    for output in spatial_visualizer.run(visualize=True, usbMode="usb2"):
        print(output)