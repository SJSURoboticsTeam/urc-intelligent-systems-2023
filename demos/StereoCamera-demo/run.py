import sys
sys.path.append( '../../')
from Vision.modules.StereoCamera import StereoCamera

if __name__ == "__main__":
    spatial_visualizer = StereoCamera()
    # Run the camera and iterate over the output values
    for output in spatial_visualizer.run(visualize=False, usbMode="usb3"):
        print(output)