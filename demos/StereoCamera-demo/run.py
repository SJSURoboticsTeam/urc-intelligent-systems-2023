import sys
sys.path.append( '../../')
from Vision.modules.StereoCamera import StereoCamera

if __name__ == "__main__":
    spatial_visualizer = StereoCamera()
    print(spatial_visualizer.run(visualize=True))