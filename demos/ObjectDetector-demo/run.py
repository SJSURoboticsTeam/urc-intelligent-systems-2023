import sys
sys.path.append( '../../')
from Autonomous_Systems.ObjectDetector import ObjectDetector

if __name__ == "__main__":
    object_detector = ObjectDetector(lidar_port='/dev/ttyUSB0', VISUALIZE=False, MaxDistance=4.0)
    for camera_boxes, lidar_data in object_detector.calculate_objects():
        print("Position:", camera_boxes, "Distance:", lidar_data, "meters")