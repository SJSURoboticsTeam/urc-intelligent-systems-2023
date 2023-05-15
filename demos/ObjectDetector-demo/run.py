import sys
sys.path.append( '../../')
from Autonomous_Systems.ObjectDetector import ObjectDetector

if __name__ == "__main__":
    object_detector = ObjectDetector(lidar_port='/dev/tty.usbserial-0001')
    for camera_boxes, lidar_data in object_detector.calculate_objects():
        print(camera_boxes, lidar_data)