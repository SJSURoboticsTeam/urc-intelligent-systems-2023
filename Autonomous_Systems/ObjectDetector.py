import sys
import numpy as np

sys.path.append('../')
from Vision.modules.StereoCamera import StereoCamera
from Vision.modules.LiDARModule import LiDARModule

class ObjectDetector:
    def __init__(self, lidar_port):
        self.stereo_camera = StereoCamera()
        self.lidar = LiDARModule(lidar_port)

    def calculate_objects(self):
        for camera_boxes, lidar_data in zip(self.stereo_camera.run(visualize=True, usbMode='usb2'), self.lidar.run()):
            x = []
            y = []
            for scan in lidar_data:
                try:
                    distance, angle = scan
                    x.append(distance * np.sin(np.radians(angle)))
                    y.append(distance * np.cos(np.radians(angle)))
                except:
                    continue

            X = np.array(list(zip(x, y)))
            # Identify your object cluster here
            object_cluster = None

            if object_cluster is None:
                print('No object detected')
                return

            boxes_to_check = self.get_boxes_to_check(object_cluster)
            z_val, box_detected = self.get_z_value_and_box(camera_boxes.splitlines(), boxes_to_check)

            if z_val is not None:
                print(f"LiDAR detected object in cluster '{object_cluster}'")
                yield object_cluster, z_val
            elif box_detected:
                print(f"Camera detected object in cluster '{object_cluster}'")
                yield object_cluster, None
            else:
                print(f"No matching box found for cluster '{object_cluster}'")

    def get_boxes_to_check(self, object_cluster):
        if object_cluster == 'Center':
            return ['Box 2', 'Box 5']
        elif object_cluster == 'Left':
            return ['Box 1', 'Box 4']
        elif object_cluster == 'Right':
            return ['Box 3', 'Box 6']

    def get_z_value_and_box(self, camera_boxes, boxes_to_check):
        obstacle_detected = False
        min_z_val = None
        depth_threshold = 1.0

        for box_name in boxes_to_check:
            for box_data in camera_boxes:
                if box_name in box_data:
                    z_index = box_data.find("Z:")
                    z_val = float(box_data[z_index + 2:].split('m')[0].strip())
                    if z_val <= depth_threshold:
                        obstacle_detected = True
                        if min_z_val is None or z_val < min_z_val:
                            min_z_val = z_val
        return min_z_val, obstacle_detected