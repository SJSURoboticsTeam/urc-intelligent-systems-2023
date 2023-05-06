import sys
sys.path.append('../../')
from Vision.modules.StereoCamera import StereoCamera
from Vision.modules.LiDARModule import LiDARModule

class ObjectDetector:
    def __init__(self, lidar_port):
        self.stereo_camera = StereoCamera()
        self.lidar = LiDARModule(lidar_port)  # Change this port name to match your device

    def calculate_objects(self, camera_boxes=None, lidar_data=None):
        camera_boxes = self.stereo_camera.run(visualize=True)
        self.lidar.start_device()

        object_cluster = self.lidar.get_clusters()

        if object_cluster is None:
            print('No object detected')
            return

        boxes_to_check = self.get_boxes_to_check(object_cluster)
        z_val = self.get_z_value(camera_boxes, boxes_to_check)

        if z_val is not None:
            return object_cluster, z_val
        else:
            print(f"No matching box found for cluster '{object_cluster}'")

    def get_boxes_to_check(self, object_cluster):
        if object_cluster == 'Center':
            return ['Box 2', 'Box 5']
        elif object_cluster == 'Left':
            return ['Box 1', 'Box 4']
        elif object_cluster == 'Right':
            return ['Box 3', 'Box 6']

    def get_z_value(self, camera_boxes, boxes_to_check):
        for box_name in boxes_to_check:
            if box_name in camera_boxes:
                box_data_start = camera_boxes.find(box_name)
                box_data = camera_boxes[box_data_start:]
                z_index = box_data.find("Z:")
                z_val = box_data[z_index + 2:].split(',')[0].strip()
                print(box_name, z_val)
                return z_val
