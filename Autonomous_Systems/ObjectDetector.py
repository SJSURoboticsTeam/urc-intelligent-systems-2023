import sys
import numpy as np
import threading
import queue

sys.path.append('../')
from Vision.modules.StereoCamera import StereoCamera
from Vision.modules.LiDARModule import LiDARModule

class ObjectDetector:
    def __init__(self, lidar_port, VISUALIZE=True, MaxDistance=4.0):
        self.stereo_camera = StereoCamera()
        self.lidar = LiDARModule(lidar_port)
        self.VISUALIZE = VISUALIZE
        self.MaxDistance = MaxDistance
        self.camera_queue = queue.Queue()
        self.lidar_queue = queue.Queue()

    def lidar_thread(self):
        for lidar_data in self.lidar.run():
            self.lidar_queue.put(lidar_data)

    def camera_thread(self):
        for camera_boxes in self.stereo_camera.run(visualize=self.VISUALIZE, usbMode='usb2'):
            self.camera_queue.put(camera_boxes)

    def calculate_objects(self):
        threading.Thread(target=self.lidar_thread).start()
        threading.Thread(target=self.camera_thread).start()

        while True:
            camera_boxes = self.camera_queue.get()
            lidar_data = self.lidar_queue.get()

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
            if np.mean(x) < -0.5:
                object_cluster = 'Left'
            elif np.mean(x) > 0.5:
                object_cluster = 'Right'
            else:
                object_cluster = 'Center'

            boxes_to_check = self.get_boxes_to_check(object_cluster)
            z_val, box_detected = self.get_z_value_and_box(camera_boxes.split('\n'), boxes_to_check)

            if z_val is not None:
                print(f"Detected object in cluster '{object_cluster}'")
                yield object_cluster, z_val
            elif box_detected:
                print(f"Camera only detected object in cluster '{object_cluster}'")
                yield object_cluster, None
            else:
                print(f"No matching box found for cluster '{object_cluster}'")

    def get_boxes_to_check(self, object_cluster):
        if object_cluster == 'Center':
            return ['Box 5', 'Box 2']
        elif object_cluster == 'Left':
            return ['Box 1', 'Box 4']
        elif object_cluster == 'Right':
            return ['Box 3', 'Box 6']

    def get_z_value_and_box(self, camera_boxes, boxes_to_check):
        obstacle_detected = False
        min_z_val = None
        if boxes_to_check is None:
            boxes_to_check = []
        for box_name in boxes_to_check:
            for box_data in camera_boxes:
                if box_name in box_data:
                    try:
                        z_index = box_data.find("Z:")
                        z_val_str = box_data[z_index + 3:box_data.find("m", z_index)].strip()
                        z_val = float(z_val_str)
                        if z_val <= self.MaxDistance:
                            obstacle_detected = True
                            if min_z_val is None or z_val < min_z_val:
                                min_z_val = z_val
                    except ValueError:
                        print(f"Cannot convert Z value to float: '{z_val_str}' in box_data: {box_data}")
        return min_z_val, obstacle_detected
