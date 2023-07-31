import numpy as np
import cv2
import math

class MockStereoCamera:
    def __init__(self):
        pass

    def run(self, visualize=False, usbMode="usb3"):
        # Define random values for mocking
        mock_spatials = [{'x': random.uniform(-5000, 5000), 'y': random.uniform(-5000, 5000), 'z': random.uniform(-5000, 5000)} for _ in range(6)]
        centroid_mock = [(100, 100), (200, 200), (300, 300), (400, 400), (500, 500), (600, 600)]
        delta = 98

        while True:
            disp = np.random.randint(0, 256, size=(800, 1280), dtype=np.uint8)
            disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

            for i, (spatials, centroid) in enumerate(zip(mock_spatials, centroid_mock)):
                self.display_spatials(disp, spatials, centroid, delta)
                output = self.print_spatials(spatials, i + 1)
                yield output  # Yield the output value

            if visualize:
                # Show the frame
                cv2.imshow("depth", disp)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

    def display_spatials(self, disp, spatials, centroid, delta):
        x, y = centroid
        cv2.rectangle(disp, (x - delta, y - delta), (x + delta, y + delta), (255, 255, 255), 2)
        cv2.putText(disp, "X: " + ("{:.1f}m".format(spatials['x'] / 1000) if not math.isnan(spatials['x']) else "--"), (x - 20, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(disp, "Y: " + ("{:.1f}m".format(spatials['y'] / 1000) if not math.isnan(spatials['y']) else "--"), (x - 20, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(disp, "Z: " + ("{:.1f}m".format(spatials['z'] / 1000) if not math.isnan(spatials['z']) else "--"), (x - 20, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    def print_spatials(self, spatials, box_id):
        x_val = "{:.1f}m".format(spatials['x'] / 1000) if not math.isnan(spatials['x']) else "--"
        y_val = "{:.1f}m".format(spatials['y'] / 1000) if not math.isnan(spatials['y']) else "--"
        z_val = "{:.1f}m".format(spatials['z'] / 1000) if not math.isnan(spatials['z']) else "--"
        return f"Box {box_id}: X: {x_val}, Y: {y_val}, Z: {z_val}"

