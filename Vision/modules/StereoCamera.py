import sys
import cv2
import depthai as dai
from Vision.modules.calc import HostSpatialsCalc
from Vision.modules.utility import *
import numpy as np
import math


class StereoCamera:
    def __init__(self):
        self.pipeline = dai.Pipeline()

    def display_spatials(self, text_helper, disp, spatials, centroid, delta):
        x, y = centroid
        text_helper.rectangle(disp, (x - delta, y - delta), (x + delta, y + delta))
        text_helper.putText(disp, "X: " + ("{:.1f}m".format(spatials['x']/1000) if not math.isnan(spatials['x']) else "--"), (x - 20, y - 50))
        text_helper.putText(disp, "Y: " + ("{:.1f}m".format(spatials['y']/1000) if not math.isnan(spatials['y']) else "--"), (x - 20, y - 20))
        text_helper.putText(disp, "Z: " + ("{:.1f}m".format(spatials['z']/1000) if not math.isnan(spatials['z']) else "--"), (x - 20, y + 10))


    def print_spatials(self, spatials, box_id):
        x_val = "{:.1f}m".format(spatials['x'] / 1000) if not math.isnan(spatials['x']) else "--"
        y_val = "{:.1f}m".format(spatials['y'] / 1000) if not math.isnan(spatials['y']) else "--"
        z_val = "{:.1f}m".format(spatials['z'] / 1000) if not math.isnan(spatials['z']) else "--"
        return(f"Box {box_id}: X: {x_val}, Y: {y_val}, Z: {z_val}")


    def pipeline_setup(self):
        # Define sources and outputs
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        self.stereo.initialConfig.setConfidenceThreshold(255)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(False)

        # Linking
        monoLeft.out.link(self.stereo.left)
        monoRight.out.link(self.stereo.right)

        xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        self.stereo.depth.link(xoutDepth.input)

        xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("disp")
        self.stereo.disparity.link(xoutDepth.input)


    def run(self, visualize=False, usbMode="usb3"):
        self.pipeline_setup()
        # Connect to device and start pipeline

        if usbMode == "usb3":
            usb2ModeChoice = False
        elif usbMode == "usb2":
            usb2ModeChoice = True

        with dai.Device(self.pipeline, usb2Mode=usb2ModeChoice) as device:
            # Output queue will be used to get the depth frames from the outputs defined above
            depthQueue = device.getOutputQueue(name="depth")
            dispQ = device.getOutputQueue(name="disp")

            text_helpers = [TextHelper() for _ in range(6)]
            host_spatials = [HostSpatialsCalc(device) for _ in range(6)]

            y = 150
            x = 300
            delta = 98
            for hs in host_spatials:
                hs.setDeltaRoi(delta)

            while True:
                depthData = depthQueue.get()

                centroids = [(x-200, y-50), (x+10, y-50), (x+220, y-50), (x-200, y+150), (x+10, y+150), (x+220, y+150)]
                spatials_data = [hs.calc_spatials(depthData, centroid) for hs, centroid in zip(host_spatials, centroids)]

                # Get disparity frame for nicer depth visualization
                disp = dispQ.get().getFrame()
                disp = (disp * (255 / self.stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
                disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

                for i, (text_helper, spatials, centroid) in enumerate(zip(text_helpers, spatials_data, centroids)):
                    self.display_spatials(text_helper, disp, spatials[0], centroid, delta)
                    output = self.print_spatials(spatials[0], i + 1)
                    yield output  # Yield the output value

                if visualize:
                    # Show the frame
                    cv2.imshow("depth", disp)

                    key = cv2.waitKey(1)
                    if key == ord('q'):
                        break
