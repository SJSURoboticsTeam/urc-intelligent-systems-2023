#!/usr/bin/env python3

import cv2
import depthai as dai
from calc import HostSpatialsCalc
from utility import *
import numpy as np
import math

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

stereo.initialConfig.setConfidenceThreshold(255)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(False)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")
stereo.depth.link(xoutDepth.input)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("disp")
stereo.disparity.link(xoutDepth.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth")
    dispQ = device.getOutputQueue(name="disp")

    #add 9 rectangles to the frame to detect depth

    text0 = TextHelper()
    text1 = TextHelper()
    text2 = TextHelper()
    text3 = TextHelper()
    text4 = TextHelper()
    text5 = TextHelper()

    # text6 = TextHelper()
    # text7 = TextHelper()
    # text8 = TextHelper()


    hostSpatials0 = HostSpatialsCalc(device)
    hostSpatials1 = HostSpatialsCalc(device)
    hostSpatials2 = HostSpatialsCalc(device)
    hostSpatials3 = HostSpatialsCalc(device)
    hostSpatials4 = HostSpatialsCalc(device)
    hostSpatials5 = HostSpatialsCalc(device)


    # hostSpatials6 = HostSpatialsCalc(device)
    # hostSpatials7 = HostSpatialsCalc(device)
    # hostSpatials8 = HostSpatialsCalc(device)


    y = 200
    x = 300
    step = 3
    delta = 5
    hostSpatials0.setDeltaRoi(delta)
    hostSpatials1.setDeltaRoi(delta)
    hostSpatials2.setDeltaRoi(delta)
    hostSpatials3.setDeltaRoi(delta)
    hostSpatials4.setDeltaRoi(delta)
    hostSpatials5.setDeltaRoi(delta)


    # hostSpatials6.setDeltaRoi(delta)
    # hostSpatials7.setDeltaRoi(delta)
    # hostSpatials8.setDeltaRoi(delta)
    print("Use WASD keys to move ROI.\nUse 'r' and 'f' to change ROI size.")

    while True:
        depthData = depthQueue.get()
        # Calculate spatial coordiantes from depth frame
        spatials, centroid = hostSpatials0.calc_spatials(depthData, (x-200,y-50)) # centroid == x/y in our case
        spatials1, centroid1 = hostSpatials1.calc_spatials(depthData, (x-10,y-50)) # centroid == x/y in our case
        spatials2, centroid2 = hostSpatials2.calc_spatials(depthData, (x+200,y-50)) # centroid == x/y in our case

        spatials3, centroid3 = hostSpatials3.calc_spatials(depthData, (x-200,y+100)) # centroid == x/y in our case
        spatials4, centroid4 = hostSpatials4.calc_spatials(depthData, (x-10,y+100)) # centroid == x/y in our case
        spatials5, centroid5 = hostSpatials5.calc_spatials(depthData, (x+200,y+100)) # centroid == x/y in our case


        # spatials6, centroid6 = hostSpatials6.calc_spatials(depthData, (x+600,y+100)) # centroid == x/y in our case
        # spatials7, centroid7 = hostSpatials7.calc_spatials(depthData, (x+700,y+100)) # centroid == x/y in our case
        # spatials8, centroid8 = hostSpatials8.calc_spatials(depthData, (x+800,y+100)) # centroid == x/y in our case


        # Get disparity frame for nicer depth visualization
        disp = dispQ.get().getFrame()
        disp = (disp * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
        disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

        text0.rectangle(disp, (x-290-delta, y-delta), (x-100+delta, y-200+delta))
        text0.putText(disp, "X: " + ("{:.1f}m".format(spatials['x']/1000) if not math.isnan(spatials['x']) else "--"), (x - 225, y - 30))
        text0.putText(disp, "Y: " + ("{:.1f}m".format(spatials['y']/1000) if not math.isnan(spatials['y']) else "--"), (x - 225, y - 60))
        text0.putText(disp, "Z: " + ("{:.1f}m".format(spatials['z']/1000) if not math.isnan(spatials['z']) else "--"), (x - 225, y - 90))

        text1.rectangle(disp, (x-80-delta, y-delta), (x+120+delta, y-200+delta))
        text1.putText(disp, "X: " + ("{:.1f}m".format(spatials1['x']/1000) if not math.isnan(spatials1['x']) else "--"), (x - 20, y - 30))
        text1.putText(disp, "Y: " + ("{:.1f}m".format(spatials1['y']/1000) if not math.isnan(spatials1['y']) else "--"), (x - 20, y - 60))
        text1.putText(disp, "Z: " + ("{:.1f}m".format(spatials1['z']/1000) if not math.isnan(spatials1['z']) else "--"), (x - 20, y - 90))

        text2.rectangle(disp, (x+340-delta, y-delta), (x+130+delta, y-200+delta))
        text2.putText(disp, "X: " + ("{:.1f}m".format(spatials2['x']/1000) if not math.isnan(spatials2['x']) else "--"), (x + 200, y - 30))
        text2.putText(disp, "Y: " + ("{:.1f}m".format(spatials2['y']/1000) if not math.isnan(spatials2['y']) else "--"), (x + 200, y - 60))
        text2.putText(disp, "Z: " + ("{:.1f}m".format(spatials2['z']/1000) if not math.isnan(spatials2['z']) else "--"), (x + 200, y - 90))




        text3.rectangle(disp, (x-290-delta, y+10-delta), (x-100+delta, y+180+delta))
        text3.putText(disp, "X: " + ("{:.1f}m".format(spatials3['x']/1000) if not math.isnan(spatials3['x']) else "--"), (x - 225, y + 100))
        text3.putText(disp, "Y: " + ("{:.1f}m".format(spatials3['y']/1000) if not math.isnan(spatials3['y']) else "--"), (x - 225, y + 130))
        text3.putText(disp, "Z: " + ("{:.1f}m".format(spatials3['z']/1000) if not math.isnan(spatials3['z']) else "--"), (x - 225, y + 160))

        text4.rectangle(disp, (x-80-delta, y+10-delta), (x+120+delta, y+180+delta))
        text4.putText(disp, "X: " + ("{:.1f}m".format(spatials4['x']/1000) if not math.isnan(spatials4['x']) else "--"), (x - 20, y + 100))
        text4.putText(disp, "Y: " + ("{:.1f}m".format(spatials4['y']/1000) if not math.isnan(spatials4['y']) else "--"), (x - 20, y + 130))
        text4.putText(disp, "Z: " + ("{:.1f}m".format(spatials4['z']/1000) if not math.isnan(spatials4['z']) else "--"), (x - 20, y + 160))

        text5.rectangle(disp, (x+340-delta, y+10-delta), (x+130+delta, y+180+delta))
        text5.putText(disp, "X: " + ("{:.1f}m".format(spatials5['x']/1000) if not math.isnan(spatials5['x']) else "--"), (x + 200, y + 100))
        text5.putText(disp, "Y: " + ("{:.1f}m".format(spatials5['y']/1000) if not math.isnan(spatials5['y']) else "--"), (x + 200, y + 130))
        text5.putText(disp, "Z: " + ("{:.1f}m".format(spatials5['z']/1000) if not math.isnan(spatials5['z']) else "--"), (x + 200, y + 160))
        

        #For Printing the distance values for each grid
        # print("Rectangle 0 - X: {:.2f}m, Y: {:.2f}m, Z: {:.2f}m".format(spatials['x']/1000, spatials['y']/1000, spatials['z']/1000))
        # print("Rectangle 1 - X: {:.2f}m, Y: {:.2f}m, Z: {:.2f}m".format(spatials1['x']/1000, spatials1['y']/1000, spatials1['z']/1000))
        # print("Rectangle 2 - X: {:.2f}m, Y: {:.2f}m, Z: {:.2f}m".format(spatials2['x']/1000, spatials2['y']/1000, spatials2['z']/1000))
        # print("Rectangle 3 - X: {:.2f}m, Y: {:.2f}m, Z: {:.2f}m".format(spatials3['x']/1000, spatials3['y']/1000, spatials3['z']/1000))
        # print("Rectangle 4 - X: {:.2f}m, Y: {:.2f}m, Z: {:.2f}m".format(spatials4['x']/1000, spatials4['y']/1000, spatials4['z']/1000))
        # print("Rectangle 5 - X: {:.2f}m, Y: {:.2f}m, Z: {:.2f}m".format(spatials5['x']/1000, spatials5['y']/1000, spatials5['z']/1000))



        # text6.rectangle(disp, (x+600-delta, y+600-delta), (x+600+delta, y+600+delta))
        # text6.putText(disp, "X: " + ("{:.1f}m".format(spatials6['x']/1000) if not math.isnan(spatials6['x']) else "--"), (x + 610, y + 620))
        # text6.putText(disp, "Y: " + ("{:.1f}m".format(spatials6['y']/1000) if not math.isnan(spatials6['y']) else "--"), (x + 610, y + 635))
        # text6.putText(disp, "Z: " + ("{:.1f}m".format(spatials6['z']/1000) if not math.isnan(spatials6['z']) else "--"), (x + 610, y + 650))

        # text7.rectangle(disp, (x+700-delta, y+700-delta), (x+700+delta, y+700+delta))
        # text7.putText(disp, "X: " + ("{:.1f}m".format(spatials7['x']/1000) if not math.isnan(spatials7['x']) else "--"), (x + 710, y + 720))
        # text7.putText(disp, "Y: " + ("{:.1f}m".format(spatials7['y']/1000) if not math.isnan(spatials7['y']) else "--"), (x + 710, y + 735))
        # text7.putText(disp, "Z: " + ("{:.1f}m".format(spatials7['z']/1000) if not math.isnan(spatials7['z']) else "--"), (x + 710, y + 750))

        # text8.rectangle(disp, (x+800-delta, y+800-delta), (x+800+delta, y+800+delta))
        # text8.putText(disp, "X: " + ("{:.1f}m".format(spatials8['x']/1000) if not math.isnan(spatials8['x']) else "--"), (x + 810, y + 820))
        # text8.putText(disp, "Y: " + ("{:.1f}m".format(spatials8['y']/1000) if not math.isnan(spatials8['y']) else "--"), (x + 810, y + 835))
        # text8.putText(disp, "Z: " + ("{:.1f}m".format(spatials8['z']/1000) if not math.isnan(spatials8['z']) else "--"), (x + 810, y + 850))


        # Show the frame
        cv2.imshow("depth", disp)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
