from pathlib import Path
from typing import List
import cv2
import depthai as dai
import numpy as np
import time

"""
Spatial yolo example
  Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
  Can be used for tiny-yolo-v3 or tiny-yolo-v4 networks
"""

# Get argument first
nnBlobPath = str(
    (
        Path(__file__).parent
        / Path("./trained_models/rockdetection/best-simplified5s.blob")
    )
    .resolve()
    .absolute()
)

if not Path(nnBlobPath).exists():
    raise FileNotFoundError(f'Blob path not found"')

syncNN = False  # True

# Create pipeline
pipeline = dai.Pipeline()
pipeline.setOpenVINOVersion(dai.OpenVINO.VERSION_2022_1)

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
nnNetworkOut = pipeline.create(dai.node.XLinkOut)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutDepth.setStreamName("depth")
nnNetworkOut.setStreamName("nnNetwork")

# Properties
camRgb.setPreviewSize(640, 640)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(30)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Align depth map to the perspective of RGB camera, on which inference is done
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
stereo.setSubpixel(True)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.8)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(80)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
spatialDetectionNetwork.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
spatialDetectionNetwork.setIouThreshold(0.5)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

# Connect to device and start pipeline
maxSize = 2
fontColor = (0, 0, 255)  # red
with dai.Device(pipeline) as device:
    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=maxSize, blocking=False)
    detectionNNQueue = device.getOutputQueue(
        name="detections", maxSize=maxSize, blocking=False
    )
    depthQueue = device.getOutputQueue(name="depth", maxSize=maxSize, blocking=False)
    networkQueue = device.getOutputQueue(
        name="nnNetwork", maxSize=maxSize, blocking=False
    )

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)
    printOutputLayersOnce = True

    while True:
        inPreview = previewQueue.get()
        inDet = detectionNNQueue.get()
        depth = depthQueue.get()
        inNN = networkQueue.get()

        if printOutputLayersOnce:
            toPrint = "Output layer names:"
            for ten in inNN.getAllLayerNames():
                toPrint = f"{toPrint} {ten},"
            print(toPrint)
            printOutputLayersOnce = False

        frame = inPreview.getCvFrame()
        depthFrame = depth.getFrame()  # depthFrame values are in millimeters

        depth_downscaled = depthFrame[::maxSize]
        if np.all(depth_downscaled == 0):
            min_depth = (
                0  # Set a default minimum depth value when all elements are zero
            )
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(
            depthFrame, (min_depth, max_depth), (0, 255)
        ).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        counter += 1
        current_time = time.monotonic()
        if (current_time - startTime) > 1:
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width = frame.shape[1]
        for detection in detections:
            roiData = detection.boundingBoxMapping
            roi = roiData.roi
            roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
            topLeft = roi.topLeft()
            bottomRight = roi.bottomRight()
            xmin = int(topLeft.x)
            ymin = int(topLeft.y)
            xmax = int(bottomRight.x)
            ymax = int(bottomRight.y)
            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1)

            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            label = detection.label

            increment = 15
            start = y1 + 20

            def putTexts(texts: List[str]):
                for idx, text in enumerate(texts):
                    cv2.putText(
                        frame,
                        text,
                        (x1 + 10, start + increment * idx),
                        cv2.FONT_HERSHEY_TRIPLEX,
                        0.5,
                        fontColor,
                    )

            if detection.spatialCoordinates.z != 0:
                dep = detection.spatialCoordinates.z
            else:
                dep = 1e-6  # epsilon to avoid division by 0

            # DFOV / HFOV / VFOV = 120° / 95° / 70°
            # FOV = 2 x working distance x tan (AFOV/2) - somehow this gives 2* what we need, so ignore 2x
            dist = detection.spatialCoordinates.z  # in mm
            deg2rad = np.pi / 180
            vfov = np.tan(70 * deg2rad) * dist  # in mm
            hPercent = (ymax - ymin) / frame.shape[1]  # px / px = percent
            h = hPercent * vfov

            # H is the horizontal angle (angle from center of camera to object's horizontal position)
            # V is the vertical angle (angle from center of camera to object's vertical position)
            putTexts(
                [
                    "rock",
                    "{:.2f}".format(detection.confidence * 100),
                    f"X: {int(detection.spatialCoordinates.x)} mm",
                    f"Y: {int(detection.spatialCoordinates.y)} mm",
                    f"Z: {int(detection.spatialCoordinates.z)} mm",
                    f"H*: {int(np.arctan2(detection.spatialCoordinates.x, dep) * 180 / np.pi)} deg",
                    f"V*: {int(np.arctan2(detection.spatialCoordinates.y, dep) * 180 / np.pi)} deg",
                    f"HE: {int(h)} mm",  # height in mm; max we can roll over is ~130mm
                ]
            )

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

        cv2.putText(
            frame,
            "NN fps: {:.2f}".format(fps),
            (2, frame.shape[0] - 4),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.4,
            color,
        )
        # cv2.imshow("depth", depthFrameColor)
        cv2.imshow("rgb", frame)

        if cv2.waitKey(1) == ord("q"):
            break
