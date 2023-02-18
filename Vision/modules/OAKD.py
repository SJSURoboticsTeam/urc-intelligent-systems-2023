import depthai
import numpy as np

def get_camera_intrinsics(device):
    """Gets the camera intrinsics from the device (camera matrix and distortion coefficients).
    It returns them as mtx, dist. It takes a depthai.Device object as an argument.

    """
    calibData = device.readCalibration()

    # get the camera matrix
    mtx = np.array(calibData.getCameraIntrinsics(depthai.CameraBoardSocket.RGB))
    # get the distortion coefficients
    dist = np.array(calibData.getDistortionCoefficients(depthai.CameraBoardSocket.RGB))

    return mtx, dist

class OakD():
    def __init__(self): # TODO: add video size to constructor
        self.pipeline = depthai.Pipeline()

        # First, we want the Color camera as the output
        self.cam_rgb = self.pipeline.createColorCamera()
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_13_MP)
        self.cam_rgb.setVideoSize(3840, 2160)

        # XLinkOut is a "way out" from the device. Any data you want to transfer to host need to be send via XLink
        self.xout_rgb = self.pipeline.createXLinkOut()
        # For the rgb camera output, we want the XLink stream to be named "rgb"
        self.xout_rgb.setStreamName("rgb")
        # Linking camera preview to XLink input, so that the frames will be sent to host
        self.cam_rgb.video.link(self.xout_rgb.input)

        self.device = depthai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue("rgb")

        self.mtx, self.dist = get_camera_intrinsics(self.device)

    def read(self):
        """Gets a frame from the camera. It has the same signature as the read() from an OpenCV video capture
        object.

        It returns ret, frame where ret indicated whether the frame was successfully grabbed or if there was an error"""

        frame = self.q_rgb.get()

        ret = True

        if frame is None:
            ret = False

        return ret, frame
