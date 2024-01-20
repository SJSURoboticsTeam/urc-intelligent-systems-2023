from Vision.modules.localization import CameraLocalizer
from datetime import datetime
import time

if __name__ == "__main__":
    cam = CameraLocalizer("./trained_models/rockdetection/best-simplified5s.blob")

    block = False
    cam.start(blocking=block)
    if not block:
        startTime = datetime.now()
        seconds = 30
        while (datetime.now() - startTime).total_seconds() < seconds:
            print(cam.getDetections())
            time.sleep(1)
        cam.stop()
