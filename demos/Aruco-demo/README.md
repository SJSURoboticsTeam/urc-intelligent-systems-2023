# Aruco Tag Detector

## Testing Maximum Aruco Tag Distance

Use the notebooks `aruco-tag-detection.ipynb` and `aruco-tag-detection-oakd.ipynb` to test the maximum distance at which camera can detect Aruco tags.
The challenge will have Aruco tags indicating the true target location anywhere from 5m to 20m (15 feet to 67 feet) away from the provided GPS coordinates.
We need to make sure that the camera can detect the Aruco tags at the maximum distance (or farther for a safety margin).

### Creating the Tags

First, you have to print out the Aruco tag. You can just print `aruco_0.png` on one of the school printers. 
It may be a bit distorted, but it should be good enough for testing. The tag should be 15x15cm, which will take up most
of the width of the page. If it's a little large or a little small, that's okay too.

add ls as the key for the last bit
1 red - default
2 blue
3 flashing green
### Setting up the Camera

For a normal USB camera, you can just plug it in and use the `arugo-tag-detection.ipynb` notebook.
If you are using the Oak-d camera, you'll have to install the software (see Adrien's instructions [here](https://github.com/SJSURoboticsTeam/urc-intelligent_systems-2023/blob/main/Vision/README.md))
and use the `aruco-tag-detection-oakd.ipynb` notebook. You will also need to install `opencv-contrib-python` version `4.6.0.66` (earlier versions are okay, but later versions break the API).

```bash
pip install opencv-contrib-python==4.6.0.66
```

### Testing the Distance

This is a two-person job. Ryan has done it with me before, so he should know the general steps, but I will repeat them here for completeness.  

1. Go out into the hall near Eng 141.
2. Have one person carry the tag to the end of the hall and have the other person run the notebook and observe the screen (if you don't see the OpenCV window when you start the notebook, look for it in the taskbar and try to open it)
3. Point the camera down the hall at the tag. **The Aruco tag detection algorithm is sensitive to motion blur, so try to keep the camera steady (maybe attach it to the laptop or put it on the floor)
4. Have the person carrying the tag walk towards the camera 5-10 feet at a time then stop. **Be sure to stop and hold the tag steady for a few seconds before moving again.**
5. When the tag is detected, a green box should appear around it. If it only appears a few times, then it means the algorithm is having a hard time detecting the tag and it is still too far away.
6. Once the tag is consistently detected, have the person carrying the tag stop moving.
7. Measure the distance from where the tag stopped to the camera. Most tape measures are only 25-30 feet long, so you may have to take several measurements.
8. It's worth repeating the experiement a couple of times to make sure the results are consistent. 


**NOTE:** You may want to try running the notebooks before you go out into the hall to make sure that they are working properly.

## ArucoDetector Class Demo

`run.py` is a demo of how to use the `ArucoDetector` class. Start it with:

```bash
python3 run.py
```

Once the program is running, point the camera at an Aruco tag and the tag ID and location will be displayed on the screen.
This tag should be from the 4x4_50 dictionary, which is used by the URC competition.

If you don't have a tag handy, just open up `aruco_0.png` on your computer and point the camera at it.

If you want to use a different camera, change the `camera_id` variable in `run.py`. 

