# Sensor Array

The objective of SensorArray is to read in data from all onboard sensors, combine that into a single world view, make this world view easil available to anyone, and run a toggleable visualization of the world view.

## world view

By world view I mean current state of the environment. This can include the positions of obstacles, maybe a history of where they were, air temperature, altitude, slope, or anything that will be needed for autonomy to chart a path to the goal. Basically what ever algorithm we run to make autonomy decissions, the world view should be enough of an input to make decissions.

As of right now I thinking the world view must be a list of obstacles surrounding the rover. This list will be fed by the camera and the lidar. For now it will be a text file containing a 3d array of shape (<num obstacles>, <num points in obstacle>, 2<r,theta>).

We can have one process for the camera and lidar each to write their data into 2 files, and a seperate process that reads from each of the sensor files and combines them into the world view file.

