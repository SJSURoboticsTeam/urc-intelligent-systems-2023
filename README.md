# urc-intelligent_systems-2023

Contains the intelligent systems module and sensor logic for the SJSU Robotics' 3-Wheeled Mars Rover

## [Documentation Site](https://sjsuroboticsteam.github.io/urc-intelligent-systems-2023/index.html)

## Setup

To get started, you will need to install the following dependencies, you can do this by using a virtual environment and the requirements.txt file in the root directory of the project.

```sh
python -m venv venv
source venv/bin/activate

# if you are running not on a Raspberry Pi, run the below
pip install -r env_files/all_requirements.txt

# If you are running on a Raspberry Pi, run the below
pip install -r env_files/rover_requirements.txt
```

![classes](https://github.com/SJSURoboticsTeam/urc-intelligent-systems-2023/assets/50222631/3d47c3d4-b21a-463e-9739-bc99b61f450f)

## Running

To get the mission started, first run `python3 rover.py` from within the `unified_frameworks` directory on the Raspberry Pi. This
starts up the bridge on the rover side and gets all necessary sensors running.

From there, you can run `python3 captain.py` from within the `unified_frameworks` directory on your local machine
to begin the pathfinding and visualization.
