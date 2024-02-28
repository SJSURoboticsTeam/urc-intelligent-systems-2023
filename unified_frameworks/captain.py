"""This script is responsible for taking in the path from pathfinder 
and determining a command to send to the rover"""

import os
root = os.path.realpath(os.getcwd()+ os.sep+"..")
import sys
sys.path.append(root)
from modules.WiFi import WiFi, Modes, make_command

