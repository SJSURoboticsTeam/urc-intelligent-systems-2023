import sys
import re
root = (next(re.finditer(".*unified_frameworks", __file__)).group())
sys.path.append(root) if root not in sys.path else None
from sensor_array.LidarClass import Lidar
from bridge import client_side
# import asyncio
# import websockets
from threading import Thread
import json
import time

class WirelessBridgeLidar(list):
    def __init__(self):
        self.path = "/lidar"
        