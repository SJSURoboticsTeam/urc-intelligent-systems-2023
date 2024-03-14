import asyncio
import websockets
import json

data = {
    # path: value
    # "/hello": "kitty",
    # "/general": "kenobi"
}

async def send_data(ws, path):
    print("Connection on", path)
    if path not in data and path != "/":
        await ws.send("")
        return
    try:
        while True:
            d = json.dumps(list(data.keys())) if path == "/" else data[path]
            await ws.send(str(d))
            # if path in data:
            #     data[path]+='+'
            await asyncio.sleep(0.1)
    except websockets.exceptions.ConnectionClosedOK:
        print("[Connection Closed OK]:", path)
    except websockets.exceptions.ConnectionClosedError:
        print("[Connection Closed Error]:", path)

# _stay_alive = False
async def start_server(is_alive):
    server = websockets.serve(send_data, "localhost", 8765)
    async with server:
        # Server starts when entering this context
        while is_alive():
            await asyncio.sleep(1)
        # Server closes when exiting this context
    print("[Closed Server]")

def blocking_start_server(is_alive):
    try:
        asyncio.run(start_server(is_alive))
    except KeyboardInterrupt:
        print("[Keyboard Interrupted Server]")

import os
import sys
root = os.path.realpath(os.path.join(__file__, "..", "..", ".."))
sys.path.append(root) if root not in sys.path else None
from unified_frameworks.unified_utils import Service

service = Service(blocking_start_server, "Rover Side Server Thread")


# if False:
if __name__=='__main__':
    import time
    service.start_service()
    try:
        time.sleep(3)
        paths = [("/hello","world"), ("/general", "kenobi"), ("/Ala", "kazam")]
        for i, j in paths:
            data[i]=j
            time.sleep(1)
        while True:
            for i in data:
                data[i]=data[i][:-1]+chr(ord(data[i][-1])+1)
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    service.stop_service()

