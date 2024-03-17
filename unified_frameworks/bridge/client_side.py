import asyncio
import websockets
import json


data = {

}

async def sync_path(uri, path, is_running):
    # print(f"Synchronizing with {path}")
    async with websockets.connect(uri+path) as ws:
        while is_running():
            d = await ws.recv()
            data[path]=d
    # print(f"Ending Connection on {path}")

connection_tasks = {

}

async def create_connections(uri, is_running):
    async with websockets.connect(uri) as ws:
        while is_running():
            try:
                d = await asyncio.wait_for(ws.recv(), 1)
            except asyncio.TimeoutError:
                # print(f"Time out on {uri}")
                # print("Trying again") if is_running() else None
                continue
            
            # print("Synchronizing", d)
            paths = json.loads(d)
            for p in paths:
                if f'{p} synchronizing task' in connection_tasks: continue
                t = asyncio.create_task(sync_path(uri, p, is_running), name=f'{p} synchronizing task')
                connection_tasks[f'{p} synchronizing task']=t
                t.add_done_callback(lambda task: connection_tasks.pop(task.get_name()))
            # print("ending this round")


def blocking_start_client(is_running):
    try:
        asyncio.run(create_connections('ws://localhost:8765', is_running))
        # asyncio.run(create_connections('ws://192.168.1.130:8765', is_running))
    except KeyboardInterrupt:
        print("[Keyboard Interrupted Client]")
    print("[Closing Client]")

import os
import sys
root = os.path.realpath(os.path.join(__file__, "..", "..", ".."))
sys.path.append(root) if root not in sys.path else None
from unified_frameworks.unified_utils import Service

service = Service(blocking_start_client, "Client Side thread")


if __name__=='__main__':
    service.start_service()
    import time
    ts = time.time()
    try:
        while time.time()-ts < 10:
            print(json.dumps(data, indent=2))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    service.stop_service()