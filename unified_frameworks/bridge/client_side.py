import asyncio
import websockets
import json

config = {
    'verbose': False
}

data = {

}

async def sync_path(uri, path, is_running):
    print(f"Synchronizing with {path}") if config['verbose'] else None
    async with websockets.connect(uri+path) as ws:
        while is_running():
            d = await ws.recv()
            data[path]=d
    print(f"Ending Connection on {path}") if config['verbose'] else None

connection_tasks = {

}

_connection_created = False
async def create_connections(uri, is_running):
    global _connection_created
    print("Attempting to connect from client side of bridge") if config['verbose'] else None
    async with websockets.connect(uri) as ws:
        _connection_created = True
        while is_running():
            try:
                d = await asyncio.wait_for(ws.recv(), 1)
            except asyncio.TimeoutError:
                # print(f"Time out on {uri}")
                # print("Trying again") if is_running() else None
                continue
            
            print("Total synchronous paths", d) if config['verbose'] else None
            paths = json.loads(d)
            for p in paths:
                if f'{p} synchronizing task' in connection_tasks: continue
                t = asyncio.create_task(sync_path(uri, p, is_running), name=f'{p} synchronizing task')
                connection_tasks[f'{p} synchronizing task']=t
                t.add_done_callback(lambda task: connection_tasks.pop(task.get_name()))
            # print("ending this round")

_connection_failed = False
def blocking_start_client(is_running):
    try:
        # asyncio.run(create_connections('ws://localhost:8765', is_running))
        asyncio.run(create_connections('ws://192.168.1.130:8765', is_running))
    except KeyboardInterrupt:
        print("[Keyboard Interrupted Client]")
    except ConnectionRefusedError:
        global _connection_failed
        _connection_failed = True
        print("[Bridge Refused Connection]")
    print("[Closing Client]")

import os
import sys
root = os.path.realpath(os.path.join(__file__, "..", "..", ".."))
sys.path.append(root) if root not in sys.path else None
from unified_frameworks.unified_utils import Service
import time
service = Service(blocking_start_client, "Client Side thread")

def open_bridge():
    service.start_service()
    while not _connection_created and not _connection_failed:
        time.sleep(1)
    return _connection_created and not _connection_failed
def close_bridge():
    service.stop_service()

def bridge_is_up():
    return service.is_running()


if __name__=='__main__':
    # service.start_service()
    config['verbose']=True
    b = open_bridge()
    print(f"Bridge{' was not' if not b else ''} Opened")
    if not b: exit(0)
    ts = time.time()
    try:
        while time.time()-ts < 10:
            print(json.dumps(data, indent=2))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    # service.stop_service()
    close_bridge()