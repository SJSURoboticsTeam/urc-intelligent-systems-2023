import asyncio
import websockets
import json


data = {

}
stay_connected = True

async def sync_path(uri, path):
    print(f"Synchronizing with {path}")
    async with websockets.connect(uri+path) as ws:
        while stay_connected:
            d = await ws.recv()
            data[path]=d

connection_tasks = {

}

async def create_connections(uri):
    async with websockets.connect(uri) as ws:
        while stay_connected:
            d = await ws.recv()
            print("Synchronizing", d)
            paths = json.loads(d)
            for p in paths:
                if f'{p} synchronizing task' in connection_tasks: continue
                t = asyncio.create_task(sync_path(uri, p), name=f'{p} synchronizing task')
                connection_tasks[f'{p} synchronizing task']=t
                # def popit(p):
                #     print(f"popping {p} from {connection_tasks.keys()}")
                #     connection_tasks.pop(p)
                t.add_done_callback(lambda task: connection_tasks.pop(task.get_name()))


async def start_client():
    t = asyncio.create_task(create_connections('ws://localhost:8765'))
    import time
    ts = time.time()
    while time.time()-ts < 5:
        print(json.dumps(data, indent=2))
        await asyncio.sleep(0.5)
    global stay_connected
    stay_connected = False
    await t

if __name__=='__main__':
    asyncio.run(start_client())