import asyncio
import websockets
import json

data = {
    # path: value
    "/hello": "kitty",
    "/general": "kenobi"
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
            if path in data:
                data[path]+='+'
            await asyncio.sleep(1)
    except websockets.exceptions.ConnectionClosedOK:
        print("[Connection Closed OK]:", path)
    # except KeyboardInterrupt:
    #     print("[Keyboard Interrupted Server]")

start_server = websockets.serve(send_data, "localhost", 8765)
asyncio.get_event_loop().run_until_complete(start_server)
try:
    asyncio.get_event_loop().run_forever()
except KeyboardInterrupt:
    print("[Keyboard Interrupted Server]")