import asyncio
import websockets

async def receive_data():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        while True:
            data = await websocket.recv()
            print(f"Received: {data}")

asyncio.get_event_loop().run_until_complete(receive_data())
