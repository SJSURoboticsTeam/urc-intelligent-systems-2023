import asyncio
import websockets

async def receive_data():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        count = 0
        while count < 10:
            data = await websocket.recv()
            print(f"Received: {data}")
            count+=1

if __name__=='__main__':
    asyncio.get_event_loop().run_until_complete(receive_data())
