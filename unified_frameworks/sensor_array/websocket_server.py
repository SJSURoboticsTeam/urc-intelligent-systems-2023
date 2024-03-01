import asyncio
import websockets


async def data_stream(websocket, path):
    count = 0
    while True:
        # Replace this with your actual data source or generation logic
        count += 1
        data = str(count)
        await websocket.send(data)
        await asyncio.sleep(1)  # Adjust the delay as needed

start_server = websockets.serve(data_stream, "localhost", 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()