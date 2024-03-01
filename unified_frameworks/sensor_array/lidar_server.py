import websockets
import asyncio
import lidar

# Creating WebSocket server
async def server(websocket):
 
    try:
        while True:
            # Receiving values from client (lidar)
            # RAW DATA is in tuples
            # lidar_data = tuple()
            # lidar_data = None

            # if lidar_data == "":
            #     print("None")

            # This is going to be an example of how we'll be sending data
            for i in range(4):
                await websocket.send(f"Heyo here is a count {i}")
                await asyncio.sleep(1)

    except websockets.exceptions.ConnectionClosedOK:
        # await websocket.send("[SERVER] Disconnecting from server")
        # await asyncio.sleep(1)
        print("[SERVER] Client disconnected from server!")
    except websockets.ConnectionClosedError:
        print("Internal Server Error.")
    except KeyboardInterrupt:
        print("Keyboard Interrupt occurred!")
 
 
async def main():
    async with websockets.serve(server, "0.0.0.0", 8765):
        await asyncio.Future()  # run forever
 
if __name__ == "__main__":
    print("[SERVER] Server ON")
    asyncio.run(main())