import websockets
import asyncio
import json
from gps_compass.actual_gps_compass import ActualGPSCompass

# gps = ActualGPSCompass() # Commented this cuz it should not run if the file is not __main__
print("successfully made actual gps")

clients = []  # @note List of clients connected in the server
buffer = None


# Creating WebSocket server
# @note Server listens for ConnectionClosedOK
# @note Sends data from the server to the client via websockets
# @note checks if the lidar we are sending data from is either FakeLidar or ActualLidar
# @note If current lidar is FakeLidar then we utilize NumpyEncoder.
async def sendServerDataToClient(websocket):
    clients.append(websocket)
    print("[SERVER] Client has connected to the server")
    # await websocket.send(json.dumps("[SERVER] You have connected to the server!"))
    # await asyncio.sleep(0.01)
    isConnectedStill = True

    try:
        while isConnectedStill:
            buffer = json.dumps(
                {"gps": gps.get_cur_gps(), "angle": gps.get_cur_angle()}
            )

            if len(clients) > 0:
                await websocket.send(buffer)
                await asyncio.sleep(0.2)

    except websockets.exceptions.ConnectionClosedOK:
        print("[SERVER] Client disconnected from server!")
        # await websocket.send(json.dumps("[SERVER] You have disconnected from the server"))
        # await asyncio.sleep(0.01)
        clients.remove(websocket)
    except websockets.ConnectionClosedError:
        print("[SERVER] Internal Server Error.")
        # await websocket.send(json.dumps("[SERVER] Internal Server error has occurred!"))
        # await asyncio.sleep(0.01)


async def startServer():
    async with websockets.serve(sendServerDataToClient, "0.0.0.0", 8777):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    gps = ActualGPSCompass()
    try:
        print("[SERVER] Server ON")
        asyncio.run(startServer())
    except KeyboardInterrupt:
        print("[SERVER] Keyboard Interrupt occurred!")
