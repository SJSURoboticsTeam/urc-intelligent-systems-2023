import websockets
import asyncio
import rplidar
from rplidar import RPLidar
import json
import sys
import serial.tools.list_ports
import serial


def getDevicePort():
    ports = serial.tools.list_ports.comports()

    for port in ports:
        if "USB" in port.device:
            return port.device

port = getDevicePort()

print(f"Port = {getDevicePort()}")

if port is None:
    print("Port not found!")
    sys.exit(1)

lidar = RPLidar(port)
clients = [] # List of clients connected in the server
buffer = None

# Creating WebSocket server
async def serverConnections(websocket):
    global lidar

    clients.append(websocket)

    try:
        while True:
            # Receiving values from client (lidar)
            # RAW DATA is in tuples
            for scan in lidar.iter_scans():
                buffer = json.dumps(scan)

                if len(clients) > 0:
                    data = await data_buffer.get()
                    await websocket.send(data)
                    await asyncio.sleep(0.01)

    except websockets.exceptions.ConnectionClosedOK:
        print("[SERVER] Client disconnected from server!")
        clients.remove(websocket)
    except websockets.ConnectionClosedError:
        print("Internal Server Error.")
    except rplidar.RPLidarException:
        print("RPLidarException has been caught!")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        lidar = RPLidar(port)
 
 
async def runningServer():
    async with websockets.serve(serverConnections, "0.0.0.0", 8765):
        await asyncio.Future()  # run forever
 
if __name__ == "__main__":
    try:
        print("[SERVER] Server ON")
        asyncio.run(runningServer())
    except KeyboardInterrupt:
        print("Keyboard Interrupt occurred!")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
