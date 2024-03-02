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

    if len(ports) > 0:
        for port, desc, hwid in sorted(ports):
            if hwid != "n/a":
                return port
    return None
port = getDevicePort()

print(f"Port = {getDevicePort()}")

if port is None:
    print("Port not found!")
    sys.exit(1)

lidar = RPLidar(port)


# Creating WebSocket server
async def server(websocket):
    global lidar

    try:
        while True:
            # Receiving values from client (lidar)
            # RAW DATA is in tuples
            # lidar_data = tuple()
            # lidar_data = None

            
            for scan in lidar.iter_scans():
                # print(json.dumps(scan))
                await websocket.send(json.dumps(scan))
                await asyncio.sleep(0.005)

    except websockets.exceptions.ConnectionClosedOK:
        # await websocket.send("[SERVER] Disconnecting from server")
        # await asyncio.sleep(1)
        print("[SERVER] Client disconnected from server!")
    except websockets.ConnectionClosedError:
        print("Internal Server Error.")
    except rplidar.RPLidarException:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        lidar = RPLidar(port)
 
 
async def main():
    async with websockets.serve(server, "0.0.0.0", 8765):
        await asyncio.Future()  # run forever
 
if __name__ == "__main__":
    try:
        print("[SERVER] Server ON")
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Keyboard Interrupt occurred!")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()