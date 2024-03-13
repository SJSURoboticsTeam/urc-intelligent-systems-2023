# Socket Bridge

One general problem that has crept up is transferring data between the rover and the human control interface. When this appeared for streaming lidar data we tackled this by creating a lidar server and lidar client connected by a websocket connection. The lidar client pretends to be a regular lidar so the rest of the stack does not care where the lidar data is actually coming from. This pattern proved successful so the idea is to extend this pattern as a general solution and apply that solution to all data that might be streamed from human to rover or vice-versa.

## Goal: Make a magic pipe

Any device should be able to extend this pipe to handle the data production and consumption in the same code file while abstracting over how the data gets from production to consumption.

### OK, maybe magic pipeline is not easily feasible

but I think its feasible to create a magic dict thats synchronized across the bridge. [Client Side](client_side.py) and [Rover Side](rover_side.py) together make a sample of this bridge. When the **Rover Side** is running, any **Client Side** that starts up will synchronize its data to the data of the **Rover Side**.

Try it out

```sh
python rover_side.py
```

```sh
python client_side.py # do this twice on two different terminals at once
```
