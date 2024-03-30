import depthai

# Initialize the pipeline
pipeline = depthai.Pipeline()

# Load the config file
board_config = depthai.BoardConfig("path/to/config.json")

# Start the pipeline
pipeline.start(board_config)

# Continuously retrieve frames from the pipeline
while True:
    # Get the latest frame
    packet = pipeline.get_available_packet()

    # Check if a packet is available
    if packet is None:
        continue

    # Get the depth information from the frame
    depth = packet.getData(depthai.CAMERA_T1_LEFT).reshape((packet.height, packet.width))

    # Find the closest obstacle
    closest_obstacle = depth.min()

    # Calculate the width of the obstacle
    obstacle_width = depth.shape[1]

    # Print the distance and width of the obstacle
    print("Distance:", closest_obstacle, "Width:", obstacle_width)

# Stop the pipeline
pipeline.stop()
