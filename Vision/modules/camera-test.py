from depthai_sdk import OakCamera

with OakCamera() as oak:
    # Create stereo component, initialize left/right MonoCamera nodes for 800P and 60FPS
    stereo = oak.create_stereo('1020p', fps=60)

    # Visualize normalized and colorized disparity stream
    oak.visualize(stereo.out.depth)
    # Start the pipeline, continuously poll
    oak.start(blocking=True)