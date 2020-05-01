from time import sleep
import pyrealsense2 as rs


def capture_depthframe(width=1280, height=720, exposure=0, laser_power=240, depth_preset=1):

    pipeline = rs.pipeline()
    config   = rs.config()

    # Set resolution and check if it's valid
    config.enable_stream(rs.stream.depth, width, height)
    if (config.can_resolve(pipeline) == False):
        print("Resolution not supported")
        return
    sensor = config.resolve(pipeline).get_device().first_depth_sensor()

    # Set remaining parameters
    if (exposure == 0):
        sensor.set_option(rs.option.enable_auto_exposure, True)
    else:
        sensor.set_option(rs.option.exposure, exposure)
    sensor.set_option(rs.option.laser_power, laser_power)
    sensor.set_option(rs.option.visual_preset, depth_preset)

    # Get a depth frame
    pipeline.start(config)
    if (exposure == 0):         # Stabilize autoeposure, if enabled
        for _ in range(5):
            pipeline.wait_for_frames().get_depth_frame()
            sleep(0.2)
    frame = pipeline.wait_for_frames().get_depth_frame()
    pipeline.stop()
    config.disable_all_streams()

    return frame


def get_metadata(frame):
    metadata = {}
    metadata['resolution (px)'] = f"{frame.get_width()} x {frame.get_height()} px"
    metadata['autoexposure']  = frame.get_frame_metadata(rs.frame_metadata_value.auto_exposure)
    metadata['exposure (ms)'] = frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure) # docs say ms, but not really sure
    metadata['laser power (mW)']  = frame.get_frame_metadata(rs.frame_metadata_value.frame_laser_power)
    return metadata


if __name__ == "__main__":

    resolutions = [(1280, 720), (848, 480), (640, 480), (640, 360), (480, 270)]
    
    for res in resolutions:
        for exposure in range(6500, 9000, 500):
            for laserpower in range(150, 300, 60):
                frame = capture_depthframe(*res, exposure, laserpower)
                print(get_metadata(frame))





