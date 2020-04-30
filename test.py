import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2


def capture_depthframe(width=1280, height=720, exposure=8500, laser_power=240, depth_preset=1):

    pipeline = rs.pipeline()
    config   = rs.config()

    # Set resolution and check if it's valid
    config.enable_stream(rs.stream.depth, width, height)
    if (config.can_resolve(pipeline) == False):
        print("Resolution not supported")
        return

    # Enable stream
    sensor = config.resolve(pipeline).get_device().first_depth_sensor()

    # Set exposure, laser_power and depth_preset
    if (exposure == 0):
        sensor.set_option(rs.option.enable_auto_exposure, True)
    else:
        sensor.set_option(rs.option.exposure, exposure)
    sensor.set_option(rs.option.laser_power, laser_power)
    sensor.set_option(rs.option.visual_preset, depth_preset)


    # Get a depth frame
    pipeline.start(config)
    frame = pipeline.wait_for_frames().get_depth_frame()
    pipeline.stop()
    config.disable_all_streams()

    return frame


def get_frame_metadata(frame):
    metadata = {}
    metadata['resolution (px)'] = f"{frame.get_width()} x {frame.get_height()} px"
    metadata['autoexposure']  = frame.get_frame_metadata(rs.frame_metadata_value.auto_exposure)
    metadata['exposure (ms)'] = frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure) # docs say ms, but not really sure
    metadata['laser power (mW)']  = frame.get_frame_metadata(rs.frame_metadata_value.frame_laser_power)
    return metadata


if __name__ == "__main__":

    # cam = ParameterExplorer()

    frame = capture_depthframe()
    print(get_frame_metadata(frame))
    
    frame2 = capture_depthframe(848, 480, 6500, 150, 1)
    print(get_frame_metadata(frame2))
    
    frame3 = capture_depthframe(480, 270, 8500, 150, 1)
    print(get_frame_metadata(frame3))

