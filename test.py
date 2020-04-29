import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2


if __name__ == "__main__":

    # Initialize realsense stuff
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720)
    profile = config.resolve(pipeline)
    device = profile.get_device()
    pipeline.start()

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()

    sensor = device.first_depth_sensor()
    pipeline.stop()
    print(sensor.get_option(rs.option.laser_power))
    sensor.set_option(rs.option.laser_power, 50)
    print(sensor.get_option(rs.option.laser_power))
    pipeline.start()

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()


