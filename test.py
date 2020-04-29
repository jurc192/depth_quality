import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2


if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16)

    profile = config.resolve(pipeline)

    device = profile.get_device()
    sensor = device.first_depth_sensor()

    # What can this depth_sensor do?
    # sensor.profiles -> list of supported stream profiles (fps+res+etc.)
    # sensor.get_supported_options() ->
    #   option.exposure,
    #   option.gain,
    #   option.enable_auto_exposure,
    #   option.visual_preset,
    #   option.laser_power,
    #   option.emitter_enabled,
    #   option.frames_queue_size,
    #   option.asic_temperature,
    #   option.error_polling_enabled,
    #   option.projector_temperature,
    #   option.output_trigger_enabled,
    #   option.depth_units,
    #   option.stereo_baseline,
    #   option.inter_cam_sync_mode,
    #   option.emitter_on_off,
    #   option.global_time_enabled

