import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2

class ParameterExplorer:
    
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config   = rs.config()


    def capture_depthframe(self, width=1280, height=720, exposure=8500, laser_power=240, depth_preset=1):

        # Set resolution and check if it's valid
        self.config.enable_stream(rs.stream.depth, width, height)
        if (self.config.can_resolve(self.pipeline) == False):
            print("Resolution not supported")
            return

        # Enable stream
        self.sensor = self.config.resolve(self.pipeline).get_device().first_depth_sensor()

        # Set exposure, laser_power and depth_preset
        if (exposure == 0):
            self.sensor.set_option(rs.option.enable_auto_exposure, True)
        else:
            self.sensor.set_option(rs.option.exposure, exposure)
        self.sensor.set_option(rs.option.laser_power, laser_power)
        self.sensor.set_option(rs.option.visual_preset, depth_preset)


        # Get a depth frame
        self.pipeline.start(self.config)
        frame = self.pipeline.wait_for_frames().get_depth_frame()
        self.pipeline.stop()
        self.config.disable_all_streams()

        return frame


def get_frame_metadata(frame):
    metadata = {}
    metadata['resolution (px)'] = f"{frame.get_width()} x {frame.get_height()} px"
    metadata['autoexposure']  = frame.get_frame_metadata(rs.frame_metadata_value.auto_exposure)
    metadata['exposure (ms)'] = frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure) # docs say ms, but not really sure
    metadata['laser power (mW)']  = frame.get_frame_metadata(rs.frame_metadata_value.frame_laser_power)
    return metadata


if __name__ == "__main__":

    cam = ParameterExplorer()

    frame = cam.capture_depthframe()
    print(get_frame_metadata(frame))
    
    frame2 = cam.capture_depthframe(848, 480, 6500, 150, 1)
    print(get_frame_metadata(frame2))
    
    frame3 = cam.capture_depthframe(480, 270, 8500, 150, 1)
    print(get_frame_metadata(frame3))

