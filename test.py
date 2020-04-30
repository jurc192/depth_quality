import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2


supported_resolutions = [
    (1280, 720),
    (848, 480),
    (640, 480),
    (640, 360),
    (480, 270)
]

exposures = [6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000]
laser_powers = [i for i in range(0, 360, 30)]


class ParameterExplorer:
    
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config   = rs.config()


    def capture_depthframe(self, resolution, exposure, laser_power, depth_preset=1):
        """ Capture a single depth frame with given parameter settings
        
        resolution  - (width, height) pixels
        exposure    - miliseconds
        laser_power - mW
        depth_preset- [0,5] Custom, Default, Hand, High Accuracy, High Density, Medium Density
        
        Returns pyrealsense2.depth_frame object
        """

        # Set resolution and check if it's valid
        self.config.enable_stream(rs.stream.depth, resolution[0], resolution[1])
        if (self.config.can_resolve(self.pipeline) == False):
            print("Resolution not supported")
            return
        self.sensor = config.resolve(self.pipeline).get_device().first_depth_sensor()

        # Set exposure
        if (exposure == 0):
            self.sensor.set_option(rs.option.auto_exposure, True)
        else:
            self.sensor.set_option(rs.option.exposure, exposure)

        # Set laser power
        self.sensor.set_option(rs.option.laser_power, laser_power)

        # Set depth preset
        self.sensor.set_option(rs.option.visual_preset, depth_preset)


    @staticmethod
    def get_frame_metadata(frame):
        metadata = {}
        metadata['resolution (px)'] = f"{frame.get_width()} x {frame.get_height()} px"
        metadata['autoexposure']  = frame.get_frame_metadata(rs.frame_metadata_value.auto_exposure)
        metadata['exposure (ms)'] = frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure) # docs say ms, but not really sure
        metadata['laser power (mW)']  = frame.get_frame_metadata(rs.frame_metadata_value.frame_laser_power)
        return metadata


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

    print(get_metadata(depth_frame))
    


