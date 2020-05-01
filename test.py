import pyrealsense2 as rs
import open3d as o3d
import cv2
import numpy as np
from time import sleep


def get_intrinsics(width=1280, height=720):
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.depth, width, height)
    pipeline.start(config)
    intrinsics = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    pipeline.stop()
    return intrinsics


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
    
    config.disable_all_streams()
    pipeline.stop()

    return frame


def get_metadata(frame):
    metadata = {}
    metadata['resolution (px)'] = f"{frame.get_width()} x {frame.get_height()} px"
    metadata['autoexposure']  = frame.get_frame_metadata(rs.frame_metadata_value.auto_exposure)
    metadata['exposure (ms)'] = frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure) # docs say ms, but not really sure
    metadata['laser power (mW)']  = frame.get_frame_metadata(rs.frame_metadata_value.frame_laser_power)
    return metadata


def save_depth_raw(filename, depthmap):
    depthmap = np.asanyarray(depthmap.get_data())
    np.savetxt(filename, depthmap, fmt="%u")

def save_depth_colorized(filename, depthmap):
    depthmap = np.asanyarray(depthmap.get_data())
    colorized = cv2.applyColorMap(cv2.convertScaleAbs(depthmap, alpha=0.03), cv2.COLORMAP_JET)
    cv2.imwrite(filename, colorized)

def save_pointcloud(filename, depthmap, intrinsics):
    # Convert realsense depthmap and intrinsics into open3d formats
    depthmap = np.asanyarray(depthmap.get_data())
    depthmap = o3d.geometry.Image(depthmap)
    intr = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width,
        intrinsics.height,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.ppx,
        intrinsics.ppy
    )
    pointcloud = o3d.geometry.PointCloud.create_from_depth_image(depthmap, intr)
    o3d.io.write_point_cloud(filename, pointcloud)


if __name__ == "__main__":

    resolutions = [(1280, 720), (848, 480), (640, 480), (640, 360), (480, 270)]
    
    # for res in resolutions:
    #     for exposure in range(6500, 9000, 500):
    #         for laserpower in range(150, 300, 60):
                
    #             depthframe = capture_depthframe(*res, exposure, laserpower)
    #             intrinsics = get_intrinsics(*res)

    #             filename = f"na_{res[0]}x{res[1]}_{exposure}_{laserpower}"
    #             save_depth_raw(f"{filename}.raw")
    #             save_depth_colorized(f"{filename}.png")
    #             save_pointcloud(f"{filename}.ply")


    res = (848, 480)
    exposure = 9000
    laserpower = 300

    print("Init")
    depthframe = capture_depthframe(*res, exposure, laserpower)
    print("Depthframe OK")
    intrinsics = get_intrinsics(*res)
    print("Intrinsics OK")
    print(intrinsics)

    filename = f"na_{res[0]}x{res[1]}_{exposure}_{laserpower}"
    print(f"Filename: {filename}")
    save_depth_raw(f"{filename}.raw", depthframe)
    print("depth_raw OK")
    save_depth_colorized(f"{filename}.png", depthframe)
    print("depth_colorized OK")
    save_pointcloud(f"{filename}.ply", depthframe, intrinsics)
    print("pointcloud OK")
