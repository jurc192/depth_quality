#
#   depth_acquisition.py
#   Script for collecting depth images using different parameter settings
#
#   Usage: ./depth_acquisition.py
#   1. Select exposure times that seem reasonable (using 'up/down', 'enter' to add, 'q' to finish)
#   2. Place the camera at given distance
#   3. Press ENTER and wait
#
#   Warning: might need a USB3 connection (due to a bug in rs firmware)
#
import pyrealsense2 as rs
import open3d as o3d
import cv2
import numpy as np
from pathlib import Path
from time import sleep


def get_intrinsics(width=1280, height=720):
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.depth, width, height)
    pipeline.start(config)
    intrinsics = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    pipeline.stop()
    return intrinsics


def save_depth_raw(filename, depthmap):
    depthmap = np.asanyarray(depthmap.get_data())
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(filename, depthmap, fmt="%u")


def save_depth_colorized(filename, depthmap):
    depthmap = np.asanyarray(depthmap.get_data())
    colorized = cv2.applyColorMap(cv2.convertScaleAbs(depthmap, alpha=0.03), cv2.COLORMAP_JET)
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
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
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(filename, pointcloud)


def capture_depthmap(width=848, height=480, exposure=8500, laser_power=150, depth_preset=1):

    pipeline = rs.pipeline()
    config   = rs.config()
    
    # Enable depth stream with given resolution
    config.enable_stream(rs.stream.depth, width, height)
    if (config.can_resolve(pipeline) == False):
        print("Resolution not supported")
        return

    # Set parameters
    sensor = config.resolve(pipeline).get_device().first_depth_sensor()
    sensor.set_option(rs.option.depth_units, 0.0001)     # Smallest depth unit (in meters)
    sensor.set_option(rs.option.enable_auto_exposure, False)
    sensor.set_option(rs.option.exposure, exposure)
    sensor.set_option(rs.option.laser_power, laser_power)
    sensor.set_option(rs.option.visual_preset, depth_preset)

    # Get depth frame
    pipeline.start(config)
    frame = pipeline.wait_for_frames().get_depth_frame()
    config.disable_all_streams()
    pipeline.stop()
    return frame


def exposure_preview():
    """ Opens a window for previewing video streams (color, depth, IR).
        Select desired exposure time(s) with arrow keys and pres enter to add to a list. 
        Press Q to exit, when finished.
        Returns a list of selected exposure times
    """

    exposures = []
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 1, 640,480, rs.format.y8, 30)
    config.enable_stream(rs.stream.infrared, 2, 640,480, rs.format.y8, 30)
    pipeline.start(config)
    sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
    sensor.set_option(rs.option.enable_auto_exposure, False)

    while True:
        # Get depth, color and IR frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        irLeft_frame = frames.get_infrared_frame(1)
        irRight_frame = frames.get_infrared_frame(2)

        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        irLeft_frame = np.asanyarray(irLeft_frame.get_data())
        irRight_frame = np.asanyarray(irRight_frame.get_data())

        # Colorize, convert and display everything
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        imagesTop    = np.hstack((color_image, depth_colormap))
        imagesBottom = cv2.cvtColor(np.hstack((irLeft_frame, irRight_frame)), cv2.COLOR_GRAY2BGR)
        images = np.vstack((imagesTop, imagesBottom))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        
        exposure = depth_frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure)
        key = cv2.waitKey(2)
        if key == ord('q') or key == 27:                        # ESC or Q (exit)
            cv2.destroyAllWindows()
            break

        elif key == ord('w') or key == 82:                      # UP or W (increase exposure)
            exposure = exposure + 2000
            sensor.set_option(rs.option.exposure, exposure)
            print(f"exposure {exposure}")

        elif key == ord('s') or key == 84:                      # DOWN or S (decrease exposure)
            exposure = exposure - 2000
            sensor.set_option(rs.option.exposure, exposure)
            print(f"exposure {exposure}")

        elif key == 13:                                         # ENTER (add to exposure list)
            cv2.destroyAllWindows()
            exposures.append(exposure)
            print(f"Added {exposure} to exposure list")

    pipeline.stop()
    return exposures


def find_largest_index(directory):
    files = Path(directory).glob('*')     # File naming regex, rather than all files
    nlist = []
    for f in files:
        dist, res, exp, lpow, preset, n = f.stem.split('_')
        nlist.append(int(n))
    # print(nlist)
    # print(max(nlist))
    if not nlist:
        return 0
    else:
        return max(nlist) + 1


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: ./depth_acquisition <distance> <exposure> <lpow> <preview> <destination>")
        sys.exit()
    
    dist, exp, lpow, preview, dstfolder = sys.argv[1:]
    print(f"{dist}, {exp}, {lpow}, {dstfolder}, {preview}") # Check if input params work ok

    # Preview positioning and exposure
    if preview == True:
        exposure_preview()

    for i in range(50):
        # Capture depthmaps
        depthmap = capture_depthmap(848, 480, exp, lpow)
        save_depth_raw(f"{dist}_848x480_{exp}_{lpow}_1_{i}.raw", depthmap)

        