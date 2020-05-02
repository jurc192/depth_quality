#
#   depth_acquisition.py
#   Script that enables collecting depth images at different distances, with variable parameters
#

# 1. Preview and set exposure time
# 2. For every distance:
    # - position the camera
    # - capture images
    # - save to files
import pyrealsense2 as rs
import cv2
import numpy as np

def exposure_preview():

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
        if key == ord('q') or key == 27:    # esc
            break

        elif key == ord('w'):
            exposure = exposure + 2000
            sensor.set_option(rs.option.exposure, exposure)
            print(f"exposure {exposure}")

        elif key == ord('s'):
            exposure = exposure - 2000
            sensor.set_option(rs.option.exposure, exposure)
            print(f"exposure {exposure}")

        elif key == 13:     # enter
            cv2.destroyAllWindows()
            exposures.append(exposure)
            print(f"Added {exposure} to exposure list")

    pipeline.stop()
    return exposures


if __name__ == "__main__":
    
    distances   = []
    resolutions = []
    laserpowers = []
    exposures   = sorted(set(exposure_preview()))
    
    print(f"\nSelected exposures: {exposures}")

    # for dist in distances:
    #     input("Place the camera to distance: {dist}cm")
    #     for res in resolutions:
    #         for exp in exposures:
    #             for lpow in laserpowers:
    #                 depthframe = capture_depthframe(*res, exposure, laserpower)
    #                 intrinsics = get_intrinsics(*res)
    #                 filename = f"{dist}_{res[0]}x{res[1]}_{exposure}_{laserpower}"
    #                 save_depth_raw(f"{filename}.raw", depthframe)
    #                 save_depth_colorized(f"{filename}.png", depthframe)
    #                 save_pointcloud(f"{filename}.ply", depthframe, intrinsics)
                    