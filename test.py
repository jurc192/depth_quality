from time import sleep
import pyrealsense2 as rs
import cv2
import numpy as np
import open3d as o3d



def depth_to_pointcloud(depthmap, intrinsics):

    depth_image = o3d.geometry.Image(depthmap)
    intr = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width,
        intrinsics.height,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.ppx,
        intrinsics.ppy
    )
    o3ddepth = o3d.geometry.PointCloud.create_from_depth_image(depth_image, intr)
    return o3ddepth



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
    
    intrinsics = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    config.disable_all_streams()
    pipeline.stop()

    return frame, intrinsics


def get_metadata(frame):
    metadata = {}
    metadata['resolution (px)'] = f"{frame.get_width()} x {frame.get_height()} px"
    metadata['autoexposure']  = frame.get_frame_metadata(rs.frame_metadata_value.auto_exposure)
    metadata['exposure (ms)'] = frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure) # docs say ms, but not really sure
    metadata['laser power (mW)']  = frame.get_frame_metadata(rs.frame_metadata_value.frame_laser_power)
    return metadata


if __name__ == "__main__":

    resolutions = [(1280, 720), (848, 480), (640, 480), (640, 360), (480, 270)]
    
    # for res in resolutions:
    #     for exposure in range(6500, 9000, 500):
    #         for laserpower in range(150, 300, 60):
    #             frame = capture_depthframe(*res, exposure, laserpower)
    #             print(f"na_{res[0]}x{res[1]}_{exposure}_{laserpower}.ply")
    #             print(get_metadata(frame))
    #             print()


    depth_raw, intr   = capture_depthframe(exposure=9000)
    depth_raw         = np.asanyarray(depth_raw.get_data())
    depth_color       = cv2.applyColorMap(cv2.convertScaleAbs(depth_raw, alpha=0.03), cv2.COLORMAP_JET)

    np.savetxt('testimage.raw', depth_raw, fmt="%u")        # Save raw depth image
    cv2.imwrite('testimage.png', depth_color)                                        # Save colorized depth image
    o3d.io.write_point_cloud('testimage.ply', depth_to_pointcloud(depth_raw, intr))  # Save pointcloud

    # # Save pointcloud
    # intr = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    # depth_image = o3d.geometry.Image(depth_image)
    # intr = o3d.camera.PinholeCameraIntrinsic(
    #     intrinsics.width,
    #     intrinsics.height,
    #     intrinsics.fx,
    #     intrinsics.fy,
    #     intrinsics.ppx,
    #     intrinsics.ppy
    # )
    # o3ddepth = o3d.geometry.PointCloud.create_from_depth_image(depth_image, intr)

    

    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', depth_raw)
    # cv2.waitKey(0)







