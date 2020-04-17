#
#   Custom depth quality measurement tool
#
# Usage: python pydepth_quality.py <input_file.ply> 

# What do we want?
# - live preview
# - button press
# - taking a depth image frame
# - calculating best fit plane
# - calculating RMSE
# - displaying results

# Use Open3D library for vizualization, start with reading .ply from file


import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2
import sys

# if len(sys.argv) < 2:
#     print("Usage: ./pydepth_quality.py <pcl file>")
#     sys.exit(1)

# pcl = o3d.io.read_point_cloud(sys.argv[1])
# if (pcl):
#     o3d.visualization.draw_geometries([pcl], window_name='Point cloud visualization', width=800, height=600)
# else:
#     print("Bad input file")


def display_pointcloud(depth_image, intrinsics):

    # depth (open3d.geometry.Image) –
    # intrinsic (open3d.camera.PinholeCameraIntrinsic) –
    # extrinsic (numpy.ndarray[float64[4, 4]], optional) – array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])
    # depth_scale (float, optional, default=1000.0) –
    # depth_trunc (float, optional, default=1000.0) –
    # stride (int, optional, default=1) –

    print(intrinsics)
    print(depth_image)

    depth_image = o3d.geometry.Image(depth_image)
    intr = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width,
        intrinsics.height,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.ppx,
        intrinsics.ppy
    )

    # depth_image = o3d.geometry.Image(depth_image)
    o3ddepth = o3d.geometry.PointCloud.create_from_depth_image(depth_image, intr)
    o3d.visualization.draw_geometries([o3ddepth], width=800, height=600)







if __name__ == "__main__":

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            
            key = cv2.waitKey(0)
            if key == ord('q') or key == 27:
                break
            elif key == 13:
                print("Getting video stream intrinsics...")
                intr = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
                display_pointcloud(depth_image, intr)

    finally:

        # Stop streaming
        pipeline.stop()
