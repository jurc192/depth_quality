#
#   Custom depth quality measurement tool
#

# TODO:
# OK - live preview
# OK - button press
# OK - taking a depth image frame
# - calculating best fit plane
# - calculating RMSE
# - displaying results

import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2



def display_pointcloud(depth_image, intrinsics):

    # Convert rs2 depthmap into o3d depthmap
    depth_image = o3d.geometry.Image(depth_image)
    intr = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width,
        intrinsics.height,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.ppx,
        intrinsics.ppy
    )
    o3ddepth = o3d.geometry.PointCloud.create_from_depth_image(depth_image, intr)

    # calculate best-fit plane

    o3d.visualization.draw_geometries([o3ddepth], width=800, height=600)


def main():

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:

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
            images = np.hstack((color_image, depth_colormap))

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            
            key = cv2.waitKey(0)
            if key == ord('q') or key == 27:    # esc
                break

            elif key == 13:     # enter
                cv2.destroyAllWindows()
                intr = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
                display_pointcloud(depth_image, intr)

    
    except:
        print(e)

    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()

