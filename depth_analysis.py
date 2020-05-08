#
#   depth_analysis.py
#   Script for analyzing planarity of pointclouds (wall scans)
#

# import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import sys
from pathlib import Path
from sklearn import linear_model


def plane_fit_RMSE(points, depth_unit=0.0001):
    """ Calculate best-fit plane for a given pointcloud and return rmse (in meters)
        Depth unit is specified in meters, set to smallest value (100um) by default
    """

    # Form the system of equations in matrix form: Aw = z
    A = []
    z = []
    for p in points:
        A.append([p[0], p[1], 1])
        z.append(p[2])
    
    # Find best-fit plane using linear regression
    reg = linear_model.LinearRegression()
    reg.fit(A, z)
    a = reg.coef_[0]
    b = reg.coef_[1]
    c = 1
    d = reg.intercept_

    distances = np.array([((a*x + b*y + c*z - d) / np.sqrt(a*a + b*b + c*c)) for x,y,z in points])
    rmse      = np.sqrt(np.sum((distances**2)) / distances.size)
    return rmse * depth_unit


def parse_params(folder):
    """ Utility function to parse all .ply files in a folder and return lists of used settings
        Assuming naming convention: distance_resolution_exposure_laserpower.ply
    """
    distances   = set()
    resolutions = set()
    exposures   = set()
    laserpowers = set()
    files = Path(sys.argv[1]).glob('*.ply')
    for f in files:
        dist, res, exp, lpow = f.stem.split('_')
        distances.add(int(dist))    # Float might be better
        resolutions.add(res)
        exposures.add(int(exp))
        laserpowers.add(int(lpow))

    return (
        sorted(list(distances)),
        sorted(list(resolutions)),
        sorted(list(exposures)),
        sorted(list(laserpowers))
    )


def centered_crop(image, percents):
    """ Return a cropped image, centre aligned, percentage or original size (width) """

    height, width = image.shape[:2]
    roiw = int(percents/100 * width)
    roih = int(percents/100 * height)

    x,y = ((width-roiw)//2 , (height-roih)//2)
    image[y:y+roih , x:x+roiw] = 1
    # return image[y:y+roih , x:x+roiw]


def depth_to_pointcloud(depthmap):

    # Get intrinsics
    # Hardcoded values for RealSense D435i camera, using video_stream_profile.get_intrinsics() function
    height, width = depthmap.shape[:2]
    if (height, width) == (720, 1280):
        fx, fy, ppx, ppy = 635.201, 635.201, 639.165, 366.071
    elif (height, width) == (480, 848):
        fx, fy, ppx, ppy = 420.821, 420.821, 423.447, 244.022
    elif (height, width) == (480, 640):
        fx, fy, ppx, ppy = 381.121, 381.121, 319.499, 243.643
    elif (height, width) == (360, 640):
        fx, fy, ppx, ppy = 317.601, 317.601, 319.583, 183.036
    elif (height, width) == (270, 480):
        fx, fy, ppx, ppy = 238.201, 238.201, 239.687, 137.277
    else:
        print(f"Wrong depthmap resolution: {width} x {height}")
        return 0
    intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, ppx,ppy)
    print(intrinsics)

    # Convert to o3d format
    depthmap   = o3d.geometry.Image(depthmap)
    pointcloud = o3d.geometry.PointCloud.create_from_depth_image(depthmap, intrinsics)
    print(intrinsics.intrinsic_matrix)
    return pointcloud



if __name__ == "__main__":

    import cv2

    if len(sys.argv) < 2:
        print("Usage: ./depth_analysis.py <input_file.raw>")
        sys.exit()

    # Parse raw file
    depthmap_raw = np.loadtxt(sys.argv[1], dtype='uint16')
    pointcloud = depth_to_pointcloud(depthmap_raw)
    o3d.visualization.draw_geometries([pointcloud], width=800, height=600)

    # Create a mask
    mask = np.zeros(depthmap_raw.shape, dtype='uint16')
    percents = 65
    height, width = mask.shape[:2]
    roiw = int(percents/100 * width)
    roih = int(percents/100 * height)
    x,y = ((width-roiw)//2 , (height-roih)//2)
    mask[y:y+roih , x:x+roiw] = 1

    # Apply mask to depthmap
    cropped = mask * depthmap_raw
    print(cropped.dtype)
    print(mask.dtype)
    print(depthmap_raw.dtype)
    print(cropped)
    print(cropped[300:400, 600:700])
    print(cropped.shape)

    pointcloud = depth_to_pointcloud(cropped)
    o3d.visualization.draw_geometries([pointcloud], width=800, height=600)

    # # Display
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cropped, alpha=0.03), cv2.COLORMAP_JET)
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', depth_colormap)
    # cv2.waitKey(0)


