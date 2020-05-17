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
import matplotlib.pyplot as plt


def plane_fit_RMSE(points, depth_unit=0.0001):
    """ Calculate best-fit plane for a given pointcloud and return rmse (in meters)
        Depth unit is specified in meters, set to smallest value (100um) by default
    """

    # Form the system of equations in matrix form: Aw = z
    A = np.column_stack((points[:,0], points[:,1], np.ones((points.shape[0], 1), dtype='uint16')))
    z = points[:,2]
    # A = []
    # z = []
    # for p in points:
    #     A.append([p[0], p[1], 1])
    #     z.append(p[2])
    
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


def depth_to_pointcloud(depthmap, roi=100):
    """ Convert depthmap (uint16) into a point cloud. ROI -> percentage of original size """

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

    # Crop depthmap
    if roi < 100:
        mask = np.zeros(depthmap_raw.shape, dtype='uint16')
        height, width = mask.shape[:2]
        roih, roiw = int(roi/100 * height), int(roi/100 * width)
        x,y = ((width-roiw)//2 , (height-roih)//2)  # Top-left point of ROI
        mask[y:y+roih , x:x+roiw] = 1
        depthmap = mask * depthmap_raw               # Apply mask to the depthmap

    # Convert to o3d format
    depthmap   = o3d.geometry.Image(depthmap)
    pointcloud = o3d.geometry.PointCloud.create_from_depth_image(depthmap, intrinsics)
    return pointcloud


def parse_params(folder):
    """ Utility function to parse all files in a folder and return lists of used settings
        Assuming naming convention: distance_resolution_exposure_laserpower.<extension>
    """
    distances   = set()
    resolutions = set()
    exposures   = set()
    laserpowers = set()
    files = Path(sys.argv[1]).glob('./*/*')     # File naming regex, rather than all files
    for f in files:
        dist, res, exp, lpow = f.stem.split('_')
        distances.add(int(dist))    # Float might be better
        resolutions.add(res)
        exposures.add(int(exp))
        laserpowers.add(int(lpow))

    return (
        sorted(list(distances)),
        sorted(list(resolutions), key = lambda item: int(item.split('x')[0])),  # Sort numerically
        sorted(list(exposures)),
        sorted(list(laserpowers))
    )


if __name__ == "__main__":

    import csv

    if len(sys.argv) < 3:
        print("Usage: ./depth_analysis.py <input_folder> <output_filename>")
        sys.exit()

    infolder = sys.argv[1]
    outfile = sys.argv[2]
    dist = sys.argv[3]
    
    ## Parse Experiment 8
    nframes   = 50
    res       = (848,480)
    exposures = [8500, 7500, 6500, 5500, 4500, 3500, 2500, 1500]
    # exposures = [8500, 7500]
    lpowers   = [150, 180, 210, 240]
    # lpowers   = [150, 180]

    results = np.zeros((nframes,len(lpowers), len(exposures)))
    print(f"Results shape: {results.shape}")

    # For every exp, lpow combination = configuration, do:
    for i, exp in enumerate(exposures):
        for j, lpow in enumerate(lpowers):

            # Get all frames, calc rmse for each
            frames = Path(infolder).glob(f"{dist}_{res[0]}x{res[1]}_{exp}_{lpow}_1_*")
            for k, frame in enumerate(frames):
                depthmap     = np.loadtxt(frame, dtype='uint16')
                pointcloud   = depth_to_pointcloud(depthmap)
                rmse = plane_fit_RMSE(np.array(pointcloud.points)) * 1000 # convert to millimeters
                print(f"{exp}\t{lpow}\t{k}: {rmse}")
                results[k, j, i] = rmse
                # if k == nframes-1:
                #     break

    # Save individual measurements
    for i in range(nframes):
        np.savetxt(f'{infolder}/results/{i}_{outfile}', results[i], fmt='%.6f', delimiter=' ', newline='\n')

    # Save average
    avgresults = np.mean(results, axis=0)
    np.savetxt(f'{infolder}/results/avg_{outfile}', avgresults, fmt='%.6f', delimiter=' ', newline='\n')
    
    # # Plot something
    # plt.imshow(avgresults, cmap='hot', interpolation='nearest')
    # plt.show()