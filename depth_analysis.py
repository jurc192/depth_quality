#
#   depth_analysis.py
#   Script for analyzing planarity of pointclouds (wall scans)
#


import pyrealsense2 as rs
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


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("Usage: ./depth_analysis.py <input_directory>")
        print("\t<input_directory>  directory containing .ply files with naming convention:")
        print("\t\tdistance_resolution_exposure_laserpower.ply")
        sys.exit()

    distances, resolutions, exposures, laserpowers = parse_params(sys.argv[1])
    
    # Example usage: observing distance
    res = resolutions[0]
    exp = exposures[0]
    lpow = laserpowers[0]
    print(f"\nObserving distance using resolution {res}, exposure {exp}, laserpower {lpow} mW")
    for dist in distances:
        filename = f"{sys.argv[1]}/{dist}_{res}_{exp}_{lpow}.ply"
        points  = np.asanyarray(o3d.io.read_point_cloud(filename).points)
        print(f"RMSE ({dist} cm):\t{plane_fit_RMSE(points) * 1000:.6f} mm")


    # Example usage: observing laser power
    dist = "30"
    res = "848x480"
    exp = "8500"
    print(f"\nObserving laser power at distance {dist}cm, resolution {res}, exposure {exp}")
    for lpow in laserpowers:
        filename = f"{sys.argv[1]}/{dist}_{res}_{exp}_{lpow}.ply"
        points  = np.asanyarray(o3d.io.read_point_cloud(filename).points)
        print(f"RMSE ({lpow} mW):\t{plane_fit_RMSE(points) * 1000:.6f} mm")

