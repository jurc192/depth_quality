import pyrealsense2 as rs
import open3d as o3d
import cv2
import numpy as np
from pathlib import Path
import os
import sys
from sklearn import linear_model
from math import sqrt


def rms(errors):
    return np.sum((errors**2)) / len(errors)


def dist_to_plane(x, y, z, a, b, c, d):
    return (a*x + b*y + c*z - d) / sqrt(a*a + b*b + c*c)


def plane_fit_RMSE(points, depth_unit=0.0001):
    
    # Form the system of equations in matrix form: Aw = z
    A = []
    z = []
    for p in points:
        A.append([p[0], p[1], 1])
        z.append(p[2])
    
    # Perform linear regression
    reg = linear_model.LinearRegression()
    reg.fit(A, z)
    a = reg.coef_[0]
    b = reg.coef_[1]
    c = 1
    d = reg.intercept_
    # print(f"Best fitting plane: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    
    distances = []
    for p in points:
        distances.append(dist_to_plane(*p, a, b, c, d))

    return rms(np.array(distances)) * depth_unit


def parse_params(folder):
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

    resolutions = ['480x270', '640x360', '848x480', '1280x720']
    # res = resolutions[0]
    exp = exposures[0]
    lpow = laserpowers[0]
    
    for res in resolutions:
        print(f"\nresolution {res}\texposure {exp}\tlpow {lpow}")
        for dist in distances:
            filename = f"{sys.argv[1]}/{dist}_{res}_{exp}_{lpow}.ply"
            points = np.asanyarray(o3d.io.read_point_cloud(filename).points)
            print(f"RMSE ({dist}):\t{plane_fit_RMSE(points) * 1000:.6f} mm")

    # points = np.asanyarray(o3d.io.read_point_cloud(sys.argv[1]).points)
    # print(f"RMSE: {plane_fit_RMSE(points)} m")

