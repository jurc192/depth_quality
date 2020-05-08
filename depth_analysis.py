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


    def centered_roi(image, percentage):
        """ Return a cropped image, with percentage of its width """

        height, width = image.shape[:2]
        roiw = int(percents/100 * width)
        roih = int(percents/100 * height)

        x,y = ((width-roiw)//2 , (height-roih)//2)
        return image[y:y+roih , x:roiw]



if __name__ == "__main__":

    import cv2

    # Parse raw file
    rawdata = np.loadtxt('experiment1/raw/90_1280x720_8500_150.raw', dtype='uint16')
    colorized = cv2.applyColorMap(cv2.convertScaleAbs(rawdata, alpha=0.03), cv2.COLORMAP_JET)
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', colorized)
    cv2.waitKey(0)
    roiimage = centered_roi(colorized, 52)
    print(roiimage)
    cv2.imshow('RealSense', roiimage)
    cv2.waitKey(0)


    # # Calculate width and height of the ROI
    # percents = 50
    # height, width = rawdata.shape
    # roiw     = int(percents/100 * width)
    # roih     = int(percents/100 * height)
    
    # tl = ((width-roiw)//2 , (height-roih)//2)
    # tb = ((width+roiw)//2 , (height+roih)//2)

    # cropped = rawdata[tl:tl+roih, 200:200+500]
    # colorized = cv2.rectangle(colorized, tl, tb, (255, 255, 255), 3)

    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', colorized)
    # cv2.waitKey(0)



