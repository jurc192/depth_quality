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
import csv


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
        mask = np.zeros(depthmap.shape, dtype='uint16')
        height, width = mask.shape[:2]
        roih, roiw = int(roi/100 * height), int(roi/100 * width)
        x,y = ((width-roiw)//2 , (height-roih)//2)  # Top-left point of ROI
        mask[y:y+roih , x:x+roiw] = 1
        depthmap = mask * depthmap               # Apply mask to the depthmap

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


def distance_to_exposure(inputfolder, outputfile, distances, exposures):
    with open(f"{inputfolder}/{outputfile}", 'w') as f:
        writer = csv.writer(f)
        writer.writerow(["dist (cm) / exposure (us)"]+exposures)
        for dist in distances:
            results_mm = []
            print(f"Distance {dist} cm")
            for exp in exposures:
                filename     = f"{dist}_848x480_{exp}_150.raw"
                depthmap_raw = np.loadtxt(f"{inputfolder}/raw/{filename}", dtype='uint16')
                pointcloud   = depth_to_pointcloud(depthmap_raw)
                rmse = plane_fit_RMSE(np.array(pointcloud.points))
                results_mm.append(rmse*1000)
                print(f"Exposure {exp}us:\t{rmse*1000:.6f}mm", end='\n')
            print([(f"{res:.4f}") for res in results_mm])
            writer.writerow([dist]+[(f"{res:.4f}") for res in results_mm])


def distance_to_laserpower(inputfolder, outputfile, distances, laserpowers):
    with open(f"{inputfolder}/{outputfile}", 'w') as f:
        writer = csv.writer(f)
        writer.writerow(["dist (cm) / laserpower (mW)"]+laserpowers)
        data = []
        for dist in distances:
            results_mm = []
            print(f"Distance {dist} cm")
            for lpow in laserpowers:
                filename     = f"{dist}_848x480_5500_{lpow}.raw"
                depthmap_raw = np.loadtxt(f"{inputfolder}/raw/{filename}", dtype='uint16')
                pointcloud   = depth_to_pointcloud(depthmap_raw)
                rmse = plane_fit_RMSE(np.array(pointcloud.points))
                results_mm.append(rmse*1000)
                print(f"Lpower {lpow}mW:\t{rmse*1000:.6f}mm", end='\n')
            print([(f"{res:.4f}") for res in results_mm])
            data.append(results.results_mm)
            writer.writerow([dist]+[(f"{res:.4f}") for res in results_mm])
        print(data)


def distance_to_roi(inputfolder, outputfile, distances, rois):
    with open(f"{inputfolder}/{outputfile}", 'w') as f:
        writer = csv.writer(f)
        writer.writerow(["dist (cm) / roi (%)"]+rois)
        for dist in distances:
            results_mm = []
            print(f"Distance {dist} cm")
            for roi in rois:
                filename     = f"{dist}_848x480_5500_150.raw"
                depthmap_raw = np.loadtxt(f"{inputfolder}/raw/{filename}", dtype='uint16')
                pointcloud   = depth_to_pointcloud(depthmap_raw, roi)
                rmse = plane_fit_RMSE(np.array(pointcloud.points))
                results_mm.append(rmse*1000)
                print(f"ROI {roi}%:\t{rmse*1000:.6f}mm", end='\n')
            print([(f"{res:.4f}") for res in results_mm])
            writer.writerow([dist]+[(f"{res:.4f}") for res in results_mm])


def plot_heatmap(filename):

    ## PARSE CSV
    datamatrix = []
    xticks = []
    yticks = []

    with open(filename) as f:
        csv_reader = csv.reader(f, delimiter=',')
        xticks = next(csv_reader)[1:]                   # header
        for row in csv_reader:
            yticks.append(row[0])
            datamatrix.append(row[1:])

    # print(np.array(datamatrix, dtype=float))

    fig, ax = plt.subplots()
    im = ax.imshow(np.array(datamatrix, dtype=float))

    ax.set_xticks(np.arange(len(xticks)))
    ax.set_yticks(np.arange(len(yticks)))
    ax.set_xticklabels(xticks)
    ax.set_yticklabels(yticks)

    # Rotate the tick labels and set their alignment.
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
            rotation_mode="anchor")

    # Loop over data dimensions and create text annotations.
    for i in range(len(xticks)):
        for j in range(len(yticks)):
            text = ax.text(j, i, datamatrix[j][i],
                        ha="center", va="center", color="w")

    ax.set_title("Test plot")
    fig.tight_layout()
    plt.show()





if __name__ == "__main__":

    from collections import defaultdict
    import matplotlib.pyplot as plt
    import matplotlib

    if len(sys.argv) < 2:
        print("Usage: ./depth_analysis.py <input_folder>")
        sys.exit()

    inputfolder = sys.argv[1]
    distances, resolutions, exposures, laserpowers = parse_params(inputfolder)
    rois = [100, 95, 90, 85, 80, 75, 70]

    # distance_to_exposure(inputfolder, 'dist_exposure.csv', distances, exposures)

    plot_heatmap(sys.argv[1])

        # print()

        # datamat = []
        # for k, v in d.items():
        #     datamat.append(v)
        # datamat = np.array(datamat)
        # print(np.array(datamat))


        # # ## 
        # fig, ax = plt.subplots()
        # im = ax.imshow(datamat)

        # ax.set_xticks(np.arange(len(d.keys())))      # distances
        # ax.set_yticks(np.arange(len(header[1:])))   # exposures
        # ax.set_xticklabels(d.keys())
        # ax.set_yticklabels(header[1:])

        # # Rotate the tick labels and set their alignment.
        # plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
        #         rotation_mode="anchor")

        # # Loop over data dimensions and create text annotations.
        # # for i in range(len(vegetables)):
        # #     for j in range(len(farmers)):
        # #         text = ax.text(j, i, harvest[i, j],
        # #                     ha="center", va="center", color="w")

        # ax.set_title("Harvest of local farmers (in tons/year)")
        # fig.tight_layout()
        # plt.show()

