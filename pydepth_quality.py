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
import cv2
import sys

if len(sys.argv) < 2:
    print("Usage: ./pydepth_quality.py <pcl file>")
    sys.exit(1)

pcl = o3d.io.read_point_cloud(sys.argv[1])
if (pcl):
    o3d.visualization.draw_geometries([pcl], window_name='Point cloud visualization', width=800, height=600)
else:
    print("Bad input file")




