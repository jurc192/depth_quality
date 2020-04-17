#
#   Custom depth quality measurement tool
#

# What do we want?
# - live preview
# - button press
# - taking a depth image frame
# - calculating best fit plane
# - calculating RMSE
# - displaying results

# Use Open3D library for vizualization, start with reading .ply from file

import pyrealsense2 as rs
import cv2

