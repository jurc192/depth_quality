import pyrealsense2 as rs
import open3d as o3d
import cv2
import numpy as np
from pathlib import Path
import sys
from sklearn import linear_model
from math import sqrt

def rmse(errors):
    return np.sum((errors**2)) / len(errors)

def dist_to_plane(x, y, z, a, b, c, d):
    return (a*x + b*y + c*z - d) / sqrt(a*a + b*b + c*c)


# Read pointcloud
if len(sys.argv) < 2:
    print("Usage: ./depth_analysis.py <input_file.ply>")
    sys.exit()
points = np.asanyarray(o3d.io.read_point_cloud(sys.argv[1]).points)

# Form the system of equations in matrix form: Aw = z
A = []
z = []
for p in points:
    A.append([p[0], p[1], 1])
    z.append(p[2])

# print(points)
# print("\n")
# print(np.matrix(A))
# print("\n")
# print(np.matrix(z).T)

# Perform linear regression
reg = linear_model.LinearRegression()
reg.fit(A, z)

# Get plane formula
a = reg.coef_[0]
b = reg.coef_[1]
c = 1
d = reg.intercept_

print(f"{a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")


# Calculate distances
distances = []
for p in points:
    distances.append(dist_to_plane(*p, a, b, c, d))

# print(min(distances))
# print(max(distances))
# print(f"RMSE: {rmse(np.array(distances)) * 100} um")
print(f"RMSE: {rmse(np.array(distances)) * 0.0001} m")

