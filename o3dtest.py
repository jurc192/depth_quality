import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("experiment1/ply/30_1280x720_8500_240.ply")

print(
    "Find the plane model and the inliers of the largest planar segment in the point cloud."
)
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=250)

[a, b, c, d] = plane_model
print(f"Plane model: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcd.select_down_sample(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])

outlier_cloud = pcd.select_down_sample(inliers, invert=True)

o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])