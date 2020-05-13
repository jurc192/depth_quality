# Depth Quality testing

Use `depth_acquisition.py` to obtain wall depthmaps at different distances.  
Use `depth_analysis.py`    to measure plane-fit-RMSE



## Depth acquisition

Use `def capture_depthmap(width=1280, height=720, exposure=8500, laser_power=240, depth_preset=1)` to capture depthmaps using different settings.   

| Available resolutions (width, height) in pixels: |
| ------------------------------------------------ |
| 1280 x 720                                       |
| 848  x 480                                       |
| 640  x 480                                       |
| 640  x 360                                       |
| 480  x 270                                       |
| 424  x 240                                       |



|                        | Min  | Max    | Default |
| ---------------------- | ---- | ------ | ------- |
| **Exposure** (??*)     | 1    | 165000 | 8500    |
| **Laser power** (mW**) | 0    | 360    | 240     |

*documentation says *ms* but the implementation/numbers looks more like *us*
**steps of 30mW 



| Depth preset index | Depth preset name |
| ------------------ | ----------------- |
| 0                  | Custom            |
| 1                  | Default           |
| 2                  | Hand              |
| 3                  | High Accuracy     |
| 4                  | High Density      |
| 5                  | Medium Density    |





## Depth analysis

Use `plane_fit_RMSE(points, depth_unit=0.0001)` to calculate RMSE of a given `pointcloud` in meters. Depth unit is set to smallest possible value (0.0001 m), but can be changed if pointcloud was captured using different settings.

**Best-fit plane** is calculated using [Linear Regression](https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.LinearRegression.html#sklearn.linear_model.LinearRegression) from scikit learn [library](https://scikit-learn.org/stable/modules/linear_model.html#ordinary-least-squares). 

**Signed distances** to a plane are calculated using:

![](.readme_images/distancetoplane.png)

**Root-mean square errors** are calculated using:

![](.readme_images/rmse.png)

Utility function `parse_params` and examples in the script assume naming convention of `.ply` files:
`distance_resolution_exposure_laserpower.ply`

Example dataset can be found [here](https://drive.google.com/open?id=1PRAM1WC2O3LjU8KLo7OZq_YXtz5iKds1) (captures of a wall at different distances)

