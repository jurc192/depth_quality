#
#   depth_acquisition.py
#   Script that enables collecting depth images at different distances, with variable parameters
#

# 1. Preview and set exposure time
# 2. For every distance:
    # - position the camera
    # - capture images
    # - save to files


if __name__ == "__main__":
    
    distances   = []
    resolutions = []
    laserpowers = []
    exposures   = exposure_preview()

    for dist in distances:
        input("Place the camera to distance: {dist}cm")
        for res in resolutions:
            for exp in exposures:
                for lpow in laserpowers:
                    depthframe = capture_depthframe(*res, exposure, laserpower)
                    intrinsics = get_intrinsics(*res)
                    filename = f"{dist}_{res[0]}x{res[1]}_{exposure}_{laserpower}"
                    save_depth_raw(f"{filename}.raw", depthframe)
                    save_depth_colorized(f"{filename}.png", depthframe)
                    save_pointcloud(f"{filename}.ply", depthframe, intrinsics)
                    