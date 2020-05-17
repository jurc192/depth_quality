import sys
from pathlib import Path
import numpy as np
import pyrealsense2 as rs
from time import sleep
from multiprocessing import Process
import subprocess

def save_depth_raw(filename, depthmap):
    depthmap = np.asanyarray(depthmap.get_data())
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(filename, depthmap, fmt="%u")

def capture_depthmaps_csv(width=848, height=480, exposure=8500, laser_power=150, depth_preset=1, nframes=1, name='depth'):
    
    pipeline = rs.pipeline()
    config   = rs.config()

    # Enable depth stream with given resolution
    config.enable_stream(rs.stream.depth, width, height)
    if (config.can_resolve(pipeline) == False):
        print("Resolution not supported")
        raise Exception

    # Set parameters
    sensor = config.resolve(pipeline).get_device().first_depth_sensor()
    sensor.set_option(rs.option.depth_units, 0.0001)     # Smallest depth unit (in meters)
    sensor.set_option(rs.option.enable_auto_exposure, False)
    sensor.set_option(rs.option.exposure, exposure)
    sensor.set_option(rs.option.laser_power, laser_power)
    sensor.set_option(rs.option.visual_preset, depth_preset)

    # Get depth frame
    pipeline.start(config)
    for i in range(nframes):
        filename = f"{name}_{i}.raw"
        depthmap = pipeline.wait_for_frames().get_depth_frame()
        save_depth_raw(filename, depthmap)
    config.disable_all_streams()
    pipeline.stop()


def curse_the_camera():
    for i in range(3):
        subprocess.run(["espeak", "Fucking camera died again"], capture_output=True)


if __name__ == "__main__":

    from depth_acquisition import exposure_preview

    if len(sys.argv) < 2:
        print("Usage: ./depthgrabber <distance>")
        sys.exit()

    distance = sys.argv[1]
    width, height = (848, 480)
    nframes   = 50

    exposure_preview()

    # BASELINE SETTINGS
    exp = 8500
    lpow = 150
    destination = 'experiment9_base'
    while True:
        filename = f"{destination}/{distance}_{width}x{height}_{exp}_{lpow}_1"
        print(f"Capturing base {filename}")
        args = {
            'width': width, 'height': height, 
            'exposure': exp, 
            'laser_power': lpow, 
            'depth_preset': 1, 
            'nframes': nframes, 
            'name': filename
            }
        subproc =  Process(target=capture_depthmaps_csv, kwargs=args)
        subproc.start()
        subproc.join(10)
        if subproc.exitcode is not 0:
            print(f"Capturing failed w/ exitcode {subproc.exitcode}")
            subproc.terminate()
            curse_the_camera()
            input("Reconnect the camera and press ENTER")
        else:
            break


    # BEST SETTINGS
    exp = 6500
    lpow = 240
    destination = 'experiment9_best'
    while True:
        filename = f"{destination}/{distance}_{width}x{height}_{exp}_{lpow}_1"
        print(f"Capturing best {filename}")
        args = {
            'width': width, 'height': height, 
            'exposure': exp, 
            'laser_power': lpow, 
            'depth_preset': 1, 
            'nframes': nframes, 
            'name': filename
            }
        subproc =  Process(target=capture_depthmaps_csv, kwargs=args)
        subproc.start()
        subproc.join(10)
        if subproc.exitcode is not 0:
            print(f"Capturing failed w/ exitcode {subproc.exitcode}")
            subproc.terminate()
            curse_the_camera()
            input("Reconnect the camera and press ENTER")
        else:
            break
