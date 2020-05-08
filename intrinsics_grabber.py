import pyrealsense2 as rs


def get_intrinsics(width=1280, height=720):
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.depth, width, height)
    pipeline.start(config)
    intrinsics = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    pipeline.stop()
    return intrinsics

if __name__ == "__main__":
    
    resolutions = [(1280, 720), (848, 480), (640, 480), (640, 360), (480, 270)]
    for res in resolutions:
        intr = get_intrinsics(*res)
        print(intr)
