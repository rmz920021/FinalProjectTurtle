import pyrealsense2 as rs
import yaml

def saveCameraConfig(depth_stream_width, depth_stream_height):
    # Create a pipeline and start it
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, depth_stream_width, depth_stream_height, rs.format.z16, 30)

    pipeline_profile = pipeline.start(config)

    # Get depth stream intrinsics
    depth_stream = pipeline_profile.get_stream(rs.stream.depth)  # rs.video_stream_profile
    intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

    # Create a dictionary to hold the intrinsic parameters
    intrinsics_dict = {
        "width": intrinsics.width,
        "height": intrinsics.height,
        "fx": intrinsics.fx,
        "fy": intrinsics.fy,
        "ppx": intrinsics.ppx,
        "ppy": intrinsics.ppy,
        "model": str(intrinsics.model),  # Convert enum to string
        "coeffs": intrinsics.coeffs
    }

    # Export to a YAML file
    with open("camera_intrinsics.yaml", "w") as file:
        yaml.dump(intrinsics_dict, file)

    print("Intrinsic parameters saved to 'camera_intrinsics.yaml'.")

    # Stop the pipeline
    pipeline.stop()
