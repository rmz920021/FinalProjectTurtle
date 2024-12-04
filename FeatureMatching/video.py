import cv2
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

def capture_frame():
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        return None, None

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    return depth_image, color_image

# ORB Feature Matching Function
def orb_feature_matching(image1, image2):
    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(image1, None)
    kp2, des2 = orb.detectAndCompute(image2, None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    return kp1, kp2, matches

# 3D Localization from Point Cloud
def localize_keypoints(depth_frame, kp, intrinsics):
    points_3d = []
    for point in kp:
        # Convert 2D keypoints to 3D
        x, y = int(point.pt[0]), int(point.pt[1])
        depth = depth_frame[y, x] * 0.001  # Convert depth to meters
        if depth > 0:
            # Deproject pixel to 3D using intrinsics
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
            points_3d.append(point_3d)
    return np.array(points_3d)

# Main Function
try:
    # Load scout TurtleBot's last image
    scout_image = cv2.imread('../Assets/feature-1_Color.png', cv2.IMREAD_GRAYSCALE)

    # Intrinsic parameters of the RealSense camera
    profile = pipeline.get_active_profile()
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    while True:
        depth_frame, color_frame = capture_frame()
        if depth_frame is None or color_frame is None:
            continue

        # Convert to grayscale for ORB
        current_image = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)

        # Perform ORB feature matching
        kp1, kp2, matches = orb_feature_matching(scout_image, current_image)

        # Visualize matches
        matched_image = cv2.drawMatches(scout_image, kp1, current_image, kp2, matches[:10], None)
        cv2.imshow('Feature Matching', matched_image)

        # Get 3D points of matched keypoints
        points_3d = localize_keypoints(depth_frame, [kp2[m.trainIdx] for m in matches], intrinsics)
        
        # (Optional) Use 3D points to navigate
        print("3D Points:", points_3d)

        # Break the loop with a key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()