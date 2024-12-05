import cv2
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt

def capture_frame(pipeline):
    """Capture depth and color frames from RealSense."""
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        return None, None

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    return depth_image, color_image

def orb_feature_matching(image1, image2):
    """Perform ORB feature detection and matching."""
    orb = cv2.ORB_create(nfeatures=20)
    # Detect and compute features
    kp1, des1 = orb.detectAndCompute(image1, None)
    kp2, des2 = orb.detectAndCompute(image2, None)

    # Check if descriptors are valid
    if des1 is None or des2 is None:
        print("Error: No descriptors found in one or both images.")
        return [], [], []

    # Ensure descriptors are uint8
    if des1.dtype != np.uint8 or des2.dtype != np.uint8:
        des1 = des1.astype(np.uint8)
        des2 = des2.astype(np.uint8)

    # Match features using Brute Force Matcher
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)

    # Sort matches by distance (quality)
    matches = sorted(matches, key=lambda x: x.distance)
    return kp1, kp2, matches

def map_2d_to_3d(depth_frame, keypoints, intrinsics):
    """Map 2D keypoints to 3D coordinates using depth frame and intrinsics."""
    points_3d = []
    for kp in keypoints:
        x, y = int(kp.pt[0]), int(kp.pt[1])  # Keypoint pixel coordinates
        if x >= depth_frame.shape[1] or y >= depth_frame.shape[0]:
            continue  # Ignore keypoints outside depth frame bounds

        depth = depth_frame[y, x] * 0.001  # Convert depth to meters (scale factor 0.001)

        if depth > 0:  # Ignore invalid depth values
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
            points_3d.append(point_3d)
    return np.array(points_3d)

def main():
    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # Load scout TurtleBot's last image
    scout_image = cv2.imread('../Assets/set-1_Color.png', cv2.IMREAD_GRAYSCALE)
    if scout_image is None:
        print("Error: Could not load scout image. Check the file path.")
        return

    # Resize the scout image to match RealSense resolution
    scout_image = cv2.resize(scout_image, (640, 480))

    # Get camera intrinsics
    profile = pipeline.get_active_profile()
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

    try:
        while True:
            # Capture RealSense frames
            depth_frame, color_frame = capture_frame(pipeline)
            if depth_frame is None or color_frame is None:
                print("Error: Failed to capture frames from RealSense.")
                continue

            # Convert color frame to grayscale for ORB processing
            current_image = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)

            # Perform ORB feature matching
            kp1, kp2, matches = orb_feature_matching(scout_image, current_image)

            if not matches:
                print("No matches found. Skipping frame.")
                continue

            # Extract matched keypoints in the current frame
            matched_keypoints = [kp2[m.trainIdx] for m in matches]

            # Map matched keypoints to 3D coordinates
            points_3d = map_2d_to_3d(depth_frame, matched_keypoints, intrinsics)
            print("Matched 3D Points:", points_3d)

            # Visualize matches
            matched_image = cv2.drawMatches(scout_image, kp1, current_image, kp2, matches[:20], None)
            cv2.imshow('Feature Matching and 3D Mapping', matched_image)

            # Exit on 'q' key press
            if cv2.waitKey(0) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
