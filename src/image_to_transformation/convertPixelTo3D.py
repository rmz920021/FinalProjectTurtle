import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import matplotlib.image as mpimg
import yaml
import cv2 as cv
from get2DCoord import main


def loadCameraConfig(depth_img):
    with open("camera_intrinsics.yaml", "r") as file:
        loaded_intrinsics = yaml.safe_load(file)

    intrinsics = rs.intrinsics()
    intrinsics.width = depth_img.shape[1]
    intrinsics.height = depth_img.shape[0]
    intrinsics.fx = loaded_intrinsics["fx"]
    intrinsics.fy = loaded_intrinsics["fy"]
    intrinsics.ppx = loaded_intrinsics["ppx"]
    intrinsics.ppy = loaded_intrinsics["ppy"]
    intrinsics.model = rs.distortion.brown_conrady
    intrinsics.coeffs = loaded_intrinsics["coeffs"]

    return intrinsics


def readInstantImage(rgb_img_path, depth_img_path, display=False):
    input_file = depth_img_path
    npimg = np.fromfile(input_file, dtype=np.uint16)
    imageSize = (240, 320)

    depth_img = npimg.reshape(imageSize)
    rgb_img = mpimg.imread(rgb_img_path)

    print(f"RGB image shape: {rgb_img.shape}\nDepth image shape: {depth_img.shape}")
    if display:
        fig, axes = plt.subplots(1, 2)
        axes[0].imshow(depth_img)
        axes[1].imshow(rgb_img)
        axes[0].axis("off")
        axes[1].axis("off")
        plt.show()
    return rgb_img, depth_img


def map2DTo3DFromAssignedPoint(rgb_img, depth_img, pixel_coord, intrinsic_matrix):
    # Scale factor between RGB and depth resolutions
    scale_y = depth_img.shape[0] / rgb_img.shape[0]
    scale_x = depth_img.shape[1] / rgb_img.shape[1]
    print(pixel_coord)
    given_rgb_coord = pixel_coord  # (row, col)

    # Map to depth image coordinate
    depth_coord = (int(given_rgb_coord[0] * scale_y), int(given_rgb_coord[1] * scale_x))
    depth_value = depth_img[depth_coord[0], depth_coord[1]]  # Depth in millimeters

    # Convert 2D depth pixel to 3D point
    depth_pixel = [depth_coord[1], depth_coord[0]]  # [x, y] in (col, row)
    depth_in_meters = depth_value / 1000.0  # Convert mm to meters

    point_3d = rs.rs2_deproject_pixel_to_point(intrinsic_matrix, depth_pixel, depth_in_meters)

    print(f"3D coordinate: {point_3d}")


def map2DTo3D(rgb_img, depth_img, pixel_coords, intrinsic_matrix):
    # Scale factor between RGB and depth resolutions
    scale_y = depth_img.shape[0] / rgb_img.shape[0]
    scale_x = depth_img.shape[1] / rgb_img.shape[1]
    homogenous_coord = []
    cnt = 0
    for pixel_coord in pixel_coords:
        depth_coord = (int(pixel_coord[0] * scale_y), int(pixel_coord[1] * scale_x))
        depth_value = depth_img[depth_coord[1], depth_coord[0]]  # Depth in millimeters

        # Convert 2D depth pixel to 3D point
        depth_pixel = [depth_coord[0], depth_coord[1]]  # [x, y] in (col, row)
        depth_in_meters = depth_value / 1000.0  # Convert mm to meters

        point_3d = rs.rs2_deproject_pixel_to_point(intrinsic_matrix, depth_pixel, depth_in_meters)
        if all([ v == 0.0 or v == -0.0 for v in point_3d ]):
            # print(f"x, y, z coord are all 0.0")
            cnt += 1
            continue

        point_3d.extend([1])
        # print(f"3D coordinate: {point_3d}")
        homogenous_coord.append(point_3d)
    return homogenous_coord, cnt

    

# depth_img_path = '../Assets/dot-1_Depth.raw'
# rgb_img_path = '../Assets/dot-1_Color.png'

# rgb = cv.imread(rgb_img_path)
# clicked_coord = main(rgb_img_path)

# rgb_img, depth_img = readInstantImage(rgb_img_path, depth_img_path, display=False)
# intrinsic_matrix = loadCameraConfig(depth_img)

# try:
#     map2DTo3DFromAssignedPoint(rgb_img, depth_img, clicked_coord[0], intrinsic_matrix)
# except:
#     print("----------------------------")
#     print("You didnt clicked any points")
