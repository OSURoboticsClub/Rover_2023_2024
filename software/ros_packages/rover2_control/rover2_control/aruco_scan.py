import cv2
import numpy as np

def get_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def detect_first_aruco_marker(auton_class, image, aruco_dict_type=cv2.aruco.DICT_4X4_50):
    if image is None:
        if auton_class is not None:
            auton_class.get_logger().info("Image is None")
        return None, None

    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None or len(corners) == 0:
        print("No arucos detected")
        return None, None

    img_width = gray.shape[1]
    img_height = gray.shape[0]
    corner = corners[0]
    img_corners = corner[0]
    top_left = img_corners[0]
    bottom_right = img_corners[2]
    distance_pxls = get_distance(top_left, bottom_right)
    marker_width_pct = distance_pxls / img_width

    center_x_pct, center_y_pct = get_marker_center_percentage(corner[0], img_width, img_height)

    return center_x_pct, marker_width_pct


def get_marker_center_percentage(corner_points, img_width, img_height):
    """
    Calculates the center of an Aruco marker as a percentage of the image dimensions.

    Parameters:
    - corner_points (np.array): 4x2 array of marker corner coordinates.
    - img_width (int): Width of the image.
    - img_height (int): Height of the image.

    Returns:
    - (float, float): Center coordinates as a percentage (x%, y%).
    """
    center_x = np.mean(corner_points[:, 0])  # Mean of X coordinates
    center_y = np.mean(corner_points[:, 1])  # Mean of Y coordinates

    center_x_pct = (center_x / img_width)
    center_y_pct = (center_y / img_height)

    return center_x_pct, center_y_pct

def get_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

if __name__ == "__main__":
    image = cv2.imread("aruco_real_life1.png")
    detect_first_aruco_marker(None, image)
