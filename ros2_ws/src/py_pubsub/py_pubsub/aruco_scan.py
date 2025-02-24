import cv2
import numpy as np

def get_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def detect_first_aruco_marker(image_path, aruco_dict_type=cv2.aruco.DICT_4X4_50):
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()

    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if len(corners) == 0:
        print("No arucos detected")
        return

    img_width = gray.shape[1]
    img_height = gray.shape[0]
    corner = corners[0]
    img_corners = corner[0]
    top_left = img_corners[0]
    bottom_right = img_corners[2]
    distance_pxls = get_distance(top_left, bottom_right)
    marker_width_pct = (distance_pxls / img_width)

    center_x_pct, center_y_pct = get_marker_center_percentage(corner[0], img_width, img_height)
    print(f"Marker ID {ids[0][0]} Center: ({center_x_pct:.2f}, {center_y_pct:.2f})")
    print(f"Marker width as a percent of image: {marker_width_pct:.2f}")

    cv2.circle(image, (int(center_x_pct * img_width), int(center_y_pct * img_height)), 5, (0, 255, 0), -1)
    cv2.imshow("Aruco Detection", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

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

if __name__ == "__main__":
    detect_first_aruco_marker("aruco_real_life1.png")
