import cv2
import numpy as np

def generate_aruco_marker(marker_id, marker_size=200, border_size=20, aruco_dict_type=cv2.aruco.DICT_4X4_50, save_path="aruco_marker.png"):
    """
    Generates an Aruco marker with a white border and saves it.

    Parameters:
    - marker_id (int): ID of the Aruco marker.
    - marker_size (int): Size of the marker in pixels (without the border).
    - border_size (int): Size of the white border in pixels.
    - aruco_dict_type (cv2.aruco): Type of Aruco dictionary.
    - save_path (str): File path to save the marker image.

    Returns:
    - marker_with_border (np.array): Generated marker image with border.
    """
    # Load the specified dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)

    # Create an empty black image for the marker
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)

    # Generate the marker
    cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_image)

    # Create a new image with a white border
    total_size = marker_size + 2 * border_size  # Total image size including border
    marker_with_border = 255 * np.ones((total_size, total_size), dtype=np.uint8)  # White background

    # Place the marker at the center
    marker_with_border[border_size:border_size + marker_size, border_size:border_size + marker_size] = marker_image

    # Save the marker with the border
    cv2.imwrite(save_path, marker_with_border)
    print(f"Aruco marker (ID: {marker_id}) saved as {save_path} with a {border_size}px white border.")

    return marker_with_border


if __name__ == "__main__":
    # Parameters for the Aruco marker
    marker_id = 10  # Choose an ID between 0 and the max allowed by the dictionary
    marker_size = 300  # Size of the marker (excluding the border)
    border_size = 50  # Border size in pixels
    save_path = "aruco2.png"

    # Generate and save the marker with a white border
    generate_aruco_marker(marker_id, marker_size, border_size, cv2.aruco.DICT_4X4_50, save_path)

    # Display the generated marker
    marker_image = cv2.imread(save_path)
    cv2.imshow("Generated Aruco Marker with Border", marker_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
