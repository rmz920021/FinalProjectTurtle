import cv2
import numpy as np


def mouse_callback(event, x, y, flags, param):
    """
    Mouse callback function to display coordinates on the image.
    """
    image, window_name = param
    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button click
        print(f"Clicked at: ({x}, {y})")  # Print the coordinates

        # Draw a circle on the image where clicked
        cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

        # Display the coordinates on the image
        updated_image = image.copy()
        cv2.putText(updated_image, f"({x}, {y})", (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Update the window
        cv2.imshow(window_name, updated_image)


def setup_image_window(image, window_name="Image"):
    """
    Sets up the OpenCV window and binds the mouse callback function.
    """
    # Display the image in a window
    cv2.imshow(window_name, image)

    # Set the mouse callback to capture clicks
    cv2.setMouseCallback(window_name, mouse_callback, [image, window_name])

    print(f"Click on the image to see the 2D coordinates in '{window_name}'. Press 'q' to exit.")


def main(image_path):
    """
    Main function to display the image and allow user interaction.
    """
    # Example image (replace with your actual image)
    image = cv2.imread(image_path)

    window_name = "Image"
    setup_image_window(image, window_name)

    # Wait for 'q' key to exit
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()