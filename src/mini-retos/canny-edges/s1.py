import sys

import cv2

"""
CODE 1: IMAGE PROCESSING WITH SLIDERS FOR THRESHOLD

This program takes the image name as an argument (e.g., "dog.jpg"), reads it,
applies a Gaussian blur, and then the Canny edge detector. Additionally, it creates two
sliders to modify the Canny thresholds (threshold1 and threshold2) in real-time. 
The program ends when the 'x' key is pressed.
"""


def process_image():
    if len(sys.argv) < 2:
        print("Usage: python3 code.py image_with_extension")
        sys.exit(1)

    # Get the image name from the arguments
    image_name = sys.argv[1]

    # Read the original image
    img = cv2.imread(image_name)

    # Check if the image loaded correctly
    if img is None:
        print(f"Could not load image: {image_name}")
        sys.exit(1)

    # Window to display the image with edges
    cv2.namedWindow("Image with Canny")

    # Empty callback function for the trackbars
    def nothing(x):
        pass

    # Create two trackbars for threshold1 and threshold2
    cv2.createTrackbar("Threshold1", "Image with Canny", 100, 500, nothing)
    cv2.createTrackbar("Threshold2", "Image with Canny", 200, 500, nothing)

    while True:
        # Convert the original image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(gray, (3, 3), 0)

        # Get the trackbar values in real-time
        t1 = cv2.getTrackbarPos("Threshold1", "Image with Canny")
        t2 = cv2.getTrackbarPos("Threshold2", "Image with Canny")

        # Apply the Canny edge detector with the current thresholds
        edges = cv2.Canny(blur, t1, t2)

        # Display the resulting image
        cv2.imshow("Image with Canny", edges)

        # Wait 1 ms between updates
        key = cv2.waitKey(1) & 0xFF
        # If 'x' is pressed, exit
        if key == ord("x"):
            break

    cv2.destroyAllWindows()


process_image()
