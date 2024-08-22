import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# can adjust it based on cabinet condition
TARGET_AREA = 10000

# adjust debug rectangle
COLOR = (0, 0, 255)
THICKNESS = 2

class CabinetEdgeDetector:
    def __init__(self, image_path = 'shelves2.jpeg'):
        rospy.init_node('cabinet_edge_detector')
        # Load the image
        self.image_path = image_path
        self.image = cv2.imread(image_path)
        self.count = 0

    def preprocess_image(self):
        """
        Preprocess the image to prepare it for contour detection.
        Steps include grayscale conversion, blurring, and thresholding.
        Returns:
            dilated (np.array): The processed binary image.
        """
        # Convert to grayscale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Apply Median Filtering
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Bilateral Filtering to smooth while preserving edges
        bilateral_filtered = cv2.bilateralFilter(blurred, 9, 75, 75)

        # Apply Adaptive Thresholding
        adaptive_thresh = cv2.adaptiveThreshold(bilateral_filtered, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                cv2.THRESH_BINARY, 13, 2)

        # Apply Erosion and Dilation to clean up the image
        kernel_erode = np.ones((5, 5), np.uint8)
        kernel_dilate = np.ones((3, 3), np.uint8)
        eroded = cv2.erode(adaptive_thresh, kernel_erode, iterations=2)
        dilated = cv2.dilate(eroded, kernel_dilate, iterations=1)

        cv2.imshow('dilated', dilated)

        return dilated

    def debug_contours(self, processed_image):
        """
        Find and draw bounding boxes around contours that exceed the TARGET_AREA.
        Args:
            processed_image (np.array): The processed binary image from preprocessing.
        """
        contours, _ = cv2.findContours(processed_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > TARGET_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                print(f'Layer {x}, {y}, {w}, {h}, {area}, center: ({center_x}, {center_y})')
                cv2.rectangle(self.image, (x, y), (x + w, y + h), COLOR, THICKNESS)
                cv2.circle(self.image, (center_x, center_y), 5, COLOR, -1)  # Blue dot for the center

        image_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        plt.figure(figsize=(10, 10))
        plt.imshow(image_rgb)
        plt.axis('off')
        plt.show()

    def run(self, processed_image):
        """
        Find bounding boxes around contours that exceed the TARGET_AREA.
        Return the center coordinates of bounding boxes.
        Args:
            processed_image (np.array): The processed binary image from preprocessing.
        Returns:
            bounding_boxes_center (list of tuples): The (x, y) coordinates of the centers.
        """
        bounding_boxes_center = []

        contours, _ = cv2.findContours(processed_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > TARGET_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # Store the center coordinates
                bounding_boxes_center.append((center_x, center_y))

        return bounding_boxes_center

if __name__ == "__main__":
    try:
        # Example usage:
        # image_path = 'shelves2.jpeg'
        processor = CabinetEdgeDetector(image_path='/home/mustar/catkin_ws/src/grocery/images/edge_detection.jpeg')
        processed_image = processor.preprocess_image()
        processor.debug_contours(processed_image)
        cabinet_list = processor.run(processed_image)
        print(cabinet_list)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Error with cabinet_edge_detector.py")


