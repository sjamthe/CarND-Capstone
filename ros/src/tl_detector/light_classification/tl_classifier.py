from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        result = TrafficLight.UNKNOWN

        # Step 1: Converting to the correct color scheme
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Step 3: Clean image
        image_blur = cv2.GaussianBlur(image, (7,7), 0)
        image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)

        # Step 3: Defining filters
        # Filter by the color
        min_red1 = np.array([0, 100, 70])
        max_red1 = np.array([10, 256, 256])

        mask1 = cv2.inRange(image_blur_hsv, min_red1, max_red1)

        # Filter by brightness
        min_red2 = np.array([170, 100, 70])
        max_red2 = np.array([180, 256, 256])

        mask2 = cv2.inRange(image_blur_hsv, min_red2, max_red2)

        # Take these two mask and combine them
        mask = mask1 + mask2

        # Step 4: Segmentaion
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask_clean = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)

        _, contours, _ = cv2.findContours(mask_clean, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len (contours) > 0:
           result = TrafficLight.RED
        return result
