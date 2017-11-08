from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def red_mask(self, image_blur_hsv):

        min_red1 = np.array([0, 100, 70])
        max_red1 = np.array([10, 256, 256])
        mask1 = cv2.inRange(image_blur_hsv, min_red1, max_red1)

        # Filter by brightness
        min_red2 = np.array([170, 100, 70])
        max_red2 = np.array([180, 256, 256])

        mask2 = cv2.inRange(image_blur_hsv, min_red2, max_red2)

        # Take these two mask and combine them
        mask = mask1 + mask2
        return mask

    def yellow_mask(self, image_blur_hsv):
        
        min_yellow = np.array([20, 128, 128])
        max_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(image_blur_hsv, min_yellow, max_yellow)
    
        return mask

    def detect_color(self, image, color):
      
        # Step 1: Converting to the correct color scheme
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Step 3: Clean image
        image_blur = cv2.GaussianBlur(image, (7,7), 0)
        image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)

        # Step 3: Defining color filters
        if(color == TrafficLight.RED):
            mask = self.red_mask(image_blur_hsv)
        elif(color == TrafficLight.YELLOW):
            mask = self.yellow_mask(image_blur_hsv)
        else:
            return TrafficLight.UNKNOWN

        # Step 4: Segmentaion
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
        mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask_clean = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)

        _, contours, _ = cv2.findContours(mask_clean, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            return color
        else:
            return TrafficLight.UNKNOWN

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #cv2.imshow("classify",image)
        #cv2.waitKey(5)
        #TODO implement light color prediction
        
        #detect for all red, yellow
        result = self.detect_color(image, TrafficLight.RED)
        if result == TrafficLight.RED:
            return result

        result = self.detect_color(image, TrafficLight.YELLOW)
        if result == TrafficLight.YELLOW:
            return result
        
        return TrafficLight.UNKNOWN
