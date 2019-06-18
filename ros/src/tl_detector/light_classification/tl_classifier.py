from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import rospy

# Term 1 !!!!!!
class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):

        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #rospy.logwarn("HERE")
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        RED_LOW = np.array([0, 120, 120])
        RED_HIGH = np.array([10, 255, 255])

        mask1 = cv2.inRange(hsv_img, RED_LOW, RED_HIGH)
        red = cv2.countNonZero(mask1)
        if red > 50:
            return TrafficLight.RED

        return TrafficLight.UNKNOWN
