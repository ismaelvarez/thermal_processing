import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge


def get_min_max_point(image, threshold=30000):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='mono16')
    cv_image = cv2.GaussianBlur(cv_image, (41, 41), 0)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(cv_image)
    return maxLoc, maxVal, minLoc, minVal


def get_max_temp_area(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='mono8')

    blur = cv2.GaussianBlur(cv_image, (5, 5), 0)
    _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    return mask


def get_min_temp_area(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='mono16')

    mask = cv2.inRange(cv_image, 0, 10000)
    return np.column_stack(np.where(mask == 0))


def get_temperature(value, max_temp, min_temp, max_image_value):
    normalized = value / max_image_value
    return (normalized * (max_temp - min_temp)) + min_temp
