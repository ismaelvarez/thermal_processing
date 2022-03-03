import cv2
from cv_bridge import CvBridge


def get_hottest_point(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='mono16')
    gray = cv2.GaussianBlur(cv_image, (41, 41), 0)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)

    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

    cv2.circle(cv_image, maxLoc, 5, (0, 0, 255), 2)
    #rospy.loginfo(rospy.get_caller_id() + 'maxLoc: %s; maxVal: %s; temp: %s', maxLoc,
    #              maxVal, get_temperature(maxVal, 100, -20, 65535),)
    ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    return ros_image, maxLoc, maxVal


def get_temperature(value, max_temp, min_temp, max_image_value):
    normalized = value / max_image_value
    return (normalized * (max_temp - min_temp)) + min_temp
