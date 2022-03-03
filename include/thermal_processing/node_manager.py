import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32
from thermal_processing import features
from thermal_processing.msg import TempInfo

IMAGE_PUBLISHER = None
DATA_PUBLISHER = None

TEMPERATURE = None


def set_temp(data):
    global TEMPERATURE
    TEMPERATURE = data


def callback(data):
    image, position, max_temperature = features.get_hottest_point(data)

    if IMAGE_PUBLISHER:
        IMAGE_PUBLISHER.publish(image)

    if DATA_PUBLISHER and TEMPERATURE:
        # rospy.loginfo(rospy.get_caller_id() + 'maxLoc: %s; temp: %s ', position,
        #               features.get_temperature(max_temperature, TEMPERATURE.maxT, TEMPERATURE.minT, 65535), )
        DATA_PUBLISHER.publish(features.get_temperature(max_temperature, TEMPERATURE.maxT, TEMPERATURE.minT, 65535))


def init_publishers():
    global IMAGE_PUBLISHER, DATA_PUBLISHER
    IMAGE_PUBLISHER = rospy.Publisher('optris/thermal_camera_view_max', Image, queue_size=10)
    DATA_PUBLISHER = rospy.Publisher('optris/thermal_info', Float32, queue_size=10)


def init_subscribers():
    rospy.Subscriber('optris/image_raw', Image, callback)
    rospy.Subscriber('optris/test', TempInfo, set_temp)


def init():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('thermal-processing', anonymous=True)

    init_publishers()
    init_subscribers()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

