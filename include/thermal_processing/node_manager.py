import rospy
from sensor_msgs.msg import Image
from thermal_processing import features
from thermal_processing import utils
from gara_messages.msg import IRCameraTemperatureInfo, TemperatureScale

MONO16_MAX_VALUE = 65535

IMAGE_PUBLISHER = None
TEMPERATURE_PUBLISHER = None

AREA_MODE = "area"
POINT_MODE = "point"

TEMPERATURE_CALCULATION_MODE = "area"

MODE = ""
MAX_THRESHOLD = 0
MIN_THRESHOLD = 0


def callback(data):
    scale = rospy.wait_for_message(rospy.get_param('~temperatureScaleTopic'), TemperatureScale)

    if TEMPERATURE_CALCULATION_MODE == AREA_MODE:
        position_max, max_value, position_min, min_value = \
            features.calculate_temperature_by_areas(data, MAX_THRESHOLD, MIN_THRESHOLD)

    if TEMPERATURE_CALCULATION_MODE == POINT_MODE:
        position_max, max_value, position_min, min_value = features.get_min_max_point(data)
        
    if IMAGE_PUBLISHER:
        IMAGE_PUBLISHER.publish(utils.draw_min_max_temp(data, position_max, position_min))
        # IMAGE_PUBLISHER.publish(utils.draw(data, features.get_max_temp_area(data)))

    if TEMPERATURE_PUBLISHER:
        # rospy.loginfo(rospy.get_caller_id() + 'maxLoc: %s; temp: %s ', position_max,
        #               features.get_temperature(max_value, TEMPERATURE.max, TEMPERATURE.min, MONO16_MAX_VALUE), )
        ir_info = IRCameraTemperatureInfo()
        ir_info.temperatureMax = features.get_temperature(max_value, scale.max, scale.min, MONO16_MAX_VALUE)
        ir_info.temperatureMin = features.get_temperature(min_value, scale.max, scale.min, MONO16_MAX_VALUE)
        ir_info.positionMax = position_max
        ir_info.positionMin = position_min
        TEMPERATURE_PUBLISHER.publish(ir_info)


def init_publishers():
    global IMAGE_PUBLISHER, TEMPERATURE_PUBLISHER
    IMAGE_PUBLISHER = rospy.Publisher('image_view', Image, queue_size=10)
    TEMPERATURE_PUBLISHER = rospy.Publisher('temperature_info', IRCameraTemperatureInfo,
                                            queue_size=10)


def init_subscribers():
    rospy.loginfo(rospy.get_param('~thermalImageTopic'))
    rospy.Subscriber(rospy.get_param('~thermalImageTopic'), Image, callback)


def set_params():
    global TEMPERATURE_CALCULATION_MODE
    global MAX_THRESHOLD
    global MIN_THRESHOLD

    try:
        TEMPERATURE_CALCULATION_MODE = rospy.get_param('~mode')
    except Exception:
        rospy.loginfo(rospy.get_caller_id() + ': Exception')
        TEMPERATURE_CALCULATION_MODE = "point"

    rospy.loginfo(rospy.get_caller_id() + ': Mode=%s' % TEMPERATURE_CALCULATION_MODE)
    try:
        MAX_THRESHOLD = rospy.get_param('~max_threshold')
    except Exception:
        MAX_THRESHOLD = 200

    rospy.loginfo(rospy.get_caller_id() + ': Max Threshold=%s' % MAX_THRESHOLD)

    try:
        MIN_THRESHOLD = rospy.get_param('~min_threshold')
    except Exception:
        MIN_THRESHOLD = 40

    rospy.loginfo(rospy.get_caller_id() + ': Min Threshold=%s' % MIN_THRESHOLD)


def init():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('thermal_processing', anonymous=True)

    set_params()

    init_publishers()
    init_subscribers()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

