import rospy
from sensor_msgs.msg import Image
from thermal_processing import features
from thermal_processing import utils
from gara_messages.msg import IRCameraTemperatureInfo
from optris_drivers.msg import TemperatureScale

MONO16_MAX_VALUE = 65535

IMAGE_PUBLISHER = None
TEMPERATURE_PUBLISHER = None

AREA_MODE ="area"
POINT_MODE = "point"

TEMPERATURE_CALCULATION_MODE = "area"


def callback(data):
    scale = rospy.wait_for_message('thermal_image_temperature_scale', TemperatureScale)

    if TEMPERATURE_CALCULATION_MODE == AREA_MODE:
        position_max, max_value, position_min, min_value = \
            features.calculate_temperature_by_areas(data, rospy.get_param('~max_threshold'),
                                                    rospy.get_param('~min_threshold'))

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
    rospy.Subscriber('image_raw', Image, callback)


def init():
    global TEMPERATURE_CALCULATION_MODE
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('thermal-processing', anonymous=True)

    TEMPERATURE_CALCULATION_MODE = rospy.get_param('~mode')

    init_publishers()
    init_subscribers()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

