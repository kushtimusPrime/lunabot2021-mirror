#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String


JOYSTICK_BUTTON_A = 0
JOYSTICK_BUTTON_B = 1
JOYSTICK_BUTTON_X = 2
JOYSTICK_BUTTON_Y = 3
JOYSTICK_BUTTON_LB = 4
JOYSTICK_BUTTON_RB = 5
JOYSTICK_BUTTON_OPTION = 6
JOYSTICK_BUTTON_START = 7
JOYSTICK_BUTTON_POWER = 8
JOYSTICK_BUTTON_LEFT_STICK = 9
JOYSTICK_BUTTON_RIGHT_STICK = 10

JOYSTICK_AXIS_LEFT_X = 0
JOYSTICK_AXIS_LEFT_Y = 1
JOYSTICK_AXIS_LT = 2
JOYSTICK_AXIS_RIGHT_X = 3
JOYSTICK_AXIS_RIGHT_Y = 4
JOYSTICK_AXIS_RT = 5
JOYSTICK_AXIS_DPAD_X = 6
JOYSTICK_AXIS_DPAD_Y = 7

class TankSpeeds:

     # Constructor
    def __init__(self):
        # Initialize drive speed publishers
        self.publishers = {
            "pub_left_speed": rospy.Publisher('left_speed', Float64, queue_size=0),
            "pub_right_speed": rospy.Publisher('right_speed', Float64, queue_size=0),
        }

        # Initialize subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.process_joystick_data)

    def process_joystick_data(self,msg):

        #set speeds
        left_speed_float=msg.axes[JOYSTICK_AXIS_LEFT_Y]
        right_speed_float=msg.axes[JOYSTICK_AXIS_RIGHT_Y]

        #publish speeds
        self.publishers["pub_left_speed"].publish(left_speed_float)
        self.publishers["pub_right_speed"].publish(right_speed_float)

if __name__ == '__main__':
    # Initialize as ROS node
    rospy.init_node('tank_speeds')

    # Create a TankSpeeds object
    control = TankSpeeds()

    # Ready to go
    rospy.loginfo("Tank Speeds initialized...")

    # Loop continuously
    rospy.spin()
