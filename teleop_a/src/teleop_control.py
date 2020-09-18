#!/usr/bin/env python

# ----------------------------------------------- #
# Table of index number of /joy.buttons:
# Index   Button name on the actual controller
#
# 0   A (Lead Screw)
# 1   B (Buckets)
# 2   X (Hinge)
# 3   Y (Conveyor)
# 4   LB
# 5   RB
# 6   back (accel_decel)
# 7   start (direction)
# 8   power
# 9   Button stick left
# 10  Button stick right

# Table of index number of /joy.axes:
# Index   Axis name on the actual controller
#
# 0   Moving left joystick left (+) and right (-) changes rotational velocity
# 1   Moving left joystick up (+) and down (-) changes linear velocity
# 2   LT
# 3   Left/Right Axis stick right
# 4   Up/Down Axis stick right
# 5   RT
# 6   cross key left/right      (buckets / hinge accel)
# 7   cross key up/down         (hinge / lead screw accel)
# ----------------------------------------------- #

# import required packages
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


class TeleopControl:

    # Constructor
    def __init__(self):
        # Initialize drive speed publishers
        self.publishers = {
            "pub_drive_cmd": rospy.Publisher('drive_cmd', Twist, queue_size=0),
            "pub_dep_spool": rospy.Publisher('dep_spool', Float64, queue_size=0),
            "pub_dep_linacc": rospy.Publisher('dep_linacc', Float64, queue_size=0),
            "pub_exc_pose": rospy.Publisher('exc_pose', Pose2D, queue_size=0)
        }

        self.state = {
            #TODO : add states
            "acc_dir": 1.0,
            "acc_dir_toggle": 0,
            "acc_step": 0.05,
            "dep_spool_on": False,
            "dep_linacc_on": False,
            "dep_spool_out": 0.0,
            "dep_linacc_out": 0.0,
            "dep_spool_toggle": 0,
            "dep_linacc_toggle": 0,
            "dep_spool_start": 0.1,
            "dep_linacc_start": 0.1
        }

        # Initialize subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.process_joystick_data)

    # update the speed of a digging motor or linear actuator
    def update_dig_motor(self, id, toggle, acc_toggle):

        # Check if we should change on / off
        if self.state[id + "_toggle"] == 1 and toggle == 0:

            # Toggle on / off
            self.state[id + "_on"] = not self.state[id + "_on"]

            # If turning on, set speed to default, else set to 0
            self.state[id + "_out"] = self.state[id + "_start"] if self.state[id + "_on"] else 0.0

        # Check if we should accel / decel
        if self.state[id + "_on"]:
            if self.state[id + "_acc_toggle"] != 0 and not self.check_button(acc_toggle, id):

                # Apply increment
                new_vel = self.state[id + "_out"] + self.state["acc_dir"] * self.state["acc_step"]

                # Limit speed value
                self.state[id + "_out"] = min(1.0, max(-1.0, new_vel))

        # Update toggle values
        self.state[id + "_toggle"] = toggle
        self.state[id + "_acc_toggle"] = acc_toggle if self.check_button(acc_toggle, id) else 0.0

    # Checks correspondence between accel buttons and dig motor ids
    def check_button(self, accel, id):

        # Check if accel value is in corresponding group
        if id in ["cnv", "hng"] and accel == 1:
            return True
        if id in ["bkt", "lsc"] and accel == -1:
            return True

        # Doesn't match
        return False


    # --- CALLBACK FUNCTIONS --- #

    # Callback for joystick data
    def process_joystick_data(self, msg):
        # Get motor velocity commands
        left_input = msg.axes[JOYSTICK_AXIS_LEFT_Y]
        right_input = msg.axes[JOYSTICK_AXIS_RIGHT_Y]

        # Get direction and accel_decel toggle states
        acc_dir_toggle = msg.buttons[JOYSTICK_BUTTON_RB]

        # Get dig motor toggle button states
        dep_spool_toggle = msg.buttons[JOYSTICK_BUTTON_X]
        dep_linacc_toggle = msg.buttons[JOYSTICK_BUTTON_A]

        # Get dig motor accel button states
        # TODO: plsfix
        bkt_hng_acc_toggle = msg.axes[JOYSTICK_AXIS_DPAD_X]
        lsc_cnv_acc_toggle = msg.axes[JOYSTICK_AXIS_DPAD_Y]

        # TODO: get data for Pose2D for excavation from controller

        # Update accel_direction
        if self.state["acc_dir_toggle"] == 1 and acc_dir_toggle == 0:
            self.state["acc_dir"] *= -1.0

        # Update dir and acc toggle
        self.state["acc_dir_toggle"] = acc_dir_toggle

        # Update each dig motor speeds, toggles, accel_toggles
        #TODO: PLSFIX THIS
        self.update_dig_motor("dep_spool", dep_spool_toggle, bkt_hng_acc_toggle)
        self.update_dig_motor("dep_linacc", dep_linacc_toggle, lsc_cnv_acc_toggle)

        # --- PUBLISH COMMAND MESSAGES --- #

        # Drive twist
        drive_msg=Twist()
        drive_msg.linear.x=(left_input+right_input)/2.0
        drive_msg.angular.z=(left_input-right_input)/2.0
        self.publishers["pub_drive_cmd"].publish(drive_msg)

        # TODO: publish excavation data

        # Dig motor speeds
        for id in ["dep_spool", "dep_linacc"]:

            # Create message
            dig_msg = Float64()
            dig_msg.data = self.state[id + "_out"]

            # Publish
            self.publishers["pub_" + id].publish(dig_msg)

if __name__ == '__main__':
    # Initialize as ROS node
    rospy.init_node('teleop_control')

    # Create a TeleopControl object
    control = TeleopControl()

    # Ready to go
    rospy.loginfo("Teleop Control initialized...")

    # Loop continuously
    rospy.spin()
