#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String

class DutyCycle:

    #Constructor
    def __init__(self):

        # Initialize drive speed publishers
        self.publishers = {
            "pub_left_control": rospy.Publisher('left_control', Float64, queue_size=0),
            "pub_right_control": rospy.Publisher('right_control', Float64, queue_size=0),
        }

        self.left_speed = rospy.Subscriber("left_speed",Float64,self.left_callback)
        self.right_speed = rospy.Subscriber("right_speed",Float64,self.right_callback)
        

    def left_callback(self,data):

        #set duty cycle
        tl_duty_cycle = data.data*100
        bl_duty_cycle = data.data*100

        #publish duty cycles
        self.publishers["pub_left_control"].publish(tl_duty_cycle)

    def right_callback(self,data):

        #set duty cycle
        tr_duty_cycle = data.data*100
        br_duty_cycle = data.data*100

        #publish duty cycles
        self.publishers["pub_right_control"].publish(tr_duty_cycle)



if __name__ == '__main__':
    # Initialize as ROS node
    rospy.init_node('duty_cycle')

    # Create a TankSpeeds object
    control = DutyCycle()

    # Ready to go
    rospy.loginfo("Duty Cycle initialized...")

    # Loop continuously
    rospy.spin()
