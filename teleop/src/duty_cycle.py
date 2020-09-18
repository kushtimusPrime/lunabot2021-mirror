#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String

class DutyCycle:

    #Constructor
    def __init__(self):

        # Initialize drive speed publishers
        self.publishers = {
            "pub_tl_duty_cycle": rospy.Publisher('tl_duty_cycle', Float64, queue_size=0),
            "pub_tr_duty_cycle": rospy.Publisher('tr_duty_cycle', Float64, queue_size=0),
            "pub_bl_duty_cycle": rospy.Publisher('bl_duty_cycle', Float64, queue_size=0),
            "pub_br_duty_cycle": rospy.Publisher('br_duty_cycle', Float64, queue_size=0),
        }

        self.left_speed = rospy.Subscriber("left_speed",Float64,self.left_callback)
        self.right_speed = rospy.Subscriber("right_speed",Float64,self.right_callback)
        

    def left_callback(self,data):

        #set duty cycle
        tl_duty_cycle = data.data*75
        bl_duty_cycle = data.data*75

        #publish duty cycles
        self.publishers["pub_tl_duty_cycle"].publish(tl_duty_cycle)
        self.publishers["pub_bl_duty_cycle"].publish(bl_duty_cycle)

    def right_callback(self,data):

        #set duty cycle
        tr_duty_cycle = data.data*75
        br_duty_cycle = data.data*75

        #publish duty cycles
        self.publishers["pub_tr_duty_cycle"].publish(tr_duty_cycle)
        self.publishers["pub_br_duty_cycle"].publish(br_duty_cycle)



if __name__ == '__main__':
    # Initialize as ROS node
    rospy.init_node('duty_cycle')

    # Create a TankSpeeds object
    control = DutyCycle()

    # Ready to go
    rospy.loginfo("Duty Cycle initialized...")

    # Loop continuously
    rospy.spin()