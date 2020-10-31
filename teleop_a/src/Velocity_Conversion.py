#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist

class Velocity:
    def __init__(self):

       		# Initialize drive speed publishers
        	self.publishers = {
            	"pub_left_control": rospy.Publisher('left_control', Float64, queue_size=0),
           		"pub_right_control": rospy.Publisher('right_control', Float64, queue_size=0),
        	}

        	self.left_speed = rospy.Subscriber("cmd_vel",Twist,self.method)

    def method(self,data):
    
        # get data from twist
        linear = data.linear.x
        angular = data.angular.z

    # fixed separation distance from wheels
        r = float(1)

    # gets a and b numbers using r, linear, angular
        a = float(linear - r * angular)
        b = float(linear + r * angular)

    # gets linear velocity for each side of robot (a = left side) (b = right side)
        a_Percentage = float(a / 0.34)
        b_Percentage = float(b / 0.34)

    # scales each side to the maximum linear velocity
        if(abs(a_Percentage)>1 or abs(b_Percentage)>1):
            if(abs(a_Percentage) > abs(b_Percentage)):
                a = 100*(a_Percentage / abs(a_Percentage))
                #Do whatever
                b = 100*(b_Percentage / abs(a_Percentage))
            else:
                b = 100*(b_Percentage / abs(b_Percentage))
                a = 100*(a_Percentage / abs(b_Percentage))
        else:
            a=a_Percentage*100
            b=b_Percentage*100
    
        threshhold = 0.20

        if(a>0):
            a = 0.55*a+45
	elif(a == 0):
            a = 0
	else:
            a = 0.55*a-45
	if(b>0):
            b = 0.55*b+45
	elif(b == 0):
            b = 0
	else:
            b = 0.55*b-45

        self.publishers["pub_left_control"].publish(a)
        self.publishers["pub_right_control"].publish(b)
        return a, b



if __name__ == '__main__':
    # Initialize as ROS node
    rospy.init_node("TankConversion")

    # Create TankSpeeds object
    velocity = Velocity()

    # Ouputs ready to go
    rospy.loginfo("Velocity initialized....")

    # loops continuously 
    rospy.spin()
    

