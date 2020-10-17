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

    
def speeds_from_twist(self,data):
        linear = data.linear.x
        angular = data.angular.z
        r = float(0.1425)


        theta = angular*250
        r = linear*(100/0.340)

        v_a = r*(45-theta%90)/45
        v_b = min(100,abs(2*r+v_a),abs(2*r-v_a))

        if(theta<-90):
            v_b=-v_b
            v_a=-v_a
        
        if(theta<0):
            v_a=-v_a

    self.publishers["pub_left_control"].publish(v_a)
    self.publishers["pub_right_control"].publish(v_b)

    if theta<-90: return -v_b,-v_a
    if theta<0: return -v_a,v_b
    if theta<90: return v_b,v_a



def method(self,data):
    
    # get data from twist
    linear = data.linear.x
    angular = data.angular.z

    # fixed separation distance from wheels
    r = float(0.1425)

    # gets a and b numbers using r, linear, angular
    a = float(linear - r * angular)
    b = float(linear + r * angular)

    # gets linear velocity for each side of robot (a = left side) (b = right side)
    a_Percentage = float(a / linear)
    b_Percentage = float(b / linear)

    # scales each side to the maximum linear velocity
    if(a_Percentage > b_Percentage):
        a = 1
        b = b_Percentage / a_Percentage
    else:
        b = 1
        a = a_Percentage / b_Percentage
    
    
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
    

