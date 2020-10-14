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

        	self.left_speed = rospy.Subscriber("cmd_vel",Twist,self.speeds_from_twist)

    def speeds_from_twist(self,data):
        linear=data.linear.x
        angular = data.angular.z

        theta=angular*250
        r=linear*(100/0.340)

        v_a=r*(45-theta%90)/45
        v_b=min(100,abs(2*r+v_a),abs(2*r-v_a))

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

		# normalize angular speed value to [-180, 180)
    	# normalize linear speed value to [0, 100]
        #theta = angular*250       		
    	#r = linear*(100/0.340)        				 

    	# falloff of main motor
    	#v_a = r * (45 - theta % 90) / 45
    	# compensation of other motor
    	#v_b = min(100,abs( 2 * r + v_a), abs(2 * r - v_a)) 


   		 #direction cases
    	#if theta < -90: return -v_b, -v_a
    	#if theta < 0:   return -v_a, v_b
    	#if theta < 90:  return v_b, v_a
    		
		#self.publishers["pub_left_control"].publish(v_a)
        #self.publishers["pub_right_control"].publish(v_b)
    

   		
if __name__ == '__main__':
    # Initialize as ROS node
    rospy.init_node('velocity')

    # Create a TankSpeeds object
    velocity = Velocity()

    # Ready to go
    rospy.loginfo("Velocity initialized...")

    # Loop continuously
    rospy.spin()



        #linear = data.linear.x
		