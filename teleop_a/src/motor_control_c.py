#! /usr/bin/env python
#include <ros/ros.h>
#include <geometry_msg/Twist.h>


int x_linear = 0; ##fake forward and backward velocity MAX 0.34
int z_angular = 0; ##fake angular veloctiy about z axis MAX 0.72

class Velocity: 

	def __init__(self):

       		# Initialize drive speed publishers
        	self.publishers = {
            	"pub_left_control": rospy.Publisher('left_control', Float64, queue_size=0),
           		"pub_right_control": rospy.Publisher('right_control', Float64, queue_size=0),
        	}

        	self.left_speed = rospy.Subscriber("cmd_vel",Twist,self.speeds_from_twist)
        		
	##CREATE A CALLBACK TO TAKE TWIST DATA FROM CMD_VEL TO GET X, AND ROT Z FROM TWIST 

	def speeds_from_twist(data):
		linear = data.linear.x
		angular = data.angular.z
		return self.throttle_angle_to_thrust(linear,angular)


	def throttle_angle_to_thrust(linear_v, angular_v):
   		# normalize angular speed value to [-180, 180)
    		# normalize linear speed value to [0, 100]
    		theta = angular_v*250       		
    		r = linear_v*(100/0.340)        				 

    		# falloff of main motor
    		v_a = r * (45 - theta % 90) / 45
    		# compensation of other motor
    		v_b = min(100,abs( 2 * r + v_a), abs(2 * r - v_a)) 


   		 #direction cases
    		if theta < -90: return -v_b, -v_a
    		if theta < 0:   return -v_a, v_b
    		if theta < 90:  return v_b, v_a
    		
			self.publishers["pub_left_control"].publish(v_a)
        	self.publishers["pub_right_control"].publish(v_b)
	
		
	

if __name__ == '__main__':
	##Initalize tank_speed_control Node
	rospy.init_node('tank_speed_control')

	##Control object created 
	control = Control()

	# Loop continuously
   	 rospy.spin()
