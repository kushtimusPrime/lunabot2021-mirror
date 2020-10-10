#! /usr/bin/env python
#include <ros/ros.h>
#include <geometry_msg/Twist.h>


int x_linear = 0; ##fake forward and backward velocity MAX 0.34
int z_angular = 0; ##fake angular veloctiy about z axis MAX 0.72



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
    return v_a, -v_b

	


def start():
	
	pub = rospy.Publisher("joystick_reader", String ,queue_size=10)

	#subscribed to joystick input on topic Joy
	rospy.Subscriber('/joy', Joy, callback)

	##Initalize tank_speed_control Node
	rospy.init_node('tank_speed_control_c')


if __name__ == '__main__':
	start()
