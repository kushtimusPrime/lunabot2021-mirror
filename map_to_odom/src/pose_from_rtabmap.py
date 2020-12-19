#!/usr/bin/python
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Float64
import tf

#not sure if this is the place we want to publish to
robot_pose_publisher = rospy.Publisher('rtabmap/rover_pose', PoseStamped, queue_size=0)

def main():
    rospy.init_node("pose_from_rtabmap")

    listener = tf.TransformListener()

    counter = 0

    while not rospy.is_shutdown():
        try:
            robot_pose = PoseStamped()
            
            (trans,rot) = listener.lookupTransform('base_link_r', 'map_r', rospy.Time(0))

            robot_pose.header.frame_id = "base_link_r"
            robot_pose.header.stamp = rospy.Time.now()
            robot_pose.header.seq = counter

            robot_pose.pose.position.x = trans[0]
            robot_pose.pose.position.y = trans[1]
            robot_pose.pose.position.z = trans[2]
            
            robot_pose.pose.orientation.x = rot[0]
            robot_pose.pose.orientation.y = rot[1]
            robot_pose.pose.orientation.z = rot[2]
            robot_pose.pose.orientation.w = rot[3]

            robot_pose_publisher.publish(robot_pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to publish robot pose (in map frame) because of an error with transform from /map_r --> /base_link_r")

        counter = counter + 1

if __name__ == '__main__':
    main()
