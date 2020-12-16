#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import tf
import tf_conversions

MARKER_POSE_TOPIC = '/charuco/marker_pose'

rospy.init_node('charuco_link')

tfBuffer = tf2_ros.Buffer()
tf1Broadcaster = tf.TransformBroadcaster()
tf1Listener = tf.TransformListener()
tfListener = tf2_ros.TransformListener(tfBuffer)

marker_pose_publisher = rospy.Publisher('charuco/marker_pose', PoseStamped, queue_size=0)

first_translation = (None, None, None)
first_yaw = None

def charuco_callback(data):
    try:
        pos = data.pose

        # Transform is represented as a tuple of a translation and rotation
        # Gets latest available transform from the map frame to the base_link frame
        (trans_base_map,rot_base_map) = tf1Listener.lookupTransform('map','base_link', rospy.Time(0)) #Time(0) gets latest tf

        # Extract euler angles from the rotation quaternion
        (roll_base_map,pitch_base_map,yaw_base_map) = tf_conversions.transformations.euler_from_quaternion(rot_base_map)

        #is this right?
        theta_charuco_map = pos.orientation.z - yaw_base_map

        x_map_charuco = pos.position.x - trans_base_map[0]*np.cos(theta_charuco_map) + trans_map_base[1]*np.sin(theta_charuco_map)
        y_map_charuco = pos.position.y - trans_base_map[1]*np.cos(theta_charuco_map) - trans_map_base[0]*np.sin(theta_charuco_map)
        z_map_charuco = 0

        if not first_yaw:
            first_translation = (x_map_charuco,y__map_charuco,z_map_charuco)
            first_yaw = theta_charuco_map

        q = tf_conversions.transformations.quaternion_from_euler(0,0, theta_charuco_map - first_yaw)


        translation = (x_map_charuco - first_translation[0], y_map_charuco - first_translation[1], z_map_charuco - first_translation[2])
        tf1Broadcaster.sendTransform(translation,q,rospy.Time.now(),'/map','/charuco_link')

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn('Unable to publish /charuco_link --> /map because of an error with transform from /map --> /base_link')

def main():
    rospy.Subscriber(MARKER_POSE_TOPIC, PoseStamped, charuco_callback)
    rospy.spin()


if __name__ == '__main__':
    main()