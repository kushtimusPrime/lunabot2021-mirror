#!/usr/bin/python3
import json
import rospy
import tf2_ros
from tf_conversions.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, TransformStamped

DYNAMIC_FILENAME = 'tf_file_dynamic.json'

class Transformer:
    def __init__(self, topic, frame_id, child_id):
        self.frame_id = frame_id
        self.child_id = child_id
        self.pose_sub = rospy.Subscriber(topic, Pose, self.get_tf)
        self.broadcaster = tf2_ros.TransformBroadcaster()
    
    def get_tf(self, pose_msg):
        tf_msg = tf2_ros.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.child_id
        tf_msg.transform.translation = pose_msg.position
        tf_msg.transform.rotation = pose_msg.orientation
        self.broadcaster.sendTransform(tf_msg)

if __name__ == '__main__':
    rospy.init('tf_dynamic_broadcaster')
    with open(DYNAMIC_FILENAME) as f:
        transforms = json.load(f)['transforms']
    
    for transform in transforms:
        Transformer(transforms['topic'], transforms['frame_id'], transforms['child_frame_id'])

    rospy.spin()
