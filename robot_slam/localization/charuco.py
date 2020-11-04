#!/usr/bin/python
import rospy
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Float64
from cv_bridge import CvBridge
import tf2_ros
import tf
import tf_conversions
import yaml
bridge = CvBridge()


IMAGE_TOPIC = "/camera/color/image_raw"

rospy.init_node('aruco_localization')
BOARD_FILE = rospkg.RosPack().get_path('robot_slam') + '/board.yaml'
CALIBRATION_DATA_FILE = rospkg.RosPack().get_path('robot_slam') + '/localization/calibration/camera_b.yaml'

cal_data = yaml.load(open(BOARD_FILE, 'r'), Loader=yaml.Loader)
# board = cal_data['board']
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard_create(
    cal_data['num_cols'],
    cal_data['num_rows'],
    cal_data['chess_size'],
    cal_data['marker_size'],
    dictionary
)
parameters = cv2.aruco.DetectorParameters_create()


imboard = board.draw((1000, 1400))
cv2.imwrite('charuco.jpg', imboard)

cal_data = yaml.load(open(CALIBRATION_DATA_FILE, 'r'), Loader=yaml.Loader)
mtx = np.asarray(cal_data['camera_matrix'])
dist = np.asarray(cal_data['dist_coefficients'])
p_rvec = None
p_tvec = None

cap = cv2.VideoCapture("/home/nisala/catkin_ws/src/robot_slam/realsense.mkv")
decimator = 0

bridge = CvBridge()
tfBuffer = tf2_ros.Buffer()
tf1Broadcaster = tf.TransformBroadcaster()
tf1Listener = tf.TransformListener()
tfListener = tf2_ros.TransformListener(tfBuffer)
raw_image_publisher = rospy.Publisher('camera/raw_image', Image, queue_size=0)
detected_marker_image_publisher = rospy.Publisher('camera/detected_image', Image, queue_size=0)
marker_pose_publisher = rospy.Publisher('charuco/marker_pose', PoseStamped, queue_size=0)
robot_pose_publisher = rospy.Publisher('charuco/rover_pose', PoseStamped, queue_size=0)
marker_detected_publisher = rospy.Publisher('charuco/marker_detected', Bool, queue_size=0)
raw_distance_publisher = rospy.Publisher('charuco/raw_distance', Float64, queue_size=0)
counter = 0


def callback(data):
    global decimator
    global p_rvec
    global p_tvec
    global counter

    frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray2 = gray

    marker_detected = Bool()
    marker_detected.data = False

    res = cv2.aruco.detectMarkers(gray, board.dictionary, parameters=parameters)
    cv2.aruco.refineDetectedMarkers(gray, board, res[0], res[1], res[2])

    if len(res[0]) > 0:
        for r in res[0]:
            cv2.cornerSubPix(
                gray,
                r,
                winSize=(3, 3),
                zeroZone=(-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
            )
        res2 = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray, board)

        if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 3 == 0:
            cv2.aruco.drawDetectedCornersCharuco(gray, res2[1], res2[2])
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(res2[1], res2[2], board, mtx, dist, rvec=p_rvec, tvec=p_tvec)
            if retval:
                print('rvec:', rvec)
                print('tvec:', tvec)
                p_rvec = rvec
                p_tvec = tvec

                # Compute pose of Camera relative to world
                # dst, _ = cv2.Rodrigues(rvec)
                # R = dst.T
                # tvec = np.dot(-R, tvec)
                # rvec, _ = cv2.Rodrigues(R)

                marker_detected.data = True
                marker_pose = PoseStamped()
                quat = tf.transformations.quaternion_from_euler(rvec[1, 0], rvec[0, 0], -rvec[2, 0])
                marker_pose.header.frame_id = "camera_link"
                marker_pose.header.stamp = rospy.Time.now()
                marker_pose.header.seq = counter
                marker_pose.pose.position.x = tvec[0, 0] / 100
                marker_pose.pose.position.y = tvec[2, 0] / 100
                marker_pose.pose.position.z = tvec[1, 0] / 100
                marker_pose.pose.orientation.x = quat[0]
                marker_pose.pose.orientation.y = quat[1]
                marker_pose.pose.orientation.z = quat[2]
                marker_pose.pose.orientation.w = quat[3]
                marker_pose_publisher.publish(marker_pose)
                raw_distance_publisher.publish(((tvec[2, 0] ** 2) + (tvec[0, 0] ** 2) + (tvec[1, 0] ** 2)) ** 0.5)

                try:
                    # First we need to get the estimate of the robot pose from the odometry data
                    (trans_base_odom, rot_base_odom) = tf1Listener.lookupTransform('odom', 'base_link', rospy.Time(0))

                    # Extract rotation from quaternion
                    (roll_base_odom, pitch_base_odom, yaw_base_odom) = tf_conversions.transformations.euler_from_quaternion(rot_base_odom)

                    pos = marker_pose.pose.position

                    # theta_odom_map is the error between the particle filter estimated orientation vs the odometry estimated orientation
                    # Once again assumes 2D robot - robot does not fly or roll
                    theta_odom_map = pos.z - yaw_base_odom

                    #Next, we find the difference in the position estimate between the particle filter and the odometry
                    x_odom_map = pos.x - trans_base_odom[0]*np.cos(theta_odom_map) + trans_base_odom[1]*np.sin(theta_odom_map)
                    y_odom_map = pos.y - trans_base_odom[1]*np.cos(theta_odom_map) - trans_base_odom[0]*np.sin(theta_odom_map)
                    z_odom_map = 0 # Assuming the robot is not magically flying



                    # Finally, we convert back to quaternion
                    print(theta_odom_map)
                    q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta_odom_map)

                    # Publish transform
                    tf1Broadcaster.sendTransform((x_odom_map, y_odom_map, z_odom_map), q, rospy.Time.now(), "/odom", "/map")

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Unable to publish transformation from /map --> /odom because of an error with transform from /odom --> /base_link")

                # robot_pose = PoseStamped()
                # trans = tfBuffer.lookup_transform('base_link', 'map', rospy.Time(0))
                # robot_pose.header.frame_id = "base_link"
                # robot_pose.header.stamp = rospy.Time.now()
                # robot_pose.header.seq = counter
                # robot_pose.pose.position.x = trans.transform.translation.x
                # robot_pose.pose.position.y = trans.transform.translation.y
                # robot_pose.pose.position.z = trans.transform.translation.z
                # robot_pose.pose.orientation.x = trans.transform.rotation.x
                # robot_pose.pose.orientation.y = trans.transform.rotation.y
                # robot_pose.pose.orientation.z = trans.transform.rotation.z
                # robot_pose.pose.orientation.w = trans.transform.rotation.w
                # robot_pose_publisher.publish(robot_pose)
                # detected_marker_image_publisher.publish(bridge.cv2_to_imgmsg(gray2, '8UC1'))

                counter = counter + 1

    # cv2.imshow('frame', gray2)
    decimator += 1

    marker_detected_publisher.publish(marker_detected)
    raw_image_publisher.publish(bridge.cv2_to_imgmsg(gray2, '8UC1'))


def main():
    rospy.Subscriber(IMAGE_TOPIC, Image, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
