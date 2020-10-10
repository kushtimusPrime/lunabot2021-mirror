#!/usr/bin/python
import cv2
import time
import yaml
import rospkg
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
rospy.init_node('aruco_localization')
raw_image_publisher = rospy.Publisher('camera/raw_image', Image, queue_size=0)
charuco_found_publisher = rospy.Publisher('camera/found_image', Image, queue_size=0)
image_found_publisher = rospy.Publisher('camera/marker_found', Bool, queue_size=0)
#raw_image_publisher.init_node()
#charuco_found_publisher.init_node()
#image_found_publisher.init_node()
print(cv2.__version__)

BOARD_FILE = rospkg.RosPack().get_path('robot_slam') + '/board.yaml'
#CALIBRATION_DATA_DIR = rospkg.RosPack().get_path('robot_slam')
#CALIBRATION_DATA_FILE = CALIBRATION_DATA_DIR + '/cam.yaml'

DEVICE_NUM = 0
#SAVE_PATH = "./calib_images/"
#SAVE_PATH_YAML = "./calib_data/cam.yaml"
CAPTURE_RATE = 1
PREVIEW_TIME = 10

bridge = CvBridge()

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

cal_data = yaml.load(open(BOARD_FILE, 'r'), Loader=yaml.Loader)
board = cv2.aruco.CharucoBoard_create(
    cal_data['num_cols'],
    cal_data['num_rows'],
    cal_data['chess_size'],
    cal_data['marker_size'],
    dictionary
)
parameters =  cv2.aruco.DetectorParameters_create()

stream = cv2.VideoCapture(DEVICE_NUM)

allCorners = []
allIds = []
decimator = 0
frame_count = 0
start_time = time.time()

while True:

    ret, gray = stream.read()

    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

    if time.time() - start_time > CAPTURE_RATE:

        res = cv2.aruco.detectMarkers(gray, board.dictionary, parameters=parameters)
        # TODO not using refineDetectedMarkers
        cv2.aruco.refineDetectedMarkers(gray, board, res[0], res[1], res[2])

        print(res[0], res[1])
        if len(res[0])>0:
            for r in res[0]:
                cv2.cornerSubPix(
                    gray, 
                    r,
                    winSize = (3,3),
                    zeroZone = (-1,-1),
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
                )

            # TODO not using interpolateCornersCharuco
            res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
                cv2.aruco.drawDetectedCornersCharuco(gray,res2[1],res2[2])

                cv2.imshow("frame", gray)
                charuco_found_publisher.publish(bridge.cv2_to_imgmsg(gray, '8UC1'))
                image_found_publisher.publish(True)
                if cv2.waitKey(PREVIEW_TIME * 1000) & 0xFF == ord('y'):

                    #cv2.imwrite(CALIBRATION_DATA_DIR + "calib_img" + str(frame_count) + ".png", gray)
                    print("Captured frame #" + str(frame_count))
                    frame_count += 1

                    start_time = time.time()
            else:
                image_found_publisher.publish(False)
##
    cv2.imshow('frame', gray)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    decimator+=1

    imsize = gray.shape

    raw_image_publisher.publish(bridge.cv2_to_imgmsg(gray, '8UC1'))

yaml.dump({
        'num_cols': cal_data['num_cols'],
        'num_rows': cal_data['num_rows'],
        'chess_size': cal_data['chess_size'],
        'marker_size': cal_data['marker_size'],
        'imsize': imsize
    }, 
    open(BOARD_FILE, 'w'))

stream.release()
cv2.destroyAllWindows()
