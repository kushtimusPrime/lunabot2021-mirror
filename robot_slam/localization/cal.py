#!/usr/bin/python
# Import required packages
import cv2
import os
import yaml
import numpy as np

OPEN_PATH = "./calib_images/"
SAVE_PATH_YAML = "./calib_data/cam-mac.yaml"

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

cal_data = yaml.load(open('../board.yaml', 'r'), Loader=yaml.Loader)
board = cv2.aruco.CharucoBoard_create(
            cal_data['num_cols'],
            cal_data['num_rows'],
            cal_data['chess_size'],
            cal_data['marker_size'],
            dictionary
        )

parameters =  cv2.aruco.DetectorParameters_create()

# termination criteria for finding sub-pixel corner positions
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# load path names of images to be used for calibration
fnames = os.listdir(OPEN_PATH)

# list to store 'image points' - these are the coordinates of the corners that
# we find in the image using detectMarkers.
image_points = []

imsize = (4000, 4000)
aruco_ids = []
decimator = 0
frame_count = 0

# Extract corner information from each of the images
for fname in fnames:

    img = cv2.imread(OPEN_PATH + fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    res = cv2.aruco.detectMarkers(gray, board.dictionary, parameters=parameters)
    cv2.aruco.refineDetectedMarkers(gray, board, res[0], res[1], res[2])

    print(res[0], res[1])
    if len(res[0])>0:
        for r in res[0]:
            cv2.cornerSubPix(gray, r,
                winSize = (3,3),
                zeroZone = (-1,-1),
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001))
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
            cv2.aruco.drawDetectedCornersCharuco(gray,res2[1],res2[2])

            cv2.imshow("frame", gray)

            # Wait for response
            if cv2.waitKey(0) & 0xFF == ord('y'):

                # Add points to the full list as well as another copy of object points
                image_points.append(res2[1])
                aruco_ids.append(res2[2])

                # Done with this frame
                print("-- processed " + fname + " --")

            # Otherwise, skip the frame
            else:
                print("-- skipped frame " + fname + " --")

    else:
        print("-- unable to find corners in " + fname + ", skipped frame --")

cv2.destroyAllWindows()

# TODO calibrateCameraCharucoExtended vs current
ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(image_points,aruco_ids,board,imsize,None,None,flags=cv2.CALIB_ZERO_TANGENT_DIST)

print("\n=========\n RESULTS\n=========\n")
print("Dist. Coeffifients: ")
print(dist)
print("\nCamera Matrix")
print(mtx)
print("\nData saved to " + SAVE_PATH_YAML)

data = {"camera_matrix": np.asarray(mtx).tolist(),
        "dist_coefficients": np.asarray(dist).tolist(),
        "all_corners": np.asarray(image_points).tolist(),
        "all_ids": np.asarray(aruco_ids).tolist()}
out = open(SAVE_PATH_YAML, 'w')
yaml.dump(data, out)
