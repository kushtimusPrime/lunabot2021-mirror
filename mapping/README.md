# mapping

This package uses an Xbox Kinect or Intel Realsense D435i to create a costmap of the environment using RTABMAP.

Freenect_throttle.launch will run freenect and the Xbox Kinect will start capturing the environment.
```
roslaunch mapping freenect_throttle.launch
```

Freenect_throttle_rtabmap.launch will run just RTABMAP.
```
roslaunch mapping freenect_throttle.launch
```

full_mapping.launch will run both of the above launch files.
```
roslaunch mapping full_mapping.launch
```

rtabmap_realsense.launch will run RTABMAP and activate the Realsense to start capturing images and create a costmap of the environment
```
roslaunch mapping rtabmap_realsense.launch
```
