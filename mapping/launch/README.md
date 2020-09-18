# mapping

This package uses an Xbox Kinect to create a costmap of the environment using RTabMap.

Freenect_throttle.launch will run freenect and the Xbox Kinect will start capturing the environment.
```
roslaunch mapping freenect_throttle.launch
```

Freenect_throttle_rtabmap.launch will run just rtabmap.
```
roslaunch mapping freenect_throttle.launch
```

full_mapping.launch will run both of the above launch files.
```
roslaunch mapping full_mapping.launch
```
