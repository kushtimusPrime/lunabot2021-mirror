# move_base_kinect

This package will launch the move base node which is the core of the navigation stack. The RTABMAP node with the realsense will use that map and odometry as input to the move_base package. The output is a velocity Twist message. If you want the car to move, this package must be ran on the main computer with the realsense attached, while the teleop a package must be run on the pi.
``` 
roslaunch move_base_kinect full_system.launch 
```

This is the primary launch file for this package that will accomplish the above goal.
