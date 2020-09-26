#move_base_kinect
This package will launch rtabmap with the kinect or realsense and use that map and odometry as input to the move_base package. The output is a velocity Twist message.
``` roslaunch move_base_kinect kinect_move_base.launch ```
This is the primary launch file for this package that will accomplish the above goal.
