# move_base_static_sim
This package will use launch ROS mapserver, which will take still images and create a costmap that is used as an input for Move Base. The simulator node will generate a simulated odoometry topic that is also fed to the move_base node. Using rviz, you can then publish a navigation goal and you will be able to see the robot move to the goal at the generated velocities.
```
roslaunch move_base_static_sim sim.launch
```
The sim.launch file launches all of the necessary nodes to accomplish the above goal.
