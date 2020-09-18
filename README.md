# lunabot_2021_master
Rong Wang™ Edition®

1. This is a catkin project, therefore you will need to create a catkin workspace to contain all the packages. Create a folder called `catkin_ws` and within it, a folder called `src`. Navigate to the catkin_ws directory. This can be done via the command line with the following commands: 
```
mkdir -p catkin_ws/src  # Create catkin_ws and catkin_ws/src directories
cd catkin_ws
```
2. You then need to clone this repository into the src directory of your catkin workspace. Make sure to execute this command from inside the top level of the workspace.
```
git clone https://github.com/vanderbiltrobotics/lunabot-2020.git src
```
The `src` folder of your catkin_ws should now contain all the packages in the repository.

3. Finally, you can build your workspace with:
```
catkin_make              # Build all packages in the workspace
```
4. Sourcing the catkin_ws

Running the command `source {Path to catkin_ws}/catkin_ws/devel/setup.bash` or `source /opt/ros/melodic/setup.bash` will allow ROS to find your built packages. You will need to run this command each time you open a new terminal if you want ROS to find your packages. To avoid this, you can add that command to the end of your .bashrc and it will run automatically whenever you open a terminal.


## Notes on testing code

1. Make a launch file (Look at the ros_teleop.launch file because it's a pretty simple example of a launch file and you can just build up from there).
2. Once you `catkin_make` and `source devel/setup.bash` in your catkin_ws, then you can run your package with:
```
roslaunch [package_name] [launch_file_name]
```
3. To see the interface and learn about the nodes and topics currently running, run the following:
```
rqt_graph
```
4. To see what topics are being published, run the following:
 ```
 rostopic list
 ```
 5. To see what a certain topic is actually publishing, run the following:
 ```
 rostopic echo [topic_name]
 ```
## Notes on developing these packages

If you are working on a new version of a particular package, do it on a separate branch! The master branch should always contain the latest *working version* of each package. Once your new version of a package is working and thoroughly tested, it can be merged into the master branch.
