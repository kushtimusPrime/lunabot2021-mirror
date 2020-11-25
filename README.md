# lunabot_2021_master
Rong Wang™ Edition®

1. This is a catkin project, therefore you will need to create a catkin workspace to contain all the packages. Create a folder called `catkin_ws` and within it, a folder called `src`. Navigate to the catkin_ws directory. This can be done via the command line with the following commands: 
```
mkdir -p catkin_ws/src  # Create catkin_ws and catkin_ws/src directories
cd catkin_ws
```
2. You then need to clone this repository into the src directory of your catkin workspace. Make sure to execute this command from inside the top level of the workspace.
```
git clone https://github.com/vanderbiltrobotics/lunabot-2021.git src
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

## Notes on Decentralized ROS
Throughout testing ROS code, it will become very important to test our code on actual machines. Below is an example where you have a catkin workspace on your computer (COMP1) and you have the same branch of the workspace on a Raspberry Pi attached to the machine (PI 1).

Prerequisite: Same wifi connection/VPN connection
The best way to verify that the connection is strong enough is by making a terminal chatroom between COMP1 and PI1 via netcat.

COMP 1
```
netcat -l [Port Number (ex. 1234)]
```

PI 1
```
netcat [COMP1 IP Address] [Same Port Number (ex. 1234)]
```

You should be able to type a message on either the COMP1 terminal or the PI1 terminal, hit enter, and then see that message on the other terminal.

Main Step: Setup ROS_MASTER_URI
ROS_MASTER_URI is a ROS variable that tells all the ROS nodes where to look to find the master (roscore). By default, ROS_MASTER_URI is set to http://localhost:11311. However, in this particular scenario, the goal is to have a single roscore shared by two machines (COMP1 and PI1).
ROS_IP is another ROS variable that specifies the IP address of the local machine to ROS nodes. While its usually okay to leave default, it can sometimes cause errors if not specified.

COMP1
```
export ROS_MASTER_URI=http://[COMP1 IP Address]:11311/
export ROS_IP=[COMP1 IP Address]
roscore
```

PI1
```
export ROS_MASTER_URI=http://[COMP 1 IP Address]:11311/
export ROS_IP]=[PI1 IP Address]
roslaunch ...
```

The reason why this will be very helpful is that many ROS visualization tools (rviz, rtabmapviz) will lag on the pi, so being able to see those visualizations on a computer while running ROS code on a pi connected to the machine is the ticket.


