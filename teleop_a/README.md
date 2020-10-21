# teleop_a
This package can tank drive a raspberry pi car with a USB Xbox Controller
```
roslaunch teleop_a teleop_car.launch
```
This launch file will run the joy node, and convert the Y stick values to left and right motor output percentages, which are then used in the car's drive command.
