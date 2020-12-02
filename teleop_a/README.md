# teleop_a
This package can tank drive a raspberry pi car with a USB Xbox Controller
```
roslaunch teleop_a teleop_car.launch
```
This launch file will run the joy node, and convert the Y stick values to left and right motor output percentages, which are then used in the car's drive command.

```
roslaunch teleop_a teleop_car_c.launch
```
This launch file is to be used in association with the move_base node because it takes in the velocity topic and convert that to motor control commands for the pi car.
