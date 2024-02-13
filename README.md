# Shapes-Organising-Robot

## Background
This is a project I did as part of my 'Sensors and Control' subject's major assignment, where I was tasked to apply computer vision in combination with robotics to do a task.

The robot used was a UR3e, with an RGB-D camera (Intel Realsense D435i), which used an OnRobot RG2 Gripper, to manipulate different 3D printed shapes.

Using the depth data, I was able to ensure that the gripper could hold shapes of different sizes. Using OpenCV I was able to classify the shape so the robot could separate and organise the different shapes.

## Connections
Both the robot and gripper used ethernet to connect to the P.C, so it was connected to a router, which was connected to my P.C.

The camera was connected to my P.C via USB. It was mounted on RG2 gripper, as it was hard to mount it on the UR3e's wrist (as it was cylindrical, and therefore was very prone to slipping). A 3D printed bracket was then used to securely hold the camera onto the RG2 Gripper.

## Code Components

### RGB-D Camera
Initialise camera:
```bash
roslaunch realsense2_camera rs_rgbd.launch 
```

The RGB-D data is encoded, however it is decoded and printed onto a topic when running:
rosrun depth_image_processing image_to_xyz 

To make the processing of the images easier, I added another file, which has 2 threshold points. This means that it can make pixels which are outside of range 'black', meaning image processing is more accurate and faster. So run this instead of the similar above command:
```bash
rosrun depth_image_processing image_to_xyz 
```
### Gripper
To control the RG2 Gripper, first run:
```bash
roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=192.168.1.1 
```
I wrote a an RG2.py file which publishes a message to the OnRobotRGOutput topic to control the gripper.

### Robot
To control the UR3e you beed to run the UR ROSDRIVER. However this process can be done by running the launch file:
```bash
roslaunch ur3e_rg2_moveit_config ur3e_bringup.launch 
```
Then on pendant run the externalControl program.

Then to run the UR3e with MoveIt!:
```bash
roslaunch ur3e_rg2_moveit_config demo_planning_execution.launch 
```
In the rviz window of MoveIt! find the config file to initialise the environment:
/home/selimon/ur_ws/src/ur3e_rg2_moveit_config/config/config_with_trail_animation.rviz


## Project Demonstration
### Demonstration
https://youtu.be/gzgtYLjqDSw

### Bottle Sorting Project (Using Soda Cans)
A similar project was also done where I identified popular sorting cans as a project as many items (particularly alcohol bottles), are being sent back to their manufacturer so they can be reused. But first they have to be sorted to their brand:
https://youtu.be/UHeZQ_xx6JE
