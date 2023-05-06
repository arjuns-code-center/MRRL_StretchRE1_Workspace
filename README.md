# MRRL_StretchRE1_Workspace
Stretch RE1 robot code for motion, image capture, and other functionality. 

# UMass Amherst Mechatronics and Robotics Research Lab
Author: Arjun Viswanathan, CompE 2024 at UMass Amherst

Date Created: 2/16/23
Last date modified: 5/5/23

Dr. Frank Sup, Gina Georgadarellis

Stretch RE1 is a robot developed by hello robot inc, and is used by this lab at UMass to help in nursing. From simple motion to picking things up and mapping paths around a room, this robot is very versatile in how it can be used in the medical field to help with simple tasks. 

I have developed code using the stretch_body API in Python that will take keyboard input and move different parts of Stretch. It is simple in design and easy to understand, and runs with ROS Melodic in a Linux environment. Everything required for it is in this repository. 

There are two packages created. Simplemotion is a package that runs teleop commands as well as autonomous commands on Stretch. Capture_image package is used for camera work on Stretch, which is work for the future. 

# Simplemotion Package
There is a base keyboard_teleop.py file that takes inputs from the keyboard and moves different parts of Stretch. Also, there are 3 autonomous commands (located in the auto_commands folder)
- avoidObstacles.py
- box.py
- followObjects.py

To run just the teleoperated commands, use the following code in the command line
```roslaunch simplemotion teleop_keyboard.launch```
To run teleop with autonomous commands, use the following code in the command line
```roslaunch simplemotion teleop_with_auto.launch```

The avoidObstacles module has two classes: SimpleAvoid and BetterAvoid. SimpleAvoid is a base version of collision avoidance where Stretch processes just what is in front of it, and takes an action accordingly. This opens up a significant flaw where Stretch deadlocks between two states and continuously oscillates between two actions. BetterAvoid, however, uses a one-step lookbehind (a previous state) to take its action. This way, it does not deadlock like SimpleAvoid does. 

The followObjects module will use the same strategy as used in BetterAvoid, but it turns toward you instead of away from you. This way, Stretch will follow you at a distance, and stop when he gets close. If you get too close, then he will back up. A small flaw is that just the LiDAR is not enough to track the actual target. This is where computer vision is required, which is work for the future. 

To run the BetterAvoid algorithm, use the following code in the command line
```roslaunch simplemotion autocommands.launch avoidObstacles:=1 box:=0 followObjects:=0```
To run the Box algorithm, use the following code in the command line
```roslaunch simplemotion autocommands.launch avoidObtacles:=0 box:=1 followObjects:=0```
To run the FollowObjects algorithm, use the following code in the command line
```roslaunch simplemotion autocommands.launch avoidObtacles:=0 box:=0 followObjects:=1```
