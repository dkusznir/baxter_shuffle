# ME 495 baxter_shuffle


# Baxter Shuffle
#### ME 495: Embedded Systems in Robotics
#### _Li-Chun Lu_
#### _Weiyuan Deng_
#### _Daniel Lynch_
#### _Dorian Kusznir_


### Introduction

This project uses Baxter to perform a simplified version of the classic [shell game](https://en.wikipedia.org/wiki/Shell_game). The original idea is that an object will be placed under one cup and Baxter will shuffle each of the three cups for a period of time. One of the audience will then guess which cup contains the object by placing their hand in front of the cup. Baxter will grasp the cup and lift it, then place the cube back down on the table. The shell game is simplified in two ways: instead of using cups and an object hidden inside one of the cups, this version uses three cubes, one of which has a marker on the underside; also, instead of shuffling the blocks randomly, Baxter moves each block to a position chosen randomly from a set of specified positions. The [shuffle algorithm]() section explains this simplification in greater detail.

### Equipment and Hardware Requirements

1. Baxter Robot

2. [ROS Indigo](http://wiki.ros.org/ROS/Installation) on Ubuntu 14.04

3. Three cubes

4. Marker

5. Table

6. Tape

### Preliminary Steps

To setup Baxter and workstation, please follow the [tutorial](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup) from Rethink Robotics.

### Project Overview

1. Move the gripper to home position

2. Sweep the table to detect the cubes

3. Store the position of the three cubes

4. Move the gripper to the first cube 

5. Pick up the cube

6. Move the gripper to a random position

7. Place the cube on the table

8. Store the new location of the cube

9. Repeat step 4~8 to the rest two cubes

10. Move the gripper to a random cube

11. Pick up the cube

12. Move the gripper back to one of the original position

13. Place the cube on the table

14. Repeat step 10~13 to the rest two cubes

### Implementation
##### Shuffle Algorithm

### Reference

### Scripts

### Further Improvements

There are some tasks we can improve and implement in the future to make the algorithm more robust. 

The first imporvement is the implement the inverse kinematics and motion planning for the limb motion. We control the limb simply providing a position in space and hope the IK service will give us a soution. However, we found the solution is not guarantee to find even we use random noise and current limb joint pose as seeds. The entire limb motion could be better if we can calculate severl waypoints along the initial pose and the final pose to reduce IK service error.

The second imporvement is to try different method for detecting objects on the table. We could have the end gripper and camera travel  above the workspaec, inspect the enitre area and mark the location where it see object and marked the location for future useage. In this approach, we may reduce the coordinate conversion error when using camera pixel coordiante or the vector from camera to the center of object. 

The third improvement is to generate the random point that is within Baxter workspae. We have trouble having the Baxter go to the random point generated from the algorithm and the success rate the not high. In the end, we just create a list of locations that are solvable in IK. 

### Conclusion

It was a great experience to work with Baxter Robot. Though we came across many challenges, we finally made it. There is still a lot of work to do from our original idea. We'll try to improve our skills and make it better.

Matlab version of shuffle algorithm [*Baxtershuffle.m*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/Baxtershuffle.m)

shuffle algorithm python [*Baxtershuffle.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/Baxtershuffle.py)

controller node, but not used in the program [*controller.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/controller.py)

we use this node to try debug and test Baxter left arm movement [*move_limb.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/move_limb.py)

node is modified to convert pixel image coordinate to baxter workspac… [*object_detection.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/object_detection.py)

added pseudocode definition of random function [*rnd_loc_gen_notes.txt*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/rnd_loc_gen_notes.txt)

[*tracking.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/tracking.py)








example

![Image of Part 1 Question 2](https://github.com/ME495-EmbeddedSystems/homework-3-f2016-WeiyuanDeng/blob/starter/screenshots/q2.png)

