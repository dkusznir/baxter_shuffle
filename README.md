# ME 495 baxter_shuffle


# Baxter Shuffle
#### ME 495: Embedded Systems in Robotics
#### _Li-Chun Lu_
#### _Weiyuan Deng_
#### _Daniel Lynch_
#### _Dorian Kusznir_


### Introduction

This project uses Baxter to perform a simplified version of the classic [shell game](https://en.wikipedia.org/wiki/Shell_game). The shell game is simplified in two ways: instead of using cups and an object hidden inside one of the cups, this version uses three cubes, one of which has a marker on the underside; also, instead of shuffling the blocks randomly, Baxter moves each block to a position chosen randomly from a set of specified positions. The [shuffle algorithm]() section explains this simplification in greater detail.

### Equipment and Hardware Requirements

1. Baxter Robot

2. ROS Indigo on Ubuntu 14.04

3. Three cubes

4. Marker

5. Table

6. Tape

### Preliminary Steps

### Project Overview

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

Matlab version of shuffle algorithm [*Baxtershuffle.m*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/Baxtershuffle.m)

shuffle algorithm python [*Baxtershuffle.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/Baxtershuffle.py)

controller node, but not used in the program [*controller.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/controller.py)

we use this node to try debug and test Baxter left arm movement [*move_limb.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/move_limb.py)

node is modified to convert pixel image coordinate to baxter workspac… [*object_detection.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/object_detection.py)

added pseudocode definition of random function [*rnd_loc_gen_notes.txt*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/rnd_loc_gen_notes.txt)

[*tracking.py*](https://github.com/dkusznir/baxter_shuffle/blob/master/src/tracking.py)








example

![Image of Part 1 Question 2](https://github.com/ME495-EmbeddedSystems/homework-3-f2016-WeiyuanDeng/blob/starter/screenshots/q2.png)

