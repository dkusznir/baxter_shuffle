# ME 495 baxter_shuffle


# Baxter Shuffle
#### ME 495: Embedded Systems in Robotics
#### _Li-Chun Lu_
#### _Weiyuan Deng_
#### _Daniel Lynch_
#### _Dorian Kusznir_


### Introduction

This project uses Baxter to perform a simplified version of the classic [shell game](https://en.wikipedia.org/wiki/Shell_game). The original idea is that an object will be placed under one cup and Baxter will shuffle each of the three cups for a period of time. One of the audience will then guess which cup contains the object by placing their hand in front of the cup. Baxter will grasp the cup and lift it, then place the cube back down on the table. The shell game is simplified in two ways: instead of using cups and an object hidden inside one of the cups, this version uses three cubes, one of which has a marker on the underside; also, instead of shuffling the blocks randomly, Baxter moves each block to a position chosen randomly from a set of specified positions. The [shuffle algorithm](#####Shuffle Algorithm) section explains this simplification in greater detail.

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
  The shuffle algorithm went through several revisions before the group settled on the following algorithm:
  ```
  0. The three blocks start from known "home" positions.
  For each block:
    1. A new position for the current block is chosen randomly from an array of known positions in the workspace
    2. Baxter moves the current block from its home position to the new position
  Steps 1 and 2 can be repeated for an arbitrary number of shufflings. Currently only 1 iteration is implemented.

  Once shuffling is complete, Baxter will move the blocks back to the home positions 
  in a different order than their original order, finishing the shuffling sequence.
  Steps 1 and 2 can be repeated for an arbitrary number of shuffle iterations. 
  (Currently only 1 iteration is implemented.)
  Once shuffling is complete, Baxter will move the blocks back to the home positions in a different order than their original order, finishing the shuffling sequence.
  ```
  The shuffling algorithm uses two arrays: one array consists of the target positions used during shuffling, and the other array is a 3x2 array of the current position of each block which is updated after each iteration of shuffling.

#####

### Reference

### Scripts
object_detection.py
	Node name: 'left_hand_camera_detection'
	Function:
		This node converts the pixel-coordinate representation of the center of the detected cube to coordinates in Baxter's workspace. In the callback function for this node, we set the camera resolution to 640x400. When the program starts, the robot moves to a pre-defined home position. In Baxter's workspace, the camera identifies the cube by its green face, marks the perimeter of the cube and generates camera-frame coordinates of the cube's center point. 
			Tried: (PinholeCameraModel(), image_geometry package -->> image_geometry.PinholeCameraModel.projectPixelTo3dRay), didn't work because although Baxter would move towards the specified target location, the final location of the end-effector was always off some distance from the target location.
			2nd way: followed example "Worked_ExampleVisualServoing" [link], implementing the "image pixel to Workspace coordinate conversion" [link]. This method worked some of the time but resulted in unreliable pick-and-place performance, especially because of difficulty tuning the camera calibration factor (cc).
	Subscribes to:
		/cameras/left_hand_camera/image
	Publishes to:
		'convertPixeltoCoordinate'
		This will publish the center point of the detected cube using the OpenCV image processor.

tracking.py
	Node name: 'limbs_tracking'
	Function:
		This node receives the Baxter-frame coordinates of a point from the 'left_hand_camera_detection' node. It uses the x-y coordinates of this point, along with a fixed z-coordinate and orientation (represented as a quaternion).
		Within this node there is a Python function named 'control_baxter_arm()' that solves the inverse kinematics problem and moves Baxter's arm accordingly. Function control_baxter_arm().
		First, move to home position. Once at home position, uses convertPixeltoCoordinate and moves Baxter's left arm to the green cube described by this point.
		getPoint()
			Goes to home position, receives the target position. Set global flag to make sure arm receives target position before moving. Once the arm is at its target position, the flag is reset. This functions like an interrupt service routine (ISR). Once at the target position, the function saves the position into an array that keeps track of the blocks' locations.
	Subscribes to:
		'/convertPixeltoCoordinate'
		'/robot/limb/left/endpoint_state'
	Publishes to:
		(none)

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

