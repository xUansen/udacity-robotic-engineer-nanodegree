## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Steps to complete the project:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[DH_config]: ./misc_images/DH_config.png
[theta_img]: ./misc_images/theta_diagram.jpg
[result]: ./misc_images/result.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The project kuka-arm is configured as the following images. The arm contains six revolute joints connected to each other in a linear fashion. The DH parameter can be derived from the URDF file.

![alt text][DH_CONFIG]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | ---   | --- | --- | ---
0->1 | 0      | 0     | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | q2 -pi/2
2->3 | 0      | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2   | 0     | 0 | q5
5->6 | -pi/2  | 0     | 0 | q6
6->EE | 0     | 0     | 0.303 | 0   |

##### Each joint tranformation matrix is created by substituting the values into the following equation
```

TM = [[        cos(θ),       -sin(θ),       0,         a],
      [ sin(θ)*cos(α), cos(θ)*cos(α), -sin(α), -sin(α)*d],
      [ sin(θ)*sin(α), cos(θ)*sin(α),  cos(α),  cos(α)*d],
      [             0,             0,       0,         1]]
```

Using the transformation matrix formula above, here are the joint transformation matrices for the arm:

```
Joint 0_1: [[ cos(θ1), -sin(θ1),  0,     0],
          [ sin(θ1),  cos(θ1),  0,     0],
          [       0,        0,  1,  0.75],
          [       0,        0,  0,     1]]
```

```
Joint 1_2: [[ sin(θ2),  cos(θ2),  0,  0.35],
          [       0,        0,  1,     0],
          [ cos(θ2), -sin(θ2),  0,     0],
          [       0,        0,  0,     1]]
```

```
Joint 2_3: [[ cos(θ3), -sin(θ3),  0,  1.25],
          [ sin(θ3),  cos(θ3),  0,     0],
          [       0,        0,  1,     0],
          [       0,        0,  0,     1]]
```

```
Joint 3_4: [[ cos(θ4), -sin(θ4),  0, -0.054],
          [       0,        0,  1,    1.5],
          [-sin(θ4), -cos(θ4),  0,      0],
          [       0,        0,  0,      1]]
```

```
Joint 4_5: [[ cos(θ5), -sin(θ5),  0,      0],
          [       0,        0, -1,      0],
          [ sin(θ5),  cos(θ5),  0,      0],
          [       0,        0,  0,      1]]
```

```
Joint 5_6: [[ cos(θ6), -sin(θ6),  0,      0],
          [       0,        0,  1,      0],
          [-sin(θ6), -cos(θ6),  0,      0],
          [       0,        0,  0,      1]]
```

```
Joint 6_7 [[1, 0, 0, 0],
           [0, 1, 0, 0],
           [0, 0, 1, 0.303],
           [0, 0, 0, 1]]
```

#### Base to Gripper Homogeneous Transformation Matrix
By applying chain of transformation and substituing the values,
the total transformation matrix becomes (R_corr is the correction matrix): 
```

T_total = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 *T6_7* R_corr
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Inverse kinematics are used to calculate the joint angles required to move the arm's gripper to the position.
The problem is decoupled into two parts: position calculation and orientation calculation. 
The first step is to calculate wrist center position by constructing rotation matrix R0_6 based on target roll, pitch yaw required. The theta1, theta3, theta2 are calcualted based on the diagram following. After these parameters are calcualted, the wrist center can be positioned. 
Then we need to calculate theta4, theta5, theta6. This is done by using R0_3 and R0_6 to extract R3_6.

##### Inverse Kinematic Position

![alt-text][theta_img]


##### Inverse Kinematic Orientation

If we know the gripper pose, we can also obtain one transform matrix between `base_link` and `gripper_link`. 

The `roll, pitch, yaw` are calculated with `tf.transformations.euler_from_quaternion` with default rotation sequence from quaternion.
The spherical wrist rotation matrix:
```
Rrpy = rot_yaw(y)* rot_pitch(p) * rot_roll(r) * R_corr
```
- `Rrpy` is the rotation matrix of the gripper's current roll, pitch and yaw
- `R_corr` is the rotation matrix of gripper correction matrix that rotates the gripper around and around Y axis by -90 degrees.

So we have R0_6, then we could get R3_6.
The theta4, theta5, theta6 can calculated from the R3_6.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  
The implementation result follows the steps from last section. 
![result][result]

#### Improvement
1. The planning path can be optimized because the path generated by Moveit!. 

2. The robot arm movement is not steady. This could be improved by tuning controller (PID)

3. The IK calculation is slow using current method. Symbolic method could be replaced by using numpy package.


