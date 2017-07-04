## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[image4]: ./misc_images/DHparams.PNG
[image5]: ./misc_images/theta1.PNG
[image6]: ./misc_images/theta2.PNG
[image7]: ./misc_images/theta3.PNG
[image8]: ./misc_images/wrist_joint_eq.PNG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This is the Writeup for Kinematics Pick & Place Project.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

 Here is derived DH parameters of Kuka KR210 robot based on `kr210.urdf.xacro file.`

|   i   | &alpha;(i-1) |  a(i-1)     |  d(i)    | &theta;(i) |
| :---: | ----------:  | ---------:  | ------:  | --------:  |
| 1     | 0            | 0           | 0.75     | &theta;_1  |
| 2     | -&pi;/2      | 0.35        | 0        | &theta;_2-&pi;/2 |
| 3     | 0            | 1.25        | 0        | &theta;_3  |
| 4     | -&pi;/2      | -0.054      | 1.50     | &theta;_4  |
| 5     | &pi;/2       | 0           | 0        | &theta;_5  |
| 6     | -&pi;/2      | 0           | 0        | &theta;_6  |
| G     | 0            | 0           | 0.303    | 0          |

KR210 robot image and link assignments and joint rotations axes are shown below.
The parameters are obtained using following convention.
![alt text][image1]

![alt text][image4]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


 Individual transform matrices about each joint are obtained using the DH table with below Sympy calculations.

```python
    ### KUKA KR210 ###
    # DH Parameters
    s = {alpha0:     0, a0:      0, d1:  0.75, 
         alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2, 
         alpha2:     0, a2:   1.25, d3:     0, 
         alpha3: -pi/2, a3: -0.054, d4:  1.50, 
         alpha4:  pi/2, a4:      0, d5:     0, 
         alpha5: -pi/2, a5:      0, d6:     0, 
         alpha6:     0, a6:      0, d7: 0.303, q7: 0}

    ### Homogeneous Transforms
    # base_link to link1
    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                   [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                   [                   0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                   [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                   [                   0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   [                   0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                   [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                   [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                   [                   0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                   [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                   [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                   [                   0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                   [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                   [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                   [                   0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)

    T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                   [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                   [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                   [                   0,                   0,            0,               1]])
    T6_G = T6_G.subs(s)

    # Composition of Homogeneous Transforms
    T0_2 = simplify(T0_1 * T1_2) # base_link to link2
    T0_3 = simplify(T0_2 * T2_3) # base_link to link3
    T0_4 = simplify(T0_3 * T3_4) # base_link to link4
    T0_5 = simplify(T0_4 * T4_5) # base_link to link5
    T0_6 = simplify(T0_5 * T5_6) # base_link to link6
    T0_G = simplify(T0_6 * T6_G) # base_link to linkG
```
 
 Also, a homogeneous transform matrix from base_link to gripper_link with consideration of difference in orientation of the grippler_link frame. 
Actual values can be obtained by assigning above DH parameter values and actual theta values.

```python
    # Correction Needed to Account of Orientation Difference Between Definition of 
    # Gripper_Link in URDF versus DH Convention
    R_z = Matrix([[     cos(np.pi), -sin(np.pi),             0,   0],
                  [     sin(np.pi),  cos(np.pi),             0,   0],
                  [              0,           0,             1,   0],
                  [              0,           0,             0,   1]])
    R_y = Matrix([[  cos(-np.pi/2),           0, sin(-np.pi/2),   0],
                  [              0,           1,             0,   0],
                  [ -sin(-np.pi/2),           0, cos(-np.pi/2),   0],
                  [              0,           0,             0,   1]])
    R_corr = simplify(R_z * R_y)


    # Total Homogeneous Transform Between Base_link and Gripper_link with
    # Orientation Correction Applied
    T_total = simplify(T0_G * R_corr)
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Since the last three joints in KR210 robot are revolute and their joint axes intersect at a single point, IK problem can be decoupled into Inverse Position and Inverse Orientation problems.

##### Step1: find the location of the wrist center relative to the base frame.
 The wrist center position `wx, wy and wz` is calculated below equation, where Rrpy is rotation matrix of the end-effector relative to the base frame. And `(d6+d7)` indicates displacement from the WC to the EE.
```python
    Rrpy = Matrix([[     cos(pitch)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw),  cos(roll)*sin(pitch)+cos(yaw)*sin(roll)*sin(yaw)],
                   [     cos(pitch)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw)],
                   [             -sin(pitch),                             sin(roll)*cos(pitch),                              cos(roll)*cos(pitch)]])

    Rrpy = Rrpy * R_corr[0:3,0:3]

    wx = px - (d6+d7) * Rrpy[0,2]
    wy = py - (d6+d7) * Rrpy[1,2]
    wz = pz - (d6+d7) * Rrpy[2,2]
```

##### Step2: find joint variables, theta1, theta2 and theta3.

`theta1` is determined by the wrist center position as shown below figure.
![alt text][image5]
```python
    theta1 = atan2(wy, wx)
    theta1 = theta1.evalf(subs=s)
```

`theta3` is determined using geometry and cosine rule as shown below figure.
![alt text][image6]
```python
    L2 = a2               # arm length of joint2
    L35 = sqrt(a3**2 +d4**2) # arm length of joint3
    theta31 = atan2(-a3, d4) # arm angle of joint3
    r = sqrt(wx**2 + wy**2) - a1 # distance between joint1 and a point projected xy-plane from the wrist center
    S = wz - d1 # distance along z-axis between joint1 and the wrist center
    L25 = sqrt(r**2 + S**2)
    D = (L25**2 - L2**2 - L35**2) / (2*L2*L35)
    theta32 = atan2(sqrt(1-D**2), D)

    theta3 = theta32 - theta31 - pi/2
    theta3 = theta3.evalf(subs=s)
```

`theta2` is also determined using geometry and cosine rule as shown below figure.
![alt text][image7]
```python
    theta22 = atan2(S, r)
    D2 = (L2**2 + L25**2 - L35**2) / (2*L2*L25)
    theta21 = atan2(sqrt(1-D2**2), D2)
    theta2 = pi/2 - (theta21 + theta22)
    theta2 = theta2.evalf(subs=s)
```

##### Step3: find joint variables, theta4, theta5 and theta6.
last three joint angles are a set of Euler angles corresponsing to a rotation matrix between joint3 to joint6.
Therefore, the angles are obtained by folloing equation.

![alt text][image8]

Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:
`R0_6 = Rrpy`
where, Rrpy = Homogeneous RPY rotation between base_link and gripper_link

```python
    RHS = R0_3_inv * Rrpy
    r11, r12, r13 = RHS[0,0], RHS[0,1], RHS[0,2]
    r21, r22, r23 = RHS[1,0], RHS[1,1], RHS[1,2]
    r31, r32, r33 = RHS[2,0], RHS[2,1], RHS[2,2]

    # # Euler angles from rotation matrix
    theta5 = atan2(sqrt(r13**2 + r33**2), r23)
    theta4 = atan2(r33, -r13)
    theta6 = atan2(-r22, r21)
    theta5 = theta5.evalf(subs={q1: theta1, q2: theta2, q3: theta3 })
    theta4 = theta4.evalf(subs={q1: theta1, q2: theta2, q3: theta3 })
    theta6 = theta6.evalf(subs={q1: theta1, q2: theta2, q3: theta3 })
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

we have equations describing all six joint variables, next we will turn our kinematic analysis into a ROS python node which will perform the Inverse Kinematics for the pick and place operation.

My `IK_server.py` contains basically all the code that I explained above sections. Using the code, the robot can successfully complet pick and place cycles. My first attempt took much time to calculate IK process to find joint angles. Then, I moved processing which is needed only once to outside of trajectory calculation loop, and get faster. However, I think more speed up is needed further. 

