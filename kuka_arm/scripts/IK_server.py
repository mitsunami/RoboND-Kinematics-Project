#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Define DH param symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 


# Joint angle symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i


# Modified DH params
s = {alpha0:     0, a0:      0, d1:  0.75, 
     alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2, 
     alpha2:     0, a2:   1.25, d3:     0, 
     alpha3: -pi/2, a3: -0.054, d4:  1.50, 
     alpha4:  pi/2, a4:      0, d5:     0, 
     alpha5: -pi/2, a5:      0, d6:     0, 
     alpha6:     0, a6:      0, d7: 0.303, q7: 0}


# Define Modified DH Transformation matrix
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


# Create individual transformation matrices
#T0_2 = simplify(T0_1 * T1_2) # base_link to link2
#T0_3 = simplify(T0_2 * T2_3) # base_link to link3
#T0_4 = simplify(T0_3 * T3_4) # base_link to link4
#T0_5 = simplify(T0_4 * T4_5) # base_link to link5
#T0_6 = simplify(T0_5 * T5_6) # base_link to link6
#T0_G = simplify(T0_6 * T6_G) # base_link to linkG
T0_3 = simplify(T0_1 * T1_2 * T2_3) # base_link to link3
T0_6 = simplify(T0_3 * T3_4 * T4_5 * T5_6) # base_link to link6
T0_G = simplify(T0_6 * T6_G) # base_link to linkG


# Correction Needed to Account of Orientation Difference Between Definition of 
# Gripper_Link in URDF versus DH Convention
R_z = Matrix([[     cos(pi), -sin(pi),          0,   0],
              [     sin(pi),  cos(pi),          0,   0],
              [           0,        0,          1,   0],
              [           0,        0,          0,   1]])
R_y = Matrix([[  cos(-pi/2),        0, sin(-pi/2),   0],
              [           0,        1,          0,   0],
              [ -sin(-pi/2),        0, cos(-pi/2),   0],
              [           0,        0,          0,   1]])
R_corr = simplify(R_z * R_y)


# Total Homogeneous Transform Between Base_link and Gripper_link with
# Orientation Correction Applied
T_total = simplify(T0_G * R_corr)

R0_6 = T0_6[0:3, 0:3]
R0_3 = T0_3[0:3, 0:3]
R0_3_inv = R0_3 ** -1



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
		    # calculate wrist center position
            print "\n[", x, "/", len(req.poses), "]"
            print "(px, py, pz, roll, pitch, yaw) = ", px, py, pz, roll, pitch, yaw
            Rrpy = Matrix([[     cos(pitch)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw),  cos(roll)*sin(pitch)+cos(yaw)*sin(roll)*sin(yaw)],
                           [     cos(pitch)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw)],
                           [             -sin(pitch),                             sin(roll)*cos(pitch),                              cos(roll)*cos(pitch)]])

            Rrpy = Rrpy * R_corr[0:3,0:3]

            wx = px - (d6+d7) * Rrpy[0,2]
            wy = py - (d6+d7) * Rrpy[1,2]
            wz = pz - (d6+d7) * Rrpy[2,2]

            theta1 = atan2(wy, wx)

            L2 = a2               # arm length of joint2
            L35 = sqrt(a3**2 +d4**2) # arm length of joint3
            theta31 = atan2(-a3, d4) # arm angle of joint3
            r = sqrt(wx**2 + wy**2) - a1 # distance between joint1 and a point projected xy-plane from the wrist center
            S = wz - d1 # distance along z-axis between joint1 and the wrist center
            L25 = sqrt(r**2 + S**2)
            D = (L25**2 - L2**2 - L35**2) / (2*L2*L35)
            theta32 = atan2(sqrt(1-D**2), D)

            theta3 = theta32 - theta31 - pi/2


            theta22 = atan2(S, r)
            D2 = (L2**2 + L25**2 - L35**2) / (2*L2*L25)
            theta21 = atan2(sqrt(1-D2**2), D2)
            theta2 = pi/2 - (theta21 + theta22)
            #print "(L2, L35, r, S, D, theta32) = ", L2.evalf(subs=s), L35.evalf(subs=s), r.evalf(subs=s), S.evalf(subs=s), D.evalf(subs=s), theta32.evalf(subs=s)
            #print "theta21, 22 = ", theta21.evalf(subs=s), theta22.evalf(subs=s)

            theta1 = theta1.evalf(subs=s)
            theta3 = theta3.evalf(subs=s)
            theta2 = theta2.evalf(subs=s)

            RHS = R0_3_inv * Rrpy
            r11 = RHS[0,0]
            r12 = RHS[0,1]
            r13 = RHS[0,2]
            r21 = RHS[1,0]
            r22 = RHS[1,1]
            r23 = RHS[1,2]
            r31 = RHS[2,0]
            r32 = RHS[2,1]
            r33 = RHS[2,2]

            # # Euler angles from rotation matrix
            theta5 = atan2(sqrt(r13**2 + r33**2), r23)
            theta4 = atan2(r33, -r13)
            theta6 = atan2(-r22, r21)
            theta5 = theta5.evalf(subs={q1: theta1, q2: theta2, q3: theta3 })
            theta4 = theta4.evalf(subs={q1: theta1, q2: theta2, q3: theta3 })
            theta6 = theta6.evalf(subs={q1: theta1, q2: theta2, q3: theta3 })

            print "(theta1, theta2, theta3, theta4, theta5, theta6) = ", theta1, theta2, theta3, theta4, theta5, theta6

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()