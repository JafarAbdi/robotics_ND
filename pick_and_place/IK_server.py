#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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
import numpy as np

def pose_error(pose1,pose2):
	xp = pose2.position.x - pose1.position.x
	yp = pose2.position.y - pose1.position.y
	zp = pose2.position.z - pose1.position.z

	xo = pose2.orientation.x - pose1.orientation.x
	yo = pose2.orientation.y - pose1.orientation.y
	zo = pose2.orientation.z - pose1.orientation.z
	wo = pose2.orientation.w - pose1.orientation.w

	ep = np.sqrt(xp*xp + yp*yp + zp*zp)
	eo = np.sqrt(xo*xo + yo*yo + zo*zo + wo*wo)

	return(ep,eo)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
        d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
        a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
        b0,b1,b2,b3,b4,b5,b6 = symbols('b0:7')
        # Create Modified DH parameters
        s = {b0:        0,          a0:        0,          d1:      0.75,      q1:q1,
             b1:    -pi/2.0,          a1:     0.35,          d2:         0,      q2:q2-pi/2.0,
             b2:        0,          a2:     1.25,          d3:         0,      q3:q3,
             b3:    -pi/2.0,          a3:   -0.054,          d4:       1.5,      q4:q4,
             b4:     pi/2.0,          a4:        0,          d5:         0,      q5:q5,
             b5:    -pi/2.0,          a5:        0,          d6:         0,      q6:q6,
             b6:        0,          a6:        0,          d7:     0.303,      q7:0
             }
        # Define Modified DH Transformation matrix
        def DH_T(ap,a,th,d):

            R_x = Matrix([[       1,       0,        0,        0],
                          [       0, cos(ap), -sin(ap),        0],
                          [       0, sin(ap),  cos(ap),        0],
                          [       0,        0,        0,       1]])

            R_z = Matrix([[ cos(th), -sin(th),        0,       0],
                          [ sin(th),  cos(th),        0,       0],
                          [       0,        0,        1,       0],
                          [       0,        0,        0,       1]])

            TX = Matrix([[1, 0, 0, a],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

            TZ = Matrix([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, d],
                         [0, 0, 0, 1]])

            DH_T = Matrix(R_x* TX * R_z * TZ)

            return DH_T
        # Create individual transformation matrices
        T0_1  = DH_T(b0,a0,q1,d1).subs(s)
        T1_2  = DH_T(b1,a1,q2,d2).subs(s)
        T2_3  = DH_T(b2,a2,q3,d3).subs(s)
        T3_4  = DH_T(b3,a3,q4,d4).subs(s)
        T4_5  = DH_T(b4,a4,q5,d5).subs(s)
        T5_6  = DH_T(b5,a5,q6,d6).subs(s)
        T6_EE = DH_T(b6,a6,q7,d7).subs(s)
        T0_EE  = T0_1 * T1_2  * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
        # Extract rotation matrices from the transformation matrices

        # Initialize service response
        joint_trajectory_list = []

        for x in xrange(0,len(req.poses)):
	    #print("pose",req.poses[x])
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

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_z = Matrix([[    cos(pi), -sin(pi),          0],
                          [    sin(pi),  cos(pi),          0],
                          [          0,        0,          1]])

            R_y = Matrix([[ cos(-pi/2.0),        0, sin(-pi/2.0)],
                          [          0,        1,          0],
                          [ -sin(-pi/2.0),       0, cos(-pi/2.0)]])

            R_corr = R_z * R_y

            #find EE rotation matrix
            r,p,y = symbols('r p y')

            RX = Matrix([[       1.0,      0,       0],
                         [       0, cos(r), -sin(r)],
                         [       0, sin(r),  cos(r)]])

            RZ = Matrix([[  cos(p),-sin(p),       0],
                         [  sin(p), cos(p),       0],
                         [       0,      0,       1]])

            RY = Matrix([[  cos(y),      0,  sin(y)],
                         [       0,      1.0,       0],
                         [ -sin(y),      0,  cos(y)]])

            R_EE = (RZ * RY * RX) * R_corr
            R_EE = R_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

            EE = Matrix([[px],
                         [py],
                         [pz]])

            WC = EE - (0.303 * R_EE[:,2])
            # Calculate joint angles using Geometric IK method
            #THETA1
            theta1 = atan2(WC[1],WC[0])

            #TRIANGLE FOR THETA 1 and 2
            a = 1.501
            b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35) ,2) + pow((WC[2]-0.75), 2))
            c = 1.25

            A = acos((b*b + c*c - a*a)/(2*b*c))
            B = acos((a*a + c*c - b*b)/(2*a*c))
            C = acos((b*b + a*a - c*c)/(2*b*a))

            #THETA2
            theta2 = pi/2 - A - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1])-0.35)

            #THETA3
            theta3 = pi/2 - (B + 0.036)

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3:theta3})
            R3_6 = R0_3.inv("LU")*R_EE
            #print("R3_6")
	   
	    #print(R3_6[0],R3_6[1],R3_6[2])
	    #print(R3_6[3],R3_6[4],R3_6[5])
            #print(R3_6[6],R3_6[7],R3_6[8])

	  
		
	    #for i in range(len(R3_6)):
	    #	if R3_6[i] < 10**(-4):
	    #		R3_6[i] = 0
	    if 0.99 < R3_6[8] < 1.01:
		theta6 = atan2(R3_6[0],R3_6[3])
		theta4 = 0
                theta5 = 0

		print(theta4,theta5,theta6)
            else:
 	
            	theta5 = atan2(R3_6[8],sqrt( 1-R3_6[8]**2 )) 
            	theta4 = atan2(R3_6[2],R3_6[4])
            	theta6 = atan2(-R3_6[6],R3_6[7])
		theta5_2 = atan2(R3_6[8],-sqrt( 1-R3_6[8]**2 )) 
                theta4_2 = atan2(-R3_6[2],-R3_6[4])
                theta6_2 = atan2(R3_6[6],-R3_6[7])
		
                if theta5_2 < theta5:
			theta5=theta5_2
                if theta6_2 < theta6:
                        theta6=theta6_2
                if theta4_2 < theta4:
                        theta4=theta4_2

	    	print('theta4',theta4,theta4_2,"theta5",theta5,theta5_2,"theta6",theta6,theta6_2)


	    #THETA4
            #theta4 = atan2(R3_6[2,2], -R3_6[0,2])

            #THETA5
            #theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])

            #THETA6
            #:theta6 = atan2(-R3_6[1,1], R3_6[1,0])


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		#print(joint_trajectory_list)
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
