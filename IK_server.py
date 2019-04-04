

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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        	# Create symbols
		###joint variables
	q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
	d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
	a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
	b0,b1,b2,b3,b4,b5,b6 = symbols('b0:7')
	
	# th, ap, a, d = symbols('th ap a d')
	
	
	#DH parameters
	s = {b0:        0,          a0:        0,          d1:      0.75,
     	b1:    -pi/2,          a1:     0.35,          d2:         0,      q2:q2-pi/2,
     	b2:        0,          a2:     1.25,          d3:         0,
     	b3:    -pi/2,          a3:   -0.054,          d4:       1.5,
     	b4:     pi/2,          a4:        0,          d5:         0,
     	b5:    -pi/2,          a5:        0,          d6:         0,
     	b6:        0,          a6:        0,          d7:     0.303,      q7:0
     	}
	
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
	
	# [cos(th)        ,-sin(th)          ,0         ,a         ]
	# [sin(th)*cos(ap),cos(ap)*cos(th)   ,-sin(ap)  ,-d*sin(ap)]
	# [sin(ap)*sin(th),sin(ap)*cos(th)   ,cos(ap)   ,d*cos(ap) ]
	# [0              ,0                 ,0         ,1         ]
	
	# DH-type transform
	# i-1 to i -> R_x( alpha i-1 )*TX( a i-1 )*R_z( theta i )*TZ( d i )
	
	T0_1 = DH_T(b0,a0,0,d1)
	T0_1 = T0_1.subs(s)
	T1_2 = DH_T(b1,a1,q2,d2)
	T1_2 = T1_2.subs(s)
	T2_3 = DH_T(b2,a2,0,d3)
	T2_3 = T2_3.subs(s)
	T3_4 = DH_T(b3,a3,0,d4)
	T3_4 = T3_4.subs(s)
	T4_5 = DH_T(b4,a4,0,d5)
	T4_5 = T4_5.subs(s)
	T5_6 = DH_T(b5,a5,0,d6)
	T5_6 = T5_6.subs(s)
	T6_G = DH_T(b6,a6,q7,d7)
	T6_G = T6_G.subs(s)
	
	T0_G = simplify(T0_1 * T1_2  * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)
	
	# R_z = Matrix([[ cos(pi), -sin(pi), 0,0],
	#               [ sin(pi), cos(pi), 0,0],
	#               [ 0, 0, 1,0],
	#               [0, 0, 0, 1]])
	# R_y = Matrix([[cos(-pi/2), 0,sin(-pi/2),0],
	#               [ 0,1, 0,0],
	#               [-sin(-pi/2), 0, cos(-pi/2),0],
	#               [0, 0, 0, 1]])
	#
	# R_corr = simplify(R_z * R_y)
	#
	#T_total = simplify(T0_G * R_corr)
	
	T_total = simplify(T0_G)

	# Extract rotation matrices from the transformation matrices
	R0_6 = T_total[0:3,0:3]
	p = T_total[0:3,3:4]

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    px,py,pz = 
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

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
