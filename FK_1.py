import numpy as np
from numpy import array
from sympy import symbols,cos,sin,pi,simplify,sqrt,atan2,var
from sympy.matrices import Matrix

###joint variables
q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
b0,b1,b2,b3,b4,b5,b6 = symbols('b0:7')

# th, ap, a, d = symbols('th ap a d')


#DH parameters
# s = {b0:        0,          a0:        0,          d1:      0.75,
#      b1:    -pi/2,          a1:     0.35,          d2:         0,      q2:q2-pi/2,
#      b2:        0,          a2:     1.25,          d3:         0,
#      b3:    -pi/2,          a3:   -0.054,          d4:       1.5,
#      b4:     pi/2,          a4:        0,          d5:         0,
#      b5:    -pi/2,          a5:        0,          d6:         0,
#      b6:        0,          a6:        0,          d7:     0.303,      q7:0
#      }
s = {b0:        0,          a0:        0,          d1:      0.33,
     b1:        0,          a1:     0.35,          d2:      0.42,      q2:q2-pi/2,
     b2:        0,          a2:        0,          d3:      1.25,
     b3:        0,          a3:     0.96,          d4:    -0.054,
     b4:        0,          a4:     0.54,          d5:         0,
     b5:        0,          a5:    0.193,          d6:     0.001,
     b6:        0,          a6:     0.11,          d7:         0,      q7:0
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

R_z = Matrix([[ cos(pi), -sin(pi), 0,0],
              [ sin(pi), cos(pi), 0,0],
              [ 0, 0, 1,0],
              [0, 0, 0, 1]])
R_y = Matrix([[cos(-pi/2), 0,sin(-pi/2),0],
              [ 0,1, 0,0],
              [-sin(-pi/2), 0, cos(-pi/2),0],
              [0, 0, 0, 1]])

R_corr = simplify(R_z * R_y)

# T_total = simplify(T0_G * R_corr)

T_total = simplify(T0_G)

T_n =  T0_1 * T1_2.evalf(subs={q2:pi/2}) * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

print(T_n[0:3,3:4])

# (cos(q2) , 0       , sin(q2), 1.196*sin(q2) + 1.803*cos(q2) + 0.35)
# (0       , 1       , 0      , 0)
# (-sin(q2), 0       , cos(q2), -1.803*sin(q2) + 1.196*cos(q2) + 0.75)
# (0       , 0       , 0      , 1)

# Calculating Spherical Wrist Center (Origin 4,5,6)

#Forward Kinematics
wrist_center = simplify(T0_1 * T1_2 * T2_3 * T3_4)*Matrix([[0],[0],[0],[1]])
wrist_center = wrist_center.evalf(subs={q2:0})
#print(wrist_center)

#Inverse Kinematics
R0_6 = T_total[0:3,0:3]
p = T_total[0:3,3:4]
i = Matrix([[0],[0],[1]])
wrist_center = p-s[d7]*R0_6*i
wrist_center = wrist_center.evalf(subs={q2:0})
#print(wrist_center)


# Calculating End Effector Position
end_effector = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)*Matrix([[0],[0],[0],[1]])
end_effector = end_effector.evalf(subs={q2:0})
#print(end_effector)

# End Effector Relative to Wrist Center
# ee_wrt_wc = end_effector-wrist_center
#print(ee_wrt_wc)
