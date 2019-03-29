import numpy as np
from numpy import array
from sympy import symbols,cos,sin,pi,simplify,sqrt,atan2
from sympy.matrices import Matrix

###joint variables
q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
b0,b1,b2,b3,b4,b5,b6 = symbols('b0:7')

#DH parameters
s = {b0:0,     a0:0,      d1:0.33+0.42,q1:,
     b1:-pi/2, a1:0.35,   d2:0,        q2:,
     b2:0,     a2:1.25,   d3:0,        q3:,
     b3:-pi/2, a3:0.054,  d4:0.96,     q4:,
     b4:pi/2,  a4:0,      d5:0.54,     q5:,
     b5:-pi/2, a5:0,      d6:0.193,    q6:,
     b6:0,     a6:0,      d7:0.11,     q7:}
