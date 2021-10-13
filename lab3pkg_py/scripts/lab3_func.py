#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	s1 = [0,0,1,0.15,0.15,0]
	s2 = [0,1,0,-0.162,0,-0.15]
	s3 = [0,1,0,-0.162,0,0.094]
	s4 = [0,1,0,-0.162,0,0.307]
	s5 = [1,0,0,0,0.162,-0.26]
	s6 = [0,1,0,-0.162,0,0.39]

	S = [s1,s2,s3,s4,s5,s6]

	M = [[0,-1,0,0.39],[0,0,-1,0.401],[1,0,0,0.2155],[0,0,0,1]]


	# ==============================================================#
	return M, S



def get_bracket_S(S):

	w3 = S[2]

	w2 = S[1]

	w1 = S[0]




	v3 = S[5]

	v2 = S[4]

	v1 = S[3]




	bracket_S = np.array([

	[0,-w3,w2,v1], 

	[w3,0,-w1,v2], 

	[-w2,w1,0,v3], 

	[0,0,0,0]

	])




	return bracket_S



"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	M,S = Get_MS()


	bracket_s = []

	for i in range(len(S)):

		bracket_s.append( get_bracket_S(S[i]) )

	bracket_s = np.array(bracket_s)

	T = np.matmul( expm(bracket_s[0]*theta1) , expm(bracket_s[1]*theta2) )
	T = np.matmul( T , expm(bracket_s[2]*theta3) ) 
	T = np.matmul( T, expm(bracket_s[3]*theta4) )
	T = np.matmul( T, expm(bracket_s[4]*theta5) )
	T = np.matmul( T, expm(bracket_s[5]*theta6) )

	T = np.matmul(T , M)




	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
