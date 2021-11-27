#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *
import math
import numpy.linalg as la
import scipy

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def change_s_formula(omega,v):
    screw=np.matrix([[0,-omega[2],omega[1],v[0]],[omega[2],0,-omega[0],v[1]],[-omega[1],omega[0],0,v[2]],[0,0,0,0]])
    return screw

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    omega1 = np.array([0,0,1])
    q1 = np.array([-150,150,10])
    v1 = -np.cross(omega1,q1)
    print(v1[0])

    omega2 = np.array([0,1,0])
    q2 = q1 + [0,120,152]
    v2 = -np.cross(omega2,q2)

    omega3 = np.array([0,1,0])
    q3 = q2 + [244,0,0]
    v3 = -np.cross(omega3,q3)

    omega4 = np.array([0,1,0])
    q4 = q3 + [213,-93,0]
    v4 = -np.cross(omega4,q4)

    omega5 = np.array([1,0,0])
    q5 = q4 + [0,83,0]
    v5 = -np.cross(omega5,q5)

    omega6 = np.array([0,1,0])
    q6 = q5 + [83,0,0]
    v6 = -np.cross(omega6,q6)


    S = []
    S.append(change_s_formula(omega1,v1))
    S.append(change_s_formula(omega2,v2))
    S.append(change_s_formula(omega3,v3))
    S.append(change_s_formula(omega4,v4))
    S.append(change_s_formula(omega5,v5))
    S.append(change_s_formula(omega6,v6))

    M = np.array([[0,-1,0,390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]])
    print(S[0])



	
	# ==============================================================#
    return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
    return_value = [None, None, None, None, None, None]

    print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])

    M,S = Get_MS()
    T = np.identity(4)
    for i in range(6):
        T = np.matmul(T,scipy.linalg.expm(S[i]*theta[i]))
    T = np.matmul(T,M)
    print("T",T)

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    print("thetas",return_value)

    return return_value



	# ==============================================================#

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

    yaw = yaw_WgripDegree*(2*np.pi)/360

    xgrip = xWgrip + 150
    ygrip = yWgrip - 150
    zgrip = zWgrip - 10

    xcen = xgrip-53.5*np.cos(yaw)
    ycen = ygrip-53.5*np.sin(yaw)
    zcen = zgrip

    thetadot1 = math.atan2(ycen,xcen)
    thetadot2 = math.atan2(110,np.sqrt(xcen**2+ycen**2-110**2))
    theta1 = thetadot1 - thetadot2

    theta6 = theta1 + np.pi/2 - yaw
    theta5 = -np.pi/2

    x3end = xcen - 83*np.cos(theta1) + (83+27)*np.sin(theta1)
    y3end = ycen - 83*np.sin(theta1) - (83+27)*np.cos(theta1)
    z3end = zcen + 82 + 59

    x1cen = 0
    y1cen = 0
    z1cen = 152

    r = np.array([x3end-x1cen,y3end-y1cen,z3end-z1cen])

    L13 = la.norm(r,ord = 2)

    theta3 = np.pi-np.arccos((244**2+213**2-L13**2)/(2*244*213))

    theta2 = - (np.arccos((244**2+L13**2-213**2)/(2*244*L13))+np.arcsin((z3end-z1cen)/L13))

    theta4 = -theta2-theta3

    return_value = lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
    print(theta1, theta2, theta3, theta4, theta5, theta6)




	# ==============================================================#
    return return_value


