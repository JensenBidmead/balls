#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	M[0,0:3] = np.array([0,-1,0])
	M[1,0:3] = np.array([0,0,1])
	M[2,0:3] = np.array([-1,0,0])
	M[0,0:3] = np.array([0,-1,0])
	M[1,0:3] = np.array([0,0,-1])
	M[2,0:3] = np.array([1,0,0])
	M[2,3] = 152 + 53.5 + 10
	M[1,3] = 223 + 59 + 150
	M[0,3] = 542 -150
	S = np.zeros((6,6))
	S = np.array([[0,0,0,0,1,0],
				[0,1,1,1,0,1],
				[1,0,0,0,0,0],
				[150,-152-10,-152-10,-152-10,0,-152-10],
				[150,0,0,0,152+10,0],
				[0,-150,244-150,457-150,-131-150,542-150]])
 

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta =np.array([theta1, theta2, theta3, theta4, theta5, theta6])
	M, S = Get_MS();
	matrixS = np.zeros((6,4,4))
	for i in range (0, 6):
		matrixS[i][0,1] = -S[2,i]
		matrixS[i][0,2] = S[1,i]
		matrixS[i][1,2] = -S[0,i]
		matrixS[i][1,0] = S[2,i]
		matrixS[i][2,0] = -S[1,i]
		matrixS[i][2,1] = S[0,i]
		matrixS[i][0,3] = S[3,i]
		matrixS[i][1,3] = S[4,i]
		matrixS[i][2,3] = S[5,i]
		matrixS[i] = matrixS[i]*theta[i]
		
		matrixS[i] = expm(matrixS[i])
	T = M
	#print(M)
	for i in range(0,6) :
		print(matrixS[5-i])
		T = matrixS[5-i]@T 
		print(str(T) + "\n")
	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	yaw_WgripDegree = yaw_WgripDegree
	g = np.array([xWgrip + 150, yWgrip - 150,zWgrip - 10])
	print(g)
	center = g - np.array([53.5*np.cos(yaw_WgripDegree/180*np.pi), 53.5*np.sin(yaw_WgripDegree/180*np.pi),0])
	print(center)
	theta1 = np.arctan2(center[1],center[0]) - np.arcsin((27 + 104)/np.sqrt(center[1]**2 + center[0]**2))
	
	end = center + np.array([np.sqrt((27 + 104)**2 + 85**2)*np.sin(theta1-np.arctan(85/(27+104))),-np.sqrt((27 + 104)**2 + 85**2)*np.cos(theta1-np.arctan(85/(27+104))), 59 +92])
	print(end)
	d2 = end[0]**2 + end[1]**2
	theta3 = np.pi - np.arccos((d2 + (end[2]-152)**2 - 244**2 -213**2)/(-2*213*244))
	theta2 = -np.arctan2(end[2]-152,np.sqrt(d2)) - np.arccos((213**2 - 244**2 - (d2 + (end[2]-152)**2 ))/(-2*244*np.sqrt(d2 + (end[2]-152)**2)))
	theta4 = -theta2 - theta3
	theta6 = np.pi/2 + theta1 - yaw_WgripDegree/180*np.pi
	theta5 = -np.pi/2


	print(theta1 * 180/np.pi)
	print(theta2 * 180/np.pi)
	print(theta3 * 180/np.pi)
	print(theta4 * 180/np.pi)
	print(theta5 * 180/np.pi)
	print(theta6 * 180/np.pi)
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
