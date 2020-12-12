import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3, joint4]
    # output: the position of end effector [x, y, z]
    
    L = 1
    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]
    joint4 = joints[3]

    if joint3 > 1.2 or joint3 < 0:
        raise ValueError("Invalid value for joint3, must be between 0 and 2 meters")
        
    #precalculated M, [S3], [S2] and [S1]
    M = np.array([[1,0,0,0], [0,1,0,3*L], [0,0,1,3*L], [0,0,0,1]])
    S4_bracket = np.array([[0,0,0,0],[0,0,-1,3*L], [0,1,0,-2*L], [0,0,0,0]])
    S3_bracket = np.array([[0,0,0,0], [0,0,0,1], [0,0,0,0], [0,0,0,0]])
    S2_bracket = np.array([[0,0,0,0], [0,0,-1,3*L], [0,1,0,0], [0,0,0,0]])
    S1_bracket = np.array([[0,-1,0,0], [1,0,0,0], [0,0,0,0], [0,0,0,0]])
    #exponential term
    S4_theta = joint4*S4_bracket
    S3_theta = joint3*S3_bracket
    S2_theta = joint2*S2_bracket
    S1_theta = joint1*S1_bracket
    #matrix exponential
    S4_exp = mr.MatrixExp6(S4_theta)
    S3_exp = mr.MatrixExp6(S3_theta)
    S2_exp = mr.MatrixExp6(S2_theta)
    S1_exp = mr.MatrixExp6(S1_theta)
    #left-multiplying 
    T04 = np.dot(S4_exp,M)
    T03 = np.dot(S3_exp,T04)
    T02 = np.dot(S2_exp,T03)
    T01 = np.dot(S1_exp,T02)
    R, p = mr.TransToRp(T01)
    x = p[0]
    y = p[1]
    z = p[2]
    print [x,y,z]
    return [x, y, z]

#test case #1: reaching straight up
joints = [0,pi/2,1,0]
forward_kinematics(joints)
#test case #2: prismatic actuation
joints = [0,0,1,0]
forward_kinematics(joints)
#test case #3: actuation of joint4 (wrist)
joints = [0,0,1,pi/2]
forward_kinematics(joints)
#test case #4: actuation of joint1 (waist)
joints = [pi/2,0,1,0]
forward_kinematics(joints)
#test case #4: invalid value of joint3
joints = [pi/2,0,3,0]
forward_kinematics(joints)


    