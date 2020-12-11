import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3, joint4]
    # ---------------------------------------------------------------------
    # Description:
    # Once Working and StillWorking are True, we have computed the first feasible inverse kinematics solution, 
    # although there are many solutions to any given position in the workspace, this will compute one of them.
    # If we iterate through all values [0-1.2]m of possible values for joint 3 and Working and StillWorking are 
    # False, then there are no solutions for our robot arm.
    
    L = 1
    x = position[0]
    y = position[1]
    z = position[2]
    #We assume joint3 is zero, if we cannot reach the desired position then joint3 += 0.1 meters
    joint1 = 0
    joint2 = 0
    joint3 = 0
    joint4 = 0
    Working = False #flag indicating if ValueError occurs due to betaB
    StillWorking = False #flag indicating if ValueError occurs due to alpha

    #joint1:
    joint1 = atan2(x,y)
    #joint2:
    Larm = sqrt(x**2+y**2)
    EBa = z - 3*L
    HBa = sqrt(Larm**2+EBa**2)

    while not Working and not StillWorking: #continues until there Working and StillWorking
        try:
            betaB = acos(((2*L+joint3)**2 + HBa**2 - L**2) / (2*(2*L+joint3)*HBa)) 
            betaA = atan2(EBa,Larm)
            joint2 = betaB + betaA
            Working = True
        except ValueError:
            joint3 += 0.1
            Working = False
            if joint3 >= 1.2:
                print("Desired position is outside of the workspace, no solution")
                break
        #joint4:
        try: 
            alpha = acos(((2*L+joint3)**2 + L**2 - HBa**2) / (2*(2*L+joint3)*L))
            joint4 = pi - alpha
            StillWorking = True
        except ValueError:
            joint3 += 0.1
            StillWorking = False
            if joint3 >= 1.2:
                print("Desired position is outside of the workspace, no solution")
                break

    if Working or StillWorking:
        print [joint1, joint2, joint3, joint4]
    return [joint1, joint2, joint3, joint4]

#test case #1: reaching straight up, actuation of joint2 (elbow) and joint3 (prismatic)
position = [0,0,7]
inverse_kinematics(position)
#test case #2: prismatic actuation, joint3
position = [0,4,3]
inverse_kinematics(position)
#test case #3: actuation of joint2 and joint4
position = [0,3,4]
inverse_kinematics(position)
#test case #4: reaching to ground
position = [0,0,0]
inverse_kinematics(position)
#test case #5: out of workspace 
position = [0,1000,0]
inverse_kinematics(position)
