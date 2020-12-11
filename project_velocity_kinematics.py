import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos
import modern_robotics as mr

def velocity_kinematics(force):
    # input: [x,y,z,fx,fy,fz] where x,y,z is the position of the end-effector and fx,fy,fz is the 
    #        desired force at the end-effector
    # output: joint torques [joint1, joint2, joint3, joint4] required to apply desired force
    # --------------------------------------------------------------------------------------
    # Description:
    # We find the joints vector in the same way as with the inverse kinematics script.
    # This script computes the moment and takes the transpose of our derived Space Jacobian then 
    # it computes the joint torques.

    L = 1
    x = force[0]
    y = force[1]
    z = force[2]
    rT = np.transpose([x,y,z]) 
    f = force[3:6:1] 
    fT = np.transpose(f)
    moment = np.cross(rT,fT) 
    F = np.array([ [moment[0]], [moment[1]], [moment[2]], [f[0]], [f[1]], [f[2]] ])

    #similar to inverse kinematics, we assume joint3 is zero, 
    #if we cannot reach the desired position, joint3 += 0.1 meters
    joint1 = 0
    joint2 = 0
    joint3 = 0
    joint4 = 0
    Working = False 
    StillWorking = False 

    #joint1:
    joint1 = atan2(x,y)
    #joint2:
    Larm = sqrt(x**2+y**2)
    EBa = z - 3*L
    HBa = sqrt(Larm**2+EBa**2)
    while not Working and not StillWorking: 
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

    #Space Jacobian - derived analytically 
    vx = -sin(joint1)*((2*L+joint3)*sin(joint2)+3*L)
    vy = cos(joint1)*((2*L+joint3)*sin(joint2)+3*L)
    vz = -cos(joint1)*((2*L+joint3)*cos(joint1)*cos(joint2))-sin(joint1)*((2*L+joint3)*sin(joint1)*cos(joint2))
    Js_bracket = np.array([[0,cos(joint1),0,cos(joint1)],[0,sin(joint1),0,sin(joint1)], [1,0,0,0], [0,-sin(joint1)*3*L,sin(joint1)*cos(joint2),vx],[0,cos(joint1)*3*L,cos(joint1)*cos(joint2),vy],[0,0,sin(joint2),vz]])
    JsT = np.transpose(Js_bracket)
    #torque = [Js]^T*F^T
    torque = np.dot(JsT,F)
    if Working or StillWorking:
        print(torque)
    return torque

#test case #1: pulling a fruit from a resting position
force = [0,4,3,0,0,1] #expected to pull up with joint2 and joint4
velocity_kinematics(force) 
#test case #2: pulling a fruit from resting position
force = [0,4,3,0,-1,0] #expected to pull away with joint3 (prismatic)
velocity_kinematics(force)
#test case #3: pulling a fruit from above
force = [0,3,4,0,-1,-1] #expected to pull down with joint2 and joint4
velocity_kinematics(force)
#test case #4: twisting arm base (joint1)
force = [0,4,3,-1,0,0] 
velocity_kinematics(force)
#test case #4: invalid position
force = [0,100,3,-1,0,0] 
velocity_kinematics(force)