from time import sleep
from robodk.robolink import *
import tools
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
import robodk.robomath as rm
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

def Rotational_matrix_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0,0],
               [np.sin(theta), np.cos(theta), 0,0],
               [0, 0, 1, 0],
               [0, 0, 0, 0]])

def Rotational_matrix_x(theta):
    return np.array([[1, 0, 0,0],
               [0, np.cos(theta), -np.sin(theta),0],
               [0, np.sin(theta), np.cos(theta), 0],
               [0, 0, 0, 0]])

def Rotational_matrix_z_inverse_z(theta):
    return np.transpose(np.array([[np.cos(theta), -np.sin(theta), 0],
                   [np.sin(theta), np.cos(theta), 0],
                   [0, 0, 1]]))

def Translation_matrix(x,y,z):
    return np.array([[0, 0, 0, x],
                  [0, 0, 0,y],
                  [0, 0, 0, z],
                  [0, 0, 0, 1]])
def Translation_matrix_inverse(x,y,z):
    return np.array([[x],
                     [y],
                     [z]])

def Inverse_transform(R_T,T):
    A = R_T
    B = R_T@(-T)
    Trans_inv = np.identity((4))
    Trans_inv[0:3,0:3] = A
    Trans_inv[0:3,3] = B.ravel()
    print(Trans_inv)
    return(Trans_inv)

# Now for the actual functions
def InitialiseSimulateB():
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

def HomeToMazzerScaleLockLeverRightTop():
    tls.mazzer_tool_attach_r_ati()
    theta = -60*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T

    R1 = Rotational_matrix_z(0,0,0)
    T1 = Translation_matrix(31.5,-59.53, -15)
    MStMSLLRT = R1 + T1

    #This part is getting the tool to be slightly rotated at the point
    theta2 = -45
    R2 = Rotational_matrix_x(theta2)
    T2 = Translation_matrix(31.5,-59.53, -15)
    MSLLRTtMSLLRTR = R2 + T2

    #inverse side matrix
    # this section is the mazzer tool 
    R3= Rotational_matrix_z((np.pi/180)*-50)
    T3= Translation_matrix(0, 0, 0)
    TCPtMT_np = R3 + T3

    #inverse matrix
    R3_inv = Rotational_matrix_z_inverse_z((np.pi/180)*-50)
    T3_inv = Translation_matrix_inverse(0,0,0)
    MTtTCP_np = Inverse_transform(R3_inv,T3_inv)

    R4 = Rotational_matrix_z(0)
    T4= Translation_matrix(0, 0, 102.82)
    MTtMTCCT_np = R4 + T4

    #inverse matrix
    R4_inv = Rotational_matrix_z_inverse_z(0)
    T4_inv = Translation_matrix_inverse(0,0,102.82)
    MTCCTtMT_np = Inverse_transform(R4_inv,T4_inv)

    #this is the cushion tip traslated to the reference frame of the lock

    MTCCTtMSLLR_np = np.array([[1,0,0,0],
                              [0,0,1,0],
                              [0,1,0,0],
                              [0,0,0,1]])
    R5 = np.array([[1,0,0],
                  [0,0,1],
                  [0,1,0]])
    T5_inv = Translation_matrix_inverse(0,0,0)
    R5_inv = np.transpose(R)
    MSLLRtMTCCT_np = Inverse_transform(R5_inv,T5_inv)

    URtTCP = URtMS_np @ MStMSLLRT @ MSLLRtMTCCT_np @ MSLLRtMTCCT_np @ MTCCTtMT_np @ MTtTCP_np

    T_URtTCP = rm.Mat(URtTCP.tolist())

    # convert numpy array into an RDK matrix


    UR5.MoveL(T_URtTCP, blocking=True)



        


# example 4x4 matrix 
#np.array([[      0,        0,                0,         x ],
#          [      0,            0,            0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])


