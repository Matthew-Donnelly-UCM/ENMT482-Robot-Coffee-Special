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
class InitialiseSimulate:
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

class HomeToMazzerScaleTop:
    tls.rancilio_tool_attach_r_ati()
    theta = -60*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T
    
    R1 = Rotational_matrix_z(0)
    T1 = Translation_matrix(-12.1,-19, 14.5)
    MStMSBB_np = R1 + T1

    R2 = Rotational_matrix_z(0)
    T2 = Translation_matrix(0,0, 20)
    MSBBtMSBBT_np = R2 + T2

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(-32,0, 28.07)
    RTtRTBB_np = R3 + T3
    
    #inverse matrix
    R3_inv = Rotational_matrix_z_inverse_z(0)
    T3_inv = Translation_matrix_inverse(-32,0, 28.07)
    RTtRTBB_inv_np = Inverse_transform(R3_inv, T3_inv)
        
    R4 = Rotational_matrix_z((np.pi/180)*-50)
    T4 = Translation_matrix(0, 0, 0)
    TCPtRT_np = R4 + T4
        
        #inverse matrix
    R4_inv = Rotational_matrix_z_inverse_z((np.pi/180)*-50)
    T4_inv = Translation_matrix_inverse(0,0,0)
    TCPtRT_inv_np = Inverse_transform(R4_inv,T4_inv)
    
    RTBBtMSBBT_np = np.array([[0,0,1,0],
                              [0,-1,0,0],
                              [1,0,0,0],
                              [0,0,0,1]])
    
       #inverse matrix simple
    RTBBtMSBBT_inv_np = RTBBtMSBBT_np
    
    URtTCP = URtMS_np @  MStMSBB_np @ MSBBtMSBBT_np @ RTBBtMSBBT_inv_np  @ RTtRTBB_inv_np @ TCPtRT_inv_np
    
    T_URtTCP = rm.Mat(URtTCP.tolist())
    
    # convert numpy array into an RDK matrix
    
    
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
    
    UR5.MoveJ(T_URtTCP, blocking=True)
class MazzerScaleTopToMazzerScale:
    
    #theta = -2.0934094900519744 #theta calculated
    theta = -60*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T

    R1 = Rotational_matrix_z(0)
    T1 = Translation_matrix(-12.1,-19, 14.5)
    MStMSBB_np = R1 + T1

    R2 = Rotational_matrix_z(0)
    T2 = Translation_matrix(-32,0, 28.07)
    RTtRTBB_np = R2 + T2

    #inverse matrix
    R2_inv = Rotational_matrix_z_inverse_z(0)
    T2_inv = Translation_matrix_inverse(-32,0, 28.07)
    RTtRTBB_inv_np = Inverse_transform(R2_inv, T2_inv)
    
    R3 = Rotational_matrix_z((np.pi/180)*-50)
    T3 = Translation_matrix(0, 0, 0)
    TCPtRT_np = R3 + T3
    
    #inverse matrix
    R3_inv = Rotational_matrix_z_inverse_z((np.pi/180)*-50)
    T3_inv = Translation_matrix_inverse(0,0,0)
    TCPtRT_inv_np = Inverse_transform(R3_inv,T3_inv)

    RTBBtMSBB_np = np.array([[0,0,1,0],
                          [0,-1,0,0],
                          [1,0,0,0],
                          [0,0,0,1]])

   #inverse matrix simple
    RTBBtMSBB_inv_np = RTBBtMSBB_np

    URtTCP = URtMS_np @  MStMSBB_np @ RTBBtMSBB_inv_np  @ RTtRTBB_inv_np @ TCPtRT_inv_np

    T_URtTCP = rm.Mat(URtTCP.tolist())

    # convert numpy array into an RDK matrix


    UR5.MoveL(T_URtTCP, blocking=True)

    
    tls.student_tool_detach()

class MazzerScaleToHome:
    theta = -60*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T
    
    R1 = Rotational_matrix_z(0)
    T1 = Translation_matrix(-12.1,-19, 14.5)
    MStMSBB_np = R1 + T1

    R2 = Rotational_matrix_z(0)
    T2 = Translation_matrix(-40,0, 32)
    MSBBtMSRP_np = R2 + T2

    R3 = Rotational_matrix_z((np.pi/180)*-50)
    T3 = Translation_matrix(0, 0, 0)
    TCPtTCPF_np = R3 + T3

    #inverse matrix
    R3_inv = Rotational_matrix_z_inverse_z((np.pi/180)*-50)
    T3_inv = Translation_matrix_inverse(0,0,0)
    TCPtTCPF_inv_np = Inverse_transform(R3_inv,T3_inv)

    R4 = np.array([[0,0,1,0],
                   [0,-1,0,0],
                   [1,0,0,0],
                   [0,0,0,0]])
    T4 = Translation_matrix(0, 0, 0)
    TCPFtMSRP = R4 + T4

    #inverse matrix
    R4_3x3 = np.array([[0,0,1,],
                   [0,-1,0,],
                   [1,0,0,]])
    R4_inv = np.transpose(R4_3x3)
    T4_inv = Translation_matrix_inverse(0,0,0)
    TCPFtMSRP_inv_np = Inverse_transform(R4_inv,T4_inv)

    URtTCP = URtMS_np @  MStMSBB_np @ MSBBtMSRP_np  @ TCPFtMSRP_inv_np @ TCPtTCPF_inv_np

    T_URtTCP = rm.Mat(URtTCP.tolist())

    Detach_program = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    Detach_program.RunCode()
    Detach_program.WaitFinished()

    UR5.MoveL(T_URtTCP, blocking=True)

    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

class HomeToMazzerTool:
    #Getting rid of the tool just from simulation, remove afterwards
    tls.rancilio_tool_detach_r_ati()

    tls.mazzer_tool_attach_r_ati()

    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

    #Maticies to solve for location 
    theta = -60*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T

    R1 = Rotational_matrix_x(-50)
    T1 = Translation_matrix(31.5,-59.53, -15)
    MStMSLLR_np = R1 + T1

    R2 = Rotational_matrix_z((np.pi/180)*-50)
    T2 = Translation_matrix(0, 0, 0)
    TCPtMT_np = R2 + T2

    #inverse
    R2_inv = Rotational_matrix_z_inverse_z((np.pi/180)*-50)
    T2_inv = Translation_matrix_inverse(0,0,0)
    TCPtMT_inv_np = Inverse_transform(R2_inv,T2_inv)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(0, 0, 102.82)
    MTtMTCCT_np = R3 + T3

    #inverse
    R3_inv = Rotational_matrix_z_inverse_z(0)
    T3_inv = Translation_matrix_inverse(0,0,102.82)
    MTtMTCCT_inv_np = Inverse_transform(R3_inv,T3_inv)

    R4 = np.array([[0,0,1,0],
                       [0,-1,0,0],
                       [1,0,0,0],
                       [0,0,0,0]])
    T4 = Translation_matrix(0, 0, 0)
    MTCCTtMSLLR = R4 + T4

    #inverse matrix
    R4_3x3 = np.array([[0,0,1,],
                       [0,-1,0,],
                       [1,0,0,]])
    R4_inv = np.transpose(R4_3x3)
    T4_inv = Translation_matrix_inverse(0,0,0)
    MTCCTtMSLLR_inv_np = Inverse_transform(R4_inv,T4_inv)

    URtTCP = URtMS_np @  MStMSLLR_np @ MTCCTtMSLLR_inv_np @ MTtMTCCT_inv_np @ TCPtMT_inv_np

    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveL(T_URtTCP, blocking=True)

# example 4x4 matrix 
#np.array([[      0,        0,                0,         x ],
#          [      0,            0,            0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])


