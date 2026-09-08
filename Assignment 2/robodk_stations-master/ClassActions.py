from time import sleep
from robodk.robolink import *
import tools
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
import robodk.robomath as rm
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

def Rotational_matrix(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0,0],
               [np.sin(theta), np.cos(theta), 0,0],
               [0, 0, 1, 0],
               [0, 0, 0, 0]])
def Translation_matrix(x,y,z):
    return np.array([[0, 0, 0, z],
                  [0, 0, 0,y],
                  [0, 0, 0, z],
                  [0, 0, 0, 1]])
class InitialiseSimulate:
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
class RancilioToMazzerScale:
    tls.rancilio_tool_attach_r_ati()
    theta = -2.0934094900519744 #theta calculated
    R = Rotational_matrix(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T

    R1 = Rotational_matrix(0)
    T1 = Translation_matrix(-12.1,-19, 14.5)
    MStMSBB_np = R1 + T1

    R2 = Rotational_matrix(0)
    T2 = Translation_matrix(-32,0, 28.07)
    RTtRTBB_np = R2 + T2
    

    R3 = Rotational_matrix(np.pi/180*-50)
    T3 = Translation_matrix(0, 0, 0)
    TCPtRT_np = R3 + T3

    RTBBtMSBB_np = np.array([[1,0,0,0],
                          [0,1,0,0],
                          [0,0,1,0],
                          [0,0,0,1]])

    URtTCP = URtMS_np @  MStMSBB_np @ 




# example 4x4 matrix 
#np.array([[      0,        0,                0,         x ],
#          [      0,            0,            0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])


