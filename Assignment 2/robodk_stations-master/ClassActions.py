from time import sleep
from robodk.robolink import *
import tools
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
import robodk.robomath as rm
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

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

    # define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
    angle = np.radians(30)
    # define an HT for a pose in the world frame, near the cup dispenser. This could be something calculated or copied from the RoboDK GUI
    T_nearcupdispenser_np = np.array([[ np.cos(angle),     -np.sin(angle),        0,  50 ],
                                    [  np.sin(angle),       np.cos(angle),        0,  50],
                                    [     0,              0,            1,              50],
                                    [  0.000000,     0.000000,     0.000000,     1.000000 ]])

    # convert numpy array into an RDK matrix
    T_nearcupdispenser = rm.Mat(T_nearcupdispenser_np.tolist())

    # reset the sim
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

    UR5.MoveL(T_nearcupdispenser)
    

    # go back home
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)


# example 4x4 matrix 
#np.array([[      0,        0,                0,         x ],
#          [      0,            0,            0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])


