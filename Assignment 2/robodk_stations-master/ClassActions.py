from time import sleep
from robodk.robolink import *
import tools
RDK = Robolink()
tls = tools.Tools(RDK)
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
    tls.rancilio_tool_attach_r_ati()
    

# example 4x4 matrix 
#np.array([[ cos(theta),     -sin(theta),     0,         x ],
#          [  sin(theta),     cos(theta),     0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])


