from time import sleep
from robodk.robolink import *
import tools

RDK = Robolink()

RDK.setRunMode(RUNMODE_SIMULATE)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
tls = tools.Tools(RDK)
tls.cup_tool_attach_r_ati()
time.sleep(2)
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
time.sleep(2)
tls.cup_tool_open_ur5()
time.sleep(2)
tls.cup_tool_shut_ur5()
time.sleep(2)
tls.cup_tool_detach_r_ati()
time.sleep(2)
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)