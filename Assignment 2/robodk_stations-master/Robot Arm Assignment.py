from time import sleep
from robodk.robolink import *
from ActionA import InitialiseSimulateA, HomeToMazzerScaleTop, MazzerScaleTopToMazzerScale, MazzerScaleToHome
from ActionB import HomeToMazzerScaleLockLeverRightTop, SlideInXDirectionAcrossLock, SlideInzDirectionAcrossLock
from ActionL import ActionL
from ActionO import ActionO
from ActionP import ActionP
from ActionQ import ActionQ
from ActionR import ActionR
from ActionS import ActionS
import tools
import numpy as np

RDK = Robolink()
tls = tools.Tools(RDK)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

#   After creating a `Robolink()` object, items within the RoboDK station tree
#   are able to be retrieved by name, item type, or both.
# Work in simulation mode
RDK.setRunMode(RUNMODE_SIMULATE)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


#Resets Simulation ready to be used
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()


Actions = [
    0,  # A
    0,  # B
    0,  # C
    0,  # D
    0,  # E
    0,  # F
    0,  # G
    0,  # H
    0,  # I
    0,  # J
    0,  # K
    0,  # L
    0,  # M
    0,  # N
    0,  # O
    0,  # P
    0,  # Q
    0,  # R
    1,  # S
    0,  # T
    0,  # U
    0,  # V
]

if Actions[0] == 1:
    #Action A
    HomeToMazzerScaleTop()
    MazzerScaleTopToMazzerScale()
    MazzerScaleToHome()
# tls.rancilio_tool_detach_r_ati()
if Actions[1] == 1:
    #Action B
    HomeToMazzerScaleLockLeverRightTop()
    SlideInXDirectionAcrossLock()
    SlideInzDirectionAcrossLock()
if Actions[11] == 1:
    ActionL()
if Actions[14] == 1:
    ActionO()
if Actions[15] == 1:
    ActionP() # Scale simulation isn't working
if Actions[16] == 1:
    ActionQ()
if Actions[17] == 1:
    ActionR()
if Actions[18] == 1:
    ActionS()