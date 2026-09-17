from time import sleep
from robodk.robolink import *
from ActionA import InitialiseSimulateA, HomeToMazzerScaleTop, MazzerScaleTopToMazzerScale, MazzerScaleToHome
from ActionB import HomeToMazzerScaleLockLeverRightTop, SlideInXDirectionAcrossLock, SlideInzDirectionAcrossLock
from ActionC import home_to_mazzer_button, mazzer_button_pressed_on, mazzer_press, mazzer_wait, mazzer_button_turn_off, mazzer_button_turn_off_pressed
from ActionO import ActionO
#from ActionP import ActionP
from ActionQ import ActionQ
from ActionR import ActionR
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


Actions = [0, 0, 1, 0, 0, 0, 0]

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
if Actions[2] == 1:
    home_to_mazzer_button()
    mazzer_button_pressed_on()
    mazzer_press()
    mazzer_wait()
    mazzer_button_turn_off()
    mazzer_button_turn_off_pressed()
if Actions[3] == 1:
    # Action O
    ActionO()
if Actions[4] == 1:
    #ActionP()
    print("hello")
if Actions[5] == 1:
    ActionQ()
if Actions[6] == 1:
    ActionR()