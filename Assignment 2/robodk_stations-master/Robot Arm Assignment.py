from time import sleep
from robodk.robolink import *
from ClassActions import InitialiseSimulate, RancilioToMazzerScale
import tools
RDK = Robolink()
tls = tools.Tools(RDK)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

InitialiseSimulate()
RancilioToMazzerScale()
