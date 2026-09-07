#*****************************************************************************
#
#   Module:         robodk_basics.py
#   Project:        UC-02-2024 Coffee Cart Modifications
#
#   Repository:     robodk_stations 
#   Target:         N/A 
#
#   Author:         Rodney Elliott
#   Date:           19 June 2025 
#
#   Description:    Example code showing basic use of the RoboDK Python API.
#
#*****************************************************************************
#
#   Copyright:      (C) 2025 Rodney Elliott
#
#   This file is part of Coffee Cart Modifications.
#
#   Coffee Cart Modifications is free software: you can redistribute it and/or
#   modify it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the license, or (at your
#   option) any later version.
#
#   Coffee Cart Modifications is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
#   Public License for more details.
#
#   You should of received a copy of the GNU General Public License along with
#   Coffee Cart Modifications. If not, see <https://www.gnu.org/licenses/>.
#
#*****************************************************************************

#*****************************************************************************
#   Doxygen file documentation
#*****************************************************************************

##
#   @file robodk_basics.py
#
#   @brief Example code showing basic use of the RoboDK Python API.
#
#   __robodk_basics__ provides a basic Python framework that students may use
#   as a starting point for their manipulator assignment.

#*****************************************************************************
#   Modules
#*****************************************************************************

##
#   @package time
#
#   The _time_ package provides various time-related functions. The only one of
#   interest here is `sleep`.
from time import sleep

##
#   @package robolink
#
#   The _robodk_ package is the distributed entry point of the RoboDK Python
#   API. It is the common parent of all sub-packages and modules that
#   constitute the Python API.
#
#   The _robolink_ sub-module is the bridge between RoboDK and Python. Items in
#   the RoboDK station tree (robots, tools, frames, programs, ...) are able to
#   be retrieved using various methods from the `Robolink()` object.
from robodk.robolink import *

##
#   @package tools
#
#   The _tools_ module contains methods to interact with tools and fixtures.
import tools

#*****************************************************************************
#   Code 
#*****************************************************************************

##
#   @brief Create a link between RoboDK and Python.
#
#   After creating a `Robolink()` object, items within the RoboDK station tree
#   are able to be retrieved by name, item type, or both.
RDK = Robolink()

# RoboDK supports a number of run modes.
#
# | Name                              | Value | Description                                                                          |
# | --------------------------------- | ----- |------------------------------------------------------------------------------------- |
# | RUNMODE_SIMULATE                  | 1     | Performs the simulation moving the robot (default)                                   |
# | RUNMODE_QUICKVALIDATE             | 2     | Performs a quick check to validate the robot movements                               |
# | RUNMODE_MAKE_ROBOTPROG            | 3     | Makes the robot program                                                              |
# | RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD | 4     | Makes the robot program and updates it to the robot                                  |
# | RUNMODE_MAKE_ROBOTPROG_AND_START  | 5     | Makes the robot program and starts it on the robot (independently from the PC)        |
# | RUNMODE_RUN_ROBOT                 | 6     | Moves the real robot from the PC (PC is the client, the robot behaves like a server) |
#
# Of these, only `RUNMODE_SIMULATE` and `RUNMODE_RUN_ROBOT` are currently
# supported, with the former being referred to in the UC-02-2024 documentation
# as running _offline_ (simulation only), and the later as _online_ (motion of
# virtual and physical robots synchronised).
RDK.setRunMode(RUNMODE_SIMULATE)

##
#   @brief Get the robot item by name.
#
#   Note that in this and all subsequent calls to the `Item()` method, both the
#   item `name` and item `type` arguments are used. Providing that all item
#   names within the RoboDK station tree are unique, the second argument is
#   optional. However it is used here out of an abundance of caution and to
#   serve as a visual reminder as to the nature of the object that is being
#   referenced.
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

##  Tools class instance.
tls = tools.Tools(RDK)

##
#   @brief Robot program item.
#
#   The first such program item to be run resets the RoboDK simulation,
#   including returning the robot to the _Home_R_ position, restoring correct
#   tool and fixture positions, and so on.
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()

# Attach the cup tool to the robot and remove from the tool stand.
#
# This is the first robot program of the example code that will result in
# movement of the physical robot if called online - that is, when a socket
# connection has been established between the lab computer running RoboDK, and
# the physical robot.
tls.cup_tool_attach_r_ati()

# Wait two seconds.
#
# Note that this, and all subsequent two second delays to program execution
# have been added simply to ensure that each robot movement appears distinct
# when viewed in RoboDK. Such delays should not appear in production code.
time.sleep(2)

# Move to the home position.
#
# There are three types of movements available within RoboDK - circular, joint,
# and linear. Students should use joint moves in their code where possible, as
# joint moves are faster than linear moves, and there are no singularities in
# joint space. There are a handful of tasks that _will_ require the use of
# linear or circular moves, but the majority of robot motions are best
# performed using joint moves.
#
# Again, it is important to note that all RoboDK move commands are, by default,
# non-blocking. In order to pause code execution until robot movement is
# complete, be sure to pass the second blocking argument.
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

# Wait two seconds.
time.sleep(2)

# Open the cup tool.
tls.cup_tool_open_ur5()

# Wait two seconds.
time.sleep(2)

# Close the cup tool.
tls.cup_tool_shut_ur5()

# Wait two seconds.
time.sleep(2)

# Return the cup tool to the tool stand and detach it from the robot.
tls.cup_tool_detach_r_ati()

# Wait two seconds.
time.sleep(2)

# Move back to the home position.
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

