__Table of Contents__

[[_TOC_]]

# 1. Introduction

__RoboDK Stations__ contains the [RoboDK][robodk] _station files_ that students
are to use as the basis of their assignment. These station files (also known as
_RDK files_) are specific to a particular robot / coffee cart combination. This
repository also contains supporting files that student will need to refer to in
order to successfully complete the assignment.

# 2. Warnings

It is of __UTMOST IMPORTANCE__ that students use the station file corresponding
to the coffee cart apparatus to which they have been assigned. Failure to do so
will result in differences in behaviour when running code offline (that is, in
simulation only) and online (when the motion of both the virtual and physical
robot is synchronised).

This is likely to result in damage to both the robot and tooling due to slight
variations in the location of fixtures across the three carts.

In order to reduce the chances that students will select the incorrect station
file, the name of each cart is prominently displayed in multiple locations on
the cart itself. Station files have then been named after the coffee cart
apparatus to which they pertain.

| Cart Name | Station File     | Robot Details                |
| --------- | -----------------| ---------------------------- |
| Robot 1   | Robot_1_2025.rdk | UR5 serial number 2019350471 |
| Robot 2   | Robot_2_2025.rdk | UR5 serial number 2020350569 |
| Robot 3   | Robot_3_2026.rdk | UR5 serial number 2023350546 |

# 3. Robot Programs

In addition to an accurate model of the coffee cart after which it is named,
each station file also contains a number of so-called _robot programs_.

## 3.1. Tool Programs

The first group of robot programs comprise those that students may use to
perform certain real-world tasks, specifically:

 - Changing robot tools
 - Actuating robot tools
 - Actuating cart fixtures

This group of robot programs are located in the _Tool_Programs_ directory of
the RoboDK station tree.

If called when online - that is, when a TCP/IP socket connection has been
established between the laboratory computer running RoboDK and the physical
robot - these programs will result in synchronous changes to the state of the
coffee cart apparatus in both simulation and the real world. This includes
movement of the robot, actuation of cart fixtures, and so on. If called when
offline, only the state of the simulation will change.

> [!important]
> Tool programs within the RoboDK station tree are not designed to be called
> from Python. Their purpose is to aid students in making the most of their
> allocated time on the robots by providing a means to quickly change tools,
> actuate fixtures, et cetera. Students must use methods from the _tools_ class
> to perform these actions from Python. See the [Tools Class](#5-tools-class)
> section below for details.

## 3.2. Visual Programs

The second group of programs only change the state of the simulation, and have
no real-world effect whether they are called when online or offline. A classic
example of this type of program is the one that may be used to reset the state
of the simulation when things go awry.

This group of programs are located in the _Visual_Programs_ directory of the
RoboDK station tree.

> [!important]
> Despite the fact that the visual programs have no real-world effect, if they
> are to be called from code that is running online, they must have their "Run
> on Robot" property set to true, otherwise RoboDK will throw an error. To set
> the property, right-click on the visual program in the station tree and check
> the "Run on Robot" box.

# 4. Python Programs

__RoboDK Stations__ also contains two example Python programs. These are
intended to provide students with a framework to use as the basis for their own
code, and to serve as an introduction to the RoboDK [Python API][python_api].

These Python programs may be called when online or offline. Note however that
these programs will have a different effect run on the laboratory computers
directly connected to the robot / coffee cart. This includes the program that
requests a Modbus Ethernet server daemon to provide the current scale weight
(grams), or to tare (zero) the scale. It is only when called from a computer
that is on the same network as the scales that the daemon is able to respond
to such requests.

## 4.1. robodk_basics

Before attempting to run the _robodk_basics_ module, it is important to clarify
if the intent is to run offline or online.

If the intent is to run online, the first step is to change the run mode on
line 102 to _RUNMODE_RUN_ROBOT_.

Next, establish a TCP/IP socket connection between the laboratory computer
running RoboDK and the physical robot by selecting _Connect -> Connect robot_
from the RoboDK menu bar. A small docked window titled _Connection to UR5_ will
then appear below the station tree. From within this window, press the
_Connect_ button and wait for the _Connection status_ bar display to change
from _disconnected_ to _connected_. Once the _Connection status_ bar has turned
green, press the _Get Position_ button to synchronise the position of the
simulated robot with the physical robot. The module is now ready to be run -
simply right-click on it in the station tree and select _Run on Robot_ to run
online.

If instead the intent is to run offline, the first step is to change the run
mode on line 102 to _RUNMODE_SIMULATE_.

Next, close the TCP/IP socket connection between the laboratory computer
running RoboDK and the physical robot by pressing the _Disconnect_ button in
the _Connection to UR5_ window. The module is now ready to be run - simply
right-click on it in the station tree and select _Run_ to run offline.

Irrespective of whether the intent is to run the _robodk_basics_ module offline
or online, it is called in the same manner.

        C:\RoboDK\Python-Embedded\python C:\<path>\robodk_basics.py

Be sure to replace `<path>` with the path to the module.

> [!tip]
> Copy the Python examples to `C:\RoboDK\Python-Embedded\` to avoid having to
> type the path.

## 4.2. robodk_scale

Before attempting to run the _robodk_scales_ module, there are two operations
that must first be performed. The first is to ensure that the requirements of
the __modbus_scale_client__ repository upon which _robodk_scales_ depends have
been satisfied.

The simplest way to ensure that the dependencies are installed correctly is to
use [pip][pip] to install the package.

        C:\> C:\RoboDK\Python-Embedded\python -m pip install modbus-scale-client

This will install the required packages to:

        C:\RoboDK\Python-Embedded\Lib\site-packages\

_robodk_scales_ may then called in the same manner as _robodk_basics_.

        C:\RoboDK\Python-Embedded\python C:\<path>\robodk_scales.py

# 5. Tools Class

The _tools_ class contains methods that replicate the functionality of the tool
programs located within the RoboDK station tree. It includes methods to attach
and detach tools, as well as actuate tools and cart fixtures. In the case of
the former, left and right hand versions have been provided so that students
may select the method that best matches the current robot configuration.

See the _robodk_basics_ module for an example of how to include the _tools_
class and use the methods it contains.

# 6. Points Spreadsheet

The _points_2026_ Excel spreadsheet contains the Cartesian coordinates of the
world and local frame origins of coffee cart fixtures, as well as the
coordinates of functional points within each fixture's local frame.

As was the case with the RoboDK station files, it is of __UTMOST IMPORTANCE__
that students use the point coordinates from the sheet corresponding to the
coffee cart apparatus to which they have been assigned. This is because the
location of points differ slightly between the three carts.
 
1. Column "A". Contains a numeric key that uniquely identifies every point
within the spreadsheet. This key is used in the [Lab Frames](#7-lab-frames)
document as the point descriptor.

2. Column "B" indicates if the point coordinates are in the
world or local frame.

3. Column "C" indicates if the point is the origin of the frame.

4. Columns "E" through "G". Contain the point Cartesian coordinates.

5. Column "H". Contains a brief description of the point location.

> [!note]
> Key points 63 and 64 are unique in that they contain geometry data for the
> _rancilio_tool_, rather than point coordinates.

# 7. Lab Frames

The _lab frames_ file contains images that illustrate the location of each key
point listed within the _points_ spreadsheet.

# 8. Documentation

Code has been documented using [Doxygen][doxygen].

# 9. License

__RoboDK Stations__ is released under the [GNU General Public License][gpl].

# 10. Authors

Code by Rodney Elliott, <rodney.elliott@canterbury.ac.nz>

Lab frames file by Kailan Paul, <kailan.paul@canterbury.ac.nz>

[robodk]: https://robodk.com
[python_api]: https://robodk.com/doc/en/PythonAPI/index.html
[pip]: https://pypi.org/project/pip/
[doxygen]: https://www.doxygen.nl
[gpl]: https://www.gnu.org/licenses/gpl-3.0.html
