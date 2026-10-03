from time import sleep
from robodk.robolink import *
import tools
from scipy.spatial.transform import Rotation as R
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
import robodk.robomath as rm
from modbus_scale_client import modbus_scale_client

IP_MAZZER_3 = "192.168.22.3"
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

def Rotational_matrix_z(theta):
    theta =  np.pi/180 * theta
    return np.array([[np.cos(theta), -np.sin(theta), 0],
               [np.sin(theta), np.cos(theta), 0],
               [0, 0, 1]])

def Rotational_matrix_x(theta):
    theta =  np.pi/180 * theta
    return np.array([[1, 0, 0],
               [0, np.cos(theta), -np.sin(theta)],
               [0, np.sin(theta), np.cos(theta)]])

def Rotational_matrix_y(theta):
    theta =  np.pi/180 * theta
    return np.array([[np.cos(theta), 0, np.sin(theta)],
               [0, 1, 0],
               [-np.sin(theta), 0, np.cos(theta)]])

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

def inverse_transform_z(theta, x, y, z):
    theta =  np.pi/180 * theta
    R_T = np.array([[ np.cos(theta),  np.sin(theta), 0],
                    [-np.sin(theta),  np.cos(theta), 0],
                    [0,               0,             1]])  # transpose of R_z(theta)
    T = np.array([x, y, z])

    Trans_inv = np.identity(4)
    Trans_inv[0:3, 0:3] = R_T
    Trans_inv[0:3, 3] = R_T @ (-T)
    return Trans_inv

def inverse_transform_matrix(R, x, y, z):
    R_T = np.transpose(R)          # inverse of a rotation matrix is its transpose
    T = np.array([x, y, z])

    Trans_inv = np.identity(4)
    Trans_inv[0:3, 0:3] = R_T
    Trans_inv[0:3, 3] = R_T @ (-T)
    return Trans_inv

def transform_matrix(R, x, y, z):
    R = R         # inverse of a rotation matrix is its transpose
    T = np.array([x, y, z])

    Trans = np.identity(4)
    Trans[0:3, 0:3] = R
    Trans[0:3, 3] = (T)
    return Trans

def deg_to_rad(angle):
    return np.pi/180 * angle

# Now for the actual functions
def InitialiseSimulateA():
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

def mazzer_pull_lever_0_degree():

    # calculated by doing the cross product of x and y vectors and finding rotation manually
    R = np.array([[-0.874, 0.203, -0.441],
                 [-0.486, -0.367, 0.794],
                 [0, 0.908, 0.420]])
    # rotation in X Y Z [65.209002 0.008858 -150.961295]
    x = 504.4
    y = -419.7
    z = 319.5

    # Mazzer Frame transform
    URtM = transform_matrix(R, x, y, z)

    #Moving to the pull lever
    R = Rotational_matrix_z(0)
    x = 78.3
    y = -134.5
    z = -80.9

    MtMDLF = transform_matrix(R, x, y, z)

    #Now making the mazzer frame flat:
    theta = -65.2
    R = Rotational_matrix_x(theta)
                
    MDLFtMDLFF = transform_matrix(R,0,0,0)

    #Rotating it in x 
    theta = 90
    R = Rotational_matrix_x(theta)

    MDLFFtMDLFFX = transform_matrix(R,0,0,0)

    theta = -50 
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    #TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(-50, 0, 67.06)
    #MTtMTDBE_np = R3 + T3
    MTDBEtMT = inverse_transform_z(0, -50, 0, 67.06)
    
    R4 = np.array([[0,1,0],
                    [1,0,0],
                    [0,0,-1]])
    
    MOBCLFFtMTDBE = inverse_transform_matrix(R4, 0, 0, 0)

    R5 = Rotational_matrix_z(90)
    MOBCLFFRtMTDBE = inverse_transform_matrix(R5, 0, 0, 0)

    
    URtTCP = URtM @ MtMDLF @ MDLFtMDLFF @ MDLFFtMDLFFX @ MOBCLFFRtMTDBE @ MOBCLFFtMTDBE @ MTDBEtMT @ MTtTCP

    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_URtTCP)), blocking=True)

def  mazzer_pull_lever_degree(x_diff, y_diff, angle) :

    # calculated by doing the cross product of x and y vectors and finding rotation manually
    R = np.array([[-0.874, 0.203, -0.441],
                 [-0.486, -0.367, 0.794],
                 [0, 0.908, 0.420]])
    # rotation in X Y Z [65.209002 0.008858 -150.961295]
    x = 504.4
    y = -419.7
    z = 319.5

    # Mazzer Frame transform
    URtM = transform_matrix(R, x, y, z)

    #Moving to the pull lever
    R = Rotational_matrix_z(0)
    x = 78.3
    y = -134.5
    z = -80.9

    MtMDLF = transform_matrix(R, x, y, z)

    #Now making the mazzer frame flat:
    theta = -65.2
    R = Rotational_matrix_x(theta)
                
    MDLFtMDLFF = transform_matrix(R,0,0,0)

    #Rotating it in x 
    theta = 90
    R = Rotational_matrix_x(theta)

    MDLFFtMDLFFX = transform_matrix(R,0,0,0)

    theta = 0
    R1 = Rotational_matrix_x(theta)
    x = x_diff
    y = 20
    z = y_diff

    MDLFFXtMDLFFX5 = transform_matrix(R1,x,y,z)


    theta = -50 
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    #TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(-50, 0, 67.06)
    #MTtMTDBE_np = R3 + T3
    MTDBEtMT = inverse_transform_z(0, -50, 0, 67.06)
    
    R4 = np.array([[0,1,0],
                    [1,0,0],
                    [0,0,-1]])
    
    MOBCLFFtMTDBE = inverse_transform_matrix(R4, 0, 0, 0)

    R5 = Rotational_matrix_z(90)
    MOBCLFFRtMTDBE = inverse_transform_matrix(R5, 0, 0, 0)

    #How much to rotate
    R6 = Rotational_matrix_y(angle)
    MOBCLFFRRtMOBCLFFR = inverse_transform_matrix(R6, 0, 0, 0)


    
    URtTCP = URtM @ MtMDLF @ MDLFtMDLFF @ MDLFFtMDLFFX @ MDLFFXtMDLFFX5 @ MOBCLFFRRtMOBCLFFR @ MOBCLFFRtMTDBE @ MOBCLFFtMTDBE @ MTDBEtMT @ MTtTCP

    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(T_URtTCP)), blocking=True)

def  mazzer_pull_lever_degree_disengage(x_diff, y_diff, angle):

    # calculated by doing the cross product of x and y vectors and finding rotation manually
    R = np.array([[-0.874, 0.203, -0.441],
                 [-0.486, -0.367, 0.794],
                 [0, 0.908, 0.420]])
    # rotation in X Y Z [65.209002 0.008858 -150.961295]
    x = 504.4
    y = -419.7
    z = 319.5

    # Mazzer Frame transform
    URtM = transform_matrix(R, x, y, z)

    #Moving to the pull lever
    R = Rotational_matrix_z(0)
    x = 78.3
    y = -134.5
    z = -80.9

    MtMDLF = transform_matrix(R, x, y, z)

    #Now making the mazzer frame flat:
    theta = -65.2
    R = Rotational_matrix_x(theta)
                
    MDLFtMDLFF = transform_matrix(R,0,0,0)

    #Rotating it in x 
    theta = 90
    R = Rotational_matrix_x(theta)

    MDLFFtMDLFFX = transform_matrix(R,0,0,0)

    theta = 0
    R1 = Rotational_matrix_x(theta)
    x = x_diff
    y = 20
    z = y_diff

    MDLFFXtMDLFFX5 = transform_matrix(R1,x,y,z)

    theta = 0
    R1_1 = Rotational_matrix_x(theta)
    x = 0
    y = 80
    z = 0

    MDLFFX5tMDLFFX5t = transform_matrix(R1_1,x,y,z)

    theta = -50 
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    #TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(-50, 0, 67.06)
    #MTtMTDBE_np = R3 + T3
    MTDBEtMT = inverse_transform_z(0, -50, 0, 67.06)
    
    R4 = np.array([[0,1,0],
                    [1,0,0],
                    [0,0,-1]])
    
    MOBCLFFtMTDBE = inverse_transform_matrix(R4, 0, 0, 0)

    R5 = Rotational_matrix_z(90)
    MOBCLFFRtMTDBE = inverse_transform_matrix(R5, 0, 0, 0)

    #How much to rotate
    R6 = Rotational_matrix_y(angle)
    MOBCLFFRRtMOBCLFFR = inverse_transform_matrix(R6, 0, 0, 0)


    
    URtTCP = URtM @ MtMDLF @ MDLFtMDLFF @ MDLFFtMDLFFX @ MDLFFXtMDLFFX5 @ MDLFFX5tMDLFFX5t @ MOBCLFFRRtMOBCLFFR @ MOBCLFFRtMTDBE @ MOBCLFFtMTDBE @ MTDBEtMT @ MTtTCP

    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(T_URtTCP)), blocking=True)

def dosing_action():
    client = modbus_scale_client.ModbusScaleClient(host = IP_MAZZER_3)

    if client.server_exists() == False:
        RDK.ShowMessage("No scale detected, output will be simulated.")

    ## Scale output (grams).
    value = client.read()
    target = 20
    tolerance = 0.1

    starting_joint = [-38.333723, -85.458041, -133.766649, -140.818270, 292.992198, -129.616853]
    mid_joint = [-37.132152, -110.515319, -105.384296, -144.148589, 294.156847, -129.623343]
    UR5.MoveJ(starting_joint, blocking=True)
    UR5.MoveJ(mid_joint, blocking=True)

    #move to initial position
    mazzer_pull_lever_0_degree()

    RDK.ShowMessage("Value = %f" % value)
    x_values = np.array([-78.843, -78.787, -78.13, -76.88, -75.0441, -72.6371, -69.6773, -66.1872])
    y_values = np.array([-2.794, 4.08868, 10.9399, 17.7078, 24.3409, 30.7888, 37.0024, 42.9344])
    x_diff = -(x_values - -78.3) 
    y_diff = y_values - -9.65464             
    angle = 5,10,15,20,25,30,35,40
    i = 0
    while(1):
        value = client.read() # Scale Output in grams
        if ((value) > (target - tolerance)):
            break

        # move to the next step of the puller
        mazzer_pull_lever_degree(x_diff[i], y_diff[i], angle[i]) 
        i += 1

        if (i + 1 == 8):
            mazzer_pull_lever_degree_disengage(x_diff[i], y_diff[i], angle[i])
            position_1 = [-12.377380, -109.368842, -109.146920, -170.020590, 359.410337, -101.437620]
            position_2 = [-34.625357, -112.320869, -102.690945, -145.206042, 313.013399, -129.555667]
            UR5.MoveJ(position_1, blocking=True)
            UR5.MoveJ(position_2, blocking=True)
            i = -1

        i += 1
    mazzer_pull_lever_degree_disengage(x_diff[i], y_diff[i], angle[i])
    position_1 = [-12.377380, -109.368842, -109.146920, -170.020590, 359.410337, -101.437620]
    position_2 = [-34.625357, -112.320869, -102.690945, -145.206042, 313.013399, -129.555667]
    UR5.MoveJ(position_1, blocking=True)
    UR5.MoveJ(position_2, blocking=True)
    final_position = [-32.826039, -104.116544, -91.478512, -253.754174, 270.092468, -85.322825]
    UR5.MoveJ(final_position, blocking=True)
    
# example 4x4 matrix 
#np.array([[      0,        0,                0,         x ],
#          [      0,            0,            0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])

