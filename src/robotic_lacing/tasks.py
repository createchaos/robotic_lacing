import compas_ghpython
from compas.geometry import Frame, Plane, Transformation
from abb_communication.clients.rfl_robot.communication.messages.messagetypes import *
import math 

#import abb_communication
#import abb_communication.clients

# Global state varaibles
state_connected = "" 
state_gripper = ""
state_joints = []
state_position = None
state_postition_gh = None
state_retracted = ""

# Work object (read manually from robot controller)
wobj = [100,100,100,.1,.1,.1,.1]

# Open gripppers
def open_gripper(communication):
    communication.send_open_gripper()
    global state_gripper
    state_gripper = "is_open"
    return state_gripper
    
# Close gripppers
def close_gripper(communication):
    communication.send_close_gripper()
    global state_gripper
    state_gripper = "is_closed"
    return state_gripper

# Transform robot pose to rhino coordinates 
def get_pose_as_plane(communication, fake_pose=None):
    global state_position
    global state_postition_gh
    pose = communication.get_current_pose_cartesian()
    if fake_pose:
        pose = fake_pose
    pose_coordinates = pose[0:3]
    pose_quaternions = pose[3:7]
    wobj_coordinates = wobj[0:3]
    wobj_quaternions = wobj[3:7]
    # Make frames with compas
    pose_frame = Frame.from_quaternion(pose_quaternions, point=pose_coordinates)
    wobj_frame = Frame.from_quaternion(wobj_quaternions, point=wobj_coordinates)
    # Define transformation using wobj
    T = Transformation.from_frame_to_frame(pose_frame, wobj_frame)
    # Do the transformation
    state_position = Frame.from_transformation(T)
    # Return rhino and compas geometry 
    state_postition_gh = compas_ghpython.draw_frame(state_position)
    return state_position
    return state_postition_gh

# Get robot joint values
def get_joints(communication, fake_joints=None):
    global state_joints
    joints = communication.get_current_pose_joint()
    if fake_joints:
        joints = fake_joints
    # Trim to 6 values
    state_joints = joints[0:6]
    # Add zeros so the list has 9 values
    state_joints.extend([0] * (9 - len(state_joints)))
    return state_joints

# Retract function
def retract(communication):
    global state_retracted
    if state_gripper == "is_open":
        print("gripper is open so I will close it")
        close_gripper(communication)
        print("gripper is now closed")
    print("now I will retract")
    # Retract along tool z See send_movel_reltool - ln 359 in communication.py
    retract_distance = 50
    communication.send_movel_reltool(0,0,-retract_distance)
    state_retracted = "is_retracted"

# Unwind function
def unwind(communication, fake_joints):
    global state_joints
    # Check if retracted
    if state_retracted != "is_retracted":
        retract(communication)
    # Get joint values
    get_joints(communication, fake_joints)
    # Move away even further the joint values at joint 1
    joint1_val = 45
    joints_safe_zone = state_joints
    joints_safe_zone[0] = joint1_val
    print(joints_safe_zone)
    communication.send_axes_absolute(joints_safe_zone, int_arr = None)
    # Now unwind fully
    joints_unwound = joints_safe_zone[0:1]
    joints_unwound.extend([0] * (9 - len(joints_unwound)))
    communication.send_axes_absolute(joints_unwound, int_arr = None)
    print(joints_unwound)
    print("unwound")
    # Move back to previous position (without path planning!)
    communication.send_axes_absolute(state_joints, int_arr = None)

# Check if robot axes need unwinding
def check_axes(communication, fake_joints):
    global state_joints
    get_joints(communication, fake_joints)
    joint_range = [165.0,110.0,110.0,160.0,120.0,400.0]
    joint_range_max = [i * .9 for i in joint_range]
    for i, value in enumerate(state_joints[0:6]):
        if abs(value) >= math.radians(joint_range_max[i]):
            unwind(communication, fake_joints)
            break
    if state_joints[2] >= math.radians(70 * .9):
        unwind(communication, fake_joints)

# Go to plane
    # standard comm function
    # change retract variable

# Bend to plane (plane input)
    # if gripping, release gripper
    # if not retracted, retract
    # check if unwinding needed
    # go to plane
    # grip
    # bend (from current plane to target plane)

# Make a node with two strands (node number)

# Lacing process: torchon ground