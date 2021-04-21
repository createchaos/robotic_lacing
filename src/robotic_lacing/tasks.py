import compas_ghpython
from compas_rhino.geometry import RhinoPlane
from compas.geometry import Frame, Plane, Transformation, Vector, Translation
from abb_communication.clients.rfl_robot.communication.messages.messagetypes import *
import math 

class robotic_lacing:

    def __init__(self):

        # State variables
        self.gripper = ""
        self.joints = []
        self.position = None
        self.position_gh = None
        self.retracted = "" 

        # Global variables
        self.wobj = [396.529,295.267,21.5105,0.000382471,0.00291474,-0.00127828,-0.999995]
        self.tcp = [0,0,107.5,0,-0.707,0,0.707]

    # Open gripppers
    def open_gripper(self, communication):
        communication.send_open_gripper()
        self.gripper = "is_open"
        #print("gripper " + self.gripper)
        #return self.gripper
            
    # Close gripppers
    def close_gripper(self,communication):
        communication.send_close_gripper()
        self.gripper = "is_closed"
        #print("gripper " + self.gripper)
        #return self.gripper

    # Toggle grippers
    def toggle_gripper(self,communication):
        #print("gripper ")
        #print(self.gripper)
        if self.gripper == "is_open":
            self.close_gripper(communication)
        elif self.gripper == "is_closed":
            self.open_gripper(communication)
        else: # if there is no state, do both to set the state
            '''Note: let's include this in a startup function
            so the gripper state is known from the beginning'''
            self.open_gripper(communication)
            self.close_gripper(communication)
            self.gripper = "is_closed"
            #return self.gripper
            #print("gripper " + self.gripper)

    # Transform robot pose to rhino coordinates
    def get_pose_as_plane(self,communication,fake_pose=None):
        tool = self.tcp
        #print(tool)
        pose = communication.get_current_pose_cartesian()
        print(pose)
        if fake_pose:
            pose = fake_pose
        pose_coordinates = pose[0:3]
        pose_quaternions = pose[3:7]
        wobj_coordinates = self.wobj[0:3]
        wobj_quaternions = self.wobj[3:7]
        tool_coordinates = tool[0:3]
        tool_quaternions = tool[3:7]

        # Make frames with compas
        pose_frame = Frame.from_quaternion(pose_quaternions, point=pose_coordinates)
        wobj_frame = Frame.from_quaternion(wobj_quaternions, point=wobj_coordinates)
        tool_frame = Frame.from_quaternion(tool_quaternions, point=tool_coordinates)

        # Define transformations
        T_robot_to_wobj = Transformation.from_frame(wobj_frame)
        T_robot_to_wobj_inverse = T_robot_to_wobj.inverse()
        T_tool = Transformation.from_frame(tool_frame)
        
        # Do the transformation
        pose_frame.transform(T_robot_to_wobj_inverse)

        # To account for the tool, we first need to transform the current pose to the origin
        T_zero = Transformation.from_frame_to_frame(pose_frame, Frame.worldXY())
        T_zero_inverse = T_zero.inverse()
        pose_frame.transform(T_zero)

        # Apply the tool transformation
        pose_frame.transform(T_tool)

        # Then transform back
        pose_frame.transform(T_zero_inverse)

        # Return rhino and compas geometry 
        self.position = pose_frame
        self.postition_gh = compas_ghpython.draw_frame(self.position)
        return self.postition_gh

    # Get robot joint values
    def get_joints(self,communication, fake_joints=None):
        self.joints
        joints = communication.get_current_pose_joint()
        if fake_joints:
            joints = fake_joints
        # Trim to 6 values
        self.joints = joints[0:6]
        # Add zeros so the list has 9 values
        self.joints.extend([0] * (9 - len(self.joints)))
        return self.joints

    # Send plane or planes
    def send_planes(self,communication, planes):
        print(len(planes))
        if len(planes) == 1:
            communication.send_pose_cartesian(planes[0])
        else:
            communication.send_pose_cartesian_list(planes)

    # Retract function
    def retract(self,communication):
        self.retracted
        retract_distance = 10
        communication.send_movel_reltool(0,0,-retract_distance, tcp=False) # Retract along tool z See send_movel_reltool - ln 359 in communication.py
        self.retracted = "is_retracted"

    # Unwind function
    def unwind(self,communication, fake_joints):
        self.joints
        # Check if retracted
        if self.retracted != "is_retracted":
            self.retract(communication)
        # Get joint values
        self.get_joints(communication, fake_joints)
        # Move away even further the joint values at joint 1
        joint1_val = 45
        joints_safe_zone = self.joints
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
        communication.send_axes_absolute(self.joints, int_arr = None)

    # Check if robot axes need unwinding
    def check_axes(self,communication, fake_joints):
        self.joints
        self.get_joints(communication, fake_joints)
        joint_range = [165.0,110.0,110.0,160.0,120.0,400.0]
        joint_range_max = [i * .9 for i in joint_range]
        for i, value in enumerate(self.joints[0:6]):
            if abs(value) >= math.radians(joint_range_max[i]):
                self. unwind(communication, fake_joints)
                break
        if self.joints[2] >= math.radians(70 * .9):
            self.unwind(communication, fake_joints)

    # Go look
    def go_look(self,communication,target,fake_pose=None):
        """The robot is getting ready to grip a strand... there is a target
        plane that it needs to 'see' with the camera. This function sends
        the robot to a good spot to look for it. """
       
        offset_value = 200
        offset_vector = [0,0,-offset_value]

        # Make a compas frame of the target plane (defined in GH)
        target_frame = RhinoPlane.from_geometry(target).to_compas()

        # Define transformations
        T_1 = Transformation.from_frame_to_frame(target_frame, Frame.worldXY()) 
        T_2 = Translation.from_vector([0,0,-offset_value]) # This should reflect the transformation to camera_look

        # Apply transformations
        target_frame.transform(T_1)     
        target_frame.transform(T_2)
        target_frame.transform(T_1.inverse())

        # Show a frame in grasshopper
        target_frame_gh = compas_ghpython.draw_frame(target_frame)
        return target_frame_gh

        # Transform to camera 0 and go there


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