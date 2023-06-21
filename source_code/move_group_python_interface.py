#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs

import tf
import copy


from tf.transformations import *
from geometry_msgs.msg import Quaternion, PoseStamped, Pose
from moveit_commander.conversions import pose_to_list
from math import pi, tau, dist, fabs, cos

from ur_msgs.srv        import SetIO, SetIORequest
from ur_python.msg      import robot_state
from moveit_msgs.msg    import Constraints, PositionConstraint
from shape_msgs.msg     import SolidPrimitive


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True
    
class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self, real="real", gripper="gripper"):
        super(MoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        self.msg_robot_state = robot_state()
        self.pub_robot_state = rospy.Publisher("robot_state", robot_state, queue_size=10)

        self.robot = moveit_commander.RobotCommander()

        self.group_name = "manipulator"
        self.manipulator    = moveit_commander.move_group.MoveGroupCommander(self.group_name)

        if real == "real":
            self.io_handler = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
            self.gripper = SetIORequest()
            self.gripper_init()

        self.planning_frame = self.manipulator.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        self.eef_link = self.manipulator.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        # self.move_to_standby()

        rospy.sleep(1)


    def move_to_standby(self):
        # self.manipulator.set_named_target("stand_by") # stand_by
        self.go_to_joint_state([-tau/4, -tau/4, -tau/4, -tau/4, tau/4, 0.0])

        self.manipulator.go(wait=True)
        self.manipulator.stop()
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        
    def gripper_init(self):
        self.gripper.fun=1
        self.gripper.pin=1
        self.grip_off()
    
        #self.gripper_open()
    
    def grip_on(self):
        self.gripper.state=1
        self.io_handler.call(self.gripper)
        rospy.sleep(1.5)
        
    def grip_off(self):
        self.gripper.state=0
        self.io_handler.call(self.gripper)
        rospy.sleep(1.5)

    # def grip_state_callback(self, data):
    #     self.msg_grip_state.change = 0
    #     self.msg_grip_state.on = data.on

    def go_to_joint_state(self, target_joints):
        current_joint = self.manipulator.get_current_joint_values()
        target_joint = copy.deepcopy(current_joint)

        print(target_joint)
        print(target_joints)
        
        target_joint[0] = target_joints[0]
        target_joint[1] = target_joints[1]
        target_joint[2] = target_joints[2]
        target_joint[3] = target_joints[3]
        target_joint[4] = target_joints[4]
        target_joint[5] = target_joints[5]

        self.msg_robot_state.move = 1
        self.pub_robot_state.publish(self.msg_robot_state)
        
        self.manipulator.go(target_joint, wait=True)
        self.manipulator.stop()
        current_joint = self.manipulator.get_current_joint_values()

        self.msg_robot_state.move = 0
        self.pub_robot_state.publish(self.msg_robot_state)

        return all_close(target_joint, current_joint, 0.01)

    def go_to_pose_abs(self, absolute_xyz, absolute_rpy):
        # xyz unit: [m], rpy unit: [rad]

        # get current pose
        current_pose = self.manipulator.get_current_pose().pose
        # print(f"<current_pose>\n{current_pose}")

        # target position
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.x = absolute_xyz[0]
        target_pose.position.y = absolute_xyz[1]
        target_pose.position.z = absolute_xyz[2]

        # Transform target_rpy to target_quat
        target_quat = quaternion_from_euler(absolute_rpy[0], absolute_rpy[1], absolute_rpy[2])

        # Apply target_quat to the orientation of target_pose
        target_pose.orientation = Quaternion(target_quat[0], target_quat[1], target_quat[2], target_quat[3])
       
        self.manipulator.set_pose_target(target_pose)


        # ====== 경로 수정 ======
        # 경로 계획 시간 설정
        self.manipulator.set_planning_time(1.5)  # 경로 계획 시간을 5.0초로 설정

        # allow_replanning 속성 설정
        self.manipulator.allow_replanning = True  # 경로 재계획 허용
        # =====================

        self.msg_robot_state.move = 1
        self.pub_robot_state.publish(self.msg_robot_state)

        # self.manipulator.go(wait=True)

        # ===========================
        self.manipulator.go(wait=False)
        init_time = rospy.Time.now().to_sec()

        while True:
            cur_time = rospy.Time.now().to_sec()
            if cur_time - init_time > 1.5:
                init_time = rospy.Time.now().to_sec()
                self.manipulator.go(wait=False)
            else: break
        # ===========================
        
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()
        current_pose = self.manipulator.get_current_pose().pose
        
        self.msg_robot_state.move = 0
        self.pub_robot_state.publish(self.msg_robot_state)

        return all_close(target_pose, current_pose, 0.01)
    
    def print_pose(self):         
        # get current pose
        current_pose = self.manipulator.get_current_pose().pose
        target_pose = copy.deepcopy(current_pose)
        current_quat = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        current_rpy = euler_from_quaternion(current_quat)   # current orientation: Quat -> Euler
        print(f"<target_pose>\n{target_pose}")
        print(f"current xyz: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}, {current_pose.position.z:.2f}), rpy: {current_rpy}")

    def go_to_pose_rel(self, relative_xyz, relative_rpy):
        # xyz unit: [m], rpy unit: [rad]

        # get current pose
        current_pose = self.manipulator.get_current_pose().pose
        # print(f"<current_pose>\n{current_pose}")

        # calculate the target position with current position and relative coordinates
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.x += relative_xyz[0]
        target_pose.position.y += relative_xyz[1]
        target_pose.position.z += relative_xyz[2]

        # Get the target orientation with current orientation(quaternion) and relative orientation(euler)
        current_quat = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        current_rpy = euler_from_quaternion(current_quat)   # current orientation: Quat -> Euler
        target_rpy = [0., 0., 0.]                           # calculate the target orientation in Euler Coordinate
        target_rpy[0] = current_rpy[0] + relative_rpy[0]
        target_rpy[1] = current_rpy[1] + relative_rpy[1]
        target_rpy[2] = current_rpy[2] + relative_rpy[2]
        
        # Transform target_rpy to target_quat
        target_quat = quaternion_from_euler(target_rpy[0], target_rpy[1], target_rpy[2])
        
        # Apply target_quat to the orientation of target_pose
        target_pose.orientation = Quaternion(target_quat[0], target_quat[1], target_quat[2], target_quat[3])

        print(f"<target_pose>\n{target_pose}")
        print(f"current xyz: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}, {current_pose.position.z:.2f}), rpy: {current_rpy}")
        print(f"target  xyz: ({target_pose.position.x:.2f}, {target_pose.position.y:.2f}, {target_pose.position.z:.2f}), rpy: {target_rpy}")

        # Plan & Move
        # self.manipulator.set_pose_target(target_pose)
        # self.manipulator.go(wait=True)
        # self.manipulator.stop()
        # self.manipulator.clear_pose_targets()

        self.manipulator.set_pose_target(target_pose)

        self.msg_robot_state.move = 1
        self.pub_robot_state.publish(self.msg_robot_state)

        self.manipulator.go(wait=True)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()
        current_pose = self.manipulator.get_current_pose().pose

        self.msg_robot_state.move = 0
        self.pub_robot_state.publish(self.msg_robot_state)

        return all_close(target_pose, current_pose, 0.01)
    

