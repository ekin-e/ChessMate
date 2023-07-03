#!/usr/bin/env python3
import rospy
import sys
import franka_gripper.msg
from std_msgs.msg import String
import json
import numpy as np
import actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
import roboticstoolbox as rtb
from spatialmath import SE3


# Class for moving Panda
class MovePanda:
    def __init__(self, piece_height, piece_width):
        self.robot = rtb.models.Panda()
        self.start_pos = None
        self.goal_pos = None
        self.piece_height = piece_height
        self.piece_width = piece_width
        self.client = None
        self.client_gripper = None
        self.init_pose = None
        self.current_pose = None


    def set_pos(self, pos):
        self.start_pos = pos[0]
        self.goal_pos = pos[1]


    def get_robot_init_pose(self):
        topic = rospy.resolve_name('~joint_states')
        rospy.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
        joint_state = rospy.wait_for_message(topic, JointState)
        self.init_pose = dict(zip(joint_state.name, joint_state.position))
        print(self.init_pose)


    def get_joint_pose(self):
        param = rospy.resolve_name('~joint_pose')
        pose = rospy.get_param(param, None)
        if pose is None:
            rospy.logerr('move_to_start: Could not find required parameter "' + param + '"')
            sys.exit(1)
        
        self.current_pose = pose
        ####TODO############
        # convert pose to sol type
        self.robot.q = pose


    def calculate_IK_pose(self, x, y, z):
        # example coordinates
        x = 0.6
        y = 0.3
        z = 0.2
        # translate to this position
        Tep = SE3.Trans(x, y, z)
        # solve IK
        sol = self.robot.ik_LM(Tep)
        ####TODO############
        # convert sol to franka_ros joint pose
        pose = None
        return pose


    def robot_result(self, result):
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logerr('move_to_start: Movement was not successful: ' + {
                FollowJointTrajectoryResult.INVALID_GOAL:
                """
                The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
                Is the 'joint_pose' reachable?
                """,

                FollowJointTrajectoryResult.INVALID_JOINTS:
                """
                The joint pose you specified is for different joints than the joint trajectory controller
                is claiming. Does you 'joint_pose' include all 7 joints of the robot?
                """,

                FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
                """
                During the motion the robot deviated from the planned path too much. Is something blocking
                the robot?
                """,

                FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
                """
                After the motion the robot deviated from the desired goal pose too much. Probably the robot
                didn't reach the joint_pose properly
                """,
            }[result.error_code])

        else:
            rospy.loginfo('move_to_start: Successfully moved into start pose')


    def move_pose(self, pose):
        max_movement = max(abs(pose[joint] - self.init_pose[joint]) for joint in pose)

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(
            # Use either the time to move the furthest joint with 'max_dq' or 500ms,
            # whatever is greater
            max(max_movement / rospy.get_param('~max_dq', 0.5), 0.5)
        )
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
        point.velocities = [0] * len(pose)

        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)

        rospy.loginfo('Sending trajectory Goal to move into initial config')
        self.client.send_goal_and_wait(goal)

        result = self.client.get_result()
        self.robot_result(result)


    def move_gripper(self, unit):
        # Creates the SimpleActionClient, passing the type of the action
        self.client_gripper = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        print("waiting for server")
        self.client_gripper.wait_for_server()

        # Creates a goal to send to the action server.
        print("creating goal")
        rospy.sleep(10) # Sleeps for 10 sec
        goal = franka_gripper.msg.MoveGoal(width=unit, speed=1.0, force=5.0)
        
        # Sends the goal to the action server.
        print("sending goal")
        self.client_gripper.send_goal(goal)

        # Waits for the server to finish performing the action.
        print("wait for result")
        self.client_gripper.wait_for_result()

        # Prints out the result of executing the action
        result = self.client_gripper.get_result()
        self.robot_result(result)


    def robot_init(self):
        print("move robot to initial position")
        action = rospy.resolve_name('~follow_joint_trajectory')
        self.client = actionlib.SimpleActionClient(action, FollowJointTrajectoryAction)
        rospy.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
        self.client.wait_for_server()

        pose = self.get_joint_pose()
        self.get_robot_init_pose()


    def pick_piece(self):
        print("picking up the piece")
        self.get_joint_pose()
        if (self.current_pose != self.init_pose):
            self.move_init()
        top_pos = self.start_pos.copy()

        # MOVING TO A POSITION ALIGNED WITH THE PIECE (HIGHER)
        top_pos[2] = self.start_pos[2] - self.piece_height * 3
        # calculate IK
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)

        self.move_gripper(0.08)
        # MOVING TO A POSITION FOR PICKING UP THE PIECE
        self.get_joint_pose()
        # edit 0.8 to find the correct position
        top_pos[2] = self.start_pos[2] - self.piece_height * 0.8
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)

        # CLOSE THE GRIPPER
        self.move_gripper(self.piece_width)

        # MOVING TO A POSITION WITH THE PIECE (HIGHER)
        self.get_joint_pose()
        top_pos[2] = self.start_pos[2] + self.piece_height * 3
        # calculate IK
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)


    def drop_piece(self):
        print("dropping the piece")
        self.get_joint_pose()
        top_pos = self.end_pos.copy()

        # MOVING TO A POSITION ALIGNED WITH THE PIECE (HIGHER)
        top_pos[2] = self.end_pos[2] - self.piece_height * 3
        # calculate IK
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)

        # MOVING TO A POSITION FOR PICKING UP THE PIECE
        self.get_joint_pose()
        # edit 0.8 to find the correct position
        top_pos[2] = self.end_pos[2] - self.piece_height * 0.8
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)

        # OPEN THE GRIPPER
        self.move_gripper(0.08)

        # MOVING TO A POSITION WITHOUT THE PIECE (HIGHER)
        self.get_joint_pose()
        top_pos[2] = self.end_pos[2] + self.piece_height * 3
        # calculate IK
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)


    def pick_and_place(self):
        # implement lowering the arm as well as closing the gripper in this function
        self.pick_piece()
        # implement moving the arm up as well as opening the gripper in this function
        self.drop_piece()
        self.move_pose(self.init_pose)

# global variables
global panda
panda = MovePanda(0.04, 0.02)

def control(data):
    global panda
    #move_coord = json.loads(data.data)
    move_coord = data
    panda.set_pos(move_coord)    
    panda.pick_and_place()
    

# def pick_and_drop_piece(data):
#     move_coord = json.loads(data.data)
#     print("Robot moving with move coordinates:")
#     print(move_coord)


def listener():
    rospy.init_node('motion_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    # rospy.Subscriber("game_controller/move", String, pick_and_drop_piece)
    # rospy.spin()


if __name__ == '__main__':
    #listener()
    rospy.init_node('motion_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    panda.robot_init()
    move_coord = [(-0.148572, -0.17441600000000002, 0.439), (0.15142800000000003, 0.12558400000000003, 0.439)]
    control(move_coord)




# LOOK AT:
# roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=effort_joint_trajectory_controller
# /effort_joint_trajectory_controller/follow_joint_trajectory/command
# /effort_joint_trajectory_controller/follow_joint_trajectory/feedback
# /effort_joint_trajectory_controller/follow_joint_trajectory/goal
# /effort_joint_trajectory_controller/follow_joint_trajectory/result
# /effort_joint_trajectory_controller/follow_joint_trajectory/status