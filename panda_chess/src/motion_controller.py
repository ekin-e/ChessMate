#!/usr/bin/env python3
import rospy
from franka_gripper.msg import GraspActionGoal, GraspGoal, GraspAction, MoveGoal, MoveAction, StopActionGoal, StopAction
from std_msgs.msg import String
import json
import numpy as np
import actionlib
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback, JointTrajectoryControllerState
import roboticstoolbox as rtb
from spatialmath import SE3


# Class for moving Panda
class MovePanda:
    def __init__(self, piece_height, piece_width):
        self.robot = rtb.models.Panda()
        # Define the z-offset value
        z_offset = 0.32
        # Get the transformation matrix of the base frame
        T_base = np.array(self.robot.base)
        # Apply the z-offset to the translation component of the transformation matrix
        T_base[2, 3] += z_offset
        # Update the transformation matrix of the base frame
        self.robot.base = T_base

        self.start_pos = None
        self.goal_pos = None
        self.piece_height = piece_height
        self.piece_width = piece_width
        self.client = None
        self.client_gripper = None
        self.init_pose = (-0.0001460298397581994, -0.7856265484913365, 3.848215033386282e-05, -2.3559752525418283, 1.4089042781328942e-05, 1.5717536590604224, 0.7853856431923942)
        self.current_pose = None


    def set_pos(self, pos):
        self.start_pos = pos[0]
        self.goal_pos = pos[1]
        print(self.start_pos)

    def print_pose(self, pose):
        for i in pose:
            print(round(i, 4))

    def get_joint_pose(self):
        topic = "/effort_joint_trajectory_controller/state"
        pose_msg = rospy.wait_for_message(topic, JointTrajectoryControllerState)
        #print(pose_msg)
        pose = pose_msg.actual.positions
        #rospy.loginfo("Received joint pose: " + str(pose))

        self.current_pose = pose
        self.robot.q = pose
        Te = self.robot.fkine(self.robot.q)  # forward kinematics
        print(Te)


    def calculate_IK_pose(self, x, y, z):
        # Define the desired orientation as a rotation matrix
        R = SE3.Ry(180, unit='deg')
        # translate to this position
        Tep = SE3.Tx(x) @ SE3.Ty(y) @ SE3.Tz(z) @ R
        # solve IK
        sol = self.robot.ik_LM(Tep)
        # convert sol to franka_ros joint pose
        joint_positions = sol[0]

        pose = joint_positions.copy()
        return pose


    def robot_result(self, result):
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logerr('Movement was not successful: ' + {
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
            rospy.loginfo('Successfully moved into goal pose')


    def move_pose(self, pose):
        # max_movement = max(abs(pose[joint] - self.init_pose[joint]) for joint in pose)
        # max_movement = max(abs(pose[i] - self.init_pose[self.robot.arm_joint_names[i]]) for i in range(len(pose)))
        # /effort_joint_trajectory_controller/follow_joint_trajectory/goal ---> might need to use this topic we'll see

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(
            # Use either the time to move the furthest joint with 'max_dq' or 500ms,
            # whatever is greater
            # max(max_movement / rospy.get_param('~max_dq', 0.5), 0.5)
            2.0
        )
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        point.positions = pose

        # goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
        point.velocities = [0] * len(pose)

        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)
        print("CURRENT POSE")
        self.print_pose(self.current_pose)
        self.client.send_goal_and_wait(goal)

        result = self.client.get_result()
        print("TARGET POSE")
        self.print_pose(pose)
        self.robot_result(result)


    def move_gripper(self, unit):
        # Creates the SimpleActionClient, passing the type of the action
        self.client_gripper = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        print("waiting for server")
        self.client_gripper.wait_for_server()

        # Creates a goal to send to the action server.
        print("creating goal")
        goal = MoveGoal(width=unit, speed=2.0)
        
        # Sends the goal to the action server.
        print("sending goal")
        self.client_gripper.send_goal(goal)

        # Waits for the server to finish performing the action.
        print("wait for result")
        self.client_gripper.wait_for_result()

        # Get the result
        result = self.client_gripper.get_result()
        print("Result:", result)
    
    def grasp(self, width, speed):
        # Create the SimpleActionClient, passing the type of the action
        self.client_gripper = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

        # Wait until the action server has started up and started listening for goals.
        print("Waiting for server")
        self.client_gripper.wait_for_server()

        # Create a goal to send to the action server.
        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = 5.0

        # Create the action goal message
        action_goal = GraspActionGoal()
        action_goal.goal = goal
        
        # Sends the goal to the action server.
        print("sending goal")
        self.client_gripper.send_goal(goal)

        # Waits for the server to finish performing the action.
        print("wait for result")
        self.client_gripper.wait_for_result()

        # Get the result
        result = self.client_gripper.get_result()
        print("Result:", result)
    
    def stop_gripper(self):
        # Creates the SimpleActionClient, passing the type of the action
        self.client_gripper = actionlib.SimpleActionClient('/franka_gripper/stop', StopAction)

        # Waits until the action server has started up and started
        # listening for goals.
        print("waiting for server")
        self.client_gripper.wait_for_server()

        # Creates a goal to send to the action server.
        goal = StopActionGoal()
        
        # Sends the goal to the action server.
        print("sending goal")
        self.client_gripper.send_goal(goal)

        # Waits for the server to finish performing the action.
        print("wait for result")
        self.client_gripper.wait_for_result()

        # Get the result
        result = self.client_gripper.get_result()
        print("Result:", result)


    def robot_init(self):
        print("move robot to initial position")
        #action = rospy.resolve_name('/effort_joint_trajectory_controller/follow_joint_trajectory')
        self.client = actionlib.SimpleActionClient('/effort_joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()
        self.get_joint_pose()


    def pick_piece(self):
        print("picking up the piece")
        self.get_joint_pose()
        if (self.current_pose != self.init_pose):
            self.move_pose(self.init_pose)
            self.move_gripper(0.03)
        top_pos = list(self.start_pos)

        # MOVING TO A POSITION ALIGNED WITH THE PIECE (HIGHER)
        top_pos[2] = self.start_pos[2] + self.piece_height * 4
        # calculate IK
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)
        print("moved to top position")
        
        # MOVING TO A POSITION FOR PICKING UP THE PIECE
        self.get_joint_pose()
        # edit 0.8 to find the correct position
        top_pos[2] = self.start_pos[2] + self.piece_height * 0.8
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)

        # CLOSE THE GRIPPER
        self.grasp(self.piece_width, 0.1)

        # MOVING TO A POSITION WITH THE PIECE (HIGHER)
        self.get_joint_pose()
        top_pos[2] = self.start_pos[2] + self.piece_height * 4
        # calculate IK
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)


    def drop_piece(self):
        print("dropping the piece")
        self.get_joint_pose()
        top_pos = list(self.goal_pos)

        # # MOVING TO A POSITION ALIGNED WITH THE PIECE (HIGHER)
        # top_pos[2] = self.goal_pos[2] + self.piece_height * 4
        # # calculate IK
        # pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # # move to position
        # self.move_pose(pose)

        # MOVING TO A POSITION FOR DROPPING THE PIECE
        self.get_joint_pose()
        # edit 0.8 to find the correct position
        top_pos[2] = self.goal_pos[2] + self.piece_height * 0.8
        pose = self.calculate_IK_pose(top_pos[0], top_pos[1], top_pos[2])
        # move to position
        self.move_pose(pose)

        # DROP PIECE
        self.stop_gripper()

        # MOVING TO A POSITION WITHOUT THE PIECE (HIGHER)
        self.get_joint_pose()
        top_pos[2] = self.goal_pos[2] + self.piece_height * 4
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
    # print(move_coord)
    panda.pick_and_place()
    

# def pick_and_drop_piece(data):
#     move_coord = json.loads(data.data)
#     print("Robot moving with move coordinates:")
#     print(move_coord)


def listener():
    rospy.init_node('motion_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    rospy.Subscriber("game_controller/move", String, control)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('motion_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    panda.robot_init()
    move_coord = [(0.675, -0.175, 0.021), (0.675, -0.175, 0.021)]
    #listener()
    control(move_coord)



# LOOK AT:
# roslaunch franka_gazebo panda.launch x:=0 y:=0 z:=0 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=effort_joint_trajectory_controller
# /effort_joint_trajectory_controller/follow_joint_trajectory/command
# /effort_joint_trajectory_controller/follow_joint_trajectory/feedback
# /effort_joint_trajectory_controller/follow_joint_trajectory/goal
# /effort_joint_trajectory_controller/follow_joint_trajectory/result
# /effort_joint_trajectory_controller/follow_joint_trajectory/status
