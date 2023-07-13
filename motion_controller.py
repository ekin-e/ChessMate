#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import sys
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from franka_gripper.msg import GraspActionGoal, GraspGoal, GraspAction, MoveGoal, MoveAction, StopActionGoal, StopAction


# Class for moving Panda
class MovePanda:
    def __init__(self, piece_height, piece_width):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.start_pos = None
        self.goal_pos = None
        self.piece_height = piece_height
        self.piece_width = piece_width
        self.client_gripper = None
        self.init_pose = (-0.0001460298397581994, -0.7856265484913365, 3.848215033386282e-05, -2.3559752525418283, 1.4089042781328942e-05, 1.5717536590604224, 0.7853856431923942)
        self.self.init_xyz = [0,0,0]
        self.current_pose = None


    def set_pos(self, pos):
        self.start_pos = pos[0]
        self.goal_pos = pos[1]


    def move_pose(self, x, y, z):
        print ("Waiting for RVIZ...")
        rospy.sleep(3)

        print ("Reference frame: %s" % self.group.get_planning_frame())
        print ("Reference frame: %s" % self.group.get_end_effector_link())
        print ("Robot Groups:")
        print (self.robot.get_group_names())
        self.current_pose = self.group.get_current_joint_values()
        print(self.current_pose)
        self.group.clear_pose_targets()

        print ("Generating plan 1")
        self.group.set_planner_id("RRTstar")
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation = self.group.get_current_pose().pose.orientation
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        self.group.set_pose_target(pose_target)
        plan_success, plan1, planning_time, error_code = self.group.plan()
        print("Plan success")
        print(plan_success)

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(5)
        self.group.execute(plan1)


    def move_gripper(self, unit):
        # Creates the SimpleActionClient, passing the type of the action
        self.client_gripper = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        print("GRIPPER MOVEMENT")

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
        print("Gripper movement result:", result)
    
    def grasp(self, width, speed):
        # Create the SimpleActionClient, passing the type of the action
        self.client_gripper = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

        # Wait until the action server has started up and started listening for goals.
        print("Waiting for server")
        self.client_gripper.wait_for_server()

        # Create a goal to send to the action server.
        goal = GraspGoal()
        goal.width = width
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
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
        print("Gripper grasping result:", result)
    
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
        print("Gripper stopping result:", result)


    def pick_piece(self):
        print("picking up the piece")
        if (self.current_pose != self.init_pose):
            self.move_pose(self.init_xyz[0], self.init_xyz[1], self.init_xyz[2]) # move_pose find the init_pose coordinates and change this part
            self.move_gripper(0.03)
        
        top_pos = list(self.start_pos)

        # MOVING TO A POSITION ALIGNED WITH THE PIECE (HIGHER)
        top_pos[2] = self.start_pos[2] + self.piece_height * 4
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])
        print("moved to top position")
        
        # MOVING TO A POSITION FOR PICKING UP THE PIECE
        # edit 0.8 to find the correct position
        top_pos[2] = self.start_pos[2] + self.piece_height * 0.8
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])
        
        # CLOSE THE GRIPPER
        # self.move_gripper(0.01)
        self.grasp(self.piece_width, 0.1)

        # MOVING TO A POSITION WITH THE PIECE (HIGHER)
        top_pos[2] = self.start_pos[2] + self.piece_height * 4
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])


    def drop_piece(self):
        print("dropping the piece")
        top_pos = list(self.goal_pos)

        # MOVING TO A POSITION FOR DROPPING THE PIECE
        # edit 0.8 to find the correct position
        top_pos[2] = self.goal_pos[2] + self.piece_height * 0.8
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])

        # DROP PIECE
        self.move_gripper(0.03)

        # MOVING TO A POSITION WITHOUT THE PIECE (HIGHER)
        top_pos[2] = self.goal_pos[2] + self.piece_height * 4
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])


    def pick_and_place(self):
        # implement lowering the arm as well as closing the gripper in this function
        self.pick_piece()
        # implement moving the arm up as well as opening the gripper in this function
        self.drop_piece()
        self.move_pose(self.init_pose) # change init pose to be xyz

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


def listener():
    rospy.init_node('motion_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    rospy.Subscriber("game_controller/move", String, control)
    rospy.spin()


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    panda.robot_init()
    move_coord = [(0.675, -0.176, 0.021), (0.675, -0.175, 0.021)]
    #listener()
    control(move_coord)







# LOOK AT:
# roslaunch franka_gazebo panda.launch x:=0 y:=0 z:=0 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=effort_joint_trajectory_controller
# /effort_joint_trajectory_controller/follow_joint_trajectory/command
# /effort_joint_trajectory_controller/follow_joint_trajectory/feedback
# /effort_joint_trajectory_controller/follow_joint_trajectory/goal
# /effort_joint_trajectory_controller/follow_joint_trajectory/result
# /effort_joint_trajectory_controller/follow_joint_trajectory/status
# init_pose = (-0.0001460298397581994, -0.7856265484913365, 3.848215033386282e-05, -2.3559752525418283, 1.4089042781328942e-05, 1.5717536590604224, 0.7853856431923942)
