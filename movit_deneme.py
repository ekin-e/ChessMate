import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# ASAGIDAKI LINKE GORE YAP
# demo.launch çalıştırdıktan sonra
# Rviz ile linklerin pozisyonunda yaptığımız değişiklikleri görebiliyoruz
# http://docs.ros.org/en/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html


# public member functions
# http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("panda_arm")
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=10)

print ("Waiting for RVIZ...")
rospy.sleep(3)

print ("Reference frame: %s" % group.get_planning_frame())
print ("Reference frame: %s" % group.get_end_effector_link())
print ("Robot Groups:")
print (robot.get_group_names())
print(group.get_current_joint_values())


group.clear_pose_targets()


print ("Generating plan 1")
group.set_planner_id("RRTstar") # planner seçimi
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation = group.get_current_pose().pose.orientation
pose_target.position.x = 0.5154
pose_target.position.y = -0.07274
pose_target.position.z = 0.1680178409655524

init_pose = (0.20569760583994678, -0.46903115825910924, -0.21801485168053691, -2.3468741699571547, -0.10967289843824174, 1.904154620806376, 0.8666427851311035)
group.go(init_pose, wait = True)

group.set_pose_target(pose_target)
plan_success, plan1, planning_time, error_code = group.plan()

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory)




rospy.sleep(2)

#group.execute(plan1)




print ("Waiting while plan1 is visualized (again)...")
rospy.sleep(1)






