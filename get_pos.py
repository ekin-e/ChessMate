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

def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(3)

    """ print ("Reference frame: %s" % group.get_planning_frame())
    print ("Reference frame: %s" % group.get_end_effector_link())
    print ("Robot Groups:")
    print (robot.get_group_names())
    print(group.get_current_joint_values()) """

    print(group.get_current_joint_values())

    print("The last position of end effector: \n", group.get_current_pose().pose.position)
   

    group.clear_pose_targets()


    print ("Generating plan 1")
    group.set_planner_id("RRTstar") # planner seçimi
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = group.get_current_pose().pose.orientation   # maintaining last orientation
    pose_target.position.x = 0.8
    pose_target.position.y = 0.05
    pose_target.position.z = 0.2

    

    #group.set_pose_target(pose_target)
    plan_success, plan1, planning_time, error_code = group.plan()


    # for display
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory)

    rospy.sleep(5)



    # execute
    #group.execute(plan1)

    print("The new position of end effector: \n", group.get_current_pose().pose.position)


    print ("Waiting while plan1 is visualized (again)...")
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass






