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


def move(group, x,y,z):
    group.clear_pose_targets()

    print ("Generating plan 1")
    # group.set_planner_id("RRTstar") # planner seçimi
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = group.get_current_pose().pose.orientation   # maintaining last orientation
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    group.set_pose_target(pose_target)
    plan_success, plan1, planning_time, error_code = group.plan()


    # for display
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan1)

def get_pos(group):
    # chess board dimensions and center coordinates
    side_len = 0.047
    center_x = 0.5259758765743129
    center_y = -0.012167901
    center_z = 0.1701256399469593

    # Calculate the minimum and maximum coordinates for the chessboard
    min_x = center_x + (3.5 * side_len)
    min_y = center_y - (3.5 * side_len)


    # Edit chess_table and add the square coordinate information using chess board dimensions 
    for row in range(8):
        for col in range(8):
            x = min_x - (row * side_len)
            y = min_y + (col * side_len)
            z = center_z
            move(group, x,y,z)


if __name__ == '__main__':
    try:
        # main()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("panda_arm")
        group.set_max_velocity_scaling_factor(0.9)
        display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory, queue_size=10)

        get_pos(group)
    except rospy.ROSInterruptException:
        pass



