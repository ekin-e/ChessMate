#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import Twist

def play_rosbag():
    rospy.init_node('play_rosbag', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    bag_file_path = '2023-08-10-10-48-28.bag'
    #rate = rospy.Rate(2)

    with rosbag.Bag(bag_file_path, 'r') as bag:
        
        for topic, msg, t in bag.read_messages(topics=['/turtle1/cmd_vel']):
            """twist_msg = Twist()
            twist_msg.linear.x = msg.linear.x
            twist_msg.linear.y = 0
            twist_msg.linear.z = 0
            twist_msg.angular.x = 0
            twist_msg.angular.y = 0
            twist_msg.angular.z = msg.angular.z
            print(msg.linear.x,msg.linear.y,msg.linear.z,"\n",msg.angular.x,msg.angular.y,msg.angular.z)"""
            pub.publish(msg)
            rospy.sleep(0.1)  # Adjust the sleep duration as needed
            #rate.sleep()
    rospy.signal_shutdown("Finished playing ROS bag.")

if __name__ == '__main__':
    try:
        play_rosbag()
    except rospy.ROSInterruptException:
        pass


