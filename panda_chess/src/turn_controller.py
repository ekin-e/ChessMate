#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


########## PUBLISH THE PLAYER MOVE TO BOARD_SENSOR ##########
def talker(data):
    pub = rospy.Publisher('turn_controller/move', String, queue_size=10)
    move = input("Please enter move in the format a1a2:\n")
    pub.publish(move)
    rospy.sleep(1)


########## SUBSCRIBE TO GAME_CONTROLLER ##########
def listener():
    rospy.init_node('turn_controller', anonymous=True)
    rospy.Subscriber("game_controller/turn", String, talker)
    rospy.spin()


########## MAIN ##########
if __name__ == '__main__':
    listener()