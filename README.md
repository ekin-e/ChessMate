# ChessMate

# Launch gazebo with example controller

roslaunch franka_gazebo panda.launch x:=0 y:=0 z:=0 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=effort_joint_trajectory_controller


# Current errors and TODO
- It doesn't send robot's first move to motion_controller. Sends rest of the moves correctly.

# V2 TODO
- SETTING UP THE BOARD AND SENDING IT TO GAME CONTROLLER
  - Need to detect the board square length

- DURING THE GAME
  - tracking

# franka_gazebo

Replace the original franka_gazebo with the one provided here. (Added models for chessboard, pieces etc)


# Needed packages
- libfranka
- franka_ros
- stockfish
- python-chess
- roboticstoolbox-python

# UML
![UML](ChessMateV1.png)
