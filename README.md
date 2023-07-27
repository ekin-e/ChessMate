# ChessMate

# Launch gazebo with example controller

roslaunch franka_gazebo panda.launch x:=0 y:=0 z:=0 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=effort_joint_trajectory_controller


# Current errors and TODO
- It automatically assumes that the player is playing the white pieces and robot is playing the black pieces. (will be fixed in the next version)
- It doesn't send robot's first move to motion_controller. Sends rest of the moves correctly.
- Gets stuck after checkmate. The is_checkmate() condition doesn't work.
- Capturing and recovering pieces not fully implemented yet!
- motion_controller needs to send a message to board_sensor indicating that the move is done (V1)
- Need to test everything together (might crash and burn)

# V2 TODO
- SETTING UP THE BOARD AND SENDING IT TO GAME CONTROLLER
  - Need to detect the board square length
  - Need to detect who is playing with white and who is playing with black

- DURING THE GAME
  - Need to find a way to detect the player move and send it to game controller in the format a1a2
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
