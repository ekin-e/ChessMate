# ChessMate

# Launch gazebo with example controller

roslaunch franka_gazebo panda.launch x:=0 y:=0 z:=0 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=effort_joint_trajectory_controller


# Current errors and TODO
- It automatically assumes that the player is playing the white pieces and robot is playing the black pieces. (will be fixed in the next version)
- It doesn't send robot's first move to motion_controller. Sends rest of the moves correctly.
- I get 2 prompts when I first start turn_controller for the first player move, then it goes back to normal. Doesn't affect the game just need to enter the first move twice
- Gets stuck after checkmate. The is_checkmate() condition doesn't work.
- motion_controller for gazebo: pick and place working but robot makes unnecessary movements.
- Capturing and recovering pieces not fully implemented yet!
- Sending 2 moves for the robot for castling and capturing pieces not implemented yet
- motion_controller for the real robot hasn't been tested yet
- motion_controller needs to send a message to board_sensor indicating that the move is done (V1)


# franka_gazebo

Replace the original franka_gazebo with the one provided here. (Added models for chessboard, pieces etc)


# packages needed
- libfranka
- franka_ros
- stockfish
- python-chess
- roboticstoolbox-python
