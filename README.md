# ChessMate

# Launch gazebo with example controller

roslaunch franka_gazebo panda.launch x:=0 y:=0 z:=0 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=effort_joint_trajectory_controller


# Current errors
- It doesn't send robot's first move to motion_controller. Sends rest of the moves correctly.

- I get 2 prompts when I first start turn_controller for the first player move, then it goes back to normal. Doesn't affect the game just need to enter the first move twice

- Gets stuck after checkmate. The is_checkmate() condition doesn't work.

- motion_controller for gazebo: pick and place not working correctly yet because after IK calculations the joint positions do not match coordinates in gazebo 

- Capturing and recovering pieces not fully implemented yet!

- Sending 2 moves for the robot for castling and capturing pieces not implemented yet because robot can't make 1 pick and place move correctly

# packages needed
- libfranka
- franka_ros
- stockfish
- python-chess
- roboticstoolbox-python
