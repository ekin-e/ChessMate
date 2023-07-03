#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from panda_chess.srv import board_sensor
import json


########## CHESS BOARD ########## 
global chess_table, side_len, center_x, center_y, center_z, min_x, max_x, min_y, max_y

# chess board dimensions and center coordinates
side_len = 0.4 / 8
center_x = 0.49
center_y = 0.0
center_z = 0.021

# Calculate the minimum and maximum coordinates for the chessboard
min_x = center_x - (3.5 * side_len)
min_y = center_y - (3.5 * side_len)

chess_table = {
    "a1": ["r", (0,0,0)],
    "b1": ["n", (0,0,0)],
    "c1": ["b", (0,0,0)],
    "d1": ["q", (0,0,0)],
    "e1": ["k", (0,0,0)],
    "f1": ["b", (0,0,0)],
    "g1": ["n", (0,0,0)],
    "h1": ["r", (0,0,0)],
    "a2": ["p", (0,0,0)],
    "b2": ["p", (0,0,0)],
    "c2": ["p", (0,0,0)],
    "d2": ["p", (0,0,0)],
    "e2": ["p", (0,0,0)],
    "f2": ["p", (0,0,0)],
    "g2": ["p", (0,0,0)],
    "h2": ["p", (0,0,0)],
    "a3": [" ", (0,0,0)],
    "b3": [" ", (0,0,0)],
    "c3": [" ", (0,0,0)],
    "d3": [" ", (0,0,0)],
    "e3": [" ", (0,0,0)],
    "f3": [" ", (0,0,0)],
    "g3": [" ", (0,0,0)],
    "h3": [" ", (0,0,0)],
    "a4": [" ", (0,0,0)],
    "b4": [" ", (0,0,0)],
    "c4": [" ", (0,0,0)],
    "d4": [" ", (0,0,0)],
    "e4": [" ", (0,0,0)],
    "f4": [" ", (0,0,0)],
    "g4": [" ", (0,0,0)],
    "h4": [" ", (0,0,0)],
    "a5": [" ", (0,0,0)],
    "b5": [" ", (0,0,0)],
    "c5": [" ", (0,0,0)],
    "d5": [" ", (0,0,0)],
    "e5": [" ", (0,0,0)],
    "f5": [" ", (0,0,0)],
    "g5": [" ", (0,0,0)],
    "h5": [" ", (0,0,0)],
    "a6": [" ", (0,0,0)],
    "b6": [" ", (0,0,0)],
    "c6": [" ", (0,0,0)],
    "d6": [" ", (0,0,0)],
    "e6": [" ", (0,0,0)],
    "f6": [" ", (0,0,0)],
    "g6": [" ", (0,0,0)],
    "h6": [" ", (0,0,0)],
    "a7": ["P", (0,0,0)],
    "b7": ["P", (0,0,0)],
    "c7": ["P", (0,0,0)],
    "d7": ["P", (0,0,0)],
    "e7": ["P", (0,0,0)],
    "f7": ["P", (0,0,0)],
    "g7": ["P", (0,0,0)],
    "h7": ["P", (0,0,0)],
    "a8": ["R", (0,0,0)],
    "b8": ["N", (0,0,0)],
    "c8": ["B", (0,0,0)],
    "d8": ["Q", (0,0,0)],
    "e8": ["K", (0,0,0)],
    "f8": ["B", (0,0,0)],
    "g8": ["N", (0,0,0)],
    "h8": ["R", (0,0,0)],
}

# Edit chess_table and add the square coordinate information using chess board dimensions 
for key, value in chess_table.items():
    col = ord(key[0]) - ord('a')
    row = int(key[1]) - 1
    x = min_x + (row * side_len)
    y = min_y + (col * side_len)
    z = center_z
    value[1] = (round(x,3), round(y,3), round(z,3))
    
    if (value[0] != " "):
        line = f"<include>\n\
    <uri>model://chess_piece</uri>\n\
    <name>chess_piece_{col+row*8}</name>\n\
    <pose>{round(x,3)+0.01} {round(y,3)} {round(z,3)} 0 0 0</pose>\n\
</include>\n"
        # UNCOMMENT THIS LINE TO PRINT THE CHESS PIECE COORDINATES
        #print(line)

########## CAPTURE SPOTS FOR WHITE PIECES ########## 
capture_table_white = {
    "a1": [" ", (0,0,0)],
    "b1": [" ", (0,0,0)],
    "c1": [" ", (0,0,0)],
    "d1": [" ", (0,0,0)],
    "e1": [" ", (0,0,0)],
    "f1": [" ", (0,0,0)],
    "g1": [" ", (0,0,0)],
    "h1": [" ", (0,0,0)],
    "a2": [" ", (0,0,0)],
    "b2": [" ", (0,0,0)],
    "c2": [" ", (0,0,0)],
    "d2": [" ", (0,0,0)],
    "e2": [" ", (0,0,0)],
    "f2": [" ", (0,0,0)],
    "g2": [" ", (0,0,0)],
    "h2": [" ", (0,0,0)],
}


########## CAPTURE SPOTS FOR BLACK PIECES ########## 
capture_table_black = {
    "a1": [" ", (0,0,0)],
    "b1": [" ", (0,0,0)],
    "c1": [" ", (0,0,0)],
    "d1": [" ", (0,0,0)],
    "e1": [" ", (0,0,0)],
    "f1": [" ", (0,0,0)],
    "g1": [" ", (0,0,0)],
    "h1": [" ", (0,0,0)],
    "a2": [" ", (0,0,0)],
    "b2": [" ", (0,0,0)],
    "c2": [" ", (0,0,0)],
    "d2": [" ", (0,0,0)],
    "e2": [" ", (0,0,0)],
    "f2": [" ", (0,0,0)],
    "g2": [" ", (0,0,0)],
    "h2": [" ", (0,0,0)],
}


########## HELPER FUNCTIONS ##########
def find_empty_spot(dict, new_element):
    for key in dict:
        if dict[key][0] == " ":
            dict[key] = new_element
            break

def castling_move(king_move, castle_move):
    data_list = [(king_move[:2], king_move[-2:]), (castle_move[:2], castle_move[-2:])]
    print(data_list)
    # king move
    chess_table[data_list[0][1]][0] = chess_table[data_list[0][0]][0]
    chess_table[data_list[0][0]][0] = " "
    # castle move
    chess_table[data_list[1][1]][0] = chess_table[data_list[1][0]][0]
    chess_table[data_list[1][0]][0] = " "

def edit_chess_table(data):
    global chess_table
    
    # Kingside castling
    if (data == "e1g1"): # Kingside castling for white
        king_move = "e1g1"
        castle_move = "h1f1"
        castling_move(king_move, castle_move)
    
    elif (data == "e8g8"): # Kingside castling for black
        king_move = "e8g8"
        castle_move = "h8f8"
        castling_move(king_move, castle_move)
    
    # Queenside castling
    elif (data == "e1c1"): # Queenside castling for white
        king_move = "e1c1"
        castle_move = "a1d1"
        castling_move(king_move, castle_move)
    
    elif(data == "e8c8"): # Queenside castling for black
        king_move = "e8c8"
        castle_move = "a8d8"
        castling_move(king_move, castle_move)

    # not castling move
    else:
        data_list = [data[i:i+2] for i in range(0, len(data), 2)]
        print(data_list)
        goal_piece = chess_table[data_list[1]][0]
        # if one of the players capture another piece update capture_table
        if goal_piece != " ":
            if goal_piece.isupper():
                print("Captured black piece")
                find_empty_spot(capture_table_black, goal_piece)
            else:
                print("Captured white piece")
                find_empty_spot(capture_table_white, goal_piece) 
        # the response coming from game_controller should be a legal move (ex: a2a4)
        chess_table[data_list[1]][0] = chess_table[data_list[0]][0]
        chess_table[data_list[0]][0] = " "


########## SERVER FUNCTIONS ##########
def handle_board_sensor(req):
    global chess_table
    # if query is get_chess_table, send the current chess_table
    if req.qry == "get_chess_table":
        res = json.dumps(chess_table)

    # if query is update_move, update chess_table with the given parameter (move) then send it
    if req.qry == "update_move":
        # move is in prm
        edit_chess_table(req.prm1)
        res = json.dumps(chess_table)
    print(chess_table)
    return res

def send_chess_table():
    service = rospy.Service('board_sensor', board_sensor, handle_board_sensor)


########## PUBLISH THE MOVE MADE BY PLAYER TO GAME_CONTROLLER ##########
# version1: won't need this for v2, will update such that camera will handle it
def turn_controller_move(data):
    pub = rospy.Publisher('board_sensor/move', String, queue_size=10)
    pub.publish(data.data)
    rospy.sleep(5)


########## SUBSCRIBED TO TURN_CONTROLLER ##########
# waiting for the player move information
def listener():
    rospy.init_node('board_sensor', anonymous=True)
    rospy.Subscriber("turn_controller/move", String, turn_controller_move)
    rospy.spin()


########## MAIN ########## 
if __name__ == "__main__":
    try:
        send_chess_table()
        listener()
    except rospy.ROSInterruptException:
        pass