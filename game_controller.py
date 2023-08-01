#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from panda_chess.srv import board_sensor
from stockfish import Stockfish
# !!!!!change this path
stockfish = Stockfish(path="/home/ekin/catkin_ws/src/panda_chess/Stockfish/src/stockfish")
from enum import Enum
import json
import chess
from std_msgs.msg import Bool

########## GAME STATES ##########
class Game_state(Enum):
    board_unknown = 0
    board_known = 1
    player_turn = 2
    robot_turn = 3
    game_over = 4
    robot_moving = 5


########## GLOBAL VARIABLES ##########
global game_state, chess_table_fen, best_move, piece_coord
chess_table_fen = ""
game_state = Game_state.board_unknown
best_move = ""
board = chess.Board()
piece_coord = {}
is_capture_move = False
is_castling_move = False
is_en_passant_move = False
move_complete = False
robot_color = 0 # 0: black, 1: white


########## HELPER FUNCTIONS FOR CONVERTING THE DICTIONARY TO FEN STRING ##########
def check_castling_availability(board):
    castling_str = ""
    # Check castling rights for White
    white_kingside = board.piece_at(chess.E1) == chess.Piece(chess.KING, chess.WHITE) and \
                    board.piece_at(chess.H1) == chess.Piece(chess.ROOK, chess.WHITE)
    white_queenside = board.piece_at(chess.E1) == chess.Piece(chess.KING, chess.WHITE) and \
                    board.piece_at(chess.A1) == chess.Piece(chess.ROOK, chess.WHITE)

    # Check castling rights for Black
    black_kingside = board.piece_at(chess.E8) == chess.Piece(chess.KING, chess.BLACK) and \
                    board.piece_at(chess.H8) == chess.Piece(chess.ROOK, chess.BLACK)
    black_queenside = board.piece_at(chess.E8) == chess.Piece(chess.KING, chess.BLACK) and \
                    board.piece_at(chess.A8) == chess.Piece(chess.ROOK, chess.BLACK)
    
    # Check if castling is available for black
    if white_kingside:
        castling_str += "K"
    if white_queenside:
        castling_str += "Q"

    # Check if castling is available for white
    if black_kingside:
        castling_str += "k"
    if black_queenside:
        castling_str += "q"
    
    # no availability for either
    if ((black_kingside == False) and (black_queenside == False) and (white_kingside == False) and (white_queenside == False)):
        castling_str = "-"
    
    return castling_str

def chess_table_to_fen(chess_table):
    global board, stockfish, piece_coord
    fen_parts = []
    fen_row = ""
    empty_count = 0

    # store the coordinates
    for key, value in chess_table.items():
        piece_coord[key] = value[1]

    # SETTING UP THE CHESS BOARD (ONLY PIECE INFORMATION)
    for rank in range(8, 0, -1):
        for file in "abcdefgh":
            square = file + str(rank)
            piece, _ = chess_table[square]

            if piece != " ":
                if empty_count > 0:
                    fen_row += str(empty_count)
                    empty_count = 0
                fen_row += piece
            else:
                empty_count += 1

            if len(fen_row) == 8 or (file == "h" and fen_row):
                if empty_count > 0:
                    fen_row += str(empty_count)
                    empty_count = 0
                fen_parts.append(fen_row)
                fen_row = ""

        if empty_count > 0:
            fen_row += str(empty_count)
            empty_count = 0

        if fen_row:
            fen_parts.append(fen_row)
            fen_row = ""

    fen_parts.reverse()
    fen_position = "/".join(fen_parts)
    
    # SETTING OTHER IMPORTANT BOARD INFORMATION
    # active color
    active_color = board.turn
    if active_color == chess.WHITE:
        active_color = "w"
    else:
        active_color = "b"
    
    # casling availability
    castling_availability = check_castling_availability(board)
    
    # - if there is no availability for en passant, the square if there is ex:"e6"
    en_passant_target = board.ep_square
    if en_passant_target is not None:
        en_passant_target = chess.square_name(en_passant_target)
    else:
        en_passant_target = "-"
    
    # The halfmove clock is typically reset to 0 whenever a capture or pawn move is made
    halfmove_clock = str(board.halfmove_clock)

    # It starts at 1 and is incremented after each pair of moves (one by White and one by Black)
    fullmove_number = str(board.fullmove_number)

    fen_update = " ".join([fen_position, active_color, castling_availability, en_passant_target, halfmove_clock, fullmove_number])
    print(stockfish.get_fen_position())
    print(stockfish.get_board_visual())
    return fen_update


########## CLIENT FUNCTIONS ##########
def get_chess_board_client():
    rospy.wait_for_service('board_sensor')
    try:
        # request and return chess_board fen
        b_sensor = rospy.ServiceProxy('board_sensor', board_sensor)
        jsn = b_sensor("get_chess_table", "")
        res = json.loads(jsn.res)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None

def get_robot_color():
    rospy.wait_for_service('board_sensor')
    try:
        # request robot piece color
        print("inside get chess board client")
        b_sensor = rospy.ServiceProxy('board_sensor', board_sensor)
        jsn = b_sensor("get_robot_color", "")
        res = json.loads(jsn.res)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None

def update_board_sensor_move(move):
    rospy.wait_for_service('board_sensor')
    try:
        # request and return chess_board fen
        b_sensor = rospy.ServiceProxy('board_sensor', board_sensor)
        jsn = b_sensor("update_move", move)
        res = json.loads(jsn.res)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None

# Get the piece's coordinates on the capture zone to recover the piece
def get_capture_piece_coord(piece):
    rospy.wait_for_service('board_sensor')
    try:
        b_sensor = rospy.ServiceProxy('board_sensor', board_sensor)
        jsn = b_sensor("get_capture_piece_coord", piece)
        res = json.loads(jsn.res)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None

# Get the pawn box coordinates
def get_pawn_box_coord():
    rospy.wait_for_service('board_sensor')
    try:
        b_sensor = rospy.ServiceProxy('board_sensor', board_sensor)
        jsn = b_sensor("get_pawn_box_coord", "")
        res = json.loads(jsn.res)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None

# Requests the player move from board_sensor
def request_move():
    rospy.wait_for_service('board_sensor')
    try:
        b_sensor = rospy.ServiceProxy('board_sensor', board_sensor)
        jsn = b_sensor("request_move", "")
        res = json.loads(jsn.res)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None


########## BUTTON FOR DETECTING PLAYER OR ROBOT TURN ##########
# edit this with the new button such that it sends True once when it's pressed
def buttonCallback(button_msg):
    if button_msg.data:
        print("pressed the button, it's robot's turn")
        move = request_move()
        update_board(move)


########## FINDS THE BEST MOVE GIVEN CURRENT CHESS BOARD ##########
# sends the move to board_sensor and requests the updated board after
def make_best_move():
    global game_state, best_move, board, stockfish, is_capture_move
    # check if stalemate
    if (board.is_stalemate() == 1):
        print("It's a stalemate")
        game_state = Game_state.game_over

    # check if checkmate
    elif (board.is_checkmate() == 1):
        print("It's a checkmate")
        game_state = Game_state.game_over
    
    else:
        best_move = stockfish.get_best_move()
        # Check if the best move is a capture move
        is_capture_move = board.is_capture(best_move)


########## CHECKS IF PLAYER MOVE IS LEGAL AND REQUESTS UPDATED BOARD ##########
# got move information from board_sensor
def update_board(data):
    global game_state, chess_table_fen, board, stockfish

    # check if move is legal
    if not stockfish.is_move_correct(data.data):
        print(data.data)
        print("Move is illegal, make a different move")

    else:
        # check if stalemate
        if (board.is_stalemate() == 1):
            print("It's a stalemate")
            game_state = Game_state.game_over

        # check if checkmate
        elif (board.is_checkmate() == 1):
            print("It's a checkmate")
            game_state = Game_state.game_over
        
        # correct player move, game continue
        else:
            chess_table = update_board_sensor_move(data.data)
            board.push(chess.Move.from_uci(data.data))
            stockfish.make_moves_from_current_position([data.data])
            chess_table_fen = chess_table_to_fen(chess_table)
            rospy.sleep(3)
            game_state = Game_state.robot_turn


########## CHECKS IF ROBOT IS DONE MOVING ##########
def robot_done_moving(data):
    global move_complete
    move_complete = True


########## GETS THE MOVE COORDINATES INCLUDING CASTLING ##########
def get_move_coord():
    global best_move, piece_coord, is_castling_move, is_en_passant_move
    
    # castling shinenigans
    if (best_move != "e1g1" and best_move != "e8g8" and best_move != "e1c1" and best_move != "e8c8"): # normal move
        move_coord = (piece_coord[best_move[:2]], piece_coord[best_move[-2:]])
        print(move_coord)
        return move_coord

    if board.is_en_passant(best_move):
        is_en_passant_move = True
        pawn_box_coord = get_pawn_box_coord()
        en_passant_target_square = best_move.to_square
        captured_pawn_square = chess.square(en_passant_target_square % 8, en_passant_target_square // 8 - 1)
        move_coord = (piece_coord[captured_pawn_square], pawn_box_coord)
        return move_coord

    elif (best_move == "e1g1"): # Kingside castling for white
        king_move = "e1g1"
        castle_move = "h1f1"
    
    elif (best_move == "e8g8"): # Kingside castling for black
        king_move = "e8g8"
        castle_move = "h8f8"
    
    elif (best_move == "e1c1"): # Queenside castling for white
        king_move = "e1c1"
        castle_move = "a1d1"
    
    elif(best_move == "e8c8"): # Queenside castling for black
        king_move = "e8c8"
        castle_move = "a8d8"

    # return 2 tuples in a list for a castling move
    move_coord = [(piece_coord[king_move[:2]], piece_coord[king_move[-2:]]), (piece_coord[castle_move[:2]], piece_coord[castle_move[-2:]])]
    print("Robot move was castling")
    print(move_coord)
    is_castling_move = True
    return move_coord


########## GAME LOOP ##########
def game_loop(data):
    global game_state, chess_table_fen, best_move, stockfish, move_complete, is_castling_move, is_en_passant_move, piece_coord, robot_color, board
    
    # Board not initialized
    if (game_state == Game_state.board_unknown):
        print("Board not initialized")
        chess_table = get_chess_board_client()
        chess_table_fen = chess_table_to_fen(chess_table)
        board.set_fen(chess_table_fen)
        stockfish.set_fen_position(chess_table_fen)
        robot_color = get_robot_color()
        game_state = Game_state.board_known

    # Initialized board
    elif (game_state == Game_state.board_known):
        print("Initialized board")
        if robot_color == 0: # robot plays black
            game_state = Game_state.player_turn
            print("Player's turn")
        else: # robot plays white
            game_state = Game_state.robot_turn
    
    # Player's turn
    elif (game_state == Game_state.player_turn):
        print("waiting for player to finish their turn")


    # Robot's turn
    elif (game_state == Game_state.robot_turn):
        print("Robot's turn")
        make_best_move()

        pub = rospy.Publisher('game_controller/move', String, queue_size=10)
        move_coord = get_move_coord()
        
        if is_capture_move:
            capture_coord = get_pawn_box_coord()
            first_move = [move_coord[0], capture_coord]
            pub.publish(json.dumps(first_move))
        
        if is_castling_move:
            first_move = [move_coord[0][0], move_coord[0][1]]
            pub.publish(json.dumps(first_move))
        
        if is_promote_move:
            # move from start to pawn_capture zone, then robot capture zone to goal
            capture_coord = get_pawn_box_coord()
            first_move = [move_coord[0], capture_coord]
            pub.publish(json.dumps(first_move))

        else:
            pub.publish(json.dumps(move_coord))

        game_state = Game_state.robot_moving

    # Waiting for the robot to move
    elif (game_state == Game_state.robot_moving):
        # wait until robot is done moving
        if move_complete:
            if is_capture_move:
                pub.publish(json.dumps(move_coord))
                is_capture_move = False
                move_complete = False

            elif is_castling_move:
                castling_second_move = [move_coord[1][0], move_coord[1][1]]
                pub.publish(json.dumps(castling_second_move))
                is_castling_move = False
                move_complete = False

            elif is_en_passant_move:
                en_passant_second_move = (piece_coord[best_move[:2]], piece_coord[best_move[-2:]])
                pub.publish(json.dumps(en_passant_second_move))
                is_en_passant_move = False
                move_complete = False

            elif is_promote_move:
                piece = best_move[-1]
                capture_coords = get_capture_piece_coord(piece)
                promote_second_move = [capture_coords, move_coord[1]]
                pub.publish(json.dumps(promote_second_move))
                is_promote_move = False
                move_complete = False

            else:
                print("Best robot move: " + best_move)
                chess_table = update_board_sensor_move(best_move)
                stockfish.make_moves_from_current_position([best_move])
                board.push(chess.Move.from_uci(best_move))
                chess_table_fen = chess_table_to_fen(chess_table)
                game_state = Game_state.player_turn
                move_complete = False

                # check if stalemate
                if (board.is_stalemate() == 1):
                    print("It's a stalemate")
                    game_state = Game_state.game_over

                # check if checkmate
                elif (board.is_checkmate() == 1):
                    print("It's a checkmate")
                    game_state = Game_state.game_over

                else:    
                    print("Player's turn")

    # game finished
    elif (game_state == Game_state.game_over):
        print("Game finished")
        rospy.signal_shutdown()


########## SUBSCRIBED TO BOARD_SENSOR ##########
def listener():
    rospy.Subscriber("motion_controller/robot_done_moving", String, robot_done_moving)
    rospy.Subscriber("arduinoTopic", Bool, buttonCallback)
    timer = rospy.Timer(rospy.Duration(0.1), game_loop)
    rospy.spin()


########## MAIN ##########
if __name__ == '__main__':
    print("Inside game_controller main")
    rospy.init_node('game_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    listener()