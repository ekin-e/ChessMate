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


########## GAME STATES ##########
class Game_state(Enum):
    board_unknown = 0
    board_known = 1
    player_turn = 2
    robot_turn = 3
    game_over = 4
    robot_moving = 5
    robot_done_moving = 6


########## GLOBAL VARIABLES ##########
global game_state, chess_table_fen, best_move, piece_coord
chess_table_fen = ""
game_state = Game_state.board_unknown
best_move = ""
board = chess.Board()
piece_coord = {}

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
        print("inside get chess board client")
        b_sensor = rospy.ServiceProxy('board_sensor', board_sensor)
        jsn = b_sensor("get_chess_table", "")
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


########## FINDS THE BEST MOVE GIVEN CURRENT CHESS BOARD ##########
# sends the move to board_sensor and requests the updated board after
def make_best_move():
    global game_state, best_move, board, stockfish
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
        game_state = Game_state.robot_moving


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


########## GETS THE MOVE COORDINATES INCLUDING CASTLING ##########
def get_move_coord():
    global best_move, piece_coord
    
    # castling shinenigans
    if (best_move != "e1g1" and best_move != "e8g8" and best_move != "e1c1" and best_move != "e8c8"): # normal move
        move_coord = (piece_coord[best_move[:2]], piece_coord[best_move[-2:]])
        print(move_coord)
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
    return move_coord


########## GAME LOOP ##########
def game_loop(data):
    global game_state, chess_table_fen, best_move, stockfish
    
    # Board not initialized
    if (game_state == Game_state.board_unknown):
        print("Board not initialized")
        chess_table = get_chess_board_client()
        chess_table_fen = chess_table_to_fen(chess_table)
        board.set_fen(chess_table_fen)
        stockfish.set_fen_position(chess_table_fen)
        game_state = Game_state.board_known
    
    # Initialized board
    elif (game_state == Game_state.board_known):
        print("Initialized board")
        # check player and robot piece colors in v2
        game_state = Game_state.player_turn
        print("Player's turn")
    
    # Player's turn
    elif (game_state == Game_state.player_turn):
        # pub to turn controller and wait for player turn
        pub = rospy.Publisher('game_controller/turn', String, queue_size=10)
        pub.publish("")
    
    # Robot's turn
    elif (game_state == Game_state.robot_turn):
        print("Robot's turn")
        make_best_move()

    # Waiting for the robot to move
    elif (game_state == Game_state.robot_moving):
        # print("waiting for the robot to move")
        move_coord = get_move_coord()
        pub = rospy.Publisher('game_controller/move', String, queue_size=10)
        pub.publish(json.dumps(move_coord))
        # wait until robot is done moving
        rospy.sleep(10)
        # might subscribe to board_sensor instead of waiting here in the later versions
        print("Best robot move: " + best_move)
        chess_table = update_board_sensor_move(best_move)
        stockfish.make_moves_from_current_position([best_move])
        board.push(chess.Move.from_uci(best_move))
        chess_table_fen = chess_table_to_fen(chess_table)
        game_state = Game_state.player_turn
        print("Player's turn")
    
    # game finished
    elif (game_state == Game_state.game_over):
        print("Game finished")
        rospy.signal_shutdown()


########## SUBSCRIBED TO BOARD_SENSOR ##########
def listener():
    rospy.init_node('game_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    rospy.Subscriber("board_sensor/move", String, update_board)
    timer = rospy.Timer(rospy.Duration(0.1), game_loop)
    rospy.spin()


########## MAIN ##########
if __name__ == '__main__':
    print("Inside game_controller main")
    listener()
