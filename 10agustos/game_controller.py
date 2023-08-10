#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from panda_chess.srv import board_sensor
from stockfish import Stockfish
# !!!!!change this path
stockfish = Stockfish(path="/home/kovan-robot/panda_chessmate/stockfish_14_linux_x64/stockfish_14_x64")
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
global game_state, best_move, piece_coord, move_coord
game_state = Game_state.board_unknown
best_move = ""
board = chess.Board()
piece_coord = {}
is_capture_move = False
is_castling_move = False
is_en_passant_move = False
is_promote_move = False
move_complete = False
robot_color = 0 # 0: black, 1: white
move_coord = None


########## HELPER FUNCTIONS FOR CONVERTING THE DICTIONARY TO FEN STRING ##########
def is_valid_fen(fen):
    # Split the FEN string into its components
    fen_parts = fen.split()

    # Check if there is exactly one part in the FEN string
    if len(fen_parts) != 1:
        print("len fen parts error")
        return False

    # Check the piece positions part
    piece_positions = fen_parts[0]
    rows = piece_positions.split('/')

    # Check that there are exactly 8 rows
    if len(rows) != 8:
        print("len fen rows error")
        return False

    # Check each row to ensure it has a valid piece count
    piece_counts = {piece: 0 for piece in "prnbqkPRNBQK"}
    for row in rows:
        count = 0
        for char in row:
            if char in piece_counts:
                piece_counts[char] += 1
                count += 1
            elif char.isdigit():
                count += int(char)
        if count != 8:
            print("count 8 error")
            return False

    # Check if the piece counts exceed the maximum allowed counts
    max_allowed_counts = {"p": 8, "n": 2, "b": 2, "r": 2, "q": 1, "k": 1, "P": 8, "N": 2, "B": 2, "R": 2, "Q": 1, "K": 1}
    for piece, count in piece_counts.items():
        if count > max_allowed_counts[piece]:
            print("max allowed counts error")
            return False

    return True


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

def chess_table_to_fen(chess_table, set_init_fen):
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

    fen_position = "/".join(fen_parts)

    #if is_valid_fen(fen_position) == False:
        #print("incorrect fen postitin")
        #return -1

    if set_init_fen:
        board.set_fen(fen_position)
    
    # SETTING OTHER IMPORTANT BOARD INFORMATION
    # active color
    # THE ACTIVE COLOR WILL ALWAYS START AS WHITE BECAUSE THERE IS NO WAY TO DETERMINE WHO'S TURN IT IS GIVEN A RANDOM BOARD STATE
    active_color = board.turn
    if active_color == chess.WHITE:
        active_color = "w"
    else:
        active_color = "b"

    print("active color", active_color)
    
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

    if set_init_fen:
        stockfish.set_fen_position(fen_update)
    
    print(stockfish.get_fen_position())
    print(stockfish.get_board_visual())
    return fen_update


########## DETECT PLAYER MOVE #########
def find_move_between_fen(board_before, fen_after):
    # Create board object from the FEN strings
    board_after = chess.Board(fen_after.split()[0])
    f2 = board_after.fen().split()[0]

    # for debugging delete these later!!!!
    print(board_before.fen())
    print(board_after.fen())

    # Generate all legal moves on the starting board for both White and Black
    all_moves = list(board_before.legal_moves)

    # Check if the move is valid and leads to the ending board
    for move in all_moves:
        board_copy = board_before.copy()
        board_copy.push(move)
        if board_copy.fen().split()[0] == f2:
            return move.uci()

    # If no move is found, return None
    return -1


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
        print("inside request move")
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
    global board
    if button_msg.data:
        print("Player pressed the button, it's robot's turn")
        chess_table_after = request_move()
        if chess_table_after != None: 
            chess_table_after_fen = chess_table_to_fen(chess_table_after, False)
            print("new detected chessboard after player move: ", chess_table_after_fen)
            move = find_move_between_fen(board, chess_table_after_fen)
            print("Player move: ", move)
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
        best_move_uci = chess.Move.from_uci(best_move)
        # Check if the best move is a capture move
        target_square = best_move_uci.to_square
        if board.piece_at(target_square) != None:
            is_capture_move = True
        else:
            is_capture_move = False

        #is_capture_move = board.is_capture(best_move_uci)
        print("is_capture_move", is_capture_move)


########## CHECKS IF PLAYER MOVE IS LEGAL AND REQUESTS UPDATED BOARD ##########
# got move information from board_sensor
def update_board(data):
    global game_state, board, stockfish
    move = data

    # check if move is legal
    if move == -1:
        print("Move is illegal, make a different move")

    elif not stockfish.is_move_correct(move):
        print(move)
        print("Move is illegal, make a different move")
        ## ADD HRI STUFF HERE

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
            print("Player move is: ", move)
            chess_table = update_board_sensor_move(move)
            board.push(chess.Move.from_uci(move))
            stockfish.make_moves_from_current_position([move])
            rospy.sleep(3)
            game_state = Game_state.robot_turn


########## CHECKS IF ROBOT IS DONE MOVING ##########
def robot_done_moving(data):
    print(data.data)
    global move_complete
    move_complete = True


########## GETS THE MOVE COORDINATES INCLUDING CASTLING ##########
def get_move_coord():
    global best_move, piece_coord, is_castling_move, is_en_passant_move, move_coord
    
    # castling shinenigans
    if (best_move != "e1g1" and best_move != "e8g8" and best_move != "e1c1" and best_move != "e8c8"): # normal move
        move_coord = (piece_coord[best_move[:2]], piece_coord[best_move[-2:]])
        print("Robot move is a regular move")
        return move_coord

    if board.is_en_passant(best_move):
        is_en_passant_move = True
        pawn_box_coord = get_pawn_box_coord()
        
        if pawn_box_coord != -1:
            en_passant_target_square = best_move.to_square
            en_passant_begin_square = best_move.from_square
            move_dir = en_passant_begin_square // 8 - en_passant_target_square // 8
            
            captured_pawn_square = chess.square(en_passant_target_square % 8, en_passant_target_square // 8 + move_dir)
            move_coord = (piece_coord[captured_pawn_square], pawn_box_coord)
            print("Robot move is en passant")
            return move_coord
        
        else:
            print("board_sensor could not return the pawn box coordinates correctly")

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
    print("Robot move is castling")
    print(move_coord)
    is_castling_move = True
    return move_coord


########## GAME LOOP ##########
def game_loop(data):
    global game_state, best_move, stockfish, move_complete, is_castling_move, is_en_passant_move, piece_coord, robot_color, board, is_capture_move, is_promote_move, move_coord
    pub = rospy.Publisher('game_controller/move', String, queue_size=10)
    # Board not initialized
    if (game_state == Game_state.board_unknown):
        print("Board not initialized")
        chess_table = get_chess_board_client()
        robot_color = get_robot_color()
        print("robot color: ", robot_color)
        if chess_table == None or robot_color == None:
            print("Board incorrectly detected. Requesting again")
        
        else:
            # check if chess_table and robot_color are correct if not request again
            chess_table_fen = chess_table_to_fen(chess_table, True)
            if chess_table_fen != -1 and robot_color != -1:
                print("Initial board position: ", chess_table_fen)
                game_state = Game_state.board_known
                
            else:
                print("Board pieces not correctly detected")


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
        print("waiting for player to finish their turn...")


    # Robot's turn
    elif (game_state == Game_state.robot_turn):
        print("Robot's turn")
        make_best_move()
        print("Robot move is: ", best_move)

        #pub = rospy.Publisher('game_controller/move', String, queue_size=10)
        move_coord = get_move_coord()
        
        if is_capture_move:
            capture_coord = get_pawn_box_coord()
            
            if capture_coord != -1:
                first_move = [move_coord[1], capture_coord]
                pub.publish(json.dumps(first_move))
                print("Robot move is a capture move. Publishing to motion_controller")

            else:
                print("board_sensor could not return the pawn box coordinates correctly")
        
        elif is_castling_move:
            first_move = [move_coord[0][0], move_coord[0][1]]
            pub.publish(json.dumps(first_move))
            print("Robot move is a castling move. Publishing to motion_controller")
        
        elif is_promote_move:
            # move from start to pawn_capture zone, then robot capture zone to goal
            capture_coord = get_pawn_box_coord()

            if capture_coord != -1:
                first_move = [move_coord[0], capture_coord]
                pub.publish(json.dumps(first_move))
                print("Robot move is a promotion move. Publishing to motion_controller")

            else:
                print("board_sensor could not return the pawn box coordinates correctly")

        # en passant or normal move
        else:
            pub.publish(json.dumps(move_coord))
            print("Robot move is a normal or an en passant move. Publishing to motion_controller")

        game_state = Game_state.robot_moving


    # Waiting for the robot to move
    elif (game_state == Game_state.robot_moving):
        # wait until robot is done moving
        if move_complete:
            if is_capture_move:
                pub.publish(json.dumps(move_coord))
                is_capture_move = False
                move_complete = False
                print("Robot is done capturing the piece. Making the second move. Publishing to motion_controller")

            elif is_castling_move:
                castling_second_move = [move_coord[1][0], move_coord[1][1]]
                pub.publish(json.dumps(castling_second_move))
                is_castling_move = False
                move_complete = False
                print("Robot is done with first castling move. Making the second move. Publishing to motion_controller")

            elif is_en_passant_move:
                en_passant_second_move = (piece_coord[best_move[:2]], piece_coord[best_move[-2:]])
                pub.publish(json.dumps(en_passant_second_move))
                is_en_passant_move = False
                move_complete = False
                print("Robot is done capturing the pawn. Making the second move. Publishing to motion_controller")

            elif is_promote_move:
                piece = best_move[-1]
                capture_coords = get_capture_piece_coord(piece)

                # check if board_sensor returned the piece coordinates correctly
                if capture_coord != -1:
                    promote_second_move = [capture_coords, move_coord[1]]
                    pub.publish(json.dumps(promote_second_move))
                    is_promote_move = False
                    move_complete = False
                    print("Robot is done capturing the pawn. Making the promotion move. Publishing to motion_controller")
                
                else:
                    print("board_sensor could not return the piece coordinates correctly. Trying again")
            
            # end of robot moves
            else:
                print("Robot is done moving. Best robot move: " + best_move)
                chess_table = update_board_sensor_move(best_move)
                stockfish.make_moves_from_current_position([best_move])
                board.push(chess.Move.from_uci(best_move))
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
                    game_state = Game_state.player_turn


    # game finished
    elif (game_state == Game_state.game_over):
        print("Game is finished")
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