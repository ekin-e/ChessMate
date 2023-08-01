#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from panda_chess.srv import board_sensor
from sensor_msgs.msg import Image
from ultralytics import YOLO
import pyrealsense2
import json
import cv2
from cv_bridge import CvBridge
import os
import numpy as np
import math
import chess.svg

########GLOBAL VARS########
global model, names, detectedList, chess_table, pawn_capture_coord, chess_table_before, chess_table_after, frame, piece_points

########## CHESS BOARD ##########
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

########## CAPTURE SPOTS FOR ROBOT PIECES ##########
capture_table_robot_pieces = []


pawn_capture_coord = (0,0,0)
frame = None
model = YOLO('temmuz25best.pt')
names = model.names

x0 = 160
y0 = 433
x1 = 205
y1 = 138
x2 = 474
y2 = 131
x3 = 540
y3 = 428

polygon_coords_input = [x0,y0,x1,y1,x2,y2,x3,y3]
points = np.array(polygon_coords_input, dtype=np.int32).reshape((-1, 2))


########## CAMERA CLASS ##########
class camera_XYZ:
    def __init__(self):
        savedir="/home/gursel/ekinYeni/ChessMate/camera_calib/camera_data/"
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.cam_mtx=np.load(savedir+'cam_mtx.npy').astype(np.float32)
        self.dist = np.load(savedir + 'dist.npy').astype(np.float32)
        self.objp = np.load(savedir + 'objp.npy').astype(np.float32)
        self.rvec = np.zeros((3, 1), dtype=np.float32)
        self.tvec = np.zeros((3, 1), dtype=np.float32)
        self.Z_world = 0
        self.rotationMatrix = None
        self.square_size = 0.05
        self.height = 0
        self.robot_color = 0 # robot is playing black
    

    def find_max_contour_area(self, contours):
        max_area = 0 - float('inf')
        max_c = None
        for c in contours:
            area = cv2.contourArea(c)
            if area > max_area:
                max_area = area
                max_c = c

        return [max_c]


    def distance(self, p1, p2):
        return math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )


    def find_outer_corners(self, img, pts):
        rows, cols, _ = img.shape
        bl_dst = br_dst = tl_dst = tr_dst = float('inf')
        for p in pts:
            p = p[0]
            if self.distance(p, (cols*0, rows*1)) < bl_dst:
                bl_dst = self.distance(p, (cols*0, rows*1))
                bl = p
            if self.distance(p, (cols*1, rows*1)) < br_dst:
                br_dst = self.distance(p, (cols*1, rows*1))
                br = p
            if self.distance(p, (cols*0, rows*0)) < tl_dst:
                tl_dst = self.distance(p, (cols*0, rows*0))
                tl = p
            if self.distance(p, (cols*1, rows*0)) < tr_dst:
                tr_dst = self.distance(p, (cols*1, rows*0))
                tr = p
        pts1 = np.float32(
            [bl,  # btm left
            br,  # btm right
            tl,  # top left
            tr]  # top right
        )
        return pts1
    

    def do_perspective_transform(self, img, pts, pts_type=1):
        rows, cols = img.shape[:2]
        bl = [cols*0, rows*1] 
        br = [cols*1, rows*1] 
        tl = [cols*0, rows*0] 
        tr = [cols*1, rows*0] 
        pts2 = np.float32([bl, br, tl, tr])
        if pts_type == 2:
            pts, pts2 = pts2, pts

        M = cv2.getPerspectiveTransform(pts,pts2)
        color = 255
        if len(img.shape) == 3:
            color = (255, 255, 255)

        img = cv2.warpPerspective(img, M, (cols, rows), borderValue=color)
        return img, M


    def create_four_points_list(self, img_orig):
        #finding all corner points and listing them by 4 points for each square
        self.height, width, shape = img_orig.shape
        w = int(width/8)
        h = int(self.height/8)
        four_points_list = []
        for multiplier in range(8):
            for multiplier2 in range(8):
                four_point = [(w*multiplier2,h*(multiplier+1)),(w*(multiplier2+1),h*(multiplier+1)),(w*multiplier2,h*multiplier),(w*(multiplier2+1),h*multiplier)]
                four_points_list.append(four_point)
        return four_points_list


    def transform_points_to_original(self, pts, M):
        pts_original = []
        for four_points in pts:
            # Convert the points to homogeneous coordinates
            pts_homogeneous = np.array(four_points, dtype=np.float32)
            # Add an extra dimension to the array
            pts_homogeneous = pts_homogeneous[:, np.newaxis]
            # Apply the inverse perspective transformation to get original points
            pts_transformed = cv2.perspectiveTransform(pts_homogeneous, M)
            # Convert back to pixel coordinates
            pts_original += [(int(pt[0][0]), int(pt[0][1])) for pt in pts_transformed]
        return pts_original


    def inverse_transform(self, four_points_list, M):
        pts_original = []
        new_M = np.linalg.inv(M)
        for pts in four_points_list:
            pts = np.array(pts, dtype=np.float32)
            pts = pts.reshape(-1, 1, 2)
            pts2 = cv2.perspectiveTransform(pts, new_M).tolist()
            pts_original.append(pts2)
        return pts_original
    

    def find_center_point(self, vertices):
        sum_x = 0
        sum_y = 0
        for i in vertices:
            sum_x += i[0][0]
            sum_y += i[0][1]
        avg_x = sum_x/len(vertices)
        avg_y = sum_y/len(vertices)
        return avg_x, avg_y
    

    def scale_coordinates_640_to_orig(self, x, y, image_copy):
        # Scale x coordinate from 640 to 480
        y_480 = int(y * image_copy.shape[0] / 640)
        x_480 = int(x * image_copy.shape[1] / 640)
        return x_480, y_480


    def find_board_coord(self, frame):
        model = YOLO("best(6).pt")
        image_copy = frame.copy()
        image = cv2.resize(image, (640,640), interpolation= cv2.INTER_AREA) #(640,640) ORIGINAL IMAGE
        results = model(image)
        
        for r in results:
            for box_info in range(len(r.boxes)):
                a = r.boxes[box_info].xyxy[0].tolist()
                x1 = round(int(a[0]),2)
                x2 = round(int(a[2]),2)
                
                y1 = round(int(a[1]),2)
                y2 = round(int(a[3]),2)
                top_left = (x1, y1)
                bottom_right = (x2, y2)

        img = image[y1:y2, x1:x2].copy()
        img_orig = img.copy()       
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 9, 3)

        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = self.ind_max_contour_area(contours)

        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)         #CUT AND FILTERED IMAGE
        c = contours[0]
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        pts = self.find_outer_corners(img, approx)

        img_orig, M = self.do_perspective_transform(img_orig, pts)
        four_points_list = self.create_four_points_list(img_orig)
        
        # Convert the points from bird-view perspective back to original image perspective
        pts_original = self.inverse_transform(four_points_list, M)
        center_list = []
        
        for points in pts_original:
            for pt in points:
                pt = (int(pt[0][0]), int(pt[0][1]))
            
            center = self.find_center_point(points)
            center_list += [center]     #MIDDLE POINTS CENTER LIST = center_list

        #from cropped to original 640*640
        #find the width and height  (x1, y1)
        zoomed_points = []
        for center in center_list:
            c = list(center)
            c[0] += x1
            c[1] += y1
            zoomed_points += [[c[0], c[1]]]
        
        row_list = [] #original image original size middle points list
        for pt in zoomed_points:
            x_480, y_480 = self.scale_coordinates_640_to_orig(pt[0], pt[1], image_copy)
            row_list += [(x_480, y_480)]

        middle_points_list = [row_list[:8],row_list[8:16],row_list[16:24],row_list[24:32],row_list[32:40],row_list[40:48],row_list[48:56],row_list[56:64]]
        return middle_points_list


    def calculate_xyz(self, pixel):
        uvPoint = np.array([pixel[0], pixel[1], 1], dtype=np.float64).reshape(3, 1)
        leftSideMat = np.linalg.inv(self.rotationMatrix) @ np.linalg.inv(self.cam_mtx) @ uvPoint
        rightSideMat = np.linalg.inv(self.rotationMatrix) @ self.tvec

        s = (self.Z_world + rightSideMat[2][0]) / leftSideMat[2][0]
        P = np.linalg.inv(self.rotationMatrix) @ (s * np.linalg.inv(self.cam_mtx) @ uvPoint - self.tvec)

        final_P = []
        final_P.append(P[0].item())
        final_P.append(P[1].item())
        final_P.append(P[2].item())

        return np.round(final_P, 3)


    def piece_on_square(self, square_coords, piece_points):
        closest_squares = {}

        for piece_name, piece_center in piece_points:
            closest_square = None
            min_distance = float('inf')

            for square_name, square_center in square_coords:
                distance = self.distance(piece_center, square_center)
                if distance < min_distance:
                    min_distance = distance
                    closest_square = square_name

            closest_squares[piece_name] = closest_square

        return closest_squares


    def add_piece_names_to_squares(self, closest_squares, square_coords):
        for piece_name, closest_square in closest_squares.items():
            for i, (square_name, square_info) in enumerate(square_coords):
                if square_name == closest_square:
                    square_info[0] = piece_name
                    square_coords[i] = (square_name, square_info)
                    break
        return square_coords


    def change_labels(self, r, box_info):
        #changing Roboflow YOLOv8 label names to FEN label names
        roboflow_labels = ["bbishop","bking","bknight","bpawn","bqueen","brook",
                            "wbishop","wking","wknight","wpawn","wqueen","wrook"]
        fen_labels = ["b","k","n","p","q","r","B","K","N","P","Q","R"]
        temporary_piece_names = names[int(r.boxes.cls[box_info])]

        for i in range(len(roboflow_labels)):
            if temporary_piece_names == roboflow_labels[i]:
                piece_names = fen_labels[i]
                break
            else:
                continue
        return piece_names
    
    def piece_pixel_dict(self, middle_points_list):
        keys = []
        fen_dict = {}
        for row in range(8):
            for col in range(8):
                key = chr(ord('a') + row) + chr(ord('1') + col)
                keys.append(key)
                if self.robot_color == 0:
                    fen_dict[key] = ["", middle_points_list[row][7-col]]
                else:
                    fen_dict[key] = ["", middle_points_list[7-row][col]]

        return fen_dict
    
    def convert_to_xyz(self, dict):
        xyz_dict = {}
        for key, value in dict:
            xyz_dict[key] = [value[0], self.calculate_xyz(value[1])]
        return xyz_dict


def is_point_inside_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def imgCallback(msg):
    global frame
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")


camera = camera_XYZ()


########## HELPER FUNCTIONS ##########
def capture_and_detect():
    global model, names, camera
    board_piece = [] # for pieces inside the board
    captured_pieces = [] # for pieces outside the board
    new_frame = frame

    if new_frame is not None:
        results = model(new_frame, conf=0.7)
        # annotated_frame = results[0].plot()
        
        for r in results:
            for box_info in range(len(r.boxes)):
                a = r.boxes[box_info].xyxy[0].tolist()
                class_name = names[int(r.boxes.cls[box_info])]
                top_left = round(int(a[0]),2), round(int(a[1]),2)
                bottom_right = round(int(a[2]),2), round(int(a[3]),2)

                # bounding boxun merkezi
                center_x = (top_left[0] + bottom_right[0]) // 2
                center_y = (top_left[1] + bottom_right[1]) // 2
                center_pixel = np.array([center_x, center_y])
                # heightOfPiece = bottom_right[1] - top_left[1]

                piece_names = camera.change_labels(r, box_info)
                
                if piece_names == "k":
                    if center_pixel[1] < camera.height/2: #height sonradan eklenecek (cornerdetection.py kodu ile)
                        camera.robot_color = 1 #white
                
                if class_name not in detectedList:
                    detectedList.append(class_name)

                center_point = (center_x, center_y)

                if is_point_inside_polygon(center_point, points):
                    print("Class:", class_name, " is inside the polygon.")
                    board_piece.append((piece_names, center_pixel))

                else:
                    print("Class:", class_name, " is OUTSIDE the polygon.")
                    captured_pieces.append((piece_names, center_pixel))
    else:
        print("Couldn't capture frame")
        return -1

    # calculations for chess_table
    middle_points_list = camera.find_board_coord(new_frame)
    square_coords = camera.piece_pixel_dict(middle_points_list)
    closest_squares = camera.piece_on_square(square_coords, board_piece)
    final_pixel_dict = camera.add_piece_names_to_squares(closest_squares, square_coords)
    final_xyz_dict = camera.convert_to_xyz(final_pixel_dict)

    return final_xyz_dict, captured_pieces


# Function for detecting the player side's move by comparing the chessboards before player move and after player move
def detect_player_move(chess_table_before, chess_table_after):
    from_square = None
    to_square = None
    promotion_piece = None

    for square in chess_table_before:
        if chess_table_before[square][0] != chess_table_after[square][0]:
            from_square = square
            break

    for square in chess_table_after:
        if chess_table_before[square][0] != chess_table_after[square][0]:
            to_square = square
            if (to_square[1] == '1' and chess_table_after[to_square][0] == "p") or \
               (to_square[1] == '8' and chess_table_after[to_square][0] == "P"):
                promotion_piece = chess_table_after[to_square][2]
            break

    if promotion_piece:
        move_type = f"{from_square}{to_square}{promotion_piece}"
    else:
        move_type = f"{from_square}{to_square}"

    return move_type


def castling_move(king_move, castle_move):
    data_list = [(king_move[:2], king_move[-2:]), (castle_move[:2], castle_move[-2:])]
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
        goal_piece = chess_table[data_list[1]][0]
        # if one of the players capture another piece update capture_table
        if goal_piece != " ":
            if camera.robot_color == 0: # player is playing white
                if goal_piece.isupper() == False:
                    print("Captured black piece") # player captured robot piece
                    capture_table_robot_pieces.append(goal_piece)

            else: # player is playing black
                if goal_piece.isupper():
                    print("Captured white piece") # player captured robot piece
                    capture_table_robot_pieces.append(goal_piece)

        # the response coming from game_controller should be a legal move (ex: a2a4)
        chess_table[data_list[1]][0] = chess_table[data_list[0]][0]
        chess_table[data_list[0]][0] = " "


########## SERVER FUNCTIONS ##########
def handle_board_sensor(req):
    global chess_table, capture_table_robot_pieces
    # if query is get_chess_table, send the current chess_table
    if req.qry == "get_chess_table":
        chess_table_after, captured_pieces = capture_and_detect()
        chess_table = chess_table_after.copy()
        res = json.dumps(chess_table)

    # if query is update_move, update chess_table with the given parameter (move) then send it
    if req.qry == "update_move":
        # move is in prm
        edit_chess_table(req.prm1)
        res = json.dumps(chess_table)

    # if query is get_capture_piece_coord, get and send the piece's location inside the capture zone
    if req.qry == "get_capture_piece_coord":
        # piece is in prm
        piece_name = req.prm1
        chess_table_after, captured_pieces = capture_and_detect()
        # for value in capture_table_robot_pieces:
        #     if value[0] == piece:
        #         res = json.dumps(value[1])
        #         break
        for piece in captured_pieces:
            if piece[0] == piece_name:
                res = json.dumps(camera.calculate_xyz(piece[1]))
                break

    # if query is get_pawn_box_coord, send the pawn capture zone coordinates
    if req.qry == "get_pawn_box_coord":
        res = json.dumps(pawn_capture_coord)

    # if query is get_robot_color, detect the robot's piece color and send it
    if req.qry == "get_robot_color":
        if camera.robot_color == -1:
            return -1
        res = json.dumps(camera.robot_color) # 0: black, 1: white

    # if query is request_move, capture a frame, compare the old and new chess_tables, update the old chess_table, and send the move information
    if req.qry == "request_move":
        chess_table_after, captured_pieces = capture_and_detect()
        move = detect_player_move(chess_table, chess_table_after)
        res = json.dumps(move)

    return res

def send_chess_table():
    service = rospy.Service('board_sensor', board_sensor, handle_board_sensor)


########## SUBSCRIBED TO TURN_CONTROLLER ##########
# waiting for the player move information
def listener():
    rospy.init_node('board_sensor', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, imgCallback)
    rospy.spin()


########## MAIN ##########
if __name__ == "__main__":
    try:
        send_chess_table()
        listener()
    except rospy.ROSInterruptException:
        pass
