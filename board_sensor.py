#!/usr/bin/env python3
import rospy
from panda_chess.srv import board_sensor
from panda_chess.srv import send_board
from sensor_msgs.msg import Image
from ultralytics import YOLO
import json
import cv2
from cv_bridge import CvBridge
import numpy as np
import math


########GLOBAL VARS########
global model, names, max_allowed_counts, chess_table, pawn_capture_coord, chess_table_before, chess_table_after, frame, piece_points, corner

########## CHESS BOARD ##########
chess_table = {}

########## CAPTURE SPOTS FOR ROBOT PIECES ##########
capture_table_robot_pieces = []

pawn_capture_coord = (0,0,0)
frame = None
model = YOLO('/home/gursel/ultralytics/agustos3best2.pt')
names = model.names

# Set the maximum number of allowed pieces for each class (e.g., 1 for "k" and 8 for "p")
max_allowed_counts = {"p": 8, "n": 2, "b": 2, "r": 2, "q": 1, "k": 1, "P": 8, "N": 2, "B": 2, "R": 2, "Q": 1, "K": 1}


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
        self.rotationMatrix = np.zeros((3, 3), dtype=np.float64)
        # Convert rvec to rotationMatrix using Rodrigues
        cv2.Rodrigues(self.rvec, self.rotationMatrix)
        self.square_size = 0.05
        self.height = 0
        self.robot_color = 0 # robot is playing black
        self.points = None

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
        dis = np.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))
        return int(dis)

    def find_outer_corners(self, img, pts):
        rows, cols, _ = img.shape
        bl_dst = br_dst = tl_dst = tr_dst = 9999999
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
        rospy.sleep(3)
        bl = [cols*0, rows*1]
        br = [cols*1, rows*1]
        tl = [cols*0, rows*0]
        tr = [cols*1, rows*0]
        pts2 = np.float32([bl, br, tl, tr])

        M = cv2.getPerspectiveTransform(pts,pts2)
        color = 255
        if len(img.shape) == 3:
            color = (255, 255, 255)

        img = cv2.warpPerspective(img, M, (cols, rows), borderValue=color)
        return img, M


    def create_four_points_list(self, img_orig):
        global corner
        #finding all corner points and listing them by 4 points for each square
        y_bird, x_bird, dim = img_orig.shape
        self.height, width, shape = img_orig.shape
        w = int(width/8)
        h = int(self.height/8)
        four_points_list = []
        corner = [(0,y_bird), (x_bird,y_bird), (0,0), (x_bird,0)]
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


    def transform_corner(self, pts, M):
        corner_original = []
        new_M = np.linalg.inv(M)
        
        pts = np.array(pts, dtype=np.float32)
        pts = pts.reshape(-1, 1, 2)
        pts2 = cv2.perspectiveTransform(pts, new_M).tolist()
        corner_original.append(pts2)
        return corner_original


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


    def find_board_coord(self, frame, x1, x2, y1, y2):
        global corner

        image_copy = frame.copy()
        image = cv2.resize(frame, (640,640), interpolation= cv2.INTER_AREA) #(640,640) ORIGINAL IMAGE

        img = image[y1:y2, x1:x2].copy()
        cv2.imshow("cut", img)
        img_orig = img.copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, -5)
        cv2.imshow("img", img)
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = self.find_max_contour_area(contours)

        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)         #CUT AND FILTERED IMAGE
        c = contours[0]
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        pts = self.find_outer_corners(img, approx)

        # if M det is 0 break and do the process again
        img_orig, M = self.do_perspective_transform(img_orig, pts)
        four_points_list = self.create_four_points_list(img_orig)

        # create the corner points and set the polygon_coords_input and points
        polygon_coords_input = []

        corner_original = self.transform_corner(corner, M)

        for a in corner_original:
            for b in a:
                b[0][0] += x1
                b[0][1] += y1
                x_o , y_o = self.scale_coordinates_640_to_orig(b[0][0], b[0][1],image_copy)
                polygon_coords_input.append(x_o)
                polygon_coords_input.append(y_o)

        self.points = np.array(polygon_coords_input, dtype=np.int32).reshape((-1, 2))
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
        """ uvPoint = np.array([pixel[0], pixel[1], 1], dtype=np.float64).reshape(3, 1)
        leftSideMat = np.linalg.inv(self.rotationMatrix) @ np.linalg.inv(self.cam_mtx) @ uvPoint
        rightSideMat = np.linalg.inv(self.rotationMatrix) @ self.tvec

        s = (self.Z_world + rightSideMat[2][0]) / leftSideMat[2][0]
        P = np.linalg.inv(self.rotationMatrix) @ (s * np.linalg.inv(self.cam_mtx) @ uvPoint - self.tvec)
        print("p", P) """
        final_P = []
        """ final_P.append(P[0].item())
        final_P.append(P[1].item())
        final_P.append(P[2].item()) """
        final_P.append(0)
        final_P.append(0)
        final_P.append(0)

        return final_P


    def piece_on_square(self, square_coords, piece_points):
        closest_squares = {}

        for piece_name, piece_center in piece_points:

            closest_square = None
            min_distance = 99999

            for square_name, square_center in square_coords.items():

                distance = self.distance(piece_center, list(square_center[1]))
                if distance < min_distance:
                    min_distance = distance
                    closest_square = square_name

            closest_squares[piece_name] = closest_square

        return closest_squares


    def add_piece_names_to_squares(self, closest_squares, square_coords):
        for piece_name, closest_square in closest_squares.items():
            for key, square_info in square_coords.items():
                if key == closest_square:
                    square_info[0] = piece_name
                    break
        return square_coords
    
    
    def piece_pixel_dict(self, middle_points_list):
        keys = []
        fen_dict = {}
        for row in range(8):
            for col in range(8):
                key = chr(ord('a') + row) + chr(ord('1') + col)
                keys.append(key)
                if self.robot_color == 0:
                    fen_dict[key] = [" ", middle_points_list[row][7-col]]
                else:
                    fen_dict[key] = [" ", middle_points_list[7-row][col]]

        return fen_dict
    
    def convert_to_xyz(self, dict):
        xyz_dict = {}
        for key, value in dict.items():
            xyz_dict[key] = [value[0], self.calculate_xyz(list(value[1]))]
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

## CHANGE FPS HERE!!!!!!!!!!!!!!!! 1 fps yeterli
def imgCallback(msg):
    global frame
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")


camera = camera_XYZ()

########## HELPER FUNCTIONS ##########
def capture_and_detect():
    global model, names, camera, frame
    board_piece = [] # for pieces inside the board
    captured_pieces = [] # for pieces outside the board
    if frame is None:
        return -1
    new_frame = frame.copy() #!!!!!!!!

    
    
    if new_frame is not None:
        chess_board_corners = get_chess_corners_client()
        x1 = chess_board_corners[0]
        x2 = chess_board_corners[1]
        y1 = chess_board_corners[2]
        y2 = chess_board_corners[3]
        

        # calculations for chess_table
        middle_points_list = camera.find_board_coord(new_frame, x1, x2, y1, y2)
        results = model(new_frame, conf=0.65)
        camera.points[[2,3]] = camera.points[[3,2]]
        cv2.polylines(new_frame, [camera.points], isClosed=True, color=(150, 220, 100), thickness=2)
        mask = np.zeros(new_frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [camera.points], (255, 255, 255))
        roi = cv2.bitwise_and(new_frame, new_frame, mask=mask)
        
        for r in results:
            for box_info in range(len(r.boxes)):
                a = r.boxes[box_info].xyxy[0].tolist()
                class_name = names[int(r.boxes.cls[box_info])]
                #confidence = r.boxes[box_info].confidence.item()  # Assuming the confidence is a scalar value

                #a = results[box_info].boxes[0].xyxy[0].tolist()
                top_left = round(int(a[0]),2), round(int(a[1]),2)
                bottom_right = round(int(a[2]),2), round(int(a[3]),2)

                # bounding boxun merkezi
                center_x = (top_left[0] + bottom_right[0]) // 2
                center_y = (top_left[1] + bottom_right[1]) // 2
                center_pixel = np.array([center_x, center_y])
                # heightOfPiece = bottom_right[1] - top_left[1]

                if class_name == "k":
                    if center_pixel[1] < camera.height / 2:
                        camera.robot_color = 1 # white !!!!!!!!

                center_point = (center_x, center_y)

                if is_point_inside_polygon(center_point, camera.points):
                    print("Class:", class_name, " is inside the polygon.")
                    board_piece.append((class_name, center_pixel))

                else:
                    print("Class:", class_name, " is OUTSIDE the polygon.")
                    captured_pieces.append((class_name, center_pixel))
        cv2.imshow("Selected ROI", roi)
        cv2.imshow("frme", new_frame)

        cv2.waitKey(0)

        
        cv2.destroyAllWindows()
        
    else:
        print("Couldn't capture frame")
        # if -1 then client should request again !!!!!!!!!!!
        return -1

    # for board detection
    # get bounding box corners from server
    
    square_coords = camera.piece_pixel_dict(middle_points_list)
    closest_squares = camera.piece_on_square(square_coords, board_piece)
    final_pixel_dict = camera.add_piece_names_to_squares(closest_squares, square_coords)
    final_xyz_dict = camera.convert_to_xyz(final_pixel_dict)

    return final_xyz_dict, captured_pieces


# Function for detecting the player side's move by comparing the chessboards before player move and after player move
#!!!!!!!!!!!!!


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
        print(chess_table)
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
    
        for piece in captured_pieces:
            if piece[0] == piece_name:
                res = json.dumps(camera.calculate_xyz(piece[1]))
                break
            else:
                res = json.dumps(-1)
                break

    # if query is get_pawn_box_coord, send the pawn capture zone coordinates
    if req.qry == "get_pawn_box_coord":
        res = json.dumps(pawn_capture_coord)

    # if query is get_robot_color, detect the robot's piece color and send it
    if req.qry == "get_robot_color":
        if camera.robot_color == -1:
            res = json.dumps(camera.robot_color) # couldn't detect color
        else:
            res = json.dumps(camera.robot_color) # 0: black, 1: white

    # if query is request_move, capture a frame, compare the old and new chess_tables, update the old chess_table, and send the move information
    if req.qry == "request_move":
        chess_table_after, captured_pieces = capture_and_detect()
        res = json.dumps(chess_table_after)

    return res


########## CLIENT FUNCTIONS ##########
def get_chess_corners_client():
    rospy.wait_for_service('send_board')
    
    try:
        # request and return chess_board fen
        b_sensor = rospy.ServiceProxy('send_board', send_board)
        jsn = b_sensor("send_board_corners", "")
        res = json.loads(jsn.res)
        return res
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None


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
