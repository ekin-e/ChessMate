import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2

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

    def find_board_coord(self, corners_new, gray_new):
            # Calculate the average (x, y) pixel coordinates of all detected corners
            center_x = np.mean(corners_new[:, 0, 0])
            center_y = np.mean(corners_new[:, 0, 1])
            center_pixel = np.array([center_x, center_y])
            print("center pixel")
            print(center_pixel)
            # Refine the detected corners to improve accuracy
            corners_refined = cv2.cornerSubPix(gray_new, corners_new, (11, 11), (-1, -1), self.criteria)
            # Find the pose (rvec, tvec) of the camera in the new image
            ret, self.rvec, self.tvec = cv2.solvePnP(self.objp, corners_refined, self.cam_mtx, self.dist)
            # Initialize rotationMatrix
            self.rotationMatrix = np.zeros((3, 3), dtype=np.float64)
            # Convert rvec to rotationMatrix using Rodrigues
            cv2.Rodrigues(self.rvec, self.rotationMatrix)
            P = self.calculate_xyz(center_pixel)
            return P

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
    
    def board_squares(self, center_P):
        square_coords = {}
        center_x = center_P[0]
        center_y = center_P[1]
        center_z = self.Z_world
        print(center_x, center_y)

        min_x = center_x + (3.5 * self.square_size)
        min_y = center_y - (3.5 * self.square_size)

        for row in range(8):
            for col in range(8):
                print("start")
                x = np.round(min_x - (row * self.square_size), 3)
                y = np.round(min_y + (col * self.square_size), 3)
                z = center_z
                key = chr(ord('a') + row) + chr(ord('1') + col)
                square_coords[key] = ["", (x,y,z)]

        return square_coords
    
    def euclidean_distance(self, p1, p2):
        return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    
    def piece_on_square(self, square_coords, piece_points):
        closest_squares = {}

        for piece_name, piece_center in piece_points:
            closest_square = None
            min_distance = float('inf')

            for square_name, square_center in square_coords:
                distance = self.euclidean_distance(piece_center, square_center)
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


global model, names, piece_points, depth_frame, color_frame, clicked_points
model = YOLO('temmuz25best.pt')
names = model.names
piece_points = []
camera = camera_XYZ()

def capture_and_detect():
    global model, names, piece_points, camera
    cap = cv2.VideoCapture(4)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break
        # Display the frame
        cv2.imshow("Camera", frame)

        gray_new = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners_new = cv2.findChessboardCorners(gray_new, (7,7), None)
        
        if ret:
            P = camera.find_board_coord(corners_new, gray_new)
        else:
            print("couldn't find corners trying again")
            continue

        print("corner center")
        print(P)
        square_coords = camera.board_squares(P)
        
        results = model(frame)
        #annotated_frame = results[0].plot()

        for r in results:
            for box_info in range(len(r.boxes)):
                a = r.boxes[box_info].xyxy[0].tolist()
                print("Top-left corner:", round(int(a[0]),2), round(int(a[1]),2))
                print("Bottom-right corner:", round(int(a[2]),2), round(int(a[3]),2))


                top_left = round(int(a[0]),2), round(int(a[1]),2)
                bottom_right = round(int(a[2]),2), round(int(a[3]),2)
                color = (255,255,255) 
                thickness = 1
                cv2.rectangle(frame, top_left, bottom_right, color, thickness)

                # bounding boxun merkezi
                center_x = (top_left[0] + bottom_right[0]) // 2
                center_y = (top_left[1] + bottom_right[1]) // 2
                center_pixel = np.array([center_x, center_y])
                center_xyz = camera.calculate_xyz(center_pixel)

                piece_names = names[int(r.boxes.cls[box_info])]

                piece_points.append((piece_names, center_xyz))

                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
                heightOfPiece = bottom_right[1] - top_left[1]

                text = f"x1, y1: ({round(int(a[0]),2)}, {round(int(a[1]),2)})"
                cv2.putText(frame, text, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color , thickness, cv2.LINE_AA)

                text = f"x2, y2 ({round(int(a[2]),2)}, {round(int(a[3]),2)})"
                cv2.putText(frame, text, (bottom_right[0], bottom_right[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color , thickness, cv2.LINE_AA)

            cv2.imshow("YOLOv8 Inference", frame)
            cv2.waitKey(0)
        break

    cv2.destroyAllWindows()
    closest_squares = camera.piece_on_square(square_coords, piece_points)
    final_square_coords = camera.add_piece_names_to_squares(closest_squares, square_coords)

    # return detected piece points
    # return piece_points
    return final_square_coords



if __name__ == '__main__':
    # piece_points = capture_and_detect()
    final_square_coords = capture_and_detect()
    print(final_square_coords)