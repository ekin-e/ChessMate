import cv2
import numpy as np
# from ultralytics import YOLO
# import pyrealsense2
# from realsense_depth import *

class camera_XYZ:
    def __init__(self):
        savedir="home/ekin/catkin_ws/src/panda_chess/camera_calib/camera_data/"
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.cam_mtx=np.load(savedir+'cam_mtx.npy')
        self.dist=np.load(savedir+'dist.npy')
        self.objp=np.load(savedir+'objp.npy')
        self.rvec = None
        self.tvec = None
        self.Z_world = 0
        self.rotationMatrix = None

    def find_board_coord(self, frame):
        gray_new = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners_new = cv2.findChessboardCorners(gray_new, (7, 7), None)
        if ret:
            # Calculate the average (x, y) pixel coordinates of all detected corners
            center_x = np.mean(corners_new[:, 0, 0])
            center_y = np.mean(corners_new[:, 0, 1])
            center_pixel = np.array([center_x, center_y])
            # Refine the detected corners to improve accuracy
            corners_refined = cv2.cornerSubPix(gray_new, corners_new, (11, 11), (-1, -1), self.criteria)
            # Find the pose (rvec, tvec) of the camera in the new image
            ret, self.rvec, self.tvec = cv2.solvePnP(self.objp[0], corners_refined, self.cam_mtx, self.dist)
            # Initialize rotationMatrix
            rotationMatrix = np.zeros((3, 3), dtype=np.float64)
            # Convert rvec to rotationMatrix using Rodrigues
            cv2.Rodrigues(self.rvec, self.rotationMatrix)
            P = self.calculate_xyz(center_pixel)
            print("3D World Coordinates:", P)
            return P


    def calculate_xyz(self, pixel):
        uvPoint = np.array([pixel[0], pixel[1], 1], dtype=np.float64).reshape(3, 1)
        leftSideMat = np.linalg.inv(self.rotationMatrix) @ np.linalg.inv(self.cam_mtx) @ uvPoint
        rightSideMat = np.linalg.inv(self.rotationMatrix) @ self.tvec

        s = (self.Z_world + rightSideMat[2][0]) / leftSideMat[2][0]
        P = np.linalg.inv(self.rotationMatrix) @ (s * np.linalg.inv(self.cam_mtx) @ uvPoint - self.tvec)

        return P


global model, names, piece_points, depth_frame, color_frame, clicked_points
# model = YOLO('last.pt')
# names = model.names
piece_points = []

def capture_and_detect():
    global model, names, piece_points
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break
        camera = camera_XYZ()
        P = camera.find_board_coord(frame)

        # results = model(frame)

        # for r in results:
        #     for box_info in range(len(r.boxes)):
        #         a = r.boxes[box_info].xyxy[0].tolist()
        #         print("Top-left corner:", round(int(a[0]),2), round(int(a[1]),2))
        #         print("Bottom-right corner:", round(int(a[2]),2), round(int(a[3]),2))


        #         top_left = round(int(a[0]),2), round(int(a[1]),2)
        #         bottom_right = round(int(a[2]),2), round(int(a[3]),2)
        #         color = (255,255,255) 
        #         thickness = 1
        #         cv2.rectangle(frame, top_left, bottom_right, color, thickness)

        #         # bounding boxun merkezi
        #         center_x = (top_left[0] + bottom_right[0]) // 2
        #         center_y = (top_left[1] + bottom_right[1]) // 2
        #         center_pixel = np.array([center_x, center_y])

        #         piece_names = names[int(r.boxes.cls[box_info])]

        #         piece_points.append((piece_names, center_pixel, depth_frame[center_y, center_x]))

        #         cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

        #         text = f"x1, y1: ({round(int(a[0]),2)}, {round(int(a[1]),2)})"
        #         cv2.putText(frame, text, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color , thickness, cv2.LINE_AA)

        #         text = f"x2, y2 ({round(int(a[2]),2)}, {round(int(a[3]),2)})"
        #         cv2.putText(frame, text, (bottom_right[0], bottom_right[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color , thickness, cv2.LINE_AA)

        #     cv2.imshow("YOLOv8 Inference", frame)
        #     cv2.waitKey(0)
        break

    cv2.destroyAllWindows()

    # return detected piece points
    # return piece_points
    return P



if __name__ == '__main__':
    # piece_points = capture_and_detect()
    P = capture_and_detect()
    print(P)
    # piece_dict = {}

    # for i in len(piece_points):
    #     piece_dict[piece_points[i][0]] = camera.calculate_xyz(piece_points[i][1])

    # print(piece_dict)
