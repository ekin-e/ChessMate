import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2
from realsense_depth import *

class camera_realtimeXYZ:
    #camera variables
    cam_mtx=None
    dist=None
    newcam_mtx=None
    roi=None
    #images
    img=None

    def __init__(self):
        savedir="home/ekin/catkin_ws/src/panda_chess/camera_calib/camera_data/"

        self.cam_mtx=np.load(savedir+'cam_mtx.npy')
        self.dist=np.load(savedir+'dist.npy')
        self.newcam_mtx=np.load(savedir+'newcam_mtx.npy')


    def calculate_xyz(u, v, Z, fx, fy, cx, cy):
        # Calculate the X and Y coordinates using the pinhole camera model
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        return X, Y, Z


global model, names, piece_points, depth_frame, color_frame, clicked_points
model = YOLO('last.pt')
names = model.names
piece_points = []


##### SECTION FOR FINDING THE CENTER PIXELS FOR THE MANUALLY ENTERED INPUTS #####
def show_distance(event, x, y, flags, param):
    global color_frame, clicked_points, depth_frame
    if event == cv2.EVENT_LBUTTONDOWN:  # Check for left mouse button click
        # Retrieve the depth value at the clicked position in the depth frame
        distance = depth_frame[y, x]

        # Display the distance on the color frame
        cv2.putText(color_frame, "{} mm".format(distance), (x, y - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

        # Draw an X marker at the clicked position
        marker_size = 10
        marker_color = (0, 0, 255)  # Red color (BGR format)
        thickness = 2

        cv2.line(color_frame, (x - marker_size, y - marker_size), (x + marker_size, y + marker_size), marker_color, thickness)
        cv2.line(color_frame, (x - marker_size, y + marker_size), (x + marker_size, y - marker_size), marker_color, thickness)

        # Print the coordinates and distance
        print("Clicked at (x={}, y={}), distance: {} mm".format(x, y, distance))
        clicked_points.append(distance/10.0)


def measure_ref_points():
    global clicked_points
    # Initialize Camera Intel Realsense
    dc = DepthCamera()
    # Create mouse event
    cv2.namedWindow("Color frame")
    cv2.setMouseCallback("Color frame", show_distance)
    clicked_points = []

    while True:
        ret, depth_frame, color_frame = dc.get_frame()
        cv2.imshow("depth frame", depth_frame)
        cv2.imshow("Color frame", color_frame)
        
        key = cv2.waitKey(1)
        if key == 27:
            break
    
        return clicked_points

def capture_and_detect():
    global model, names, piece_points
    
    cap = cv2.VideoCapture(6)       # 0  = bilgisayarın webcami
                                    # 6 = realsense'in kamerası (kamerayı takmayı unutma :D)
    while cap.isOpened():
        success, frame = cap.read()

        if success:
            results = model(frame)

            for r in results:
                for c in r.boxes.cls:
                    print(names[int(c)])
                    piece_names = names[int(c)]
                
                count = 0
                for box_info in r.boxes:
                    a = box_info.xyxy[0].tolist()
                    print("Top-left corner:", round(int(a[0]),2), round(int(a[1]),2))
                    print("Bottom-right corner:", round(int(a[2]),2), round(int(a[3]),2))


                    top_left = round(int(a[0]),2), round(int(a[1]),2)
                    bottom_right = round(int(a[2]),2), round(int(a[3]),2)
                    color = (0, 255, 0) 
                    thickness = 2
                    cv2.rectangle(frame, top_left, bottom_right, color, thickness)

                    # bounding boxun merkezi
                    center_x = (top_left[0] + bottom_right[0]) // 2
                    center_y = (top_left[1] + bottom_right[1]) // 2

                    piece_points.append((piece_names[count], (center_x, center_y)))
            
                    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

                    text = f"x1, y1: ({round(int(a[0]),2)}, {round(int(a[1]),2)})"
                    cv2.putText(frame, text, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0) , thickness, cv2.LINE_AA)

                    text = f"x2, y2 ({round(int(a[2]),2)}, {round(int(a[3]),2)})"
                    cv2.putText(frame, text, (bottom_right[0], bottom_right[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0) , thickness, cv2.LINE_AA)
                    count+=1

            cv2.imshow("YOLOv8 Inference", frame)
            cv2.waitKey(0)
        break

    cap.release()
    cv2.destroyAllWindows()

    # return detected piece points
    return piece_points



if __name__ == '__main__':
    piece_points = capture_and_detect()
    camera = camera_realtimeXYZ()
    piece_dict = {}
    
    fx = camera.cam_mtx[0, 0]
    fy = camera.cam_mtx[1, 1]
    cx = camera.cam_mtx[0, 2]
    cy = camera.cam_mtx[1, 2]
    Z = clicked_points[0]

    for i in len(piece_points):
        piece_dict[piece_points[i][0]] = camera.calculate_xyz(piece_points[i][0], piece_points[i][0], Z, fx, fy, cx, cy)

    print(piece_dict)
