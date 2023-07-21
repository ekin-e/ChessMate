import cv2
import numpy as np
from ultralytics import YOLO

class camera_realtimeXYZ:
    #camera variables
    cam_mtx=None
    dist=None
    newcam_mtx=None
    roi=None
    rvec1=None
    tvec1=None
    R_mtx=None
    Rt=None
    P_mtx=None
    board_center=None
    #images
    img=None

    def __init__(self):
        savedir="home/ekin/catkin_ws/src/panda_chess/camera_calib/camera_data/"

        self.cam_mtx=np.load(savedir+'cam_mtx.npy')
        self.dist=np.load(savedir+'dist.npy')
        self.newcam_mtx=np.load(savedir+'newcam_mtx.npy')
        self.roi=np.load(savedir+'roi.npy')
        self.rvec1=np.load(savedir+'rvec1.npy')
        self.tvec1=np.load(savedir+'tvec1.npy')
        self.R_mtx=np.load(savedir+'R_mtx.npy')
        self.Rt=np.load(savedir+'Rt.npy')
        self.P_mtx=np.load(savedir+'P_mtx.npy')
        self.board_center = None

        s_arr=np.load(savedir+'s_arr.npy')
        self.scalingfactor=s_arr[0]

        self.inverse_newcam_mtx = np.linalg.inv(self.newcam_mtx)
        self.inverse_R_mtx = np.linalg.inv(self.R_mtx)

    def calculate_XYZ(self,u,v):                          
        # Solve: From Image Pixels, find World Points
        uv_1=np.array([[u,v,1]], dtype=np.float32)
        uv_1=uv_1.T
        suv_1=self.scalingfactor*uv_1
        xyz_c=self.inverse_newcam_mtx.dot(suv_1)
        xyz_c=xyz_c-self.tvec1
        XYZ=self.inverse_R_mtx.dot(xyz_c)

        return XYZ


global model, names, piece_points
model = YOLO('last.pt')
names = model.names
piece_points = []


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
    for i in len(piece_points):
        piece_dict[piece_points[i][0]] = camera.calculate_XYZ(piece_points[i][0], piece_points[i][0])

    print(piece_dict)

