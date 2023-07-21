#https://docs.opencv.org/3.3.0/dc/dbb/tutorial_py_calibration.html

import numpy as np
import cv2
import glob
import pyrealsense2
from realsense_depth import *

##### SOME GLOBAL VARIABLES #####
global savedir, cam_mtx, dist, newcam_mtx, roi, cx, cy, fx, color_frame, clicked_points, depth_frame, total_points_used, X_center, Y_center, Z_center

#load camera calibration
savedir="home/ekin/catkin_ws/src/panda_chess/camera_calib/camera_data/"
cam_mtx=np.load(savedir+'cam_mtx.npy')
dist=np.load(savedir+'dist.npy')
newcam_mtx=np.load(savedir+'newcam_mtx.npy')
roi=np.load(savedir+'roi.npy')

#load center points from New Camera matrix
cx=newcam_mtx[0,2]
cy=newcam_mtx[1,2]
fx=newcam_mtx[0,0]

total_points_used=10
X_center=10.9
Y_center=10.7
Z_center=43.4


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
        clicked_points.append(((x,y), distance/10.0))


def measure_ref_points():
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

        if len(clicked_points) is not 9:
            print("Selected points exceeded 9, start selecting again from the start")
            clicked_points = []
        
        key = cv2.waitKey(1)
        if key == 27:
            break
    
    return clicked_points


#MANUALLY INPUT YOUR MEASURED POINTS HERE
#ENTER (X,Y,d*)
#d* is the distance from your point to the camera lens. (d* = Z for the camera center)
#we will calculate Z in the next steps after extracting the new_cam matrix
#world center + 9 world points

def enter_measured_points(clicked_points):
    global X_center, Y_center, Z_center, total_points_used
    # 4,7  10,7  16,7  4,17  10,17  16,17  4,27  10,27  16,27
    worldPoints=np.array([[X_center,Y_center,Z_center],
                        [4.0, 7.0, clicked_points[0][1]],
                        [10.0, 7.0, clicked_points[1][1]],
                        [16.0, 7.0, clicked_points[2][1]],
                        [4.0, 17.0, clicked_points[3][1]],
                        [10.0, 17.0, clicked_points[4][1]],
                        [16.0, 17.0, clicked_points[5][1]],
                        [4.0, 27.0, clicked_points[6][1]],
                        [10.0, 27.0, clicked_points[7][1]],
                        [16.0, 27.0, clicked_points[8][1]]], dtype=np.float32)

    #[u,v] center + 9 Image points
    imagePoints=np.array([[cx,cy],
                        [clicked_points[0][0][0], clicked_points[0][0][1]],
                        [clicked_points[1][0][0], clicked_points[1][0][1]],
                        [clicked_points[2][0][0], clicked_points[2][0][1]],
                        [clicked_points[3][0][0], clicked_points[3][0][1]],
                        [clicked_points[4][0][0], clicked_points[4][0][1]],
                        [clicked_points[5][0][0], clicked_points[5][0][1]],
                        [clicked_points[6][0][0], clicked_points[6][0][1]],
                        [clicked_points[7][0][0], clicked_points[7][0][1]],
                        [clicked_points[8][0][0], clicked_points[8][0][1]]], dtype=np.float32)

    for i in range(1,total_points_used):
        #start from 1, given for center Z=d*
        #to center of camera
        wX=worldPoints[i,0]- X_center
        wY=worldPoints[i,1]- Y_center
        wd=worldPoints[i,2]

        d1=np.sqrt(np.square(wX)+np.square(wY))
        wZ=np.sqrt(np.square(wd)-np.square(d1))
        worldPoints[i,2]=wZ

    inverse_newcam_mtx = np.linalg.inv(newcam_mtx)
    np.save(savedir+'inverse_newcam_mtx.npy', inverse_newcam_mtx)
    
    print("solvePNP")
    ret, rvec1, tvec1=cv2.solvePnP(worldPoints,imagePoints,newcam_mtx,dist)
    np.save(savedir+'rvec1.npy', rvec1)
    np.save(savedir+'tvec1.npy', tvec1)

    print("R - rodrigues vecs")
    R_mtx, jac=cv2.Rodrigues(rvec1)
    np.save(savedir+'R_mtx.npy', R_mtx)

    print("R|t - Extrinsic Matrix")
    Rt=np.column_stack((R_mtx,tvec1))
    np.save(savedir+'Rt.npy', Rt)

    print("newCamMtx*R|t - Projection Matrix")
    P_mtx=newcam_mtx.dot(Rt)
    np.save(savedir+'P_mtx.npy', P_mtx)

    s_arr=np.array([0], dtype=np.float32)
    s_describe=np.array([0,0,0,0,0,0,0,0,0,0],dtype=np.float32)

    for i in range(0, total_points_used):
        print("=======POINT # " + str(i) +" =========================")
        print("Forward: From World Points, Find Image Pixel")

        XYZ1=np.array([[worldPoints[i,0],worldPoints[i,1],worldPoints[i,2],1]], dtype=np.float32)
        XYZ1=XYZ1.T
        suv1=P_mtx.dot(XYZ1)
        s=suv1[2,0]
        uv1=suv1/s
        s_arr=np.array([s/total_points_used+s_arr[0]], dtype=np.float32)
        s_describe[i]=s
        uv_1=np.array([[imagePoints[i,0],imagePoints[i,1],1]], dtype=np.float32)
        uv_1=uv_1.T
        suv_1=s*uv_1
        xyz_c=inverse_newcam_mtx.dot(suv_1)
        xyz_c=xyz_c-tvec1
        inverse_R_mtx = np.linalg.inv(R_mtx)
        # XYZ=inverse_R_mtx.dot(xyz_c)

    np.save(savedir+'inverse_R_mtx.npy', inverse_R_mtx)
    np.save(savedir+'s_arr.npy', s_arr)


##### EVAL FUNCTION THAT I PROBABLY MIGHT NOT USE #####
def eval(s_describe):
    s_mean, s_std = np.mean(s_describe), np.std(s_describe)
    print(">>>>>>>>>>>>>>>>>>>>> S RESULTS")
    print("Mean: "+ str(s_mean))
    print("Std: " + str(s_std))
    print(">>>>>> S Error by Point")
    for i in range(0,total_points_used):
        print("Point "+str(i))
        print("S: " +str(s_describe[i])+" Mean: " +str(s_mean) + " Error: " + str(s_describe[i]-s_mean))



########## MAIN ##########
if __name__ == '__main__':
    print("main")
    clicked_points = measure_ref_points()
    enter_measured_points(clicked_points)
    print("Done with initial perspective calibration")