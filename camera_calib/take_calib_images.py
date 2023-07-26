import cv2
import os
import numpy as np
import glob

global SQUARE_SIZE, BOARD_SIZE, top_left_corner, workingdir, savedir

SQUARE_SIZE = 0.024
BOARD_SIZE = (7,7)
top_left_corner = (0.625, 0.075, 0.0)
workingdir="C:/Users/ekine/OneDrive/Desktop/ChessMate/ChessMate-ekin/camera_calib"
savedir="C:/Users/ekine/OneDrive/Desktop/ChessMate/ChessMate-ekin/camera_calib/camera_data/"


def calculate_chessboard_coordinates():
    """
    Calculate real-world coordinates of all the squares on a chessboard.

    Parameters:
        top_left_corner (tuple): (X, Y, Z) coordinates of the top-left corner of the chessboard.
        board_size (tuple): Size of the chessboard (rows, columns).
        square_size (float): Size of each square in the chessboard (e.g., in millimeters).

    Returns:
        list: List of real-world coordinates (X, Y, Z) for all squares on the chessboard.
    """
    global SQUARE_SIZE, BOARD_SIZE, top_left_corner

    rows, cols = BOARD_SIZE
    X_top_left, Y_top_left, Z_top_left = top_left_corner

    # Initialize the list to store the real-world coordinates
    objp = []

    # Loop through each square in the chessboard
    for x in range(rows):
        for y in range(cols):
            # Calculate the real-world coordinates of the current square
            X = X_top_left - x * SQUARE_SIZE
            Y = Y_top_left - y * SQUARE_SIZE  # Assuming Y decreases downward
            Z = Z_top_left  # Assuming the chessboard lies flat on the XY-plane

            # Append the coordinates to the list
            objp.append([X, Y, Z])

    return np.array(objp, dtype=np.float32)


def capture_images_on_spacebar(objp):
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    # Initialize the video capture
    cap = cv2.VideoCapture(0)

    while True:
        # Capture a frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            return -1

        # Display the frame
        cv2.imshow("Camera", frame)

        # Check if the spacebar is pressed
        key = cv2.waitKey(1) & 0xFF
        if key == 32:  # ASCII code for spacebar
            print("do smth here")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

        # Press 'q' to exit the loop and stop capturing
        if key == ord("q"):
            break

    # Release the video capture and close the OpenCV window
    cap.release()
    cv2.destroyAllWindows()
    print(len(objpoints))
    print(len(imgpoints))

    print(">==> Starting calibration")
    ret, cam_mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    np.save(savedir+'cam_mtx.npy', cam_mtx)
    np.save(savedir+'dist.npy', dist)
    np.save(savedir+'objp.npy', objp)

if __name__ == '__main__':
    objp = calculate_chessboard_coordinates()
    # Call the function to capture images when the spacebar is pressed and process them before calibration
    capture_images_on_spacebar(objp)
