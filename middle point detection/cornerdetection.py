#with the help of https://github.com/dimasikson/chess_computer_vision_project

import cv2
import os

import pandas as pd
import numpy as np
from scipy import ndimage
import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import chess
import chess.svg

from svglib.svglib import svg2rlg
from reportlab.graphics import renderPM

from ultralytics import YOLO

def find_max_contour_area(contours):
    max_area = 0 - float('inf')
    max_c = None
    for c in contours:
        area = cv2.contourArea(c)

        if area > max_area:
            max_area = area
            max_c = c

    return [max_c]

def distance(p1, p2):
    return math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )

def find_outer_corners(img, pts):

    rows, cols, _ = img.shape

    bl_dst = br_dst = tl_dst = tr_dst = float('inf')

    for p in pts:

        p = p[0]

        if distance(p, (cols*0, rows*1)) < bl_dst:
            bl_dst = distance(p, (cols*0, rows*1))
            bl = p

        if distance(p, (cols*1, rows*1)) < br_dst:
            br_dst = distance(p, (cols*1, rows*1))
            br = p

        if distance(p, (cols*0, rows*0)) < tl_dst:
            tl_dst = distance(p, (cols*0, rows*0))
            tl = p

        if distance(p, (cols*1, rows*0)) < tr_dst:
            tr_dst = distance(p, (cols*1, rows*0))
            tr = p

    pts1 = np.float32(
        [bl,  # btm left
        br,  # btm right
        tl,  # top left
        tr]  # top right
    )

    return pts1

def do_perspective_transform(img, pts, pts_type=1):

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
model = YOLO("best(6).pt")
image = cv2.imread("image_25.jpeg")
image_copy = image.copy()       #ORIGINAL SIZE ORIGINAL IMAGE

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
copy = img.copy()           #CUT IMAGE BY DETECTION! this image has been transformed into bird_view perpective
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 9, 3)

contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = find_max_contour_area(contours)

img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)         #CUT AND FILTERED IMAGE
c = contours[0]
peri = cv2.arcLength(c, True)
approx = cv2.approxPolyDP(c, 0.02 * peri, True)
pts = find_outer_corners(img, approx)
      
print(pts)
print(c)
img_orig, M = do_perspective_transform(img_orig, pts)      #BIRD_VIEW IMAGE

def create_four_points_list(img_orig):
    #finding all corner points and listing them by 4 points for each square
    height, width, shape = img_orig.shape
    w = int(width/8)
    h = int(height/8)
    four_points_list = []
    for multiplier in range(8):
        for multiplier2 in range(8):
            four_point = [(w*multiplier2,h*(multiplier+1)),(w*(multiplier2+1),h*(multiplier+1)),(w*multiplier2,h*multiplier),(w*(multiplier2+1),h*multiplier)]
            four_points_list.append(four_point)
    return four_points_list

four_points_list = create_four_points_list(img_orig)

#Four_points_list listesindeki pointleri orijinale geri transform et!!!

def transform_points_to_original(pts, M):
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

def inverse_transform(four_points_list, M):
    pts_original = []
    new_M = np.linalg.inv(M)
    for pts in four_points_list:
        pts = np.array(pts, dtype=np.float32)
        pts = pts.reshape(-1, 1, 2)
        pts2 = cv2.perspectiveTransform(pts, new_M).tolist()
        pts_original.append(pts2)
    return pts_original

# Convert the points from bird-view perspective back to original image perspective
pts_original = inverse_transform(four_points_list, M)
print(pts_original)
def find_center_point(vertices):
    sum_x = 0
    sum_y = 0
    for i in vertices:
        sum_x += i[0][0]
        sum_y += i[0][1]
    avg_x = sum_x/len(vertices)
    avg_y = sum_y/len(vertices)
    return avg_x, avg_y
center_list = []
for points in pts_original:
    print(points)
    for pt in points:
        pt = (int(pt[0][0]), int(pt[0][1]))
        cv2.circle(copy, (pt[0], pt[1]), 2, (0,255,255), 2) #to see corner points in the copy image
    center = find_center_point(points)
    print(points, center)
    center_list += [center]     #MIDDLE POINTS CENTER LIST = center_list
    cv2.circle(copy, (int(center[0]), int(center[1])), 2, (255, 0, 0), 2)
print(center_list)
cv2.imshow("middle points detected", copy)

#from cropped to original 640*640
#find the width and height  (x1, y1)
zoomed_points = []
for center in center_list:
    c = list(center)
    print(c)
    c[0] += x1
    c[1] += y1
    zoomed_points += [[c[0], c[1]]]
    cv2.circle(image, (int(c[0]), int(c[1])), 2, (255, 255, 0), 2)

cv2.imshow("640*640 size cropped", image)

def scale_coordinates_640_to_orig(x, y):
    # Scale x coordinate from 640 to 480
    y_480 = int(y * image_copy.shape[0] / 640)
    x_480 = int(x * image_copy.shape[1] / 640)
    return x_480, y_480

row_list = [] #original image original size middle points list
for pt in zoomed_points:
    x_480, y_480 = scale_coordinates_640_to_orig(pt[0], pt[1])
    row_list += [(x_480, y_480)]
    cv2.circle(image_copy, (int(x_480), int(y_480)), 2, (255, 0, 0), 2)

middle_points_list = [row_list[:8],row_list[8:16],row_list[16:24],row_list[24:32],row_list[32:40],row_list[40:48],row_list[48:56],row_list[56:64]]

print(middle_points_list)
cv2.imshow("original size", image_copy)
cv2.waitKey(0)
cv2.destroyAllWindows

robot_color = 1 #for now
keys = []
fen_dict = {}
for row in range(8):
    for col in range(8):
        key = chr(ord('a') + row) + chr(ord('1') + col)
        keys.append(key)
        if robot_color == 0:
            fen_dict[key] = middle_points_list[row][7-col]
        else:
            fen_dict[key] = middle_points_list[7-row][col]
print(fen_dict)