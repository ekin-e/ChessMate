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

    return img
img = cv2.imread("chess.jpeg")
img_orig = img.copy()  
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 9, 3)

contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = find_max_contour_area(contours)

img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
c = contours[0]
peri = cv2.arcLength(c, True)
approx = cv2.approxPolyDP(c, 0.02 * peri, True)
pts = find_outer_corners(img, approx)
      
print(pts)
print(c)
img_orig = do_perspective_transform(img_orig, pts)
"""for point in pts:
    cv2.circle(img_original, (int(point[0]),int(point[1])), 10, (0,255,255), 2)
"""
"""cv2.drawContours(img_orig, [c], 0, (0,255,255), 3)"""
cv2.imshow("corners", img_orig)
cv2.waitKey(0)
cv2.destroyAllWindows
