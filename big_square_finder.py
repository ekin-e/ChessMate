import math
import cv2 as cv
import numpy as np
from operator import itemgetter
import random


def find_intersection(line1, line2):
    # extract points
    x1, y1 = line1[0]
    x2, y2 =line1[1]
    x3, y3  = line2[0]
    x4, y4 = line2[1]
    # compute determinant
    Px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4))/  \
        ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
    Py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4))/  \
        ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
    return int(Px), int(Py)

def mesafe_bul(point1,point2):
   x1, y1 = point1
   x2, y2 = point2
   return np.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))


def sortRowWise(m):
     
    # loop for rows of matrix
    for i in range(len(m)):
         
        # loop for column of matrix
        for j in range(len(m[i])):
             
            # loop for comparison and swapping
            for k in range(len(m[i]) - j - 1):
                 
                if (m[i][k][1] > m[i][k + 1][1]):
                     
                    # swapping of elements
                    t = m[i][k]
                    m[i][k] = m[i][k + 1]
                    m[i][k + 1] = t
def main():


    print("Merhaba! Chessboarddaki en buyuk kareyi bulma koduna hoş geldin.")
    filename = 'deneme_1.jpeg'
    src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_GRAYSCALE)

    blur=cv.GaussianBlur(src,(5,5),0.)
    dst = cv.Canny(blur, 50, 200, None, 3)


    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    #cdstP = np.copy(cdst)
    img1=cdst

    lines = cv.HoughLines(dst, 1, np.pi / 180, 100, None, 0, 0)


    #print(lines)
    points=[[],[]]
    intersections = []

    # lineları fotoğrafın üstüne çizen ve gruplandıran kısım
    # points= [[dikey linelara ait ikili tuplelar],[yatay linelara ait ikili tuplelar]]
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            
            if theta > np.pi/4 and theta< 3*np.pi/4:
            #print("red")
                cv.line(cdst, pt1, pt2, (0,0,255), 1, cv.LINE_AA)
                points[1].append([pt1,pt2])
            else:
            #print("blue")
                cv.line(cdst, pt1, pt2, (255,0,0), 1, cv.LINE_AA)
                points[0].append([pt1,pt2])
            
            #cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
            #cv.waitKey()


    #dik ve yatay çizgilerin kesişimini bulan ve gruplayıp intersectionsa yazan kod
    for i in range(0,len(points[0])):
        intersections.append([])
        for j in range(0,len(points[1])):

            point = find_intersection(points[0][i],points[1][j])
            is_near = True


            for k in range(0,len(intersections[i])):
                if intersections[i] and mesafe_bul(point,intersections[i][k])<10:
                    is_near=False
                    break

            if is_near:
                intersections[i].append(point)     

    #interctionın her bir linedaki noktaları yukardan aşağı sıralar
    sortRowWise(intersections)

    #intersectiondaki lineları sıralar soldan sağa
    if len(intersections) > 2:
        intersections= sorted(intersections, key=itemgetter(2))

        #intersections = [[en soldaki lineın yukardan aşağı noktaları ],[],.. ]


    kare_kenarlar =[]

    if intersections[0]:

        kare_kenarlar.append(intersections[0][-1])
        kare_kenarlar.append(intersections[-1][-1])
        kare_kenarlar.append(intersections[-1][0])
        kare_kenarlar.append(intersections[0][0])

        img1 =cv.line(cdst, intersections[-1][0], intersections[-1][-1], (0,255,0), thickness=3) 
        img1 =cv.line(cdst, intersections[-1][0], intersections[0][0], (0,255,0), thickness=3) 
        img1 =cv.line(cdst, intersections[0][-1], intersections[0][0], (0,255,0), thickness=3) 
        img1 =cv.line(cdst, intersections[0][-1], intersections[-1][-1], (0,255,0), thickness=3) 


    cv.imshow("en büyük kare", img1)
    #cv.imwrite("karecikler9.jpg",img)
    cv.waitKey()
   
    return kare_kenarlar
    
    


if __name__ == "__main__":
 bulunan_kare = main()
 if bulunan_kare != None:
    print("TEBRİKLER!! \nSonucun: " , bulunan_kare)        
