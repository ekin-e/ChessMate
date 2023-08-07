"""
@file hough_lines.py
@brief This program demonstrates line and Square finding with the Hough transform
"""
import sys
import math
import cv2 as cv
import numpy as np
from operator import itemgetter
import random

np.set_printoptions(threshold=sys.maxsize)

def sonuc(nerden,img):
   if nerden==0:
      cv.imshow("kesisim ve kare yok", img)
      cv.imwrite("karecikler9.jpg",img)
      cv.waitKey()
   elif nerden==1:
      cv.imshow("kesisim var ama kare yok", img)
      cv.imwrite("karecikler9.jpg",img)
      cv.waitKey()
   else:
      cv.imshow("karecikler", img)
      cv.imwrite("karecikler9.jpg",img)
      cv.waitKey()




# listenin içinde aynı tuple varsa False döner
def is_in(list,eleman):
   is_in=True
   for i in range(0,len(list)):
      if eleman[0]==list[i][0] and eleman[1]==list[i][1]:
         is_in=False
   return is_in

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

#karedeki açı kontrolu için
def aci_bulma(P1,P2,P3):
   result = math.atan2(P3[1] - P1[1], P3[0] - P1[0]) - math.atan2(P2[1] - P1[1], P2[0] - P1[0])
   return abs(result)

def medyan_bulma(lst):
   lst.sort()
   length = len(lst)  # Get the length of the list

   if length % 2 != 0:  # Check if the length is odd
      middle_index = length // 2
      return lst[middle_index]

   # If the length is even
   first_middle_index = length // 2 - 1
   second_middle_index = length // 2
   return (lst[first_middle_index]+ lst[second_middle_index])/2



def main():

   print("Merhaba! Chessboarddaki en kücük karelerin ortalama kenar uzunluğunu bulma koduna hoş geldin.")
   filename = 'shapes.jpg'
   #filename = argv[0] if len(argv) > 0 else default_file
   # Loads an image
   src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_GRAYSCALE)

   # Check if image is loaded fine
   if src is None:
      print("Error opening image!")
      print ('Usage: hough_lines.py [image_name -- default ' + filename + '] \n')

   #görüntelenen fotoğrafın ekrana sığması için bu kodu açmalısın ama bunu açtığında fotoğraf kalitesi düşer
   # ve hata oranı artar.
      
   '''scale_percent = 60 # percent of original size
   width = int(src.shape[1] * scale_percent / 100)
   height = int(src.shape[0] * scale_percent / 100)
   dim = (width, height)
   resized = cv.resize(src, dim, interpolation = cv.INTER_AREA)'''


   
   blur=cv.GaussianBlur(src,(5,5),0.)
   dst = cv.Canny(blur, 50, 200, None, 3)
 

   # Copy edges to the images that will display the results in BGR
   cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
   #cdstP = np.copy(cdst)
   img1=cdst

   lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

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


      #kesişim noktalarını yeşil ile işaretler
      for i in range(0,len(intersections)):
         for j in range(0,len(intersections[i])):
            #cv.putText(cdst, "square", (int(intersections[i][0]),int(intersections[i][1])), cv.FONT_HERSHEY_COMPLEX, 0.5, (0, 250, 0))
            cdst = cv.circle(cdst, (int(intersections[i][j][0]),int(intersections[i][j][1])), radius=5, color=(0, 255, 0), thickness=-1)


      karecikler= []


      #for i in range(0,90):
      z=0
      while len(karecikler)<7 and z< 10000:    

         
         # print("tur",i)
         z=z+1
         #Dikey = linelar arasında random seçim
         # Yatay = line üstünde random seçim  
         Dikey = random.randint(3,len(intersections)-4)
         yatay = random.randint(3,len(intersections[Dikey])-4)

         #karelerin kenarlarının uygunluğunu test eder ve kareleri çizer
         if len(intersections[Dikey])-1 >yatay and len(intersections[Dikey+1])-1 >yatay \
            and abs(mesafe_bul(intersections[Dikey][yatay],intersections[Dikey+1][yatay])-mesafe_bul(intersections[Dikey][yatay],intersections[Dikey][yatay+1]))<10 \
            and abs(mesafe_bul(intersections[Dikey][yatay],intersections[Dikey+1][yatay])-mesafe_bul(intersections[Dikey+1][yatay],intersections[Dikey+1][yatay+1]))<10 \
            and is_in(karecikler,(Dikey,yatay)) and mesafe_bul(intersections[Dikey][yatay],intersections[Dikey+1][yatay]) > 40 \
            and abs(mesafe_bul(intersections[Dikey][yatay],intersections[Dikey+1][yatay])-mesafe_bul(intersections[Dikey][yatay+1],intersections[Dikey+1][yatay+1]))<10\
            and aci_bulma(intersections[Dikey][yatay],intersections[Dikey+1][yatay],intersections[Dikey][yatay+1]) < 1.76 \
            and aci_bulma(intersections[Dikey][yatay],intersections[Dikey+1][yatay],intersections[Dikey][yatay+1]) > 1.41 : 

            karecikler.append((Dikey,yatay))


            #print(Dikey,yatay)
            img1 =cv.line(cdst, intersections[Dikey][yatay], intersections[Dikey+1][yatay], (0,255,0), thickness=3) 
            img1 =cv.line(cdst, intersections[Dikey][yatay], intersections[Dikey][yatay+1], (0,255,0), thickness=3) 
            img1=cv.line(cdst, intersections[Dikey][yatay+1], intersections[Dikey+1][yatay+1], (0,255,0), thickness=3) 
      
            img1 =cv.line(cdst, intersections[Dikey+1][yatay], intersections[Dikey+1][yatay+1], (0,255,0), thickness=3)
            




      #tespit edilen karelerin üst kenar uzunluğunu listeye ekler.
      kenarlar=[]
      if len(karecikler)==0:
         print("Kesişim buldum ama hiç kare bulamadim! Lütfen aciyi degistir.")
         sonuc(1,cdst)
      else:   
         for i in range(0,len(karecikler)):
            kenarlar.append(mesafe_bul(intersections[karecikler[i][0]][karecikler[i][1]],intersections[karecikler[i][0]+1][karecikler[i][1]]))
         print("Bulunan kare sayisi:", len(karecikler))
         print("bulunan kenar uzunluklar:\n", kenarlar)
         ortalama_kenar = medyan_bulma(kenarlar)
         sonuc(2,img1)
         return ortalama_kenar   
      

     


   else:
      print("Hic kesisim noktasi bulunamadi!")
      print("Buldugum linelari gösteriyorum:")
      sonuc(0,cdst)
      print("Lütfen aciyi degistir!")
      return
         
   #print("kenar: " ,kenarlar)


   #probabilistic hough line transform kodu      
   '''linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

   if linesP is not None:
      for i in range(0, len(linesP)):
         l = linesP[i][0]
         cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)'''


if __name__ == "__main__":
 bulunan_kenar = main()
 if bulunan_kenar != None:
    print("TEBRİKLER!! \nSonucun: " , bulunan_kenar)
