import cv2
import imutils

'''cap = cv2.VideoCapture(0)
ret, current_frame = cap.read()
previous_frame = current_frame'''

previous_frame = cv2.imread(cv2.samples.findFile("foto_05.png"))
current_frame = cv2.imread(cv2.samples.findFile("foto_06.png"))


current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
previous_frame_gray = cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY)    

frame_diff = cv2.absdiff(current_frame_gray,previous_frame_gray)

thresh_1 = cv2.threshold(frame_diff, 20, 255, cv2.THRESH_BINARY)[1]
thresh_2 = cv2.dilate(thresh_1, None, iterations=2)

cnts = cv2.findContours(thresh_2.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

square = []

cnts = imutils.grab_contours(cnts)
for c in cnts:
	# compute the bounding box of the contour and then draw the
	# bounding box on both input images to represent where the two
	# images differ

	(x, y, w, h) = cv2.boundingRect(c)
	if w>15 and h>15:
		square.append((x, y, w, h))
		cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
		cv2.rectangle(previous_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

'''cv2.imshow('tresh',thresh_2)   
cv2.waitKey()'''
cv2.imshow('current',current_frame)   
cv2.waitKey() 
cv2.imshow('prev',previous_frame)   
cv2.waitKey()

#cv2.imshow('cnts',cnts) 
#cv2.waitKey()   
cv2.destroyAllWindows()

print(square)