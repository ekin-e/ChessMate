# chessboard middle point finder finder    
##############################################################################################################
import numpy as np
import cv2 
import sys


np.set_printoptions(threshold=sys.maxsize)

def main(argv):

	default_file = 'board.jpg'
	filename = argv[0] if len(argv) > 0 else default_file

	src = cv2.imread(cv2.samples.findFile(filename))



	if src is None:
		print("image not found!")
		return -1
	
	scale_percent = 60 # percent of original size
	width = int(src.shape[1] * scale_percent / 100)
	height = int(src.shape[0] * scale_percent / 100)
	dim = (width, height)
	resized = cv2.resize(src, dim, interpolation = cv2.INTER_AREA)


	# computations
	print(len(resized[0]),len(resized))  #1200*1200 

	katsayi= len(resized) /16

	point_array=[]

	for i in range(1,16,2):
		for j in range(1,16,2):
		# default resim üzerine yazma kodu: image = cv.circle(image, centerOfCircle, radius, color, thickness)	
			x= int(i*katsayi)
			y = int(j* katsayi)
			image = cv2.circle(resized, (x,y), radius=3, color=(255, 0, 0), thickness=-1)
			#print("i:", i, " j:",j,"\n" )
			point_array.append((x,y))

	print(point_array)
	#print(len(point_array))
	cv2.imshow("Shapes", image)
	cv2.imwrite("middle_point.jpg",image)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	return point_array



if __name__ == "__main__":
 main(sys.argv[1:])











