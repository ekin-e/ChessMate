import cv2
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

frame = None


def is_point_inside_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def imgCallback(msg):
    global frame
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")


def main():
    rospy.init_node("yoloNode")
    rospy.Subscriber("/camera/color/image_raw", Image, imgCallback)

    detectedList = []
    detectedList.clear()
    model = YOLO('temmuz28.pt')
    names = model.names


    x0 = 160
    y0 = 433
    x1 = 205
    y1 = 138
    x2 = 474
    y2 = 131
    x3 = 540
    y3 = 428

    polygon_coords_input = [x0,y0,x1,y1,x2,y2,x3,y3]
    points = np.array(polygon_coords_input, dtype=np.int32).reshape((-1, 2))
    


    while not rospy.is_shutdown():
        if frame is not None:

            results = model(frame, conf=0.7)  
            annotated_frame = results[0].plot()

            cv2.polylines(frame, [points], isClosed=True, color=(150, 220, 100), thickness=2)
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, [points], (255, 255, 255))
            roi = cv2.bitwise_and(frame, frame, mask=mask)

            for r in results:
                for box_info in range(len(r.boxes)):
                    a = r.boxes[box_info].xyxy[0].tolist()
                    class_name = names[int(r.boxes.cls[box_info])]
                    top_left = round(int(a[0]),2), round(int(a[1]),2)
                    bottom_right = round(int(a[2]),2), round(int(a[3]),2)
                    color = (0, 255, 0) 
                    thickness = 2
                    cv2.rectangle(frame, top_left, bottom_right, color, thickness)

                    # bounding boxun merkezi
                    center_x = (top_left[0] + bottom_right[0]) // 2
                    center_y = (top_left[1] + bottom_right[1]) // 2

                    heightOfPiece = bottom_right[1] - top_left[1]


                    # Kameranın konumuna göre 80 sayısı değşecek
                    if heightOfPiece < 37:
                        cv2.circle(frame, (center_x, center_y+5), 5, (255, 0, 0), -1)
                    elif heightOfPiece > 37:
                        cv2.circle(frame, (center_x, center_y+12), 5, (255, 0, 0), -1)


                    if class_name not in detectedList:
                        detectedList.append(class_name)

                    
                    center_point = (center_x, center_y)
                    if is_point_inside_polygon(center_point, points):
                        print("Class:", class_name, " is inside the polygon.")
                    else:
                        print("Class:", class_name, " is OUTSIDE the polygon.")
                    
            cv2.imshow("Selected ROI", roi)
            cv2.namedWindow("Center Points",cv2.WINDOW_NORMAL)
            cv2.namedWindow("YOLOv8 Inference",cv2.WINDOW_NORMAL)
            cv2.imshow("Center Points", frame)
            cv2.imshow("YOLOv8 Inference", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        rospy.sleep(0.1)

    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    
    main()
    
    
