import cv2
from ultralytics import YOLO


def main():

    detectedList = []
    detectedList.clear()

    model = YOLO('temmuz27.pt')
    names = model.names
    
    

    cap = cv2.VideoCapture(6)   # 0 yerine istediğin kaynağı yaz


    while cap.isOpened():
        success, frame = cap.read()

        if success:
            results = model(frame, conf=0.7)  
            annotated_frame = results[0].plot()

            for r in results:
                
            
                for box_info in range(len(r.boxes)):
                    a = r.boxes[box_info].xyxy[0].tolist()

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

                    #print(heightOfPiece)

                   #### NOTTTTT
                   # Taşların yerinin değiştiğini algılaman lazım  (son konum - ilk konum)

                    try:
                        class_name = names[int(r.boxes.cls[box_info])]
                        print(class_name, " height: ", heightOfPiece, "center: ", center_x, ",", center_y)
                    except:
                        print("bekleniyor")

                    if class_name not in detectedList:
                        detectedList.append(class_name)

                    #print(detectedList)
                    

            cv2.namedWindow("Center Points",cv2.WINDOW_NORMAL)
            cv2.namedWindow("YOLOv8 Inference",cv2.WINDOW_NORMAL)
            cv2.imshow("Center Points", frame)
            cv2.imshow("YOLOv8 Inference", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    
