import cv2
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
names = model.names

video_path = "path/to/your/video/file.mp4"
 
cap = cv2.VideoCapture(0)     # 0 yerine istediğin kaynağı yaz

while cap.isOpened():
    success, frame = cap.read()

    if success:
        results = model(frame)

        for r in results:
            for c in r.boxes.cls:
                print(names[int(c)])

            for box_info in r.boxes:
                a = box_info.xyxy[0].tolist()
                print("Top-left corner:", round(int(a[0]),2), round(int(a[1]),2))
                print("Bottom-right corner:", round(int(a[2]),2), round(int(a[3]),2))


                top_left = round(int(a[0]),2), round(int(a[1]),2)
                bottom_right = round(int(a[2]),2), round(int(a[3]),2)
                color = (0, 255, 0) 
                thickness = 2
                cv2.rectangle(frame, top_left, bottom_right, color, thickness)
                
                print("width: ", round(int(a[3]),2)-round(int(a[1]),2))

            
                text = f"x1, y1: ({round(int(a[0]),2)}, {round(int(a[1]),2)})"
                cv2.putText(frame, text, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0) , thickness, cv2.LINE_AA)

                text = f"x2, y2 ({round(int(a[2]),2)}, {round(int(a[3]),2)})"
                cv2.putText(frame, text, (bottom_right[0], bottom_right[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0) , thickness, cv2.LINE_AA)

        cv2.imshow("YOLOv8 Inference", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
