import cv2
import os

def capture_images_on_spacebar(output_folder):
    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Initialize the video capture
    cap = cv2.VideoCapture(6)

    image_count = 0

    while True:
        # Capture a frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Display the frame
        cv2.imshow("Camera", frame)

        # Check if the spacebar is pressed
        key = cv2.waitKey(1) & 0xFF
        if key == 32:  # ASCII code for spacebar
            # Save the frame to the output folder with a filename
            image_filename = os.path.join(output_folder, f"image_{image_count}.jpg")
            cv2.imwrite(image_filename, frame)
            print(f"Image saved as {image_filename}")
            image_count += 1

        # Press 'q' to exit the loop and stop capturing
        if key == ord("q"):
            break

    # Release the video capture and close the OpenCV window
    cap.release()
    cv2.destroyAllWindows()

# Set the output folder to save the captured images
output_folder = "/home/ekin/catkin_ws/src/panda_chess/camera_calib/calibration_images/"

# Call the function to capture images when the spacebar is pressed
capture_images_on_spacebar(output_folder)
