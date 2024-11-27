import cv2
import serial
from cvzone.HandTrackingModule import HandDetector
import time

# Set up serial communication (update COM port as per your system)
ser = serial.Serial('COM7', 9600)  # Replace 'COM7' with the port for your ESP8266
detector = HandDetector(detectionCon=0.8, maxHands=1)
video = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = video.read()
        frame = cv2.flip(frame, 1)
        hands, img = detector.findHands(frame)
        
        if hands:
            lmList = hands[0]
            fingerUp = detector.fingersUp(lmList)

            if fingerUp == [0, 0, 0, 0, 0]:
                print("Gesture: All fingers down")
                ser.write(b'0')  # Send '0'
            elif fingerUp == [0, 1, 0, 0, 0]:
                print("Gesture: One finger up")
                ser.write(b'1')  # Send '1'
            elif fingerUp == [0, 1, 1, 0, 0]:
                print("Gesture: Two fingers up")
                ser.write(b'2')  # Send '2'
            elif fingerUp == [0, 1, 1, 1, 0]:
                print("Gesture: Three fingers up")
                ser.write(b'3')  # Send '3'
            elif fingerUp == [0, 1, 1, 1, 1]:
                print("Gesture: Four fingers up")
                ser.write(b'4')  # Send '4'
            elif fingerUp == [1, 1, 1, 1, 1]:
                print("Gesture: Five fingers up")
                ser.write(b'5')  # Send '5'

        cv2.imshow("frame", frame)

        # Check for 'k' key press to break the loop
        if cv2.waitKey(1) & 0xFF == ord('k'):
            print("Closing camera...")
            break

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Release resources in the 'finally' block
    video.release()
    cv2.destroyAllWindows()
    ser.close()
    print("Camera and serial connection closed.")
