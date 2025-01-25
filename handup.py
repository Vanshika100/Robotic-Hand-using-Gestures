import cvzone
import cv2
from cvzone.HandTrackingModule import HandDetector
from cvzone.SerialModule import SerialObject

cap = cv2.VideoCapture(0)  

mySerial = SerialObject('/dev/tty.usbmodem1301', 9600, 1)

detector = HandDetector(maxHands=1, detectionCon=0.6)

while True:
    success, img = cap.read()
    if not success:
        print("Failed to capture video frame")
        break

    hands, img = detector.findHands(img, draw=True)  
    if hands:
        hand = hands[0] 
        lmList = hand['lmList'] 
        bbox = hand['bbox']      
        fingers = detector.fingersUp(hand) 
        
        fingers_as_str = ''.join(map(str, fingers))
        mySerial.sendData(fingers_as_str) 

    cv2.imshow('Image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
