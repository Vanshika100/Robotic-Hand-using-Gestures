# import cvzone
# import cv2
# from cvzone.HandTrackingModule import HandDetector
# from cvzone.SerialModule import SerialObject

# # Initialize video capture
# cap = cv2.VideoCapture(0)

# # Initialize serial communication (make sure the correct port is used)
# mySerial = SerialObject('/dev/tty.usbmodem1301', 9600, 1)

# # Initialize hand detector
# detector = HandDetector(maxHands=1, detectionCon=0.6)

# while True:
#     success, img = cap.read()
#     if not success:
#         print("Failed to capture video frame")
#         break

#     # Process frame for hands
#     hands, img = detector.findHands(img, draw=True)  # Returns detected hands and the processed image

#     if hands:
#         hand = hands[0]  # Get the first hand detected
#         fingers = detector.fingersUp(hand)  # Finger status (1 for open, 0 for closed)
#         print(fingers)

#         # Convert finger states to a numeric string (e.g., "01111")
#         fingers_as_str = ''.join(map(str, fingers))

#         # Send the string to Arduino
#         mySerial.sendData(fingers_as_str)

#     # Display the frame
#     cv2.imshow('Image', img)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()

# import cvzone
# import cv2
# from cvzone.HandTrackingModule import HandDetector
# from cvzone.SerialModule import SerialObject

# # Initialize video capture
# cap = cv2.VideoCapture(0)

# # Initialize serial communication (make sure the correct port is used)
# mySerial = SerialObject('/dev/tty.usbmodem1301', 9600, 1)

# # Initialize hand detector
# detector = HandDetector(maxHands=1, detectionCon=0.6)

# while True:
#     success, img = cap.read()
#     if not success:
#         print("Failed to capture video frame")
#         break

#     # Process frame for hands
#     hands, img = detector.findHands(img, draw=True)  # Returns detected hands and the processed image

#     if hands:
#         hand = hands[0]  # Get the first hand detected
#         fingers = detector.fingersUp(hand)  # Finger status (1 for open, 0 for closed)
#         print(fingers)

#         # Convert finger states to a numeric string (e.g., "01111")
#         fingers_as_str = ''.join(map(str, fingers))

#         # Send the string to Arduino
#         mySerial.sendData(fingers_as_str)

#     # Display the frame
#     cv2.imshow('Image', img)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()


import cvzone
import cv2
from cvzone.HandTrackingModule import HandDetector
from cvzone.SerialModule import SerialObject

# Initialize video capture
cap = cv2.VideoCapture(0)  

# Initialize serial communication (adjust the port if needed)
mySerial = SerialObject('/dev/tty.usbmodem1301', 9600, 1)

# Initialize hand detector
detector = HandDetector(maxHands=1, detectionCon=0.6)

while True:
    success, img = cap.read()
    if not success:
        print("Failed to capture video frame")
        break

    # Process frame for hands
    hands, img = detector.findHands(img, draw=True)  # Returns detected hands and the processed image

    if hands:
        hand = hands[0]  # Get the first hand detected
        lmList = hand['lmList']  # List of 21 landmarks
        bbox = hand['bbox']      # Bounding box info: [x, y, w, h]
        fingers = detector.fingersUp(hand)  # Finger status (1 for open, 0 for closed)
        
        # Convert finger states to a numeric string (e.g., "01111")
        fingers_as_str = ''.join(map(str, fingers))
        mySerial.sendData(fingers_as_str)  # Send numeric string only, no newline

    # Display the frame
    cv2.imshow('Image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

