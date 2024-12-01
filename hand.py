import time
import numpy as np
import HandTrackingModule as htm
import serial
import cv2  # Make sure to import OpenCV

# Initialize Arduino serial connection (adjust port if necessary)
try:
    arduino = serial.Serial('/dev/tty.usbmodem1301', 9600)
    time.sleep(2)
    print("Arduino connected successfully")
except:
    print("Failed to connect to Arduino")

# Open default camera
cap = cv2.VideoCapture(0)
detector = htm.handDetector(maxHands=1, detectionCon=0.6, trackCon=0.5)

# Define maximum bending angle
FINGER_MAX_ANGLE = 90  # Maximum safe bending angle in degrees

# Joints used for calculating angles (thumb to pinky)
joint_list = [
    [2, 3, 4],  # Thumb
    [6, 7, 8],  # Index
    [10, 11, 12],  # Middle
    [14, 15, 16],  # Ring
    [18, 19, 20]   # Pinky
]

def calculate_angle(pointA, pointB, pointC):
    """
    Calculate the angle between three points (in degrees).
    If the palm is open, the angle should be close to 0.
    """
    BA = np.array([pointA[0] - pointB[0], pointA[1] - pointB[1]])  # Vector from B to A
    BC = np.array([pointC[0] - pointB[0], pointC[1] - pointB[1]])  # Vector from B to C
    cosine_angle = np.dot(BA, BC) / (np.linalg.norm(BA) * np.linalg.norm(BC))
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))  # Clip for numerical stability
    return angle

def map_angle(angle, in_min, in_max, out_min, out_max):
    """
    Map the angle from one range to another.
    """
    return out_min + (in_max - angle) * (out_max - out_min) / (in_max - in_min)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Failed to capture image")
        break

    frame = cv2.flip(frame, 1)  # Flip image horizontally
    frame = detector.findHands(frame)  # Detect hands
    lmList, _ = detector.findPosition(frame, draw=False)  # Get landmarks

    if lmList:
        angles = []

        for joint in joint_list:
            # Get the landmark coordinates for the joint
            pointA = lmList[joint[0]][1:]  # First joint
            pointB = lmList[joint[1]][1:]  # Middle joint
            pointC = lmList[joint[2]][1:]  # Last joint

            # Calculate the angle
            raw_angle = calculate_angle(pointA, pointB, pointC)

            # Check the bending direction
            if pointC[1] > pointB[1]:  # Finger bending towards the palm
                raw_angle = 180 - raw_angle  # Adjust angle for correct bending direction

            # Map the angle to the range 0-45 degrees
            angle_mapped = map_angle(raw_angle, 0, 180, 0, FINGER_MAX_ANGLE)
            angles.append(angle_mapped)

            # Display the mapped angle on the frame
            cv2.putText(
                frame, f"{int(angle_mapped)}°",
                (pointB[0], pointB[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
            )

        # Create output string for Arduino
        angles_str = ','.join(str(int(angle)) for angle in angles)

        if arduino.is_open:
            # Send the output to Arduino
            arduino.write(f"{angles_str}\n".encode())
            print(f"Sent to Arduino: {angles_str}")

    else:
        print("Hand not detected or incomplete landmarks.")

    # Display the frame with angles
    cv2.putText(frame, 'Press "q" to exit', (10, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
    cv2.imshow("Finger Angles", frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
if arduino.is_open:
    arduino.close()

# import time
# import numpy as np
# import HandTrackingModule as htm
# import serial

# # Initialize Arduino serial connection (adjust port if necessary)
# try:
#     arduino = serial.Serial('/dev/tty.usbmodem1301', 9600) 
#     time.sleep(2)
#     print("Arduino connected successfully")
# except:
#     print("Failed to connect to Arduino")

# # Open default camera
# cap = cv2.VideoCapture(0)
# detector = htm.handDetector(maxHands=1, detectionCon=0.6, trackCon=0.5)

# # Define maximum bending angle
# FINGER_MAX_ANGLE = 90  # Maximum safe bending angle in degrees

# # Joints used for calculating angles (thumb to pinky)
# joint_list = [
#     [2, 3, 4],  # Thumb
#     [6, 7, 8],  # Index
#     [10, 11, 12],  # Middle
#     [14, 15, 16],  # Ring
#     [18, 19, 20]   # Pinky
# ]

# def calculate_angle(pointA, pointB, pointC):
#     """
#     Calculate the angle between three points (in degrees).
#     If the palm is open, the angle should be close to 0.
#     """
#     BA = np.array([pointA[0] - pointB[0], pointA[1] - pointB[1]])  # Vector from B to A
#     BC = np.array([pointC[0] - pointB[0], pointC[1] - pointB[1]])  # Vector from B to C
#     cosine_angle = np.dot(BA, BC) / (np.linalg.norm(BA) * np.linalg.norm(BC))
#     angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))  # Clip for numerical stability
#     return angle

# def map_angle(angle, in_min, in_max, out_min, out_max):
#     """
#     Map the angle from one range to another.
#     """
#     return out_min + (in_max - angle) * (out_max - out_min) / (in_max - in_min)

# while cap.isOpened():
#     success, frame = cap.read()
#     if not success:
#         print("Failed to capture image")
#         break

#     frame = cv2.flip(frame, 1)  # Flip image horizontally
#     frame = detector.findHands(frame)  # Detect hands
#     lmList, _ = detector.findPosition(frame, draw=False)  # Get landmarks

#     if lmList:
#         angles = []
        
#         for joint in joint_list:
#             # Get the landmark coordinates for the joint
#             pointA = lmList[joint[0]][1:]  # First joint
#             pointB = lmList[joint[1]][1:]  # Middle joint
#             pointC = lmList[joint[2]][1:]  # Last joint
            
#             # Calculate the angle
#             raw_angle = calculate_angle(pointA, pointB, pointC)
            
#             # Check the bending direction
#             if pointC[1] > pointB[1]:  # Finger bending towards the palm
#                 raw_angle = 180 - raw_angle  # Adjust angle for correct bending direction
            
#             # Map the angle to the range 0-45 degrees
#             angle_mapped = map_angle(raw_angle, 0, 180, 0, FINGER_MAX_ANGLE)
#             angles.append(angle_mapped)

#             # Display the mapped angle on the frame
#             cv2.putText(
#                 frame, f"{int(angle_mapped)}°", 
#                 (pointB[0], pointB[1] - 10), 
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
#             )

#         angles_str = ','.join(str(angle) for angle in angles_output)

#         if arduino.is_open:
#             # Send the output to Arduino
#             arduino.write(f"{angles_str}\n".encode())
#             print(f"Sent to Arduino: {angles_str}")

#     else:
#         print("Hand not detected or incomplete landmarks.")

#     # Display the frame with angles
#     cv2.putText(frame, 'Press "q" to exit', (10, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
#     cv2.imshow("Finger Angles", frame)

#     # Exit on pressing 'q'
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Clean up
# cap.release()
# cv2.destroyAllWindows()
# if arduino.is_open:
#     arduino.close()
