import mediapipe as mp 
import cv2 
import numpy as np 
from matplotlib import pyplot as plt
import serial

port = 'COM17'
stm = serial.Serial(port=port, baudrate=115200, timeout=0.01)
def set_angles(angles):
    msg = ''
    
    for angle in angles:
        a = str(angle)
        while len(a) < 3:
            a = '0' + a
        msg += a
    
    msg = '<' + msg + '>'
  
    print("Sending: ", msg)    
    for c in msg:
        stm.write(bytes(c, 'utf-8'))
    data = stm.readline()
    print("Receiving: ", data)

def translate(value, leftMin, leftMax, rightMin, rightMax):

    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def compute_finger_angles(image, results, joint_list):

    angles = []

    for hand in results.multi_hand_landmarks:
        for i, joint in enumerate(joint_list):
            a = np.array([hand.landmark[joint[0]].x, hand.landmark[joint[0]].y])
            b = np.array([hand.landmark[joint[1]].x, hand.landmark[joint[1]].y])
            c = np.array([hand.landmark[joint[2]].x, hand.landmark[joint[2]].y])

            rad = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
            angle = np.abs(rad*180.0/np.pi)

            if angle > 180:
                angle = 360 - angle
            
            if i == 0:
                angle = np.interp(angle,[90,180],[0, 200])
                angle = min(180, angle)
            else:
                angle = np.interp(angle,[30,180],[0, 180])
                angle = min(180, angle)

            angles.append(int(angle))
            cv2.putText(image, str(round(angle, 2)), tuple(np.multiply(b, [640, 480]).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 2, cv2.LINE_AA)
    return image, angles


# Setup mediapipe for landmarks detection
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0)

joint_list = [ [4, 3, 2], [7, 6, 5], [11, 10, 9], [15, 14, 13], [19, 18, 17]]

with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5, max_num_hands=1) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = cv2.flip(image, 1)
        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True
        if results.multi_hand_landmarks:
            for num, hand in enumerate(results.multi_hand_landmarks):
                mp_drawing.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS, 
                                            mp_drawing.DrawingSpec(color=(0, 0, 155), thickness=2, circle_radius=4),
                                            mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2))
            image, angles = compute_finger_angles(image, results, joint_list)
            set_angles(angles) 
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow('Hand Tracking', image)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()
