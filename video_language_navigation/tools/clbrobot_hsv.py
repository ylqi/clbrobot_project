import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow("ClbrobotHSVBars")

cv2.createTrackbar("LH", "ClbrobotHSVBars", 0, 179, nothing)
cv2.createTrackbar("LS", "ClbrobotHSVBars", 0, 255, nothing)
cv2.createTrackbar("LV", "ClbrobotHSVBars", 0, 255, nothing)
cv2.createTrackbar("UH", "ClbrobotHSVBars", 179, 179, nothing)
cv2.createTrackbar("US", "ClbrobotHSVBars", 255, 255, nothing)
cv2.createTrackbar("UV", "ClbrobotHSVBars", 255, 255, nothing)

while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("LH", "ClbrobotHSVBars")
    l_s = cv2.getTrackbarPos("LS", "ClbrobotHSVBars")
    l_v = cv2.getTrackbarPos("LV", "ClbrobotHSVBars")
    u_h = cv2.getTrackbarPos("UH", "ClbrobotHSVBars")
    u_s = cv2.getTrackbarPos("US", "ClbrobotHSVBars")
    u_v = cv2.getTrackbarPos("UV", "ClbrobotHSVBars")

    lower_blue = np.array([l_h, l_s, l_v])
    upper_blue = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("ClbrobotOrgin", frame)
    cv2.imshow("ClbrobotMask", mask)
    cv2.imshow("ClbrobotResult", result)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
