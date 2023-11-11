import cv2
cap=cv2.VideoCapture("log/*")
while True:
    _, frame = cap.read()
    cv2.imshow("frame",frame)
    if cv2.waitKey(0) & 0xff == ord('q'):
        break