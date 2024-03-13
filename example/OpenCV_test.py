import cv2

cap = cv2.VideoCapture(0)

try:
    while (True):
        ret, frame = cap.read()
        cv2.imshow("Test", frame)
        if cv2.waitKey(1) & 0xFF==ord('q'):
            break

except:
    print("NOT OK")
    break

cap.release()
cv2.destroyAllWindows()
