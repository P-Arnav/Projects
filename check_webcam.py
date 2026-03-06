import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTOFOCUS,     1)
cap.set(cv2.CAP_PROP_FOCUS,         0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # manual exposure
cap.set(cv2.CAP_PROP_EXPOSURE,      -8)    # lower = darker, try -8 to -12
cap.set(cv2.CAP_PROP_BRIGHTNESS,    80)
cap.set(cv2.CAP_PROP_CONTRAST,      50)

if not cap.isOpened():
    print("ERROR: Could not open camera.")
    exit()

print("Camera opened. Press Q to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Failed to read frame.")
        break
    cv2.imshow("Webcam Test (Q to quit)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Done.")
