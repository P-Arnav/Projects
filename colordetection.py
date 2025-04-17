import serial
import cv2
import numpy as np
import time
import os

# Check if running with GUI (X11)
headless = not os.environ.get("DISPLAY")

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)
arduino.reset_input_buffer()

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -5)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 35)
cap.set(cv2.CAP_PROP_CONTRAST, 70)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

ROI_X, ROI_Y, ROI_W, ROI_H = 200, 130, 200, 200
last_color = None

def detect_and_draw(mask, color_name, color_bgr, frame, roi_offset):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:
            x, y, w, h = cv2.boundingRect(largest_contour)
            x += roi_offset[0]
            y += roi_offset[1]
            if not headless:
                cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
            return True
    return False

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        if not headless:
            cv2.rectangle(frame, (ROI_X, ROI_Y), (ROI_X + ROI_W, ROI_Y + ROI_H), (0, 255, 255), 2)

        roi = frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        red_lower1, red_upper1 = np.array([0, 100, 100]), np.array([10, 255, 255])
        red_lower2, red_upper2 = np.array([170, 100, 100]), np.array([180, 255, 255])
        green_lower, green_upper = np.array([45, 100, 100]), np.array([75, 255, 255])
        blue_lower, blue_upper = np.array([100, 100, 100]), np.array([130, 255, 255])
        black_lower, black_upper = np.array([0, 0, 0]), np.array([180, 255, 40])

        mask_red = cv2.bitwise_or(cv2.inRange(hsv, red_lower1, red_upper1), cv2.inRange(hsv, red_lower2, red_upper2))
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
        mask_black = cv2.inRange(hsv, black_lower, black_upper)

        _, black_mask_gray = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY_INV)
        mask_black = cv2.bitwise_and(mask_black, black_mask_gray)

        color_detected = None
        roi_offset = (ROI_X, ROI_Y)
        if detect_and_draw(mask_red, "Red", (0, 0, 255), frame, roi_offset):
            color_detected = 'R'
        elif detect_and_draw(mask_green, "Green", (0, 255, 0), frame, roi_offset):
            color_detected = 'G'
        elif detect_and_draw(mask_blue, "Blue", (255, 0, 0), frame, roi_offset):
            color_detected = 'B'
        elif detect_and_draw(mask_black, "Black", (50, 50, 50), frame, roi_offset):
            color_detected = 'D'

        print(f"Red: {cv2.countNonZero(mask_red)}, Green: {cv2.countNonZero(mask_green)}, "
              f"Blue: {cv2.countNonZero(mask_blue)}, Black: {cv2.countNonZero(mask_black)}")

        if color_detected and color_detected != last_color:
            arduino.write(color_detected.encode())
            print(f"Sent: {color_detected}")
            last_color = color_detected
            time.sleep(0.5)
            response = arduino.readline().decode().strip()
            if response:
                print(f"Arduino Response: {response}")

        if not headless:
            key = cv2.waitKey(1) & 0xFF
            cv2.imshow('Frame', frame)
            cv2.imshow('ROI', roi)
            if key == ord('q'):
                break

finally:
    cap.release()
    if not headless:
        cv2.destroyAllWindows()
    arduino.close()
