import cv2
from picamera2 import Picamera2
import numpy as np

picam2 = Picamera2()
picam2.preview_configuration.main.size = (800,800)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

while True:
    frame = picam2.capture_array()
    
    
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray_blurred = cv2.blur(gray_frame, (8, 8))

    detected_circles = cv2.HoughCircles(gray_blurred,
                                        cv2.HOUGH_GRADIENT, 1, 10, param1=20,
                                        param2=40, minRadius=10, maxRadius=80)

    if detected_circles is not None:

        pt = np.uint16(np.around(detected_circles))[0][0]
        a, b, r = pt[0], pt[1], pt[2]

        cv2.circle(frame, (a, b), r, (0, 255, 0), 2)

        cv2.circle(frame, (a, b), 1, (0, 0, 255), 3)
        
        cv2.circle(frame, (400, 400), 2, (255, 0, 0), 5)

        cv2.line(frame, (400, 400), (a, b), (255, 0, 127), 2)
    
    cv2.imshow("Camera", frame)
    
    if cv2.waitKey(1)==ord('q'):
        break

cv2.destroyAllWindows()
