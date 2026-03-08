#!/usr/bin/env python3

import cv2
from pupil_apriltags import Detector

# 0 is usually the FaceTime HD camera
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error: Could not open webcam. Check Privacy & Security settings.")
    exit()

detector = Detector(families="tag36h11")

print("Webcam started. Press 'q' to quit.")


i = 0 #keep these out of the loop so they don't reset -- is this how it works?
camerai = 0 #keep these out of the loop so they don't reset

while True:
    success, frame = camera.read()
    if not success:
        break
    
    while i == 0:
        cv2.imwrite('test.png', frame)
        i += 1

    # Convert to grayscale for the detector
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)


    if len(results) > 0:
        while camerai == 0:
            cv2.imwrite('aprilTest.png', frame)
            camerai += 1

    for r in results:
        # Draw a box around the tag
        (ptA, ptB, ptC, ptD) = r.corners
        cv2.line(frame, tuple(ptA.astype(int)), tuple(ptB.astype(int)), (0, 255, 0), 2)
        cv2.putText(frame, f"ID: {r.tag_id}", (int(ptA[0]), int(ptA[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Mac Testing Feed", frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()