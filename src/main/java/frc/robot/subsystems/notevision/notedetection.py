from collections import deque
import numpy as np
import argparse
import cv2

#make the logic to detect which is closest more robust from just two notes to more. 
#compare centers from depth map with realsense technology to see which is closer / farther

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

vid = cv2.VideoCapture(3)
lower_orange = np.array([0, 150, 150])
upper_orange = np.array([15, 255, 255])
pts = deque(maxlen=args["buffer"])

coord_system_origin = (100, 100)  # top left start
coord_system_scale = 10 
center_order_yvalue = [0]
count_yvalue = 0

while True:
    _, frame = vid.read()
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None

    closest_center = None
    closest_distance = float('inf')

    if len(cnts) > 0:
        for c in cnts:
            if len(c) >= 5:
                count_yvalue += 1
                ellipse = cv2.fitEllipse(c)
                center = (int(ellipse[0][0]), int(ellipse[0][1]))
                center_order_yvalue.append(int(ellipse[0][1]))

                if max(ellipse[1]) > 10 and min(ellipse[1]) > 5:
                    ring_center_artificial = (
                        int((center[0] - coord_system_origin[0]) / coord_system_scale),
                        int((center[1] - coord_system_origin[1]) / coord_system_scale)
                    )
                    if center_order_yvalue[count_yvalue] > center_order_yvalue[count_yvalue -1]:
                        cv2.ellipse(frame, ellipse, (255, 0, 0) ,2)
                        cv2.circle(frame, center, 5, (0,0,255), -1)
                    else:
                        cv2.ellipse(frame, ellipse, (0, 255, 255), 2)
                        cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    cv2.putText(frame, f"({ring_center_artificial[0]}, {ring_center_artificial[1]})",
                                (center[0] + 10, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    middle_x = frame.shape[1] // 2
                    leeway = 50 

                    if center[0] < middle_x - leeway:
                        side_text = "Left"
                    elif center[0] > middle_x + leeway:
                        side_text = "Right"
                    else:
                        side_text = "Center"

                    cv2.putText(frame, f"Ring is on the {side_text}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    distance = np.sqrt((ring_center_artificial[0] - 0)**2 + (ring_center_artificial[1] - 0)**2)

                    if distance < closest_distance:
                        closest_distance = distance
                        closest_center = ring_center_artificial

        pts.appendleft(closest_center)

        for i in range(1, len(pts)):
            if pts[i - 1] is None or pts[i] is None:
                continue
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, (int(pts[i - 1][0] * coord_system_scale + coord_system_origin[0]),
                             int(pts[i - 1][1] * coord_system_scale + coord_system_origin[1])),
                     (int(pts[i][0] * coord_system_scale + coord_system_origin[0]),
                      int(pts[i][1] * coord_system_scale + coord_system_origin[1])),
                     (0, 0, 255), thickness)

    cv2.imshow("Frame", frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    points = cv2.findNonZero(edges)

    if points is not None and len(points) >= 5:
        ellipse = cv2.fitEllipse(points)
        cv2.ellipse(frame, ellipse, (0, 255, 255), 2)

        ellipse_center_artificial = (
            int((ellipse[0][0] - coord_system_origin[0]) / coord_system_scale),
            int((ellipse[0][1] - coord_system_origin[1]) / coord_system_scale)
        )
        cv2.putText(frame, f"({ellipse_center_artificial[0]}, {ellipse_center_artificial[1]})",
                    (int(ellipse[0][0] + 10), int(ellipse[0][1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        mask = np.zeros_like(gray)
        mask_radius = int(min(ellipse[1]) / 2)
        cv2.ellipse(mask, ellipse, 255, -1)
        color_mask = cv2.inRange(hsv, lower_orange, upper_orange)
        combined_mask = cv2.bitwise_and(color_mask, mask)
        combined_mask = cv2.dilate(combined_mask, None, iterations=2)
        filtered_color = cv2.bitwise_and(frame, frame, mask=combined_mask)

        cv2.imshow("Filtered Color", filtered_color)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()