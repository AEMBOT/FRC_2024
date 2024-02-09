#without differentiation between closest and furthest notes

from collections import deque
import numpy as np
import argparse
import cv2
from networktables import NetworkTables
from networktables import NetworkTableEntry
from networktables import NetworkTablesInstance
import logging
import time
import sys

logging.basicConfig(level=logging.DEBUG)
#NetworkTable.setIPAddress("127.0.0.1")
#NetworkTable.setClientMode()
NetworkTables.initialize()
nd = NetworkTables.getTable("NotesDetection")

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

vid = cv2.VideoCapture(3)
#lower_orange = np.array([0, 150, 150])
#upper_orange = np.array([15, 255, 255])
lower_orange = np.array([0,150, 140])
upper_orange = np.array([10, 255, 255])
pts = deque(maxlen=args["buffer"])

# Artificial coordinate system parameters
coord_system_origin = (100, 100)  # Coordinates of the top-left corner of the system
coord_system_scale = 10  # Scale factor to convert pixel units to artificial coordinate units

#this is the function to output to network tables
def getNotePosition(side_text):
    if side_text == "Left":
        return "l"
    if side_text == "Right":
        return "r"
    if side_text == "Center":
        return "c"
    else:
        return ""

while True:
    _, frame = vid.read()

    # Ball tracking with color and elliptical Hough transform
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None

    closest_center = None
    closest_distance = float('inf')  # Initialize to positive infinity

    if len(cnts) > 0:
        for c in cnts:
            # Check if there are enough points to fit an ellipse
            if len(c) >= 5:
                ellipse = cv2.fitEllipse(c)
                center = (int(ellipse[0][0]), int(ellipse[0][1]))

                if max(ellipse[1]) > 10 and min(ellipse[1]) > 5:
                    # convert pixel coordinates to artificial coordinates
                    ring_center_artificial = (
                        int((center[0] - coord_system_origin[0]) / coord_system_scale),
                        int((center[1] - coord_system_origin[1]) / coord_system_scale)
                    )

                    # draw the ellipse and center on the frame
                    cv2.ellipse(frame, ellipse, (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    # display coordinates next to the ring ellipse
                    cv2.putText(frame, f"({ring_center_artificial[0]}, {ring_center_artificial[1]})",
                                (center[0] + 10, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    # Determine if the ring is on the left or right side
                    middle_x = frame.shape[1] // 2  # Middle of the frame in x-coordinate
                    leeway = 50  # Adjust the leeway as needed

                    if center[0] < middle_x - leeway:
                        side_text = "Left"
                    elif center[0] > middle_x + leeway:
                        side_text = "Right"
                    else:
                        side_text = "Center"
                    
                    nd.putString("note position", side_text)
                    nd.putNumber("note x pixel", int(ellipse[0][0]))
                    getNotePosition(side_text)

                    # Display side information
                    cv2.putText(frame, f"Ring is on the {side_text}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

                    # Calculate the distance between the centers
                    distance = np.sqrt((ring_center_artificial[0] - 0)**2 + (ring_center_artificial[1] - 0)**2)

                    # Update closest item if it is closer
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_center = ring_center_artificial

        else: 
            side_text = "testing"
            nd.putString("not detection", side_text)
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

    # Display the frame with ball tracking
    cv2.imshow("Frame", frame)

    # Filtered color frame logic
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

        # Step 7: Apply a mask to filter out color information
        mask = np.zeros_like(gray)

        # Adjust the mask size to cover only the immediate region around the ellipse
        mask_radius = int(min(ellipse[1]) / 2)
        cv2.ellipse(mask, ellipse, 255, -1)

        # Create a binary mask for the specified color range
        color_mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Combine the color mask with the previous mask
        combined_mask = cv2.bitwise_and(color_mask, mask)

        # Optionally, dilate the combined mask to extend the covered area
        combined_mask = cv2.dilate(combined_mask, None, iterations=2)

        filtered_color = cv2.bitwise_and(frame, frame, mask=combined_mask)

        # Display the filtered color frame
        cv2.imshow("Filtered Color", filtered_color)

    # Exit condition
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()
