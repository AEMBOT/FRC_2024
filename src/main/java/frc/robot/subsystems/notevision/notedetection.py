import numpy as np
import cv2

#TODO: add networktables in the pushed code to github

vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) #0.25 on windows, 1 on orange pi/linux
vid.set(cv2.CAP_PROP_EXPOSURE, -6.0) #-6.0 on windows. 100 on orange pi/linux

# Parameters for inner ellipse detection
lower_orange_inner = np.array([0, 190, 0])
upper_orange_inner = np.array([4, 255, 50])
min_area_threshold_inner = 500

# Parameters for outer ellipse detection
lower_orange_outer = np.array([0, 190, 0])
upper_orange_outer = np.array([4, 255, 50])
min_area_threshold_outer = 500
max_area_threshold_outer = 100000

coord_system_origin = (100, 100)
coord_system_scale = 5

while True:
    _, frame = vid.read()
    middle_x = frame.shape[1] // 2  # Middle of the frame in x-coordinate
    leeway = 50  # Adjust the leeway as needed
    avg_intensity = np.mean(frame)
    low_light_condition = avg_intensity < 50

    # Inner ellipse detection
    hsv_inner = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orange_mask_inner = cv2.inRange(hsv_inner, lower_orange_inner, upper_orange_inner)    
    gray_inner = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, otsu_thresh_inner = cv2.threshold(gray_inner, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    combined_mask_inner = cv2.bitwise_or(orange_mask_inner, otsu_thresh_inner)
    contours_inner, hierarchy_inner = cv2.findContours(combined_mask_inner, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    inner_ellipse_found = False

    for i, contour in enumerate(contours_inner):
        if hierarchy_inner[0][i][3] != -1:
            area = cv2.contourArea(contour)
            if area > min_area_threshold_inner:
                if len(contour) >= 5:
                    ellipse = cv2.fitEllipse(contour)
                    if ellipse[1][0] >= 0 and ellipse[1][1] >= 0:
                        cv2.ellipse(frame, ellipse, (36, 255, 12), 2)
                        inner_ellipse_found = True
                        center = (int(ellipse[0][0]), int(ellipse[0][1]))
                        cv2.circle(frame, center, 3, (0, 0, 255), -1)
                        if center[0] < middle_x - leeway:
                                side_text = "Left"
                        elif center[0] > middle_x + leeway:
                                side_text = "Right"
                        else:
                                side_text = "Center"
                        print(side_text)
                        print(ellipse[0][1])
                        print(ellipse[0][0])

    if inner_ellipse_found:
        pass
    else:
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower_orange_outer, upper_orange_outer)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None
        
        closest_center = None
        closest_distance = float('inf')

        if len(cnts) > 0:
            for c in cnts:
                area = cv2.contourArea(c)
                if min_area_threshold_outer < area < max_area_threshold_outer:
                    if len(c) >= 5:
                        ellipse = cv2.fitEllipse(c)
                        center = (int(ellipse[0][0]), int(ellipse[0][1]))

                        if max(ellipse[1]) > 10 and min(ellipse[1]) > 5:
                            ring_center_artificial = (
                                int((center[0] - coord_system_origin[0]) /coord_system_scale),
                                int((center[1] - coord_system_origin[1]) /coord_system_scale)
                            )

                            color = (0, 255, 255)  # Default color for non-closest balls

                            # Check if the current ball is the closest one
                            if closest_center is None or ring_center_artificial[1] > closest_center[1]:
                                closest_distance = np.sqrt((ring_center_artificial[0] - 0) ** 2 + (ring_center_artificial[1] - 0) ** 2)
                                closest_center = ring_center_artificial
                                color = (255, 0, 0)  # Change color to blue for the closest ball

                            cv2.ellipse(frame, ellipse, color, 2)
                            cv2.circle(frame, center, 5, color, -1)
                            if center[0] < middle_x - leeway:
                                side_text = "Left"
                            elif center[0] > middle_x + leeway:
                                side_text = "Right"
                            else:
                                side_text = "Center"
                            print(side_text)
                            print(ellipse[0][1])
                            print(ellipse[0][0])
    cv2.imshow("Frame", frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()