import numpy as np
import cv2
from networktables import NetworkTables
import logging 

logging.basicConfig(level=logging.DEBUG)
NetworkTables.initialize()
nd = NetworkTables.getTable("NotesDetection")  

def ellipseDetection(rknn_lite, frame): 
    frame = cv2.resize(frame, (640, 640))
    outputs = rknn_lite.inference(inputs=[frame])

    input0_data = outputs[0].reshape([3, -1]+list(outputs[0].shape[-2:]))
    input1_data = outputs[1].reshape([3, -1]+list(outputs[1].shape[-2:]))
    input2_data = outputs[2].reshape([3, -1]+list(outputs[2].shape[-2:]))
    
    lower_orange_inner = np.array([0, 100, 100])
    upper_orange_inner = np.array([20, 150, 150])
    min_area_threshold_inner = 500

    lower_orange_outer = np.array([0, 100, 100])
    upper_orange_outer = np.array([20, 150, 150])
    min_area_threshold_outer = 500
    max_area_threshold_outer = 100000

    coord_system_origin = (100, 100)
    coord_system_scale = 5 
    leeway = 50  

    middle_x = frame.shape[1] // 2 
    #this is really important
    y_values = [] #clear y values before each frame
    x_values = []

    # Inner ellipse detection
    hsv_inner = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orange_mask_inner = cv2.inRange(hsv_inner, lower_orange_inner, upper_orange_inner)    
    gray_inner = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, otsu_thresh_inner = cv2.threshold(gray_inner, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    combined_mask_inner = cv2.bitwise_or(orange_mask_inner, otsu_thresh_inner)
    #kernel_inner = np.ones((5, 5), "uint8")
    #combined_mask_inner = cv2.dilate(combined_mask_inner, kernel_inner)

    contours_inner, hierarchy_inner = cv2.findContours(combined_mask_inner, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    inner_ellipse_found = False

    for i, contour in enumerate(contours_inner):
        if hierarchy_inner[0][i][3] != -1:
            area = cv2.contourArea(contour)
            if area > min_area_threshold_inner:
                if len(contour) >= 5:
                    ellipse = cv2.fitEllipse(contour)
                    if ellipse[1][0] >= 0 and ellipse[1][1] >= 0:
                        #cv2.ellipse(frame, ellipse, (36, 255, 12), 2)
                        inner_ellipse_found = True
                        center = (int(ellipse[0][0]), int(ellipse[0][1]))
                        #cv2.circle(frame, center, 3, (0, 0, 255), -1)
                        if center[0] < middle_x - leeway:
                            side_text = "Left"
                        elif center[0] > middle_x + leeway:
                            side_text = "Right"
                        else:
                            side_text = "Center"
                        print(ellipse[0][0])
                        print(ellipse[0][1])
                        nd.putString("note position", side_text)
                        nd.putNumber("note x pixel", int(ellipse[0][0]))
                        nd.putNumber("note y pixel", int(ellipse[0][1]))
                        #this is really important
                        y_values.append(int(ellipse[0][1])) #appends to y value
                        nd.putNumberArray("note y pixel array", y_values)
                        x_values.append(int(ellipse[0][0]))
                        nd.putNumberArray("note x pixel array", x_values)

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

        if len(cnts) > 0:
            for c in cnts:
                # Check if there are enough points to fit an ellipse
                area = cv2.contourArea(c)
                if min_area_threshold_outer < area < max_area_threshold_outer:
                    if len(c) >= 5:
                        ellipse = cv2.fitEllipse(c)
                        center = (int(ellipse[0][0]), int(ellipse[0][1]))

                        if max(ellipse[1]) > 10 and min(ellipse[1]) > 5:
                            # convert pixel coordinates to artificial coordinates
                            ring_center_artificial = (
                                int((center[0] - coord_system_origin[0]) /coord_system_scale),
                                int((center[1] - coord_system_origin[1]) /coord_system_scale)
                            )

                            #cv2.ellipse(frame, ellipse, color, 2)
                            #cv2.circle(frame, center, 5, color, -1)
                            if center[0] < middle_x - leeway:
                                side_text = "Left"
                            elif center[0] > middle_x + leeway:
                                side_text = "Right"
                            else:
                                side_text = "Center"
                            nd.putString("note position", side_text)
                            nd.putNumber("note x pixel", int(ellipse[0][0]))
                            nd.putNumber("note y pixel",int(ellipse[0][1]))
                            #this is realy important
                            y_values.append(int(ellipse[0][1]))
                            nd.putNumberArray("note y pixel array", int(ellipse[0][1]))
                            x_values.append(int(ellipse[0][0]))
                            nd.putNumberArray("note x pixel array", x_values)
    return frame