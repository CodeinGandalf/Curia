#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import cv2
import numpy as np
import rospy


# Search for the best can:
def find_best_can(camera_index):
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        rospy.loginfo("Error, device couldn't be opened!")
        return

    # Expected ratio for the can is 2.51 with a tolerance of 20%:
    expected_aspect_ratio = 2.51
    tolerance = 0.2
    i = 0
    can = False

    while True:
        # Run while True:
        ret, frame = cap.read()
        if not ret:
            cap.release()
            cv2.destroyAllWindows()
            return False

        # Convert the picture into gray values:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow("gray", gray)

        # Blur the picture:
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        cv2.imshow("blurred", blurred)

        """
        Define the upper and lower edge for the filtering. In this case we
        expect an nearly black can with significantly brighter letters
        and other stuff that would destroy the tracking of the contours of the
        can. Based upon those pixel that have a value within the bounds the
        mask set's the value to 255. All other values get set to 0.
        """
        lower_black = 0
        upper_black = 50
        mask_black = cv2.inRange(blurred, lower_black, upper_black)

        cv2.imshow("masked black", mask_black)

        # Use the mask to get the interesting parts of the picture:
        masked_blurred = cv2.bitwise_and(blurred, blurred, mask=mask_black)

        cv2.imshow("masked blurred", masked_blurred)

        # Normalize the filtered picture:
        stretched = cv2.normalize(masked_blurred, None, 0, 255, cv2.NORM_MINMAX)

        cv2.imshow("stretched", stretched)

        # Detect the contours and search for the closed edges:
        edged = cv2.Canny(stretched, 30, 100)

        cv2.imshow("edged", edged)

        kernel = np.ones((5, 5), np.uint8)
        edged_closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)

        cv2.imshow("edged_closed", edged_closed)

        contours, _ = cv2.findContours(edged_closed, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # Set the start parameters:
        best_contour = None
        best_score = -1

        # Check all found contours:
        for contour in contours:
            # Calculat the area of the contours in pixel**2:
            area = cv2.contourArea(contour)

            # Ignore all contours that have a to small surface:
            if area > 500:
                # Get the coords of the edgepoints + lenght for the rectangle:
                x, y, w, h = cv2.boundingRect(contour)

                # Check if the leght of the sides isn't 0:
                if w == 0 or h == 0:
                    continue

                # Calculate the ratio:
                aspect_ratio = h / w
                ratio_diff = abs(aspect_ratio - expected_aspect_ratio)

                # Check if the ration diff is within the tolerance:
                if ratio_diff < tolerance:
                    can = True
                    cap.release()
                    cv2.destroyAllWindows()
                    return True

        output_frame = frame.copy()

        # Brake after a short time:
        if i < 30:
            i += 1
        elif i >= 30:
            if can is False:
                rospy.loginfo("Reached target 30\r")
                cap.release()
                cv2.destroyAllWindows()
                cv2.waitKey(2)
                cap.release()
                cv2.destroyAllWindows()
                return False

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Show the original and the result:
        cv2.imshow("Original", frame)
        cv2.imshow("Ergebnis", output_frame)

        rospy.sleep(1)

    # Close the openCV windows:
    cap.release()
    cv2.destroyAllWindows()

    rospy.loginfo(f'state of can: {can}')
