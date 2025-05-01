# -*- coding: utf-8 -*-
"""
Created on Wed Apr 30 13:46:31 2025

@author: Project Curia
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt


# Function to generate the gauss probability function:
def generate_gaussian_matrix(size_x, size_y, mu_x, mu_y, sigma):
    # Generate the matix for the normalized probability function:
    x = np.linspace(0, size_x - 1, size_x)
    y = np.linspace(0, size_y - 1, size_y)
    xx, yy = np.meshgrid(x, y)
    gaussian = np.exp(-((xx - mu_x)**2 + (yy - mu_y)**2) / (2 * sigma**2))
    gaussian /= np.max(gaussian)
    return gaussian


# Search for the best can:
def find_best_can(poseCan, camera_index, offsetCam, offsetLidar):
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Error, device couldn't be opened!")
        return

    # Expected ratio for the can is 2.51 with a tolerance of 10%:
    expected_aspect_ratio = 2.51
    tolerance = 0.2

    # Position fake LIDAR Data placed into the middle of the camera chip:
    # lidar_mu_x = int(u)
    lidar_mu_x = 640 // 2
    lidar_mu_y = 480 // 2
    picture_hight = 480
    picture_width = 640
    f_x = 4
    c_x = 640/2
    sigma = 70
    i = 0
    can = False

    deltaXGlobe = poseCan[0][0]*np.cos(poseCan[0][1])
    xGlobe = deltaXGlobe - offsetLidar

    X = xGlobe - offsetCam
    Z = poseCan[0][0]*np.sin(poseCan[0][1])

    u = X * f_x / Z + c_x
    print(f'Value of u: {u}')

    x_b = None

    # Generate the probabikity function with the LIDAR data:
    probability_map = generate_gaussian_matrix(picture_width, picture_hight,
                                               lidar_mu_x, lidar_mu_y, sigma)

    # Plot der Gaussian-Verteilung
    plt.figure(figsize=(8, 8))
    plt.imshow(probability_map, cmap='viridis', interpolation='nearest')
    plt.colorbar(label='Wahrscheinlichkeitsdichte')
    plt.title('Gaussian-Verteilung')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

    while True:
        # Run while True, later this will run only once:
        ret, frame = cap.read()
        if not ret:
            cap.release()
            cv2.destroyAllWindows()
            return False, False, False

        # Convert the picture into gray values:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow("gray", gray)

        # Blur the picture a bit:
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
        stretched = cv2.normalize(
            masked_blurred, None, 0, 255, cv2.NORM_MINMAX)

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
                    # Define the coords of the center:
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # Check the probability of the found center point:
                    if 0 <= center_x < picture_width and 0 <= center_y < picture_hight:
                        score = probability_map[int(center_y), int(center_x)]

                        # Control if this target is better then the current best option:
                        if score > best_score and score > 0.5:
                            best_contour = (x, y, w, h)
                            best_score = score
                            can = True

            output_frame = frame.copy()

            # If there is a good contour print it onto the console and into the picture:
            if best_contour:
                x_b, y_b, w_b, h_b = best_contour
                cv2.rectangle(output_frame, (x, y),
                              (x_b + w_b, y_b + h_b), (0, 255, 0), 2)
                cv2.putText(output_frame, f"Dose", (x_b, y_b - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                print(f"Center=({x_b+w_b//2}, {y_b+h_b//2}), width={w_b}, hight={h_b},\
                      ratio={h_b/w_b:.2f}, probability score={best_score:.3f}")

        if i < 100:
            i += 1
        elif i >= 100:
            print("Reached target 100")
            if x_b is not None:
                break
            else:
                cap.release()
                cv2.destroyAllWindows()
                cv2.waitKey(2)

                return False, False, False

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Show the original and the result:
        cv2.imshow("Original", frame)
        cv2.imshow("Ergebnis", output_frame)

    # Close the openCV windows:
    cap.release()
    cv2.destroyAllWindows()

    print(f'state of can: {can}')

    return x_b+w_b//2, y_b+h_b//2, can
