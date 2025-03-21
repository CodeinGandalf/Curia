# -*- coding: utf-8 -*-
"""
Created on Fri Mar 14 11:10:50 2025

@author: Fuuli_FH_Pfloeck_wo_programiered!; Sagitarius A*
"""

import tkinter as tk
from tkinter import Label, Button, Frame
import cv2
import os
import glob
import json
import numpy as np
import matplotlib.pyplot as plt
import cv2.aruco as aruco
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Deactivate the matplotlib function that automaticaly opens the plot:
plt.ioff()


# Setup the class for the full operation of the BV1 project:
class TrackArucoMarcerGUI:
    # Function to init the tkinter window:
    def __init__(self, root):
        # Define the geometry for the window as instance of the class:
        self.width = 1500
        self.height = 1000
        self.geometry = f'{self.width}x{self.height}'
        self.color = "white"

        """Define the variable root (in this variable the window of tkinter is
        safed, so root references to the tkinter window) as object of
        # the class to be able to use it in the class:"""
        self.root = root
        # Set the title for the tkinter window:
        self.root.title("ArUco Camera-tracker")
        # Set the start geometry of the tkinter window with the pixel size:
        self.root.geometry(self.geometry)

        # Define the bool for the status of the camera as instance:
        self.cap = None
        # Setup the image_count as instance:
        self.image_count = 0
        # Define the foldername for the pictures for the calibration:
        self.folder = "Kalibrierungsbilder"
        # Setup the bool to define if the camera is running or not as instance:
        self.running = True

        # Call the setup_main_screen function to start the next step:
        self.setup_main_screen()

    def setup_main_screen(self):
        # Set the backround color:
        self.root.configure(bg=self.color)

        # Check if the folder for the pictures exists; if not generate it:
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)

        # If the setup function get's called again set image_count to 0:
        if self.image_count != 0:
            self.image_count = 0

        # If the setup function get's called again set running to True:
        if not self.running:
            self.runnig = True

        # Kill all widgets that are opened:
        for widget in self.root.winfo_children():
            widget.destroy()

        # Set the title label above the camera picture:
        self.title_label = Label(self.root, text="Calibrate camera",
                                 font=("Arial", 24), bg=self.color)
        # Place the title label in row 0, column 1 sticked to the south:
        self.title_label.grid(
            row=0, column=1, sticky="s")

        # Define the place for the camera picture:
        self.video_label = Label(self.root, bg=self.color)
        # Place the video label in row 1, column 1:
        self.video_label.grid(row=1, column=1, sticky="nsew")

        # Create a frame to hold the buttons:
        self.button_frame = Frame(self.root, bg=self.color)
        self.button_frame.grid(row=2, column=1, pady=10)

        # Define the capture button:
        self.capture_button = Button(self.button_frame, text="Take a picture",
                                     command=self.capture_image, bg=self.color,
                                     font=("Arial", 15), width=12, height=1)
        # Define the position of the capture button:
        self.capture_button.grid(row=0, column=0, sticky="w")

        # Define the skip button:
        self.skip_button = Button(self.button_frame, text="Skip",
                                  command=self.skip, font=("Arial", 15),
                                  width=8, height=1, bg=self.color)
        # Define the position of the skip button:
        self.skip_button.grid(row=0, column=1, padx=20)

        # Define the calibrate button:
        self.calibrate_button = Button(self.button_frame, text="Calibrate now",
                                       command=self.calibrate, bg=self.color,
                                       font=("Arial", 15), width=12, height=1)
        # Define the position of the calibration button:
        self.calibrate_button.grid(row=0, column=2, sticky="e")

        # Configure grid columns of button_frame to expand the space equally:
        self.button_frame.grid_columnconfigure(0, weight=1)
        self.button_frame.grid_columnconfigure(1, weight=1)
        self.button_frame.grid_columnconfigure(2, weight=1)

        # Define the lable for the picture counter:
        self.counter_label = Label(self.root, text="Picture count: 0",
                                   font=("Arial", 15), bg=self.color)
        # Define the position of the counter:
        self.counter_label.grid(row=3, column=1, sticky="nsew")

        # Configure rows and columns of the grid to be expandable:
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=0)
        self.root.grid_rowconfigure(3, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=0)
        self.root.grid_columnconfigure(2, weight=1)

        # Load Camera Configuration:
        with open('cam.json', 'r') as f:
            config = json.load(f)

        # Load the device name:
        device = int(config['device'])

        # Open the Camera:
        self.cap = cv2.VideoCapture(device)

        # Update the window:
        self.update_frame()

    def update_frame(self):
        # If the camera isn't running go out of the function:
        if self.cap is None or not self.running:
            return

        # Collect the frame from the camera:
        ret, self.frame = self.cap.read()
        if ret:
            # Convert the picture from the camera to RGB for tkinter:
            frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            # Converte the array from OpenCV to an image:
            img = Image.fromarray(frame)
            # Convert the converted picture to an image for tkinter:
            self.imgtk = ImageTk.PhotoImage(image=img)
            # Make an object out of the instance
            self.video_label.imgtk = self.imgtk
            # Load the new picture onto the tkinter window:
            self.video_label.configure(image=self.imgtk)

        # Call the function again after 10ms:
        self.root.after(10, self.update_frame)

    def capture_image(self):
        # If the image counter is 0 delete all old pics before adding new pics:
        if self.image_count == 0:
            # Print onto the console that the old images get deleted:
            print("Delete all old pictures.")

            # Delete all old pictures in the folder:
            for file in os.listdir(self.folder):
                file_path = os.path.join(self.folder, file)
                os.remove(file_path)

        # Define the path for the picture:
        image_path = os.path.join(
            self.folder, f"image{self.image_count:04d}.png")

        # Safe the image at the given path:
        cv2.imwrite(image_path, self.frame)

        # Increase the image_count for the next image:
        self.image_count += 1

        # Update the counter_label in the tkinter GUI:
        self.counter_label.config(text=f"Safed pictures: {self.image_count}")

        # Print the path and name of the safed picture onto the console:
        print(f"Safed picture: {image_path}")

    def calibrate(self):
        # Print onto the console, that the camera get's calibrated:
        print("Starting the calibration")

        try:
            # Import the configuration parameters from the board.json file:
            with open('board.json', 'r') as f:
                config = json.load(f)

            squares = np.asarray(config['squares'])
            square_length_mm = config['square_length_millimeters']
            marker_length_mm = config['marker_length_millimeters']

            # Define the dimension of the aruco markers:
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

            # Define the parameters for the CharucoBoard:
            board = aruco.CharucoBoard(squares, square_length_mm,
                                       marker_length_mm, aruco_dict)

            # Setup the params to detect the aruco markers on the board:
            cparams = aruco.CharucoParameters()
            cparams.tryRefineMarkers = True
            dparams = aruco.DetectorParameters()
            detector = aruco.CharucoDetector(
                board, charucoParams=cparams, detectorParams=dparams)

            all_obj_points = []
            all_img_points = []
            image_size = None

            # Load the filenames:
            filenames = sorted(glob.glob(os.path.join(self.folder,
                                                      'image*.png')))

            # If no picture is available print an error message:
            if not filenames:
                print("No pictures found to calibrate the camera.")
                return

            # Iterate through the filenames of the pictures:
            for fname in filenames:
                # Load the picture:
                image = cv2.imread(fname)

                # In case the image_size is None define the size:
                if image_size is None:
                    image_size = image.shape[0:2]
                else:
                    # Check if the new picture size is equal to the old size:
                    assert image_size == image.shape[0:2]

                # Detecte the charuco board:
                charuco_corners, charuco_ids, _, _ = detector.detectBoard(
                    image)

                # Check if there are aruco markers detected in the picture:
                if charuco_corners is None or charuco_ids is None:
                    raise Exception('No charuco corners detected.')

                # Extract the object- and the image-points for the markers:
                obj_points, img_points = board.matchImagePoints(
                    charuco_corners, charuco_ids)
                all_obj_points.append(obj_points.reshape((-1, 3)))
                all_img_points.append(img_points.reshape((-1, 2)))

            # If there are no valide calib points print an error:
            if not all_obj_points:
                print("No valide calibration points found.")
                return

            # Get the repro. error, camera matrix and dist. coeffs.:
            reprojection_error, camera_matrix, dist_coeffs, _, _ = \
                cv2.calibrateCamera(all_obj_points, all_img_points, image_size,
                                    None, None)

            # Values for the camera calib data to fill it into the jsson file:
            calib_data = {
                'reprojection_error': reprojection_error,
                'camera_matrix': camera_matrix.tolist(),
                'dist_coeffs': dist_coeffs.tolist()
            }

            # Fill the calib data into the json file:
            with open('calib.json', 'w') as f:
                json.dump(calib_data, f, indent=4)

            # Print the reprojection error:
            print(f"Calibration done.Error: {reprojection_error:.2f}")

            # Show the calib success:
            self.show_calibration_success()

        # If there is an error with the calib raise the error message:
        except Exception as e:
            print(f"Error within the calibration: {e}")

    def show_calibration_success(self):
        # Set the bool running to false:
        self.running = False

        # Kill all widgets on the tkinter window:
        for widget in self.root.winfo_children():
            widget.destroy()

        # Generate the subplots to show the dist. of the camera:
        fig, axes = plt.subplots(1, 2, figsize=(8, 3))

        # Open the json file with the calib data for the camera:
        with open('calib.json', 'r') as f:
            calib = json.load(f)

        # Read the dist. values of the json file:
        dist_coeffs = np.array(calib['dist_coeffs'][0])

        # Plot the radial distortion of the camera:
        self._plot_radial_distortion(axes[0], dist_coeffs[0], dist_coeffs[1],
                                     dist_coeffs[4])

        # Plot the tangential distortion of the camera:
        self._plot_tangential_distortion(axes[1], dist_coeffs[2],
                                         dist_coeffs[3])

        # Generate the canvas and place it onto the tkinter window:
        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.draw()
        canvas_widget = canvas.get_tk_widget()

        # Place the canvas in the middle of the zell:
        canvas_widget.grid(row=1, column=0, columnspan=3, padx=20, pady=20,
                           sticky="nsew")

        # Set up the label with a fixed distance above the canvas:
        self.label = tk.Label(self.root, text="Calibration done",
                              font=("Arial", 24), bg=self.color)
        self.label.grid(row=0, column=0, columnspan=3, pady=10, sticky="nsew")

        # Define and place the repeat button:
        self.repeat_button = Button(self.root, text="Repeat", bg=self.color,
                                    command=self.start_repeat_process,
                                    font=("Arial", 15), width=10, height=1)
        self.repeat_button.grid(row=2, column=1, sticky="e")

        # Define and place the continue button:
        self.continue_button = Button(self.root, text="Continue",
                                      command=self.skip, bg=self.color,
                                      font=("Arial", 15), width=10, height=1)
        self.continue_button.grid(row=2, column=1, sticky="w")

        # Configure rows and columns to be expandable:
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_columnconfigure(2, weight=1)

    def _plot_radial_distortion(self, ax, k1, k2, k3):
        # Define a few parameters to calculate the radial distortion:
        n = 20
        x = np.linspace(-100, 100, n)
        y = np.linspace(-100, 100, n)

        # Generate the vector field plot:
        x, y = np.meshgrid(x, y)
        rsq = np.square(x) + np.square(y)
        k = k1 * rsq + k2 * rsq * rsq + k3 * rsq * rsq * rsq
        dx = x * k
        dy = y * k
        ax.quiver(x, y, dx, dy)
        ax.set_aspect('equal')
        ax.set_title('Radial distortion')

    def _plot_tangential_distortion(self, ax, p1, p2):
        # Define a few parameters to calculate the tangential distortion:
        n = 20
        x = np.linspace(-100, 100, n)
        y = np.linspace(-100, 100, n)

        # Generate the vector field plot:
        x, y = np.meshgrid(x, y)
        rsq = np.square(x) + np.square(y)
        xy = 2 * x * y
        dx = p1 * xy + p2 * (rsq + 2 * np.square(x))
        dy = p2 * xy + p1 * (rsq + 2 * np.square(y))
        ax.quiver(x, y, dx, dy)
        ax.set_aspect('equal')
        ax.set_title('Tangential distortion')

    def start_repeat_process(self):
        # Set the bool for the camera to true:
        self.running = True

        # Print the status onto the console:
        print(f"Running status set to {self.running}")

        # Call the function to repeat the calibration:
        self.setup_main_screen()

    def skip(self):
        # Print onto the console that skip has been pressed:
        print("Skip has been pressed.")

        # Stop the updating of the camera picture:
        self.running = False

        # Check if the camera has been released:
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            cv2.destroyAllWindows()

        # Kill all widgets on the tkinter window:
        for widget in self.root.winfo_children():
            widget.destroy()

        # Show the new screen:
        self.show_aruco_tracking()

    def update_zoom(self, val):
        # Update the zoom level based on the slider:
        self.limit = int(val)

    def show_aruco_tracking(self):
        # Load the ArUco marker configurations:
        with open('diamond.json', 'r') as f:
            config = json.load(f)

        square_length_mm = config['square_length_millimeters']
        marker_length_mm = config['marker_length_millimeters']

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        # Load the json file with the parameters for the Camera Calibration:
        with open('calib.json', 'r') as f:
            calib = json.load(f)

        dist_coeffs = np.array(calib['dist_coeffs'])
        camera_matrix = np.array(calib['camera_matrix'])

        # Load the setup parameters for the Camera:
        with open('cam.json', 'r') as f:
            config = json.load(f)

        device = int(config['device'])

        # Setup the video capture with OpenCV:
        cap = cv2.VideoCapture(device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(config['width']))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config['height']))

        # Define and place the camera feed label:
        self.camera_label = tk.Label(self.root, bg=self.color)
        self.camera_label.grid(row=1, column=0, padx=20, pady=20, sticky="e")

        # Set the title label above the camera picture:
        self.camera_title = Label(self.root, text="Camera picture",
                                  font=("Arial", 24), bg=self.color)
        # Place the title label in row 0, column 0:
        self.camera_title.grid(row=0, column=0, sticky="e")

        # Create the 3D plot:
        self.fig = plt.figure(figsize=(4, 4))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Limit the scaling of the axis:
        self.limit = 500
        self.ax.set_xlim(-self.limit, self.limit)
        self.ax.set_ylim(-self.limit, self.limit)
        self.ax.set_zlim(0, self.limit)
        self.ax.set_box_aspect([1, 1, 1])
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)

        # Place the canvas onto the tkinter window:
        plot_widget = self.canvas.get_tk_widget()
        plot_widget.grid(row=1, column=2, sticky="w")

        # Define and place the title of the plot above the plot:
        self.plot_title = Label(self.root, text="Plot", font=("Arial", 24),
                                bg=self.color)
        self.plot_title.grid(row=0, column=2, sticky="w")

        # Function to update the slider based on the entry value:
        def update_slider_from_entry(event):
            try:
                # Get the entered value:
                value = int(zoom_entry.get())

                # Check if the value is legal and then set it:
                if 100 <= value <= 900:
                    zoom_slider.set(value)
            except ValueError:
                pass

        # Function to update the entry value based on the slider:
        def update_entry_from_slider(value):
            # Delete the old value, save and update the new value:
            zoom_entry.delete(0, tk.END)
            zoom_entry.insert(0, str(int(float(value))))
            self.update_zoom(int(zoom_entry.get()))

        # Create and place the zoom label above the slider:
        self.zoom_label = tk.Label(self.root, text="Zoom Level",
                                   font=("Arial", 16), bg=self.color)
        self.zoom_label.grid(row=2, column=1, columnspan=1, pady=10,
                             sticky="n")

        # Define and place the slider:
        zoom_slider = tk.Scale(self.root, from_=100, to=900,
                               orient='horizontal',
                               command=update_entry_from_slider,
                               font=("Arial", 16), length=200,
                               sliderlength=40, width=30, bg=self.color)
        zoom_slider.grid(row=3, column=1, pady=20, sticky="n")

        # Set the start value:
        zoom_slider.set(self.limit)

        # Define and place the entry field:
        zoom_entry = tk.Entry(self.root, font=("Arial", 16),
                              width=5, justify="center", bg=self.color)
        zoom_entry.grid(row=2, column=1, pady=10, sticky="s")

        # Set the start value for the entry:
        zoom_entry.insert(0, str(zoom_slider.get()))

        # Setup the event listener for the return button:
        zoom_entry.bind("<Return>", update_slider_from_entry)

        # Configure rows and columns to be expandable:
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=0)
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_rowconfigure(3, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=0)
        self.root.grid_columnconfigure(2, weight=1)

        # Function to draw the coordinate system:
        def draw_coordinate_system(ax, origin, R, size):
            # Draw a coordinate system into the 3D plot:
            colors = ['r', 'g', 'b']
            axes = np.eye(3) * size

            for i in range(3):
                vector = R @ axes[:, i]
                ax.quiver(*origin, *vector,
                          color=colors[i], arrow_length_ratio=0.3)

        def update_plot(rvec, tvec):
            # Clear the plot:
            self.ax.clear()

            # Set the limit for the plot again:
            self.ax.set_xlim(-self.limit, self.limit)
            self.ax.set_ylim(-self.limit, self.limit)
            self.ax.set_zlim(0, self.limit)
            self.ax.set_box_aspect([1, 1, 1])

            # Set teh labels and the title again:
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Z")
            self.ax.set_title("Kamera- und Marker-Koordinatensysteme")

            # Define the position of the marker:
            marker_position = np.array([0, 0, 0])
            marker_rotation = np.eye(3)

            # Compute camera position:
            R, _ = cv2.Rodrigues(rvec)
            camera_position = (-R.T @ tvec.reshape(3, 1)).flatten()

            # Calculate the min distance from the marker to the camera:
            min_distance = np.linalg.norm(camera_position)

            # Place the distance tag between both coordinate systems:
            mid_point = (marker_position + camera_position) / 2
            self.ax.text(mid_point[0], mid_point[1], mid_point[2],
                         f"{min_distance:.2f} mm", fontsize=10, color="black",
                         bbox=dict(facecolor='white', alpha=0.5))

            # Update coordinate systems and the 3D plot:
            draw_coordinate_system(self.ax, marker_position, marker_rotation,
                                   size=50)
            draw_coordinate_system(self.ax, camera_position, R.T, size=50)

            self.ax.scatter(*marker_position, color='r', label="ArUco Marker")
            self.ax.scatter(*camera_position, color='b', label="Kamera")

            self.ax.legend()
            self.canvas.draw()

        def update_camera():
            # Collect the picture from the camera:
            success, raw_image = cap.read()

            if success:
                # If there is a picture search for the markers:
                marker_corners, marker_ids, _ = aruco.detectMarkers(raw_image,
                                                                    aruco_dict)

                # Search for the edge-points of the markers:
                if marker_corners is not None and len(marker_corners) != 0:
                    # Draw the detected markers back onto the picture:
                    aruco.drawDetectedMarkers(raw_image, marker_corners,
                                              marker_ids)

                    # Setup the charuco diamond tracking:
                    diamond_corners, diamond_ids = aruco.detectCharucoDiamond(
                        raw_image, marker_corners, marker_ids,
                        square_length_mm / marker_length_mm)

                    control = len(diamond_corners)

                    # Check if there are any markers detected:
                    if diamond_corners is not None and control != 0:
                        aruco.drawDetectedDiamonds(raw_image, diamond_corners,
                                                   diamond_ids)

                        # Calculate the transformation matrices:
                        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                            diamond_corners, square_length_mm, camera_matrix,
                            dist_coeffs)

                        # Draw the coordinate system onto the charuco diamond:
                        cv2.drawFrameAxes(raw_image, camera_matrix,
                                          dist_coeffs, rvecs[0], tvecs[0],
                                          square_length_mm)

                        # Update 3D Plot:
                        update_plot(rvecs[0], tvecs[0])
                    else:
                        # Clear the plot:
                        self.ax.clear()
                        self.fig.canvas.draw_idle()

                        # Set the limit for the plot again:
                        self.ax.set_xlim(-self.limit, self.limit)
                        self.ax.set_ylim(-self.limit, self.limit)
                        self.ax.set_zlim(0, self.limit)
                        self.ax.set_box_aspect([1, 1, 1])

                        # Set the lable for the plot again:
                        self.ax.set_xlabel("X")
                        self.ax.set_ylabel("Y")
                        self.ax.set_zlabel("Z")
                        self.ax.set_title(
                            "Kamera- und Marker-Koordinatensysteme")
                else:
                    # Clear the plot:
                    self.ax.clear()
                    self.fig.canvas.draw_idle()

                    # Set the limit for the plot again:
                    self.ax.set_xlim(-self.limit, self.limit)
                    self.ax.set_ylim(-self.limit, self.limit)
                    self.ax.set_zlim(0, self.limit)
                    self.ax.set_box_aspect([1, 1, 1])

                    # Set the lable for the plot again:
                    self.ax.set_xlabel("X")
                    self.ax.set_ylabel("Y")
                    self.ax.set_zlabel("Z")
                    self.ax.set_title("Kamera- und Marker-Koordinatensysteme")

                # Convert BGR to RGB for Tkinter:
                frame = cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                self.imgtk = ImageTk.PhotoImage(image=img)

                self.camera_label.config(image=self.imgtk)

            # Call this function again after 30ms:
            self.root.after(30, update_camera)

        # Update the camera:
        update_camera()

        # Run the tkinter mainloop:
        self.root.mainloop()

        # Destroy all open windows and release the camera:
        cap.release()
        cv2.destroyAllWindows()

    def on_close(self):
        # Stop the camera:
        self.running = False

        # Release the camera:
        if self.cap is not None:
            self.cap.release()

        # Destroy the tkinter window:
        self.root.destroy()


# Start the program:
if __name__ == "__main__":
    # Setup the tkinter window:
    root = tk.Tk()
    app = TrackArucoMarcerGUI(root)

    # Setup the function to close the class:
    root.protocol("WM_DELETE_WINDOW", app.on_close)

    # Start the main loop:
    root.mainloop()
