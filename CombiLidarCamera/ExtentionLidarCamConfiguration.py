import numpy as np
import cv2

# === Parameter einstellen ===

# Kamera-Parameter
focal_length_px = 600   # Brennweite in Pixel (muss man kalibrieren!)
sensor_width_px = 640   # Sensorbreite in Pixeln
sensor_height_px = 480  # Sensorhöhe in Pixeln

# LIDAR-Offset relativ zum Weltkoordinatenursprung (z.B. Roboterzentrum)
lidar_offset_x = 0.2  # in Metern
lidar_offset_y = 0.0  # in Metern

# Kamera-Offset relativ zum Weltkoordinatenursprung
camera_offset_x = 0.2  # in Metern
camera_offset_y = 0.0  # in Metern

# === Funktionen ===


def lidar_to_world(r, theta_deg, lidar_offset_x, lidar_offset_y):
    """
    Rechnet LIDAR-Messung (r, theta) in Weltkoordinaten (X, Y) um.
    Achtung: Annahme, dass 0° nach vorne zeigt!
    """
    theta_rad = np.deg2rad(theta_deg)
    x_local = r * np.sin(theta_rad)
    y_local = r * np.cos(theta_rad)

    x_world = x_local + lidar_offset_x
    y_world = y_local + lidar_offset_y

    return x_world, y_world


def world_to_camera(x_world, y_world, camera_offset_x, camera_offset_y):
    """
    Rechnet Weltkoordinaten in Kamerakoordinaten um.
    """
    x_cam = x_world + camera_offset_x
    y_cam = y_world + camera_offset_y

    return x_cam, y_cam


def project_to_pixel(x_cam, y_cam, focal_length_px, sensor_width_px, sensor_height_px):
    """
    Projektion von 3D (x, y) in 2D Pixelkoordinaten.
    Annahme: Kamera schaut in +Y Richtung, X = rechts, Y = vorne.
    """
    if y_cam <= 0:
        # Objekt hinter der Kamera => ignorieren
        return None

    # Perspektivische Projektion
    pixel_x = (x_cam / y_cam) * focal_length_px + (sensor_width_px / 2)
    pixel_y = sensor_height_px / 2

    return int(pixel_x), int(pixel_y)

# === Beispielmessungen ===


# Beispiel-LIDAR Messdaten: Abstand in Metern und Winkel in Grad
lidar_measurements = [
    (1.5, 0),    # direkt vor dem Sensor
    (1.7, 10),   # leicht links
    (1.6, -15),  # leicht rechts
    (2.0, 30),   # weiter links
]

# Ergebnisse sammeln
pixel_coords = []

for r, theta in lidar_measurements:
    x_world, y_world = lidar_to_world(r, theta, lidar_offset_x, lidar_offset_y)
    x_cam, y_cam = world_to_camera(
        x_world, y_world, camera_offset_x, camera_offset_y)
    pixel = project_to_pixel(
        x_cam, y_cam, focal_length_px, sensor_width_px, sensor_height_px)

    if pixel is not None:
        print(f"LIDAR (r={r:.2f}m, θ={theta:.1f}°) → Pixel {pixel}")
        pixel_coords.append(pixel)

# === Ergebnis visualisieren ===

# Bild simulieren
image = np.zeros((sensor_height_px, sensor_width_px, 3), dtype=np.uint8)

# Punkte einzeichnen
for px, py in pixel_coords:
    if 0 <= px < sensor_width_px and 0 <= py < sensor_height_px:
        cv2.circle(image, (px, py), 5, (0, 255, 0), -1)

# Anzeigen
cv2.imshow('Projektion der LIDAR-Punkte', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
