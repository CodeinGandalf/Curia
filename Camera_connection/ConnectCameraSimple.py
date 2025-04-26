import cv2


def connect_camera(camera_index=0):
    # Kamera-Verbindung herstellen
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(
            f"Fehler: Kamera mit Index {camera_index} konnte nicht geöffnet werden.")
        return

    print(
        f"Erfolgreich mit Kamera {camera_index} verbunden. Drücke 'q' zum Beenden.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Fehler beim Lesen des Kamerabildes.")
            break

        # Zeige das aktuelle Kamerabild
        cv2.imshow('Kamera', frame)

        # Wenn 'q' gedrückt wird, beende die Schleife
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Kamera freigeben und Fenster schließen
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Kamera Index 1 da eine externe Kamera angesteuert wird:
    connect_camera(camera_index=1)
