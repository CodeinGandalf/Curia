from pynput import keyboard
import time

# Steuerungs-Counter
dx = 0  # Seitliche Bewegung (rechts/links)
dy = 0  # Vor-/Rückwärtsbewegung
drot = 0
manual = False


# Tastendruck-Verarbeitung
def on_press(key):
    global dx, dy, drot, manual
    try:
        if key.char == 'w':
            dy += 1
            print("W: dy =", dy)
        elif key.char == 's':
            dy -= 1
            print("S: dy =", dy)
        elif key.char == 'd':
            dx += 1
            print("D: dx =", dx)
        elif key.char == 'a':
            dx -= 1
            print("A: dx =", dx)
        elif key.char == 'e':
            drot += 1
            print("E: drot =", drot)
        elif key.char == 'q':
            drot -= 1
            print("Q: drot =", drot)
        elif key.char == 'p':
            dx = 0
            dy = 0
            drot = 0
            print("P: Stop")
        elif key.char == 'n':
            manual = True
    except AttributeError:
        # Escape beendet den Listener
        if key == keyboard.Key.esc:
            print("Beende Programm...")
            return False


# Listener starten
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Hauptloop läuft weiter
try:
    while True:
        # Hier könnten z.B. Motorbefehle gesendet werden basierend auf dx/dy
        print(f"Aktueller Status - dx: {dx}, dy: {dy}, drot: {drot}")
        time.sleep(1)
except KeyboardInterrupt:
    print("Manuell beendet.")
