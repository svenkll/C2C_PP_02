#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Flask-Webframework für HTTP-Server
from flask import Flask, Response, redirect
# PiCamera für Zugriff auf die Raspberry Pi Kamera
from picamera import PiCamera
from picamera.array import PiRGBArray
# OpenCV für Bildverarbeitung
import cv2
import time
import atexit  # Für sauberes Beenden
# ========= Konfiguration =========
RESOLUTION    = (320, 240)   # Auflösung des Kamerabildes
FRAMERATE     = 15           # Bildrate pro Sekunde
HFLIP         = False        # Horizontales Spiegeln
VFLIP         = True         # Vertikales Spiegeln
JPEG_QUALITY  = 80           # JPEG-Komprimierungsqualität
PORT          = 8080         # Port für den Webserver
# =================================

# Flask-App initialisieren
app = Flask(__name__)

# Globale Variablen für Kamera und Rohdaten
camera = None
raw = None


def init_camera():
    """Initialisiert die Pi-Kamera exakt einmal."""
    global camera, raw
    if camera is not None:
        return  # Kamera ist bereits initialisiert
    cam = PiCamera(resolution=RESOLUTION, framerate=FRAMERATE)
    cam.hflip = HFLIP
    cam.vflip = VFLIP
    time.sleep(2)  # Sensor-Warmup-Zeit
    rb = PiRGBArray(cam, size=RESOLUTION)
    camera = cam
    raw = rb

def close_camera():
    """Kamera beim Beenden sauber schließen."""
    global camera, raw
    try:
        if camera:
            camera.close()
    except Exception:
        pass  # Fehler beim Schließen ignorieren
    camera = None
    raw = None

# close_camera wird beim Programmende automatisch aufgerufen
atexit.register(close_camera)

# --- Minimal-HTML (Plain-Response, kein Escaping) ---
INDEX_HTML = """&lt;!doctype html&gt;
&lt;html lang="de"&gt;
&lt;head&gt;&lt;meta charset="utf-8"&gt;&lt;title&gt;CameraCar Live-Stream&lt;/title&gt;&lt;/head&gt;
&lt;body&gt;
  &lt;h1&gt;CameraCar Live-Stream&lt;/h1&gt;
  /video
&lt;/body&gt;
&lt;/html&gt;"""

@app.route("/")
def index():
    # Standardseite – zeigt statisches HTML
    # Alternativ: return redirect("/video") für direkten Stream
    return Response(INDEX_HTML, mimetype="text/html")

@app.route("/video")
def video():
    # Kamera initialisieren
    init_camera()

    def gen():
        # Endlosschleife für kontinuierliche Bildaufnahme
        for f in camera.capture_continuous(raw, format="bgr", use_video_port=True):
            frame_bgr = f.array  # Bilddaten im BGR-Format
            # Bild als JPEG komprimieren
            ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            # Buffer für nächsten Frame zurücksetzen
            raw.truncate(0)
            raw.seek(0)
            if not ok:
                continue  # Fehler beim Kodieren – nächsten Frame versuchen
            # Multipart-HTTP-Stream erzeugen
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")

    # Antwort als MJPEG-Stream zurückgeben
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/health")
def health():
    # Health-Check-Endpunkt für Monitoring
    return Response("ok", mimetype="text/plain")

if __name__ == "__main__":
    # Ausgabe der registrierten Routen beim Start
    print("=== Registrierte Routen ===")
    for rule in app.url_map.iter_rules():
        print(f"{rule}  ->  {','.join(rule.methods)}")
    print("===========================")
    # Webserver starten – ohne Debug-Modus, um doppelte Kamera-Initialisierung zu vermeiden
    app.run(host="0.0.0.0", port=PORT, threaded=True, debug=False, use_reloader=False)

