#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, Response, stream_with_context, request, jsonify
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import numpy as np
import time
import atexit
import threading
from basecar import BaseCar
from basisklassen import FrontWheels, BackWheels

# ========= Server-Konfiguration =========
RESOLUTION = (320, 240)  # Kameraauflösung (w, h)
FRAMERATE = 15           # FPS
PORT = 8080
# =======================================

# ========= Algorithmen-Defaults (editierbar über UI) =========
CONFIG_DEFAULTS = {
    "ROI_BOTTOM_FRACTION": 0.45,  # unterer ROI-Anteil (0..1)
    "BAND_HEIGHT": 30,            # Höhe Suchband unten (px)
    "S_MIN": 40,                  # min. Sättigung
    "V_MIN": 40,                  # min. Helligkeit
    "BLUE_HUE_MIN": 70,           # plausibles Blaufenster (H)
    "BLUE_HUE_MAX": 140,
    "HUE_WIN": 12,                # +/- Hue-Fenster um Peak
    "MIN_BLUE_PX": 200,           # Mindestmenge an Pixeln

    # Steering & Dynamik
    "STEERING_MAX_DEG": 90.0,     # Skala Lenkwinkel (°)
    "STEERING_GAIN": 1.4,         # zusätzlicher Verstärker (1.0..2.0)
    "STEERING_GAMMA": 0.7,        # <1 = kräftiger um die Mitte
    "STEERING_MIN_ABS_DEG": 3.0,  # Mindest-Ausschlag (°) gegen Deadzone
    "SMOOTH_ALPHA": 0.3,          # 0 = kein Glätten, 0.8 = stark

    # Stream
    "JPEG_QUALITY": 80,           # Stream-Qualität (10..95)
    "DRAW_OVERLAY": True,         # Overlay (Linien/Text) anzeigen

    # Spiegelung (Software-Flip im Stream; Hardware-Flip nur beim Init)
    "HFLIP": False,
    "VFLIP": True,

    # Kalibrierung / Diagnose
    "SERVO_CENTER_DEG": 90.0,     # Neutralstellung Servo (Grad)
    "SERVO_LIMIT_LEFT": 60.0,     # linker Limit (kleinster sicherer Winkel)
    "SERVO_LIMIT_RIGHT": 120.0,   # rechter Limit (größter sicherer Winkel)
    "STEER_INVERT": False,        # Richtung invertieren (True = invertiert)
    "CAM_X_OFFSET_PX": 0          # Kamera-X-Versatz (px, + = nach rechts)
}
# =============================================================

app = Flask(__name__)

# --- Hardware / Car ---
fw = FrontWheels()
bw = BackWheels()
car = BaseCar(fw, bw)
atexit.register(car.stop)

car_state = {
    "running": False,
    "speed": 40,
    "log": []
}

# --- Threads / Locks ---
drive_thread = None
drive_thread_stop = threading.Event()
cam_lock = threading.Lock()     # schützt Initialisierung / Kamera-Zugriff
config_lock = threading.Lock()  # schützt CONFIG
video_lock = threading.Lock()   # lässt nur einen Video-Stream gleichzeitig zu

# Laufzeitkonfiguration
CONFIG = dict(CONFIG_DEFAULTS)

# --- Kamera ---
camera = None
raw = None

def init_camera():
    """Initialisiert die PiCamera genau einmal (thread-sicher)."""
    global camera, raw
    if camera is not None:
        return
    with cam_lock:
        if camera is not None:
            return
        cam = PiCamera(resolution=RESOLUTION, framerate=FRAMERATE)
        # Hardware-Flip nur EINMAL beim Init anwenden (kein Umschalten im Stream!)
        with config_lock:
            cam.hflip = bool(CONFIG.get("HFLIP", False))
            cam.vflip = bool(CONFIG.get("VFLIP", True))
        time.sleep(2)  # Warm-up
        rb = PiRGBArray(cam, size=RESOLUTION)
        camera = cam
        raw = rb

def close_camera():
    """Sauber schließen beim Beenden."""
    global camera, raw
    try:
        if camera:
            camera.close()
    except Exception:
        pass
    camera = None
    raw = None

atexit.register(close_camera)

# --- Drive-Thread (nur Stop-Überwachung; Lenkung macht der Video-Loop) ---

def drive_loop():
    print("[drive_thread] gestartet")
    while not drive_thread_stop.is_set():
        if not car_state.get("running", False):
            car.stop()
        time.sleep(0.1)
    print("[drive_thread] gestoppt")

def start_drive_thread():
    global drive_thread
    if drive_thread and drive_thread.is_alive():
        return
    drive_thread_stop.clear()
    drive_thread = threading.Thread(target=drive_loop, daemon=True)
    drive_thread.start()

def stop_drive_thread():
    global drive_thread
    drive_thread_stop.set()
    if drive_thread:
        drive_thread.join(timeout=2)
    car.stop()

atexit.register(stop_drive_thread)

# --- HTML Index-Seite mit Stream + UI ---
INDEX_HTML = """<!doctype html>
<html lang="de">
<head>
  <meta charset="utf-8">
  <title>CameraCar Live-Stream + Einstellungen</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root { color-scheme: dark; }
    body { font-family: system-ui, Arial, sans-serif; background:#111; color:#eee; margin:0; }
    header { padding:12px 16px; background:#181818; border-bottom:1px solid #222; position:sticky; top:0; z-index:10; }
    main { display:grid; grid-template-columns: 1fr 420px; gap:16px; padding:16px; }
    @media (max-width: 900px) { main { grid-template-columns: 1fr; } }
    .panel { background:#151515; border:1px solid #222; border-radius:8px; padding:12px; }
    h1 { font-size:1.15rem; margin:0; }
    h2 { margin:0 0 8px 0; font-size:1.05rem; }
    img { width:100%; height:auto; background:#000; display:block; border-radius:8px; }
    .meta { opacity:0.8; margin-top:6px; font-size:0.9rem; }
    .row { display:flex; align-items:center; gap:8px; margin:8px 0; }
    .row label { flex: 0 0 200px; font-size:0.95rem; }
    .row input[type="range"] { flex:1; }
    .row input[type="number"] { width:110px; }
    .row .val { width:72px; text-align:right; opacity:0.9; font-variant-numeric: tabular-nums; }
    .grid2 { display:grid; grid-template-columns: 1fr 1fr; gap:8px; }
    .btns { display:flex; gap:8px; margin-top:12px; }
    button { background:#2b2b2b; color:#eee; border:1px solid #444; border-radius:6px; padding:8px 10px; cursor:pointer; }
    button:hover { background:#333; }
    .sec { margin-top:14px; padding-top:10px; border-top:1px dashed #333; }
  </style>
</head>
<body>
  <header>
    <h1>CameraCar Live-Stream</h1>
  </header>
  <main>
    <div class="panel">
      <img src="/  <div class="meta">Health: <aealth/health</a></div>
      <div class="meta" id="statusText"></div>
    </div>

    <div class="panel">
      <h2>Einstellungen</h2>

      <div class="row">
        <label for="ROI_BOTTOM_FRACTION">ROI-Unterteil (%)</label>
        <input type="range" id="ROI_BOTTOM_FRACTION" min="0.1" max="0.9" step="0.01">
        <div class="val" id="ROI_BOTTOM_FRACTION_val"></div>
      </div>

      <div class="row">
        <label for="BAND_HEIGHT">Bandhöhe (px)</label>
        <input type="range" id="BAND_HEIGHT" min="10" max="120" step="1">
        <div class="val" id="BAND_HEIGHT_val"></div>
      </div>

      <div class="grid2">
        <div class="row">
          <label for="S_MIN">S_min</label>
          <input type="range" id="S_MIN" min="0" max="255" step="1">
          <div class="val" id="S_MIN_val"></div>
        </div>
        <div class="row">
          <label for="V_MIN">V_min</label>
          <input type="range" id="V_MIN" min="0" max="255" step="1">
          <div class="val" id="V_MIN_val"></div>
        </div>
      </div>

      <div class="grid2">
        <div class="row">
          <label for="BLUE_HUE_MIN">Hue min</label>
          <input type="range" id="BLUE_HUE_MIN" min="0" max="179" step="1">
          <div class="val" id="BLUE_HUE_MIN_val"></div>
        </div>
        <div class="row">
          <label for="BLUE_HUE_MAX">Hue max</label>
          <input type="range" id="BLUE_HUE_MAX" min="0" max="179" step="1">
          <div class="val" id="BLUE_HUE_MAX_val"></div>
        </div>
      </div>

      <div class="row">
        <label for="HUE_WIN">±Hue um Peak</label>
        <input type="range" id="HUE_WIN" min="0" max="40" step="1">
        <div class="val" id="HUE_WIN_val"></div>
      </div>

      <div class="row">
        <label for="MIN_BLUE_PX">Min. Blau-Pixel</label>
        <input type="range" id="MIN_BLUE_PX" min="0" max="8000" step="50">
        <div class="val" id="MIN_BLUE_PX_val"></div>
      </div>

      <div class="row">
        <label for="STEERING_MAX_DEG">Steering-Skala (°)</label>
        <input type="range" id="STEERING_MAX_DEG" min="10" max="150" step="1">
        <div class="val" id="STEERING_MAX_DEG_val"></div>
      </div>

      <div class="row">
        <label for="STEERING_GAIN">Steuer-Verstärkung (×)</label>
        <input type="range" id="STEERING_GAIN" min="0.5" max="3.0" step="0.1">
        <div class="val" id="STEERING_GAIN_val"></div>
      </div>

      <div class="row">
        <label for="STEERING_GAMMA">Kennlinie γ</label>
        <input type="range" id="STEERING_GAMMA" min="0.3" max="2.5" step="0.1">
        <div class="val" id="STEERING_GAMMA_val"></div>
      </div>

      <div class="row">
        <label for="STEERING_MIN_ABS_DEG">Mindest-Ausschlag (°)</label>
        <input type="range" id="STEERING_MIN_ABS_DEG" min="0" max="15" step="0.5">
        <div class="val" id="STEERING_MIN_ABS_DEG_val"></div>
      </div>

      <div class="row">
        <label for="SMOOTH_ALPHA">Glättung α</label>
        <input type="range" id="SMOOTH_ALPHA" min="0.0" max="0.95" step="0.01">
        <div class="val" id="SMOOTH_ALPHA_val"></div>
      </div>

      <div class="row">
        <label for="JPEG_QUALITY">JPEG-Qualität</label>
        <input type="range" id="JPEG_QUALITY" min="10" max="95" step="1">
        <div class="val" id="JPEG_QUALITY_val"></div>
      </div>

      <div class="switch">
        <input type="checkbox" id="DRAW_OVERLAY">
        <label for="DRAW_OVERLAY">Overlay (Linien/Text) anzeigen</label>
      </div>
      <div class="switch">
        <input type="checkbox" id="HFLIP">
        <label for="HFLIP">Horizontal spiegeln (H-Flip)</label>
      </div>
      <div class="switch">
        <input type="checkbox" id="VFLIP">
        <label for="VFLIP">Vertikal spiegeln (V-Flip)</label>
      </div>

      <div class="sec">
        <h2>Kalibrierung</h2>
        <div class="row">
          <label for="SERVO_CENTER_DEG">Servo-Neutral (°)</label>
          <input type="range" id="SERVO_CENTER_DEG" min="60" max="120" step="1">
          <div class="val" id="SERVO_CENTER_DEG_val"></div>
        </div>
        <div class="grid2">
          <div class="row">
            <label for="SERVO_LIMIT_LEFT">Servo-Min (°)</label>
            <input type="range" id="SERVO_LIMIT_LEFT" min="40" max="90" step="1">
            <div class="val" id="SERVO_LIMIT_LEFT_val"></div>
          </div>
          <div class="row">
            <label for="SERVO_LIMIT_RIGHT">Servo-Max (°)</label>
            <input type="range" id="SERVO_LIMIT_RIGHT" min="90" max="140" step="1">
            <div class="val" id="SERVO_LIMIT_RIGHT_val"></div>
          </div>
        </div>
        <div class="switch">
          <input type="checkbox" id="STEER_INVERT">
          <label for="STEER_INVERT">Steuerrichtung invertieren</label>
        </div>
        <div class="row">
          <label for="CAM_X_OFFSET_PX">Kamera X-Offset (px)</label>
          <input type="range" id="CAM_X_OFFSET_PX" min="-100" max="100" step="1">
          <div class="val" id="CAM_X_OFFSET_PX_val"></div>
        </div>
      </div>

      <div class="btns">
        <button id="btnStart">Start</button>
        <button id="btnStop">Stop</button>
        <button id="btnReset">Defaults</button>
        <button id="btnReload">Neu laden</button>
      </div>
    </div>
  </main>

  <script>
    const fields = [
      "ROI_BOTTOM_FRACTION","BAND_HEIGHT","S_MIN","V_MIN",
      "BLUE_HUE_MIN","BLUE_HUE_MAX","HUE_WIN","MIN_BLUE_PX",
      "STEERING_MAX_DEG","STEERING_GAIN","STEERING_GAMMA","STEERING_MIN_ABS_DEG",
      "SMOOTH_ALPHA","JPEG_QUALITY","DRAW_OVERLAY",
      "HFLIP","VFLIP","SERVO_CENTER_DEG","SERVO_LIMIT_LEFT","SERVO_LIMIT_RIGHT",
      "STEER_INVERT","CAM_X_OFFSET_PX"
    ];

    function fmt(id, val) {
      if (id === "ROI_BOTTOM_FRACTION") return Math.round(val*100) + "%";
      if (id === "SMOOTH_ALPHA") return Number(val).toFixed(2);
      if (id === "DRAW_OVERLAY" || id === "HFLIP" || id === "VFLIP" || id === "STEER_INVERT") return val ? "ein" : "aus";
      if (id === "STEERING_GAIN" || id === "STEERING_GAMMA") return Number(val).toFixed(2);
      if (id === "STEERING_MIN_ABS_DEG") return Number(val).toFixed(1);
      return String(val);
    }

    function reflect(id, val) {
      const el = document.getElementById(id);
      const vl = document.getElementById(id + "_val");
      if (!el) return;
      if (el.type === "checkbox") {
        el.checked = !!val;
        if (vl) vl.textContent = fmt(id, !!val);
      } else {
        el.value = val;
        if (vl) {
          const v = (id === "ROI_BOTTOM_FRACTION" || id === "SMOOTH_ALPHA") ? Number(val) : Number(el.value);
          vl.textContent = fmt(id, v);
        }
      }
    }

    function debounce(fn, ms=150) { let t; return (...args)=>{ clearTimeout(t); t=setTimeout(()=>fn(...args), ms); } }

    async function loadConfig() {
      const r = await fetch("/api/config");
      const cfg = await r.json();
      for (const k of fields) reflect(k, cfg[k]);
    }

    const sendPatch = debounce(async (patch) => {
      try {
        const r = await fetch("/api/config", {
          method: "PATCH",
          headers: {"Content-Type":"application/json"},
          body: JSON.stringify(patch)
        });
        const cfg = await r.json();
        for (const k of Object.keys(patch)) reflect(k, cfg[k]);
      } catch (e) { console.error("PATCH failed", e); }
    });

    function bindLive(id) {
      const el = document.getElementById(id);
      if (!el) return;
      const vl = document.getElementById(id + "_val");
      const handler = () => {
        let val;
        if (el.type === "checkbox") {
          val = el.checked;
          if (vl) vl.textContent = fmt(id, val);
          sendPatch({[id]: val});
        } else {
          val = el.value;
          if (id === "ROI_BOTTOM_FRACTION" || id === "SMOOTH_ALPHA") val = Number(val); else val = parseFloat(val);
          if (vl) vl.textContent = fmt(id, val);
          sendPatch({[id]: val});
        }
      };
      el.addEventListener("input", handler);
      el.addEventListener("change", handler);
    }

    document.getElementById("btnReset").addEventListener("click", async ()=>{
      const r = await fetch("/api/config/reset", { method:"POST" });
      const cfg = await r.json();
      for (const k of fields) reflect(k, cfg[k]);
    });
    document.getElementById("btnReload").addEventListener("click", ()=> location.reload());
    for (const k of fields) bindLive(k);
    loadConfig();

    document.getElementById("btnStart").addEventListener("click", async ()=>{ await fetch("/start", { method: "POST" }); updateStatus(); });
    document.getElementById("btnStop").addEventListener("click",  async ()=>{ await fetch("/stop",  { method: "POST" }); updateStatus(); });

    async function updateStatus() {
      try { const r = await fetch("/api/status"); const j = await r.json();
        document.getElementById("statusText").textContent = j.running ? "Fährt" : "Gestoppt";
      } catch (e) { document.getElementById("statusText").textContent = "Status unbekannt"; }
    }
    setInterval(updateStatus, 2000); updateStatus();
  </script>
</body>
</html>
"""

# --- Utils ---
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def validate_and_update_config(patch):
    """Validiert eingehende Teil-Updates und aktualisiert CONFIG thread-sicher."""
    normalized = {}
    with config_lock:
        cur = dict(CONFIG)
        # ROI Anteil
        if "ROI_BOTTOM_FRACTION" in patch:
            v = float(patch["ROI_BOTTOM_FRACTION"])
            normalized["ROI_BOTTOM_FRACTION"] = clamp(v, 0.05, 0.95)
        # Bandhöhe
        if "BAND_HEIGHT" in patch:
            v = int(float(patch["BAND_HEIGHT"]))
            normalized["BAND_HEIGHT"] = clamp(v, 4, 200)
        # S/V
        if "S_MIN" in patch:
            v = int(float(patch["S_MIN"]))
            normalized["S_MIN"] = clamp(v, 0, 255)
        if "V_MIN" in patch:
            v = int(float(patch["V_MIN"]))
            normalized["V_MIN"] = clamp(v, 0, 255)
        # Hue Fenster
        if "BLUE_HUE_MIN" in patch:
            v = int(float(patch["BLUE_HUE_MIN"]))
            normalized["BLUE_HUE_MIN"] = clamp(v, 0, 179)
        if "BLUE_HUE_MAX" in patch:
            v = int(float(patch["BLUE_HUE_MAX"]))
            normalized["BLUE_HUE_MAX"] = clamp(v, 0, 179)
        # Konsistenz min<=max erzwingen
        hmin = normalized.get("BLUE_HUE_MIN", cur["BLUE_HUE_MIN"])
        hmax = normalized.get("BLUE_HUE_MAX", cur["BLUE_HUE_MAX"])
        if hmin > hmax:
            hmin, hmax = hmax, hmin
            normalized["BLUE_HUE_MIN"], normalized["BLUE_HUE_MAX"] = hmin, hmax
        # HUE_WIN
        if "HUE_WIN" in patch:
            v = int(float(patch["HUE_WIN"]))
            normalized["HUE_WIN"] = clamp(v, 0, 40)
        # Mindestpixel
        if "MIN_BLUE_PX" in patch:
            v = int(float(patch["MIN_BLUE_PX"]))
            normalized["MIN_BLUE_PX"] = clamp(v, 0, 20000)
        # Steering-Scale
        if "STEERING_MAX_DEG" in patch:
            v = float(patch["STEERING_MAX_DEG"])
            normalized["STEERING_MAX_DEG"] = clamp(v, 1.0, 180.0)
        # Verstärker / Kennlinie
        if "STEERING_GAIN" in patch:
            v = float(patch["STEERING_GAIN"])
            normalized["STEERING_GAIN"] = clamp(v, 0.5, 3.0)
        if "STEERING_GAMMA" in patch:
            v = float(patch["STEERING_GAMMA"])
            normalized["STEERING_GAMMA"] = clamp(v, 0.3, 2.5)
        if "STEERING_MIN_ABS_DEG" in patch:
            v = float(patch["STEERING_MIN_ABS_DEG"])
            normalized["STEERING_MIN_ABS_DEG"] = clamp(v, 0.0, 15.0)
        # Glättung
        if "SMOOTH_ALPHA" in patch:
            v = float(patch["SMOOTH_ALPHA"])
            normalized["SMOOTH_ALPHA"] = clamp(v, 0.0, 0.98)
        # JPEG-Qualität
        if "JPEG_QUALITY" in patch:
            v = int(float(patch["JPEG_QUALITY"]))
            normalized["JPEG_QUALITY"] = clamp(v, 10, 95)
        # Overlay an/aus
        if "DRAW_OVERLAY" in patch:
            v = bool(patch["DRAW_OVERLAY"])
            normalized["DRAW_OVERLAY"] = v
        # H/V-Flip (wir setzen NUR im Stream als Software-Flip)
        if "HFLIP" in patch:
            normalized["HFLIP"] = bool(patch["HFLIP"]) 
        if "VFLIP" in patch:
            normalized["VFLIP"] = bool(patch["VFLIP"]) 
        # Servo-Kalibrierung / Richtung / Limits
        if "SERVO_CENTER_DEG" in patch:
            v = float(patch["SERVO_CENTER_DEG"]) 
            normalized["SERVO_CENTER_DEG"] = clamp(v, 45.0, 135.0)
        if "SERVO_LIMIT_LEFT" in patch:
            v = float(patch["SERVO_LIMIT_LEFT"]) 
            normalized["SERVO_LIMIT_LEFT"] = clamp(v, 0.0, 180.0)
        if "SERVO_LIMIT_RIGHT" in patch:
            v = float(patch["SERVO_LIMIT_RIGHT"]) 
            normalized["SERVO_LIMIT_RIGHT"] = clamp(v, 0.0, 180.0)
        if "STEER_INVERT" in patch:
            normalized["STEER_INVERT"] = bool(patch["STEER_INVERT"]) 
        if "CAM_X_OFFSET_PX" in patch:
            v = int(float(patch["CAM_X_OFFSET_PX"]))
            normalized["CAM_X_OFFSET_PX"] = clamp(v, -200, 200)
        # Übernehmen
        CONFIG.update(normalized)
        return dict(CONFIG)

# --- Routes ---
@app.route("/")
def index():
    return Response(INDEX_HTML, mimetype="text/html")

@app.route("/api/config", methods=["GET", "PATCH"])
def api_config():
    if request.method == "GET":
        with config_lock:
            return jsonify(CONFIG)
    else:
        try:
            # JSON oder Form beides akzeptieren
            if request.is_json:
                patch = request.get_json(force=True, silent=False) or {}
            else:
                patch = request.form.to_dict() or {}
        except Exception:
            return jsonify({"error": "invalid JSON"}), 400
        new_cfg = validate_and_update_config(patch)
        return jsonify(new_cfg)

@app.route("/api/config/reset", methods=["POST"])
def api_config_reset():
    with config_lock:
        CONFIG.clear()
        CONFIG.update(CONFIG_DEFAULTS)
    # ⚠️ KEIN Hardware-Flip im laufenden Stream setzen
    return jsonify(dict(CONFIG))

@app.route("/start", methods=["POST"])
def start():
    if request.is_json:
        try:
            speed = int(request.get_json().get("speed", 40))
        except Exception:
            speed = 40
    else:
        speed = int(request.form.get("speed", 40))
    car_state["speed"] = speed
    car_state["running"] = True
    start_drive_thread()
    return jsonify({"status": "started", "running": True, "speed": speed})

@app.route("/stop", methods=["POST"])
def stop():
    car.stop()
    car_state["running"] = False
    return jsonify({"status": "stopped", "running": False})

@app.route("/api/status")
def api_status():
    return jsonify({"running": car_state.get("running", False),
                    "speed": car_state.get("speed", 0)})

# --- Diagnose: Servo manuell bewegen ---
@app.route("/steer")
def steer_manual():
    try:
        deg = int(float(request.args.get("deg", 90)))
    except Exception:
        deg = 90
    car.drive(new_speed=car_state["speed"], new_angle=deg)
    return jsonify({"deg": deg})

# --- Diagnose: Snapshot speichern (Frame + Maske) ---
@app.route("/debug/snapshot")
def debug_snapshot():
    init_camera()
    if camera is None:
        return jsonify({"error": "camera not available"}), 503
    with config_lock:
        cfg = dict(CONFIG)
    # Einen Frame holen (exklusiv)
    frame_bgr = None
    with video_lock:
        for f in camera.capture_continuous(raw, format="bgr", use_video_port=True):
            frame_bgr = f.array
            raw.truncate(0)
            break
    if frame_bgr is None:
        return jsonify({"error": "no frame"}), 500
    h, w = frame_bgr.shape[:2]
    # Software-Flip
    if cfg["HFLIP"] and cfg["VFLIP"]:
        frame_bgr = cv2.flip(frame_bgr, -1)
    elif cfg["HFLIP"]:
        frame_bgr = cv2.flip(frame_bgr, 1)
    elif cfg["VFLIP"]:
        frame_bgr = cv2.flip(frame_bgr, 0)
    # ROI
    y0 = max(0, min(h - 1, int(h * (1.0 - float(cfg["ROI_BOTTOM_FRACTION"])))))
    roi = frame_bgr[y0:, :]
    # HSV & Masken
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask_sv = cv2.inRange(hsv, (0, int(cfg["S_MIN"]), int(cfg["V_MIN"])), (179, 255, 255))
    hist = cv2.calcHist([hsv], [0], mask_sv, [180], [0, 180]).ravel()
    if int(cfg["BLUE_HUE_MIN"]) > 0:
        hist[:int(cfg["BLUE_HUE_MIN"])] = 0
    if int(cfg["BLUE_HUE_MAX"]) < 179:
        hist[int(cfg["BLUE_HUE_MAX"]) + 1:] = 0
    peak = int(np.argmax(hist))
    peak_val = float(hist[peak])
    if peak_val > 0:
        low_h = max(peak - int(cfg["HUE_WIN"]), int(cfg["BLUE_HUE_MIN"]))
        high_h = min(peak + int(cfg["HUE_WIN"]), int(cfg["BLUE_HUE_MAX"]))
    else:
        low_h, high_h = 85, 135
    mask_blue = cv2.inRange(hsv, (low_h, int(cfg["S_MIN"]), int(cfg["V_MIN"])), (high_h, 255, 255))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    # Speichern
    frame_path = "/home/pi/Desktop/snap_frame.jpg"
    mask_path = "/home/pi/Desktop/snap_mask.jpg"
    cv2.imwrite(frame_path, frame_bgr)
    cv2.imwrite(mask_path, mask)
    return jsonify({"saved": True, "frame": frame_path, "mask": mask_path,
                    "low_h": int(low_h), "high_h": int(high_h), "peak": int(peak), "peak_val": int(peak_val)})

# --- Video-Stream ---
@app.route("/video")
def video():
    init_camera()
    if camera is None:
        return Response("camera not available", status=503, mimetype="text/plain")

    def gen():
        prev_angle = 0.0  # für Glättung
        try:
            # Exklusivitäts-Lock: nur ein Stream gleichzeitig
            with video_lock:
                for f in camera.capture_continuous(raw, format="bgr", use_video_port=True):
                    frame_bgr = f.array
                    h, w = frame_bgr.shape[:2]

                    # --- Config snapshot ---
                    with config_lock:
                        cfg = dict(CONFIG)
                    ROI_BOTTOM_FRACTION = float(cfg["ROI_BOTTOM_FRACTION"])
                    BAND_HEIGHT        = int(cfg["BAND_HEIGHT"])
                    S_MIN              = int(cfg["S_MIN"])
                    V_MIN              = int(cfg["V_MIN"])
                    BLUE_HUE_MIN       = int(cfg["BLUE_HUE_MIN"])
                    BLUE_HUE_MAX       = int(cfg["BLUE_HUE_MAX"])
                    HUE_WIN            = int(cfg["HUE_WIN"])
                    MIN_BLUE_PX        = int(cfg["MIN_BLUE_PX"])
                    JPEG_QUALITY       = int(cfg["JPEG_QUALITY"])
                    DRAW_OVERLAY       = bool(cfg["DRAW_OVERLAY"])
                    HFLIP              = bool(cfg["HFLIP"])
                    VFLIP              = bool(cfg["VFLIP"])
                    SERVO_CENTER_DEG   = float(cfg["SERVO_CENTER_DEG"])
                    SERVO_LIMIT_LEFT   = float(cfg["SERVO_LIMIT_LEFT"])
                    SERVO_LIMIT_RIGHT  = float(cfg["SERVO_LIMIT_RIGHT"])
                    STEER_INVERT       = bool(cfg["STEER_INVERT"])
                    CAM_X_OFFSET_PX    = int(cfg["CAM_X_OFFSET_PX"])
                    STEERING_MAX_DEG   = float(cfg["STEERING_MAX_DEG"])
                    STEERING_GAIN      = float(cfg["STEERING_GAIN"])
                    STEERING_GAMMA     = float(cfg["STEERING_GAMMA"])
                    STEERING_MIN_ABS   = float(cfg["STEERING_MIN_ABS_DEG"])
                    SMOOTH_ALPHA       = float(cfg["SMOOTH_ALPHA"])

                    # --- Software-Flip statt Hardware zur Laufzeit ---
                    if HFLIP and VFLIP:
                        frame_bgr = cv2.flip(frame_bgr, -1)  # beide Achsen
                    elif HFLIP:
                        frame_bgr = cv2.flip(frame_bgr, 1)   # horizontal
                    elif VFLIP:
                        frame_bgr = cv2.flip(frame_bgr, 0)   # vertikal

                    # --- ROI: unterer Bildbereich ---
                    y0 = int(h * (1.0 - ROI_BOTTOM_FRACTION))
                    y0 = max(0, min(h - 1, y0))  # Safety clamp
                    roi = frame_bgr[y0:, :]  # (H_roi, W)

                    # --- HSV & Masken ---
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    # 1) Vorfilter
                    mask_sv = cv2.inRange(hsv, (0, S_MIN, V_MIN), (179, 255, 255))
                    # 2) Hue-Histogramm
                    hist = cv2.calcHist([hsv], [0], mask_sv, [180], [0, 180]).ravel()
                    if BLUE_HUE_MIN > 0:
                        hist[:BLUE_HUE_MIN] = 0
                    if BLUE_HUE_MAX < 179:
                        hist[BLUE_HUE_MAX + 1:] = 0
                    peak = int(np.argmax(hist))
                    peak_val = float(hist[peak])
                    # 3) Adaptives Fenster
                    if peak_val > 0:
                        low_h = max(peak - HUE_WIN, BLUE_HUE_MIN)
                        high_h = min(peak + HUE_WIN, BLUE_HUE_MAX)
                    else:
                        low_h, high_h = 85, 135
                    # 4) finale Blau-Maske
                    mask_blue = cv2.inRange(hsv, (low_h, S_MIN, V_MIN), (high_h, 255, 255))
                    kernel = np.ones((3, 3), np.uint8)
                    mask = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, iterations=1)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
                    blue_px = int(cv2.countNonZero(mask))
                    # Fallback, wenn zu wenig Blau gefunden wurde
                    if blue_px < MIN_BLUE_PX:
                        low_h, high_h = 85, 135
                        mask = cv2.inRange(hsv, (low_h, 30, 40), (high_h, 255, 255))
                        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
                        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
                        blue_px = int(cv2.countNonZero(mask))

                    # --- Spurzentrum bestimmen ---
                    band_h = min(BAND_HEIGHT, mask.shape[0])
                    band = mask[-band_h:, :]
                    col_sum = band.sum(axis=0, dtype=np.int32)
                    lane_center_x_roi = roi.shape[1] // 2
                    if int(col_sum.max()) > 0:
                        lane_center_x_roi = int(np.argmax(col_sum))
                    else:
                        M = cv2.moments(mask)
                        if M["m00"] > 0:
                            lane_center_x_roi = int(M["m10"] / M["m00"])

                    lane_center_x = lane_center_x_roi
                    vehicle_center_x = w // 2 + CAM_X_OFFSET_PX

                    # --- Lenkung berechnen ---
                    # normiere Abweichung auf -1..+1 (Bildhälfte)
                    dev_norm = (lane_center_x - vehicle_center_x) / (w / 2.0)
                    dev_norm = max(-1.0, min(1.0, float(dev_norm)))

                    sign = 1.0 if dev_norm >= 0 else -1.0
                    mag = abs(dev_norm) ** STEERING_GAMMA
                    angle_cmd = sign * (mag * STEERING_MAX_DEG * STEERING_GAIN)

                    invert = -1.0 if STEER_INVERT else 1.0
                    steering_angle = invert * (SMOOTH_ALPHA * prev_angle + (1.0 - SMOOTH_ALPHA) * angle_cmd)

                    # Mindest-Ausschlag gegen Deadzone (wenn relevante Abweichung vorhanden)
                    if abs(steering_angle) < STEERING_MIN_ABS and abs(dev_norm) > 0.02:
                        steering_angle = STEERING_MIN_ABS * (1.0 if steering_angle >= 0 else -1.0)

                    prev_angle = steering_angle

                    center = SERVO_CENTER_DEG
                    servo_cmd = int(round(center + steering_angle))
                    # Servo-Limits
                    servo_cmd = max(int(SERVO_LIMIT_LEFT), min(int(SERVO_LIMIT_RIGHT), servo_cmd))

                    if car_state.get("running", False):
                        car.drive(new_speed=car_state["speed"], new_angle=servo_cmd)
                        car_state["log"].append({
                            "time": time.strftime("%H:%M:%S"),
                            "speed": car_state["speed"],
                            "angle": float(steering_angle),
                            "dev": float(dev_norm),
                            "px": int(blue_px),
                            "peak": int(peak),
                            "cmd": int(servo_cmd)
                        })
                        if len(car_state["log"]) > 100:
                            car_state["log"] = car_state["log"][-100:]

                    # --- Overlay / Visualisierung ---
                    if DRAW_OVERLAY:
                        # Linien
                        cv2.line(frame_bgr, (vehicle_center_x, h-1), (vehicle_center_x, h-60), (255, 0, 0), 2)
                        cv2.line(frame_bgr, (lane_center_x,   h-1), (lane_center_x,   h-60), (0, 0, 255), 2)
                        # ROI-Rahmen
                        cv2.rectangle(frame_bgr, (0, y0), (w-1, h-1), (80, 80, 80), 1)
                        # Texte
                        cv2.putText(frame_bgr, f"Steering: {steering_angle:+.1f}° cmd:{servo_cmd}", (10, 24),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)
                        cv2.putText(frame_bgr, f"dev:{dev_norm:+.2f} H:[{low_h}-{high_h}] peak:{peak}({int(peak_val)}) px:{blue_px}", (10, 44),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200,200,50), 2)
                        # Mini-Plot Spaltensummen
                        try:
                            plot_h = 40
                            plot = np.zeros((plot_h, w, 3), dtype=np.uint8)
                            cs = col_sum.astype(np.float32)
                            m = cs.max() if cs.max()>0 else 1.0
                            csn = (cs / m) * (plot_h-1)
                            for x in range(w):
                                y = int(plot_h - 1 - csn[x])
                                plot[y:, x] = (60, 180, 60)
                            frame_bgr[h-plot_h:h, 0:w] = cv2.addWeighted(frame_bgr[h-plot_h:h, 0:w], 0.4, plot, 0.6, 0)
                        except Exception:
                            pass

                    # ---- JPEG + Stream ----
                    try:
                        ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                        if not ok:
                            raw.truncate(0)
                            continue
                        yield (b"--frame\r\n" + b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")
                    except Exception as e:
                        print(f"[video] JPEG encode/yield Fehler: {e}")
                    finally:
                        # Wichtig: Puffer für nächsten capture zurücksetzen
                        raw.truncate(0)
        except (GeneratorExit, BrokenPipeError):
            return
        except Exception as e:
            print(f"[video] Fehler: {e}")
        finally:
            pass
    resp = Response(stream_with_context(gen()), mimetype="multipart/x-mixed-replace; boundary=frame")
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    return resp

@app.route("/health")
def health():
    return Response("ok", mimetype="text/plain")

if __name__ == "__main__":
    print("=== Registrierte Routen ===")
    for rule in app.url_map.iter_rules():
        print(f"{rule} -> {','.join(rule.methods)}")
    print("===========================")
    # Für Tests im LAN ok; für Dauerbetrieb besser debug=False
    app.run(host="0.0.0.0", port=PORT, threaded=True, debug=True, use_reloader=False)
