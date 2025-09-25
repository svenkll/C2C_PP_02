from dash import Dash, html, dcc, Input, Output, State, callback_context, no_update
import dash_bootstrap_components as dbc
from flask import Flask, Response
import json
import numpy as np
from cameracar import CameraCar
from basisklassen import FrontWheels, BackWheels
from basisklassen_cam import Camera

# --------------------------
# Kamera und Car Setup
# --------------------------
front = FrontWheels()
back = BackWheels()
cam = Camera(devicenumber=0,
             buffersize=10,
             skip_frame=0,
             height=480,
             width=640,
             flip=True)

proc = CameraCar(front, back, cam, [])
proc.motor_armed = False
proc.CNN_active = False

external_stylesheets = [dbc.themes.SUPERHERO]
server = Flask(__name__)
app = Dash(__name__, external_stylesheets=external_stylesheets, server=server)

# --------------------------
# Config Loader
# --------------------------
def load_config():
    try:
        with open("C2C_PP_02/config.json.json", "r") as f:
            data = json.load(f)
            lower = data.get("lower_blue_input", [90,60,60])
            upper = data.get("upper_blue_input", [130,255,180])
            data["lower_blue_input"] = lower
            data["upper_blue_input"] = upper
    except Exception:
        data = {
            "lower_blue_input": [90, 60, 60],
            "upper_blue_input": [130, 255, 180]
        }
    return data

config = load_config()

# --------------------------
# Layout
# --------------------------
app.layout = dbc.Container([

    dbc.Row([
        # dbc.Col([html.Div("Werte", id="div-1", style={"width": "100%", "maxWidth": "640px", "height": "auto", "display": "block", "margin": "0 auto"})]),
        dbc.Col([html.Div(html.Img(src="/video_stream", style={"width": "100%", "maxWidth": "640px", "height": "auto", "display": "block", "margin": "0 auto"}))]),
        dbc.Col([html.Div(html.Img(src="/video_stream_Canny", style={"width": "100%", "maxWidth": "640px", "height": "auto", "display": "block", "margin": "0 auto"}))]),
        dbc.Col([html.Div(html.Img(src="/video_stream_line", style={"width": "100%", "maxWidth": "640px", "height": "auto", "display": "block", "margin": "0 auto","marginTop": "100px"}))])
    ]),
    
    dbc.Row([
        dbc.Col(html.Div("Hallo", id="div-1"), width=12, className="text-center")
    ]),

    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="h-slider", min=0, max=180,
                                 value=[config["lower_blue_input"][0], config["upper_blue_input"][0]])])
    ]),
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="s-slider", min=0, max=255,
                                 value=[config["lower_blue_input"][1], config["upper_blue_input"][1]])])
    ]),
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="v-slider", min=0, max=255,
                                 value=[config["lower_blue_input"][2], config["upper_blue_input"][2]])])
    ]),

    html.Hr(),

    dbc.Row([
        dbc.Col([dbc.Button("Fahrt starten", id="btn-mode1", color="primary")]),
        dbc.Col([dbc.Button("Fahrt stoppen", id="btn-mode2", color="danger")]),
        dbc.Col([dbc.Button("Daten speichern", id="btn-save", color="success")]),
        dbc.Col([dbc.Switch(id="switch-armed", label="Motor Armed", value=False)]),
        dbc.Col([dbc.Switch(id="switch-Calc", label="CNN Calc", value=False)])
    ]),

    html.Hr(),

    # --------------------------
    # Status Cards
    # --------------------------
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    html.H5("Motor Status", className="card-title"),
                    html.Div(id="status-motor", className="card-text")
                ])
            ], id="card-motor", color="danger", inverse=True)
        ], width=3),

        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    html.H5("CNN Calc Status", className="card-title"),
                    html.Div(id="status-cnn", className="card-text")
                ])
            ], id="card-cnn", color="danger", inverse=True)
        ], width=3)
    ], justify="center", className="mt-3"),
    
        # ---------------- Bild ----------------
    html.Hr(),
        
    dbc.Row([
            html.H3("Pi Car Gruppe 2", 
            style={
                    "textAlign": "center",  # zentrierter Text
                    "color": "#00FFFF",    # Textfarbe (Cyan)
                    "fontFamily": "Arial", # Schriftart
                    "fontSize": "36px",    # Schriftgröße
                    "fontWeight": "bold",  # Fettdruck
                    "textShadow": "2px 2px 4px #000000", # Schatteneffekt
                    "display": "block",     # damit margin greift
                    "marginLeft": "auto",   # automatische Ränder
                    "marginRight": "auto",
                    "marginTop": "20px"
    }),
            
        # Bild aus assets/ laden
        html.Img(
    src="/assets/logo.png",
    style={
        "width": "400px",       # Bildbreite (z.B. doppelt so groß wie vorher)
        "display": "block",     # damit margin greift
        "marginLeft": "auto",   # automatische Ränder
        "marginRight": "auto",
        "marginTop": "20px"
    }
)]),    
    
], fluid=True)

# --------------------------
# Slider Callback
# --------------------------
@app.callback(
    Output("div-1", "children"),
    Input("h-slider", "value"),
    Input("s-slider", "value"),
    Input("v-slider", "value")
)
def update_div_1(h_input, s_input, v_input):
    h_low, h_up = h_input
    s_low, s_up = s_input
    v_low, v_up = v_input
    proc.lower_blue_input = np.array([h_low, s_low, v_low])
    proc.upper_blue_input = np.array([h_up, s_up, v_up])
    return f"H: {h_input} | S: {s_input} | V: {v_input}"

# --------------------------
# Motor Armed / CNN Calc Switches
# --------------------------
@app.callback(
    [Output("status-motor", "children"),
     Output("card-motor", "color")],
    Input("switch-armed", "value"),
    prevent_initial_call=True
)
def toggle_motor(armed):
    proc.is_driving = armed
    if not armed:
        proc.stop()
    return ("✅ Armed" if armed else "⛔ Unarmed",
            "success" if armed else "danger")

@app.callback(
    [Output("status-cnn", "children"),
     Output("card-cnn", "color")],
    Input("switch-Calc", "value"),
    prevent_initial_call=True
)
def toggle_cnn(active):
    proc.CNN_active = active
    return ("✅ Active" if active else "⛔ Inactive",
            "success" if active else "danger")

# --------------------------
# Buttons
# --------------------------
@app.callback(
    Output("status-motor", "children", allow_duplicate=True),
    Input("btn-mode1", "n_clicks"),
    Input("btn-mode2", "n_clicks"),
    Input("btn-save", "n_clicks"),
    State("h-slider", "value"),
    State("s-slider", "value"),
    State("v-slider", "value"),
    prevent_initial_call=True
)
def handle_buttons(mode1_clicks, mode2_clicks, save_clicks, h_val, s_val, v_val):
    ctx = callback_context
    if not ctx.triggered:
        return no_update

    button_id = ctx.triggered[0]['prop_id'].split('.')[0]

    if button_id == "btn-mode1":
        if not proc.is_driving:
            return "⚠️ Motor ist Unarmed!"
        proc.start_mode(1)
        return "✅ Running"

    elif button_id == "btn-mode2":
        proc.stop()
        proc.is_driving = False
        return "⛔ Stopped"

    elif button_id == "btn-save":
        h_low, h_up = h_val
        s_low, s_up = s_val
        v_low, v_up = v_val
        proc.lower_blue_input = np.array([h_low, s_low, v_low])
        proc.upper_blue_input = np.array([h_up, s_up, v_up])

        data = {
            "lower_blue_input": proc.lower_blue_input.tolist(),
            "upper_blue_input": proc.upper_blue_input.tolist()
        }
        with open("config.json", "w") as f:
            json.dump(data, f, indent=4)
        return "⚙️ Config gespeichert"

    return no_update

# --------------------------
# Video Streams
# --------------------------
@server.route("/video_stream")
def video_stream():
    return Response(proc.video_streams(), mimetype='multipart/x-mixed-replace; boundary=frame')

@server.route("/video_stream_Canny")
def video_stream_Canny():
    return Response(proc.video_streams_Canny(), mimetype='multipart/x-mixed-replace; boundary=frame')

@server.route("/video_stream_line")
def video_streams_lines():
    return Response(proc.video_streams_lines(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --------------------------
# Main
# --------------------------
if __name__ == "__main__":
    app.run(host="0.0.0.0", debug=True, port=8050, use_reloader=False)
