from dash import Dash, html, dcc, Input, Output, State, callback_context
import dash_bootstrap_components as dbc
from flask import Flask, Response
import json
import numpy as np
from cameracar_jk import CameraCar
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

external_stylesheets = [dbc.themes.BOOTSTRAP]
server = Flask(__name__)
app = Dash(__name__, external_stylesheets=external_stylesheets, server=server)

# --------------------------
# Config Loader (robust)
# --------------------------
def load_config():
    try:
        with open("C2C_PP_02/config.json.json", "r") as f:
            data = json.load(f)
            # Stelle sicher, dass Werte vorhanden sind
            lower = data.get("lower_blue_input", [90,60,60])
            upper = data.get("upper_blue_input", [130,255,180])
            data["lower_blue_input"] = lower
            data["upper_blue_input"] = upper
            print(f"daten lower {lower} und daten upper {upper}")
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
app.layout = html.Div([
    dbc.Row([
        dbc.Col([html.Div("Hallo", id="div-1")]),
        dbc.Col([html.Div(html.Img(src="/video_stream", width="400", height="300"))]),
        dbc.Col([html.Div(html.Img(src="/video_stream_Canny", width="400", height="300"))]),
        dbc.Col([html.Div(html.Img(src="/video_stream_line", width="400", height="300"))])
    ]),

    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="h-slider", min=0, max=180, value=[config["lower_blue_input"][0], config["upper_blue_input"][0]])])
    ]),
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="s-slider", min=0, max=255, value=[config["lower_blue_input"][1], config["upper_blue_input"][1]])])
    ]),
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="v-slider", min=0, max=255, value=[config["lower_blue_input"][2], config["upper_blue_input"][2]])])
    ]),

    html.Hr(),

    dbc.Row([
        dbc.Col([dbc.Button("Fahrmodus manual starten", id="btn-mode1", color="primary")]),
        dbc.Col([dbc.Button("Fahrmodus CNN starten", id="btn-mode2", color="secondary")]),
        dbc.Col([dbc.Button("Daten speichern", id="btn-save", color="success")]),
        dbc.Col([dbc.Switch(id="switch-armed", label="Motor Armed", value=False)]),
        dbc.Col([dbc.Switch(id="switch-Calc", label="Calc", value=False)]),
    ]),

    html.Div(id="status-text", style={"marginTop": "20px", "fontWeight": "bold"})
])

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


@app.callback(
    Output("status-text", "children", allow_duplicate=True),
    Input("switch-armed", "value"),
    prevent_initial_call=True
)


@app.callback(
    Output("status-text", "children", allow_duplicate=True),
    Input("switch-Calc", "value"),
    prevent_initial_call=True
)


def toggle_motor(armed):
    if armed:
        proc.is_driving = True
        return "Motor Status: ✅ Armed"
    else:
        proc.is_driving = False
        proc.stop()
        return "Motor Status: ⛔ Unarmed"

def CNN_active(toggle):
    if toggle:
        proc.CNN_active = True
        return "CNN Calc: ✅ Active"
    else:
        proc.CNN_active = False
        return "CNN Calc: ⛔ Inactive"

# --------------------------
# Button Callback (Dash 2.9 kompatibel)
# --------------------------
@app.callback(
    Output("status-text", "children"),
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
        return ""

    button_id = ctx.triggered[0]['prop_id'].split('.')[0]

    if button_id == "btn-mode1":
        if not proc.is_driving:
            return "⚠️ Motor ist Unarmed!"
        proc.start_mode(1)  # Muss in CameraCar implementiert sein
        return "Fahrmodus manual gestartet!"

    elif button_id == "btn-mode2":
        if not proc.is_driving:
            return "⚠️ Motor ist Unarmed!"
        proc.start_mode(2)
        return "Fahrmodus CNN gestartet!"

    elif button_id == "btn-save":
        h_low, h_up = h_val
        s_low, s_up = s_val
        v_low, v_up = v_val
    
    elif button_id == "btn-trigger":
        data = load_config()
        data["lower_blue_input"] = [h_low, s_low, v_low]
        data["upper_blue_input"] = [h_up, s_up, v_up]

        with open("config.json", "w") as f:
            json.dump(data, f, indent=4)

        return f"Daten gespeichert: lower={data['lower_blue_input']}, upper={data['upper_blue_input']}"

    return ""

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
