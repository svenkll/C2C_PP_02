from dash import Dash, html, dcc, Input, Output
import dash_bootstrap_components as dbc
from flask import Flask, Response
from cameracar import CameraCar
import numpy as np
from basisklassen import FrontWheels, BackWheels
from basisklassen_cam import Camera


front = FrontWheels()
back = BackWheels()
cam = Camera(devicenumber = 0,
            buffersize = 10,
            skip_frame = 0,
            height = 480,
            width = 640,
            flip = True,
            )

proc = CameraCar(front,back,cam, [])

external_stylesheets = [dbc.themes.BOOTSTRAP]
server = Flask(__name__)


@server.route("/test")
def test():
    return "Das ist meine Zweite Seite"


@server.route("/video_stream")
def video_stream():
    return Response(proc.video_streams(), mimetype='multipart/x-mixed-replace; boundary=frame')


@server.route("/video_stream_Canny")
def video_stream_Canny():
    return Response(proc.video_streams_Canny(), mimetype='multipart/x-mixed-replace; boundary=frame')

@server.route("/video_stream_line")
def video_streams_lines():
    return Response(proc.video_streams_lines(), mimetype='multipart/x-mixed-replace; boundary=frame')


app = Dash(__name__, external_stylesheets=external_stylesheets, server=server)

app.layout = html.Div(children=[
    dbc.Row([
        dbc.Col([html.Div("Hallo", id="div-1")]), 
        dbc.Col([html.Div(html.Img(src="/video_stream"))]),
        dbc.Col([html.Div(html.Img(src="/video_stream_Canny"))]),
        dbc.Col([html.Div(html.Img(src="/video_stream_line"), style={"width": "800px", "height": "600px"})])
        ]),
        
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="h-slider", min=0, max=180, value=[90, 130])])
    ]),
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="s-slider", min=0, max=255, value=[60, 255])])
    ]),
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="v-slider", min=0, max=255, value=[100, 180])])
    ]),
])


@app.callback(
    Output("div-1", "children"),
    Input("h-slider", "value"),
    Input("s-slider", "value"),
    Input("v-slider", "value"),
)
def update_div_1(h_input, s_input, v_input):
    h_low, h_up = h_input
    s_low, s_up = s_input
    v_low, v_up = v_input
    proc.lower_blue_input = np.array([h_low, s_low, v_low])
    proc.upper_blue_input = np.array([h_up, s_up, v_up])
    # print(h_input)
    return f"H Input {h_input} | S Input {s_input} | V Input {v_input}"

@app.callback(
    Output("div-1", "children", allow_duplicate=True),
    Input("s-slider", "value"),
    prevent_initial_call=True
)
def update_div_1(h_input):
    print(h_input)
    return f"H Input {h_input} | S Input {s_input} | V Input {v_input}"


if __name__ == "__main__":
    app.run(host="0.0.0.0", debug=True, port=8050, use_reloader=False)
