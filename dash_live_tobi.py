from dash import Dash, html, dcc, Input, Output
import dash_bootstrap_components as dbc
from flask import Flask, Response
from image_processor import Processor
import numpy as np

proc = Processor()

external_stylesheets = [dbc.themes.BOOTSTRAP]
server = Flask(__name__)


@server.route("/test")
def test():
    return "Das ist meine Zweite Seite"


@server.route("/video_stream")
def video_stream():
    return Response(proc.stream_image(), mimetype='multipart/x-mixed-replace; boundary=frame')


app = Dash(__name__, external_stylesheets=external_stylesheets, server=server)

app.layout = html.Div(children=[
    dbc.Row([
        dbc.Col([html.Div("Hallo", id="div-1")]), 
        dbc.Col([html.Div(html.Img(src="/video_stream"))]),
        # dbc.Col([html.Div(html.Img(src="/video_stream_gray"))])
        ]),
        
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="h-slider", min=0, max=180, value=[50, 100])])
    ]),
    dbc.Row([
        dbc.Col([dcc.RangeSlider(id="s-slider", min=0, max=255, value=[0, 255])])
    ])
])


@app.callback(
    Output("div-1", "children"),
    Input("h-slider", "value"),
    Input("s-slider", "value"),
)
def update_div_1(h_input, s_input):
    h_low, h_up = h_input
    s_low, s_up = s_input
    proc.lower = np.array([h_low, s_low, 0])
    proc.upper = np.array([h_up, s_up, 255])
    print(h_input)
    return "Getriggert" + str(h_input)

@app.callback(
    Output("div-1", "children", allow_duplicate=True),
    Input("s-slider", "value"),
    prevent_initial_call=True
)
def update_div_1(h_input):
    print(h_input)
    return "Getriggert" + str(h_input)


if __name__ == "__main__":
    app.run(host="0.0.0.0", debug=True, port=8050, use_reloader=False)
