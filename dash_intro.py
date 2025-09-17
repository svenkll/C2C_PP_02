from dash import Dash, html, dcc, Input, Output
import dash_bootstrap_components as dbc
from flask import Flask, Response
from basisklassen_cam import Camera
import cv2

external_stylesheets = [dbc.themes.BOOTSTRAP]
server = Flask(__name__)

app = Dash(__name__, external_stylesheets=external_stylesheets, server=server)

cam = Camera()



def generate_stream(cam):
    while True:
        frame = cam.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(gray, 100, 200)
        _, frame_as_jpeg = cv2.imencode(".jpeg", canny)
        frame_in_bytes = frame_as_jpeg.tobytes()

        frame_as_string = (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_in_bytes + b'\r\n\r\n')
        
        yield frame_as_string

@server.route("/video_stream")
def video_stream():
    return Response(generate_stream(cam), mimetype='multipart/x-mixed-replace; boundary=frame')

app.layout = html.Div(children=[
    dbc.Row([
        dbc.Col([
            html.H1("Hallo Projektphase 2"),
            html.H3("Das nutze ich als Beispiel gleich", id="beispiel-output"), 
        ]),
        dbc.Col([
            html.Div(html.Img(src="/video_stream", style={"height": "200px"}))
        ])
    ]),
    
    dbc.Row([
        
        
    dbc.Col([dcc.Slider(id="slider-1", min=0, max=200, value=20),
    dcc.Slider(id="slider-2", min=0, max=200, value=20),
    dcc.Slider(id="slider-3", min=0, max=200, value=20),]), dbc.Col([dcc.Slider(id="slider-4", min=0, max=200, value=20),
    dcc.Slider(id="slider-5", min=0, max=200, value=20),])])
    
])


@app.callback(
    Output("beispiel-output", "children"),
    Input("slider-1", "value"), 
    Input("slider-2", "value"), 
    Input("slider-3", "value"), 
    Input("slider-4", "value"), 
    Input("slider-5", "value"), 
    #prevent_initial_call=True
)
def update_header(v5, v2, v3, v4, v1):
    output = v5*20
    return output


# @app.callback(
#     Output("beispiel-output", "children", allow_duplicate=True),
#     Input("slider-5", "value"), 
#     prevent_initial_call=True
# )
# def zweites_beispiel(slider_value, v2, v3, v4, v5):
#     output = slider_value*20
#     return output

if __name__ == "__main__":
    app.run_server(host="0.0.0.0", debug=False, port=8050)
