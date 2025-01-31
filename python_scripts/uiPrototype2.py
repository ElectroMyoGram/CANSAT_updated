import dash
from dash import dcc, html
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from plotly.subplots import make_subplots
 
import numpy as np
 
# -----------------------------
# Mock data for demonstration
# -----------------------------
 
num_samples = 200
time_values = np.linspace(0, 20, num_samples)
 
# 1) Orientation: Yaw, Pitch, Roll
yaw = 40 * np.sin(0.5 * time_values) + np.random.normal(0, 2, num_samples)
pitch = 25 * np.sin(0.8 * time_values) + np.random.normal(0, 2, num_samples)
roll = 30 * np.sin(0.3 * time_values) + np.random.normal(0, 2, num_samples)
 
# 2) Atmospheric data: Altitude, Pressure, Temperature
altitude = 500 + 100 * np.sin(0.4 * time_values) + np.random.normal(0, 5, num_samples)
pressure = 101325 - (altitude * 12) + np.random.normal(0, 50, num_samples)
temperature = 15 - (altitude / 1000) * 6.5 + np.random.normal(0, 1, num_samples)
 
# 3) Flight Path: Simulate 3D trajectory (lat, lon, alt)
#   We'll store lat/lon in a small range around some reference, altitude reuses above data
gps_lat = 51.5074 + 0.01 * np.sin(0.05 * time_values) + np.random.normal(0, 0.0001, num_samples)
gps_lon = -0.1278 + 0.01 * np.cos(0.05 * time_values) + np.random.normal(0, 0.0001, num_samples)
 
# -------------------------------------------------
# Dash App Initialization
# -------------------------------------------------
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
 
# -----------------------------
# Layout: TABS + CONTENT
# -----------------------------
app.layout = html.Div([
    dcc.Tabs(id='tabs', value='tab_1', children=[
        dcc.Tab(label='Orientation', value='tab_1'),
        dcc.Tab(label='Atmospheric Data', value='tab_2'),
        dcc.Tab(label='Flight Path (3D)', value='tab_3')
    ]),
    html.Div(id='tab_content')
], style={'margin': '2%'})
 
 
# -----------------------------
# Callback to Render Tabs
# -----------------------------
@app.callback(
    Output('tab_content', 'children'),
    Input('tabs', 'value')
)
def render_tabs(tab):
    if tab == 'tab_1':
        return get_tab_orientation()
    elif tab == 'tab_2':
        return get_tab_atmosphere()
    elif tab == 'tab_3':
        return get_tab_flightpath()
    return html.Div("No tab selected.")
 
 
# -----------------------------
# Tab 1: Orientation (Yaw, Pitch, Roll)
# -----------------------------
def get_tab_orientation():
    # We'll display a single figure with 3 subplots (one for each angle),
    # or a single subplot with 3 lines. Let's do 1 subplot with 3 lines for simplicity.
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=time_values, y=yaw, mode='lines', name='Yaw'))
    fig.add_trace(go.Scatter(x=time_values, y=pitch, mode='lines', name='Pitch'))
    fig.add_trace(go.Scatter(x=time_values, y=roll, mode='lines', name='Roll'))
 
    fig.update_layout(
        title="Orientation vs. Time",
        xaxis_title="Time (s)",
        yaxis_title="Angle (degrees)",
        hovermode="x unified"
    )
 
    return html.Div([
        html.H3("Yaw, Pitch, Roll"),
        dcc.Graph(figure=fig)
    ])
 
 
# -----------------------------
# Tab 2: Atmospheric Data
# -----------------------------
def get_tab_atmosphere():
    # We'll display 3 subplots for altitude, pressure, and temperature
    fig = make_subplots(rows=3, cols=1,
                        subplot_titles=("Altitude vs Time", "Pressure vs Time", "Temperature vs Time"))
 
    # Altitude
    fig.add_trace(
        go.Scatter(x=time_values, y=altitude, name="Altitude", line=dict(color='green')),
        row=1, col=1
    )
 
    # Pressure
    fig.add_trace(
        go.Scatter(x=time_values, y=pressure, name="Pressure", line=dict(color='blue')),
        row=2, col=1
    )
 
    # Temperature
    fig.add_trace(
        go.Scatter(x=time_values, y=temperature, name="Temperature", line=dict(color='red')),
        row=3, col=1
    )
 
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
 
    fig.update_yaxes(title_text="Altitude (m)", row=1, col=1)
    fig.update_yaxes(title_text="Pressure (Pa)", row=2, col=1)
    fig.update_yaxes(title_text="Temperature (°C)", row=3, col=1)
 
    fig.update_layout(
        height=900,
        showlegend=False,
        title_text="Atmospheric Data",
        hovermode="x unified"
    )
 
    return html.Div([
        html.H3("Atmospheric Data"),
        dcc.Graph(figure=fig)
    ])
 
 
# -----------------------------
# Tab 3: 3D Flight Path
# -----------------------------
def get_tab_flightpath():
    fig = go.Figure(data=[go.Scatter3d(
        x=gps_lon,
        y=gps_lat,
        z=altitude,
        mode='lines+markers',
        line=dict(color='blue', width=2),
        marker=dict(size=3, color='blue'),
        name='Flight Path'
    )])
 
    fig.update_layout(
        title="3D Flight Path",
        scene=dict(
            xaxis_title='Longitude',
            yaxis_title='Latitude',
            zaxis_title='Altitude (m)'
        ),
        width=800,
        height=600
    )
 
    return html.Div([
        html.H3("3D Flight Path"),
        dcc.Graph(figure=fig)
    ])
 
 
# -----------------------------
# Run the App
# -----------------------------
if __name__ == '__main__':
    app.run_server(debug=True)