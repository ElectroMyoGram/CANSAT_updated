import serial
import time
import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from mpl_toolkits.mplot3d import Axes3D
from geopy.distance import geodesic
 
# Serial Port Configuration
PORT = "COM7"  # Change this based on your setup
BAUD_RATE = 115200
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
 
# Data Buffers (For Graphing)
buffer_size = 100
roll_data = deque(maxlen=buffer_size)
pitch_data = deque(maxlen=buffer_size)
yaw_data = deque(maxlen=buffer_size)
altitude_data = deque(maxlen=buffer_size)
pressure_data = deque(maxlen=buffer_size)
temperature_data = deque(maxlen=buffer_size)
gps_path = deque(maxlen=500)  # Flight path buffer
 
# ------------------------ GUI SETUP ------------------------
root = tk.Tk()
root.title("CanSat Telemetry Dashboard")
 
# Set up Matplotlib Figures
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8))
 
# Graph 1: Yaw, Pitch, Roll
ax1.set_xlim(0, buffer_size)
ax1.set_ylim(-180, 180)
ax1.set_title("Yaw, Pitch, Roll")
ax1.set_ylabel("Angle (°)")
line_roll, = ax1.plot([], [], label="Roll", color="r")
line_pitch, = ax1.plot([], [], label="Pitch", color="g")
line_yaw, = ax1.plot([], [], label="Yaw", color="b")
ax1.legend()
 
# Graph 2: Atmospheric Data
ax2.set_xlim(0, buffer_size)
ax2.set_ylim(0, 120000)  # Adjust altitude range as needed
ax2.set_title("Atmospheric Data")
ax2.set_ylabel("Altitude (m) / Pressure (Pa)")
line_altitude, = ax2.plot([], [], label="Altitude", color="orange")
line_pressure, = ax2.plot([], [], label="Pressure", color="blue")
ax2.legend()
 
# Graph 3: Temperature
ax3.set_xlim(0, buffer_size)
ax3.set_ylim(-50, 50)  # Adjust temperature range
ax3.set_title("Temperature")
ax3.set_ylabel("Temperature (°C)")
line_temperature, = ax3.plot([], [], label="Temperature", color="red")
ax3.legend()
 
# ------------------------ 3D MODEL SETUP ------------------------
fig_3d = plt.figure()
ax_3d = fig_3d.add_subplot(111, projection='3d')
 
# Dummy CanSat Model (Cube)
cube_vertices = np.array([
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1],  [1, -1, 1],  [1, 1, 1],  [-1, 1, 1]
])
cube_faces = [
    [cube_vertices[j] for j in [0, 1, 2, 3]],
    [cube_vertices[j] for j in [4, 5, 6, 7]],
    [cube_vertices[j] for j in [0, 1, 5, 4]],
    [cube_vertices[j] for j in [2, 3, 7, 6]],
    [cube_vertices[j] for j in [0, 3, 7, 4]],
    [cube_vertices[j] for j in [1, 2, 6, 5]]
]
 
def draw_3d_model(roll, pitch, yaw):
    ax_3d.clear()
    ax_3d.set_xlim(-2, 2)
    ax_3d.set_ylim(-2, 2)
    ax_3d.set_zlim(-2, 2)
    # Rotation matrices
    roll_rad, pitch_rad, yaw_rad = np.radians([roll, pitch, yaw])
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])
    R_y = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    R_z = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    # Apply Rotation
    transformed_vertices = np.dot(cube_vertices, R_x @ R_y @ R_z)
    # Draw Cube
    for face in cube_faces:
        face_transformed = np.dot(face, R_x @ R_y @ R_z)
        ax_3d.plot([v[0] for v in face_transformed], 
                   [v[1] for v in face_transformed], 
                   [v[2] for v in face_transformed], color="black")
 
# ------------------------ SERIAL DATA HANDLING ------------------------
def update_ui():
    try:
        line = ser.readline().decode().strip()
        if line:
            try:
                # Expecting: "Yaw, Pitch, Roll, Altitude, Pressure, Temperature, GPS_LAT, GPS_LON"
                values = list(map(float, line.split(",")))
                if len(values) >= 6:
                    yaw, pitch, roll, altitude, pressure, temperature = values[:6]
                    # Store Data
                    yaw_data.append(yaw)
                    pitch_data.append(pitch)
                    roll_data.append(roll)
                    altitude_data.append(altitude)
                    pressure_data.append(pressure)
                    temperature_data.append(temperature)
 
                    # GPS Data (Optional)
                    if len(values) >= 8:
                        gps_path.append((values[6], values[7]))  # (Latitude, Longitude)
 
                    # Update 3D Model
                    draw_3d_model(roll, pitch, yaw)
            except ValueError:
                print(f"Invalid Data: {line}")
 
        # Update Graphs
        line_roll.set_ydata(list(roll_data))
        line_pitch.set_ydata(list(pitch_data))
        line_yaw.set_ydata(list(yaw_data))
        line_altitude.set_ydata(list(altitude_data))
        line_pressure.set_ydata(list(pressure_data))
        line_temperature.set_ydata(list(temperature_data))
 
        plt.draw()
        plt.pause(0.001)
 
    except Exception as e:
        print("Error:", e)
 
    root.after(10, update_ui)
 
# Start UI Updates
root.after(10, update_ui)
root.mainloop()

has context menu

