import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
 
# Create Tkinter Window
root = tk.Tk()
root.title("Interactive 3D CanSat Flight Trajectory")
 
# Generate mock 3D trajectory data
num_samples = 500
time_values = np.linspace(0, 600, num_samples)
 
# Simulating altitude with some variations
altitude = 500 + 5000 * np.sin(2 * np.pi * 0.0015 * time_values) + np.random.normal(0, 50, num_samples)
altitude = np.clip(altitude, 500, 30000)  # Limiting realistic altitude range
 
# Simulating a circular GPS trajectory
gps_lat = 51.5074 + 0.01 * np.sin(2 * np.pi * 0.0005 * time_values) + np.random.normal(0, 0.0001, num_samples)
gps_lon = -0.1278 + 0.01 * np.cos(2 * np.pi * 0.0005 * time_values) + np.random.normal(0, 0.0001, num_samples)
 
# Create Matplotlib figure
fig = plt.figure(figsize=(7, 5))
ax = fig.add_subplot(111, projection='3d')
 
# Plot the 3D trajectory
ax.plot(gps_lon, gps_lat, altitude, label="Mock CanSat Flight Path", color="blue")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.set_zlabel("Altitude (m)")
ax.set_title("Simulated 3D CanSat Flight Path")
ax.legend()
 
# Embed Matplotlib figure into Tkinter window
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
canvas.draw()
 
# Add control buttons
control_frame = ttk.Frame(root)
control_frame.pack(fill=tk.X, padx=10, pady=5)
 
exit_button = ttk.Button(control_frame, text="Exit", command=root.quit)
exit_button.pack(side=tk.RIGHT, padx=5)
 
# Run the Tkinter event loop
root.mainloop()