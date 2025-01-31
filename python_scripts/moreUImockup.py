# Generate mock 3D trajectory data for CanSat flight path
 
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
 
# Number of samples
num_samples = 500
 
# Time array (simulating a 10-minute flight)
time_values = np.linspace(0, 600, num_samples)
 
# Generate altitude (smooth increase with noise, peaking at 30km)
altitude = 500 + 5000 * np.sin(2 * np.pi * 0.0015 * time_values) + np.random.normal(0, 50, num_samples)
altitude = np.clip(altitude, 500, 30000)  # Limit to realistic values
 
# Generate GPS trajectory (simulating a circular flight path)
gps_lat = 51.5074 + 0.01 * np.sin(2 * np.pi * 0.0005 * time_values) + np.random.normal(0, 0.0001, num_samples)
gps_lon = -0.1278 + 0.01 * np.cos(2 * np.pi * 0.0005 * time_values) + np.random.normal(0, 0.0001, num_samples)
 
# Store in a DataFrame
mock_trajectory = pd.DataFrame({
    "Time (s)": time_values,
    "Altitude (m)": altitude,
    "GPS Latitude": gps_lat,
    "GPS Longitude": gps_lon
})
 
# Display DataFrame

# 3D Plot of the trajectory
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
 
ax.plot(gps_lon, gps_lat, altitude, label="Mock CanSat Flight Path", color="blue")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.set_zlabel("Altitude (m)")
ax.set_title("Simulated 3D CanSat Flight Path")
ax.legend()
 
plt.show()