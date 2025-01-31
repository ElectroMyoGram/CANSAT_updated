import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
 
# 🔹 Number of samples (simulate 10 seconds of data)
num_samples = 500
time_values = np.linspace(0, 10, num_samples)  # Time in seconds
 
# 🔹 Generate Roll, Pitch, Yaw using sine waves + noise
roll = 30 * np.sin(2 * np.pi * 0.1 * time_values) + np.random.normal(0, 3, num_samples)
pitch = 25 * np.sin(2 * np.pi * 0.15 * time_values) + np.random.normal(0, 2, num_samples)
yaw = 40 * np.sin(2 * np.pi * 0.08 * time_values) + np.random.normal(0, 4, num_samples)
 
# 🔹 Atmospheric Data Simulation
altitude = 500 + 100 * np.sin(2 * np.pi * 0.05 * time_values) + np.random.normal(0, 5, num_samples)
pressure = 101325 - (altitude * 12) + np.random.normal(0, 20, num_samples)  # Simulated atmospheric pressure
temperature = 15 - (altitude / 1000) * 6.5 + np.random.normal(0, 0.5, num_samples)  # Lapse rate approximation
 
# 🔹 GPS Flight Path Simulation (Circular Path)
gps_lat = 51.5074 + 0.001 * np.sin(2 * np.pi * 0.05 * time_values) + np.random.normal(0, 0.0001, num_samples)
gps_lon = -0.1278 + 0.001 * np.cos(2 * np.pi * 0.05 * time_values) + np.random.normal(0, 0.0001, num_samples)
 
# 🔹 Store in a DataFrame
mock_data = pd.DataFrame({
    "Time (s)": time_values,
    "Roll (°)": roll,
    "Pitch (°)": pitch,
    "Yaw (°)": yaw,
    "Altitude (m)": altitude,
    "Pressure (Pa)": pressure,
    "Temperature (°C)": temperature,
    "GPS Latitude": gps_lat,
    "GPS Longitude": gps_lon
})
 
# 🔹 Save to CSV (Optional for UI Testing)
mock_data.to_csv("mock_cansat_data.csv", index=False)
 
# 🔹 Display DataFrame
 
# 🔹 Plot the Data for Visualization
plt.figure(figsize=(12, 6))
 
# 🟢 Plot Roll, Pitch, Yaw
plt.subplot(3, 1, 1)
plt.plot(time_values, roll, label="Roll", color="r")
plt.plot(time_values, pitch, label="Pitch", color="g")
plt.plot(time_values, yaw, label="Yaw", color="b")
plt.legend()
plt.title("Yaw, Pitch, Roll")
 
# 🟠 Plot Altitude & PressureSc
plt.subplot(3, 1, 2)
plt.plot(time_values, altitude, label="Altitude", color="orange")
plt.plot(time_values, pressure, label="Pressure", color="blue")
plt.legend()
plt.title("Atmospheric Data")
 
# 🔴 Plot Temperature
plt.subplot(3, 1, 3)
plt.plot(time_values, temperature, label="Temperature", color="red")
plt.legend()
plt.title("Temperature")
 
plt.tight_layout()
plt.show()
