import serial
import matplotlib.pyplot as plt
from collections import deque
import time

PORT = 'COM9' 
BAUD_RATE = 115200

ser = serial.Serial(PORT, BAUD_RATE)

buffer_size = 100
roll_data = deque(maxlen=buffer_size)
pitch_data = deque(maxlen=buffer_size)
# yaw_data = deque(maxlen=buffer_size)

plt.ion()
fig, ax = plt.subplots()
line_roll, = ax.plot([], [], label="Roll (°)", color="r")
line_pitch, = ax.plot([], [], label="Pitch (°)", color="g")
line_yaw, = ax.plot([], [], label="Yaw (°)", color="b")

ax.set_xlim(0, buffer_size)
ax.set_ylim(-180, 180)  
ax.set_xlabel("Time (samples)")
ax.set_ylabel("Angle (°)")
ax.legend()

print(f"Listening on port {PORT}...")

try:
    last_update_time = time.time()
    while True:
        ser.reset_input_buffer() 
        line = ser.readline().decode().strip()  


        if line:
            try:
                values = list(map(float, line.split(',')))
                if len(values) == 2:
                    roll_data.append(values[0])
                    pitch_data.append(values[1])
                    # yaw_data.append(values[2])

            except ValueError:
                print(f'Invalid data received: {line}')

        if time.time() - last_update_time >= 0.01:
            last_update_time = time.time()

            line_roll.set_ydata(list(roll_data))
            line_pitch.set_ydata(list(pitch_data))
            # line_yaw.set_ydata(list(yaw_data))

            line_roll.set_xdata(range(len(roll_data)))
            line_pitch.set_xdata(range(len(pitch_data)))
            # line_yaw.set_xdata(range(len(yaw_data)))

            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    ser.close()
    plt.ioff()
    plt.show()
