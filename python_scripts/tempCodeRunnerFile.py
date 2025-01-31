import serial
import matplotlib.pyplot as plt
from collections import deque

PORT = 'COM7'
BAUD_RATE = 115200

ser = serial.Serial(PORT, BAUD_RATE)

buffer_size = 100
x_data = deque(maxlen=buffer_size)
y_data = deque(maxlen=buffer_size)
z_data = deque(maxlen=buffer_size)

plt.ion()
fig, ax = plt.subplots()
line_x, = ax.plot([], [], label="AccelX", color="r")
line_y, = ax.plot([], [], label="AccelY", color="g")
line_z, = ax.plot([], [], label="AccelZ", color="b")
ax.set_xlim(0, buffer_size)
ax.set_ylim(-10, 10)
ax.legend()

print("listening on port ", PORT)

try:
    while True:
        line = ser.readline().decode().strip()

        try:
            values = list(map(float, line.split(',')))
            if len(values) == 3:
                x_data.append(values[0])
                y_data.append(values[1])
                z_data.append(values[2])

                line_x.set_ydata(list(x_data))
                line_y.set_ydata(list(y_data))
                line_z.set_ydata(list(z_data))
                line_x.set_xdata(range(len(x_data)))
                line_y.set_xdata(range(len(y_data)))
                line_z.set_xdata(range(len(z_data)))

                ax.relim()
                ax.autoscale_view()
                plt.pause(0.01)
        except ValueError:
            print(f'invalid data: {line}')
except KeyboardInterrupt:
    print("stopping...")
finally:
    ser.close()
    plt.ioff()
    plt.show()

