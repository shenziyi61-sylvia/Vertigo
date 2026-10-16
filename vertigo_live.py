import re
import time
import threading
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

PORT = "/dev/cu.usbmodem101"
BAUD = 115200
WINDOW_SECONDS = 4

times = deque()
gy_values = deque()
lock = threading.Lock()
running = True
start_time = time.monotonic()
angle_values = deque()
angle = 0.0
prev_t = None
prev_gy = None

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def read_sensor():
    global running, angle, prev_t, prev_gy

    while running:
        try:
            raw = ser.readline().decode(
                "utf-8", errors="ignore"
            ).strip()

            match = re.search(
                r"gy:\s*(-?\d+(?:\.\d+)?)", raw
            )

            if match:
                gy = float(match.group(1))
                t = time.monotonic() - start_time

                with lock:
                    if prev_t is not None:
                        dt = t - prev_t

                        # Integrate angular velocity (degrees/second)
                        angle += (prev_gy + gy) / 2 * dt

                    prev_t = t
                    prev_gy = gy

                    times.append(t)
                    gy_values.append(gy)
                    angle_values.append(angle)

        except Exception as e:
            print("Serial reading error:", e)
            break

reader = threading.Thread(target=read_sensor, daemon=True)
reader.start()

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

line_gy, = ax1.plot([], [], label="Angular velocity")
line_angle, = ax2.plot([], [], label="Estimated angle")

ax1.set_ylabel("Angular velocity (°/s)")
ax1.legend()
ax1.grid(True)

ax2.set_xlabel("Time (seconds)")
ax2.set_ylabel("Relative angle (°)")
ax2.legend()
ax2.grid(True)

status = fig.suptitle("Collecting sensor data...")

def update(_):
    with lock:
        if not times:
            return line_gy, line_angle

        ts = list(times)
        gys = list(gy_values)
        angles = list(angle_values)

    latest = ts[-1]
    cutoff = latest - WINDOW_SECONDS
    indices = [i for i, t in enumerate(ts) if t >= cutoff]

    if not indices:
        return line_gy, line_angle

    ts = [ts[i] for i in indices]
    gys = [gys[i] for i in indices]
    angles = [angles[i] for i in indices]

    line_gy.set_data(ts, gys)

    # Center the displayed angle around zero
    center = sum(angles) / len(angles)
    centered_angles = [a - center for a in angles]

    line_angle.set_data(ts, centered_angles)

    ax1.set_xlim(
        max(0, latest - WINDOW_SECONDS),
        max(WINDOW_SECONDS, latest)
    )

    for ax, values in [(ax1, gys), (ax2, centered_angles)]:
        ymin, ymax = min(values), max(values)
        padding = max((ymax - ymin) * 0.2, 2)
        ax.set_ylim(ymin - padding, ymax + padding)

    return line_gy, line_angle

def stop_collection(event):
    global running

    running = False
    ani.event_source.stop()
    status.set_text("Collection stopped")

    # Close the serial connection after the reader exits.
    def close_serial():
        reader.join(timeout=2)
        if ser.is_open:
            ser.close()

    threading.Thread(target=close_serial, daemon=True).start()
    stop_button.label.set_text("Stopped")
    stop_button.ax.set_visible(False)
    fig.canvas.draw_idle()

# Leave space at the bottom for the button
plt.subplots_adjust(bottom=0.18, hspace=0.35)

stop_ax = fig.add_axes([0.40, 0.035, 0.20, 0.075])
stop_button = Button(stop_ax, "Stop")
stop_button.on_clicked(stop_collection)

ani = FuncAnimation(
    fig, update, interval=50,
    blit=False, cache_frame_data=False
)

try:
    plt.show()
finally:
    running = False
    if ser.is_open:
        ser.close()