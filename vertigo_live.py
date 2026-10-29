"""Live, mounting-calibrated upright head-turn monitor. Run --help for options.

Angle is vertical-axis gyro-integrated displacement from a manually chosen neutral position,
not an absolute heading. Calibration requires a stationary sensor. No artificial
recentering, detrending, or high-pass filtering is applied to the angle.
"""
import argparse
import csv
import math
import re
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass

NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
FIELDS = re.compile(r"\b([ag][xyz]|t_us)\s*[:=]\s*(" + NUMBER + r")", re.I)


def parse_sample(line, axis, units):
    fields = {k.lower(): float(v) for k, v in FIELDS.findall(line)}
    if axis not in fields or not all(math.isfinite(v) for v in fields.values()):
        return None
    rate = fields[axis] * (180 / math.pi if units == "rad/s" else 1)
    stamp = fields.get("t_us")
    return rate, None if stamp is None else stamp / 1_000_000


def parse_imu(line, units="deg/s"):
    fields = {k.lower(): float(v) for k, v in FIELDS.findall(line)}
    keys = ("gx", "gy", "gz", "ax", "ay", "az")
    if any(k not in fields for k in keys) or not all(math.isfinite(v) for v in fields.values()):
        return None
    scale = 180 / math.pi if units == "rad/s" else 1
    gyro = tuple(fields[k] * scale for k in keys[:3])
    accel = tuple(fields[k] for k in keys[3:])
    stamp = fields.get("t_us")
    return gyro, accel, None if stamp is None else stamp / 1e6


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


@dataclass
class Config:
    calibration_seconds: float = 3.0
    calibration_noise: float = 0.8  # maximum standard deviation, deg/s
    max_gap: float = 0.25
    window: float = 10.0
    crossing_threshold: float = 5.0  # deg/s; hysteresis for cycle detection
    min_period: float = 0.2
    max_period: float = 10.0


class MotionTracker:
    def __init__(self, config):
        self.c = config
        self.samples = deque(maxlen=100000)
        self.reset()

    def reset(self):
        self.samples.clear()
        self.calibration = []
        self.calibration_start = None
        self.bias = None
        self.angle = 0.0
        self.prev_t = self.prev_rate = None
        self.direction = 0
        self.crossings = deque(maxlen=32)
        self.cycle_peaks = deque(maxlen=10000)
        self.active_peak = None
        self.message = "Hold sensor STILL for calibration"

    def zero(self):
        self.angle = 0.0
        self.samples.clear()
        self.crossings.clear()
        self.cycle_peaks.clear()
        self.active_peak = None
        self.direction = 0
        self.prev_t = self.prev_rate = None

    def add(self, t, raw_rate):
        if not (math.isfinite(t) and math.isfinite(raw_rate)):
            return None
        if self.bias is None:
            if self.calibration_start is None:
                self.calibration_start = t
            self.calibration.append(raw_rate)
            elapsed = t - self.calibration_start
            self.message = f"Hold STILL: {max(0, self.c.calibration_seconds - elapsed):.1f}s"
            if elapsed >= self.c.calibration_seconds and len(self.calibration) >= 20:
                noise = statistics.pstdev(self.calibration)
                if noise > self.c.calibration_noise:
                    self.calibration.clear()
                    self.calibration_start = None
                    self.message = "Calibration motion/noise detected; hold STILL to retry"
                else:
                    self.bias = statistics.fmean(self.calibration)
                    self.message = f"Calibrated offset: {self.bias:+.3f} deg/s"
                    self.calibration.clear()
            return None
        rate = raw_rate - self.bias
        if self.prev_t is not None:
            dt = t - self.prev_t
            if dt <= 0:
                return None
            if dt > self.c.max_gap:
                # Missing rotation cannot be reconstructed across a data gap.
                self.prev_t = self.prev_rate = None
                self.crossings.clear()
                self.active_peak = None
                self.direction = 0
                self.message = "Data gap: return to neutral and click Zero angle"
            else:
                self.angle += (self.prev_rate + rate) * 0.5 * dt
        # Each one-way leg (left OR right) is one user-defined cycle.
        # Consecutive direction starts bound one completed one-way turn.
        direction = 1 if rate > self.c.crossing_threshold else -1 if rate < -self.c.crossing_threshold else 0
        if direction and direction != self.direction:
            if self.active_peak is not None:
                self.cycle_peaks.append(self.active_peak)
            self.active_peak = (t, self.angle)
            if self.crossings and t - self.crossings[-1] > self.c.max_period:
                self.crossings.clear()
            self.crossings.append(t)
            self.direction = direction
        if self.direction:
            if self.active_peak is None or (self.direction > 0 and self.angle > self.active_peak[1]) or (self.direction < 0 and self.angle < self.active_peak[1]):
                self.active_peak = (t, self.angle)
        while self.cycle_peaks and self.cycle_peaks[0][0] < t - self.c.window:
            self.cycle_peaks.popleft()
        self.prev_t, self.prev_rate = t, rate
        row = (t, raw_rate, rate, self.angle)
        self.samples.append(row)
        while self.samples and self.samples[0][0] < t - self.c.window:
            self.samples.popleft()
        return row

    def frequency(self, now):
        recent = [t for t in self.crossings if now - t <= self.c.window]
        if len(recent) < 2:  # one completed one-way turn between direction changes
            return None
        periods = [b - a for a, b in zip(recent, recent[1:])]
        # Reject implausibly fast chatter rather than reporting it as exercise.
        if any(p < self.c.min_period for p in periods):
            return None
        period = statistics.median(periods)
        if now - recent[-1] > min(self.c.max_period, 2 * period):
            return None
        if not self.c.min_period <= period <= self.c.max_period:
            return None
        return 1 / period


class EarTurnTracker(MotionTracker):
    """Project all gyro axes onto the resting vertical axis.

    Appropriate for upright yaw with a rigid mounting. The accelerometer is
    used during stationary calibration, NOT as an instantaneous yaw reference.
    The axis stays fixed in the board frame; combined nodding/tilting requires
    a full attitude estimator. Tilt is flagged rather than silently corrected.
    """
    def __init__(self, config, axis="auto", invert=False):
        self.axis_choice = axis
        self.sign = -1 if invert else 1
        super().__init__(config)

    def reset(self):
        super().reset()
        self.gyro_bias = None
        self.turn_axis = None
        self.rest_accel = None
        self.accel_norm = None
        self.imu_calibration = []
        self.quality_warning = ""
        self.last_imu_t = None

    def add(self, t, gyro, accel):
        if not all(math.isfinite(v) for v in (t, *gyro, *accel)):
            return None
        if self.last_imu_t is not None and t <= self.last_imu_t:
            return None
        if self.bias is None:
            if self.last_imu_t is not None and t - self.last_imu_t > self.c.max_gap:
                self.imu_calibration.clear()
                self.calibration_start = None
            self.last_imu_t = t
            if self.calibration_start is None:
                self.calibration_start = t
            self.imu_calibration.append((gyro, accel))
            elapsed = t - self.calibration_start
            self.message = f"Head UPRIGHT, hold STILL: {max(0, self.c.calibration_seconds - elapsed):.1f}s"
            if elapsed < self.c.calibration_seconds or len(self.imu_calibration) < 20:
                return None
            gs = list(zip(*(g for g, a in self.imu_calibration)))
            ac = list(zip(*(a for g, a in self.imu_calibration)))
            mean_a = tuple(statistics.fmean(v) for v in ac)
            norm_a = math.sqrt(dot(mean_a, mean_a))
            gyro_noise = max(statistics.pstdev(v) for v in gs)
            accel_noise = math.sqrt(sum(statistics.pvariance(v) for v in ac))
            if norm_a < 1e-6 or gyro_noise > self.c.calibration_noise or accel_noise > 0.03 * norm_a:
                self.imu_calibration.clear()
                self.calibration_start = None
                self.message = "Movement/noise during calibration: hold head upright and STILL to retry"
                return None
            self.gyro_bias = tuple(statistics.fmean(v) for v in gs)
            self.rest_accel = tuple(v / norm_a for v in mean_a)
            self.accel_norm = norm_a
            if self.axis_choice == "auto":
                self.turn_axis = tuple(self.sign * v for v in self.rest_accel)
            else:
                self.turn_axis = tuple(self.sign * float(k == self.axis_choice) for k in ("gx", "gy", "gz"))
            self.bias = dot(self.gyro_bias, self.turn_axis)
            self.imu_calibration.clear()
            self.message = "Mounting calibrated. Face neutral and click Zero angle; keep head upright."
            return None
        self.last_imu_t = t
        norm = math.sqrt(dot(accel, accel))
        self.quality_warning = ""
        if norm < 0.8 * self.accel_norm or norm > 1.2 * self.accel_norm:
            self.quality_warning = "Acceleration detected; keep mounting fixed and head upright"
        elif norm > 0:
            tilt_cos = max(-1, min(1, dot(accel, self.rest_accel) / norm))
            if math.degrees(math.acos(tilt_cos)) > 15:
                self.quality_warning = "Tilt/acceleration >15 degrees: upright head-turn estimate may be inaccurate"
        return super().add(t, dot(gyro, self.turn_axis))


class SensorClock:
    """Prefer firmware sample times; unwrap a 32-bit micros() counter."""
    def __init__(self):
        self.last = None
        self.elapsed = 0.0
        self.mode = None

    def update(self, device_t, host_t):
        mode = "device" if device_t is not None else "host"
        if self.mode is not None and self.mode != mode:
            raise ValueError("Serial timestamp format changed; restart collection")
        self.mode = mode
        stamp = device_t if device_t is not None else host_t
        if self.last is None:
            self.last = stamp
            return 0.0
        delta = stamp - self.last
        if mode == "device" and delta < -(2**31 / 1e6):
            delta += 2**32 / 1e6
        if delta < 0:
            raise ValueError("Sensor timestamp moved backwards; board may have restarted")
        self.last = stamp
        self.elapsed += delta
        return self.elapsed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--axis", choices=["auto", "gx", "gy", "gz"], default="auto", help="auto uses all gyro axes and stationary accelerometer mounting calibration")
    parser.add_argument("--units", choices=["deg/s", "rad/s"], default="deg/s")
    parser.add_argument("--invert", action="store_true", help="reverse the angle sign without changing magnitude")
    parser.add_argument("--window", type=float, default=10)
    parser.add_argument("--angle-limit", type=float, default=60, help="fixed plot range +/- degrees")
    parser.add_argument("--csv", help="optional output CSV filename")
    parser.add_argument("--demo", action="store_true", help="synthetic +/-30 degree, 0.5 Hz motion with sensor bias")
    args = parser.parse_args()
    if args.window <= 0 or args.angle_limit <= 0:
        parser.error("window and angle-limit must be positive")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button

    tracker = EarTurnTracker(Config(window=args.window), args.axis, args.invert)
    lock = threading.Lock()
    stop = threading.Event()
    paused = threading.Event()
    frequency_samples = deque(maxlen=100000)
    state = {"error": "", "last_received": None, "ignored": 0, "timing": ""}
    ser = None
    if not args.demo:
        import serial
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.2)
            time.sleep(2)
            ser.reset_input_buffer()
        except serial.SerialException as exc:
            parser.exit(1, f"Cannot open {args.port}: {exc}\nClose other serial monitors and check --port.\n")
    log = open(args.csv, "w", newline="") if args.csv else None
    writer = csv.writer(log) if log else None
    if writer:
        writer.writerow(["time_s", "raw_projected_rate_deg_s", "corrected_turn_rate_deg_s", "relative_angle_deg", "gx", "gy", "gz", "ax", "ay", "az"])

    def read_sensor():
        clock = SensorClock()
        begin = time.monotonic()
        try:
            while not stop.is_set():
                host = time.monotonic()
                if args.demo:
                    t = host - begin
                    turn_rate = 0 if t < 4 else 30 * math.pi * math.cos(math.pi * (t - 4))
                    accel = (-0.10, -0.16, -0.94)
                    norm = math.sqrt(dot(accel, accel))
                    axis = tuple(v / norm for v in accel)
                    gyro = tuple(b + turn_rate * u for b, u in zip((0.5, -2.4, 2.6), axis))
                    device_t = t
                    stop.wait(0.01)
                else:
                    raw = ser.readline().decode("utf-8", errors="replace").strip()
                    if not raw:
                        continue
                    parsed = parse_imu(raw, args.units)
                    if parsed is None:
                        with lock:
                            state["ignored"] += 1
                        continue
                    gyro, accel, device_t = parsed
                    host = time.monotonic()  # timestamp AFTER the blocking read
                t = clock.update(device_t, host)
                with lock:
                    state["last_received"] = time.monotonic()
                    state["timing"] = clock.mode
                    if paused.is_set():
                        continue  # keep draining serial; never replay a paused backlog
                    row = tracker.add(t, gyro, accel)
                    if row is not None:
                        frequency_samples.append((t, tracker.frequency(t)))
                        while frequency_samples and frequency_samples[0][0] < t - args.window:
                            frequency_samples.popleft()
                if writer and row:
                    writer.writerow((*row, *gyro, *accel))
        except Exception as exc:
            with lock:
                state["error"] = f"Acquisition stopped: {exc}"
            stop.set()

    reader = threading.Thread(target=read_sensor, daemon=True)
    reader.start()
    fig, (ax_rate, ax_angle, ax_freq) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    fig.subplots_adjust(top=0.80, bottom=0.18, hspace=0.30)
    title = fig.suptitle("", fontsize=10)
    metrics = fig.text(0.5, 0.94, "Angle: --°     Frequency: -- Hz", ha="center", fontsize=16)
    rate_line, = ax_rate.plot([], [], color="#1976d2", label="Calibrated head-turn angular velocity")
    angle_line, = ax_angle.plot([], [], color="#d55e00", label="Integrated angle from neutral")
    peak_points, = ax_angle.plot([], [], "o", color="#7c3aed", markersize=5, label="Peak per one-way turn (* ongoing)")
    peak_labels = []
    freq_line, = ax_freq.plot([], [], color="#15803d", label="Frequency: each left OR right turn = 1 cycle")
    for ax in (ax_rate, ax_angle, ax_freq):
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="gray", lw=0.8)
        ax.legend(loc="upper left")
    ax_rate.set_ylabel("Angular velocity (deg/s)")
    ax_angle.set_ylabel("Relative angle (degrees)")
    ax_freq.set_xlabel("Sample time (seconds)")
    ax_freq.set_ylabel("Frequency (Hz)")
    ax_freq.set_ylim(0, 2)
    ax_angle.set_ylim(-args.angle_limit, args.angle_limit)
    ax_angle.set_yticks([-args.angle_limit, -args.angle_limit / 2, 0, args.angle_limit / 2, args.angle_limit])
    ax_angle.legend(loc="lower left", bbox_to_anchor=(0, 1.01), ncol=2, fontsize=8, frameon=False)
    ax_rate.set_ylim(-100, 100)
    ax_rate.set_xlim(0, args.window)

    def update(_):
        with lock:
            rows = list(tracker.samples)
            message = tracker.quality_warning or tracker.message
            freq = tracker.frequency(tracker.prev_t) if tracker.prev_t is not None else None
            status = dict(state)
            freq_rows = list(frequency_samples)
            current_angle = tracker.angle if tracker.bias is not None else None
            peaks = list(tracker.cycle_peaks)
            active_peak = tracker.active_peak
        if rows:
            ts, _, rates, angles = zip(*rows)
            rate_line.set_data(ts, rates)
            angle_line.set_data(ts, angles)
            ax_rate.set_xlim(max(0, ts[-1] - args.window), max(args.window, ts[-1]))
            limit = max(20, 1.2 * max(abs(v) for v in rates))
            ax_rate.set_ylim(-limit, limit)
            clipped = any(abs(v) > args.angle_limit for v in angles)
        else:
            rate_line.set_data([], [])
            angle_line.set_data([], [])
            clipped = False
        # Rebuild only labels within the scrolling window. The current turn
        # remains provisional (*) until the next direction change confirms it.
        for label in peak_labels:
            label.remove()
        peak_labels.clear()
        left, right = ax_angle.get_xlim()
        visible_peaks = [(t, a, False) for t, a in peaks if left <= t <= right]
        if active_peak is not None and left <= active_peak[0] <= right:
            visible_peaks.append((*active_peak, True))
        peak_points.set_data([t for t, a, active in visible_peaks], [max(-args.angle_limit, min(args.angle_limit, a)) for t, a, active in visible_peaks])
        for t, a, active in visible_peaks:
            y = max(-args.angle_limit, min(args.angle_limit, a))
            label = ax_angle.annotate(f"{a:+.1f}°{'*' if active else ''}",
                (t, y), xytext=(0, -13 if a >= 0 else 8),
                textcoords="offset points", ha="center", fontsize=9,
                color="#6d28d9", bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=1))
            peak_labels.append(label)
        if freq_rows:
            ft, fv = zip(*freq_rows)
            freq_line.set_data(ft, [math.nan if f is None else f for f in fv])
            ax_freq.set_ylim(0, max(1, 1.2 * max((f for f in fv if f is not None), default=0)))
        else:
            freq_line.set_data([], [])
        stale = status["last_received"] is None or time.monotonic() - status["last_received"] > 1
        if stale or paused.is_set():
            freq = None
        frequency = "Frequency: waiting for a completed one-way turn" if freq is None else f"Frequency: {freq:.2f} Hz ({60 * freq:.0f} one-way turns/min)"
        warning = status["error"] or ("No recent complete IMU samples; need ax ay az gx gy gz" if stale else message)
        if paused.is_set():
            warning = "Paused. Return to neutral, then click Continue (angle resets to zero)."
        if stop.is_set() and not status["error"]:
            warning = "Collection ended; restart the script to reconnect"
        timing = "firmware sample timing" if status["timing"] == "device" else "host arrival timing (approximate)"
        angle_text = "--" if current_angle is None else f"{current_angle:+.1f}"
        freq_text = "--" if freq is None else f"{freq:.2f}"
        metrics.set_text(f"Angle: {angle_text}°     Frequency: {freq_text} Hz")
        title.set_position((0.5, 0.89))
        title.set_text(f"{'DEMO | ' if args.demo else ''}{frequency}\n{warning}\n{timing} | upright relative head turn | all 3 gyro axes + resting gravity" + ("\nAngle exceeds plot range: adjust --angle-limit" if clipped else ""))
        return rate_line, angle_line, freq_line

    def zero(_):
        with lock:
            tracker.zero()
            frequency_samples.clear()

    def calibrate(_):
        with lock:
            tracker.reset()
            frequency_samples.clear()

    def pause(_):
        with lock:
            paused.set()

    def resume(_):
        with lock:
            if stop.is_set():
                return
            if not paused.is_set():
                return
            # Rotation while paused is unknown. Begin at the user's neutral pose.
            if tracker.bias is None:
                tracker.reset()
            else:
                tracker.zero()
                tracker.message = "Continued from new neutral position"
            frequency_samples.clear()
            paused.clear()

    buttons = []
    for left, label, callback in [(0.07, "Zero angle", zero), (0.29, "Recalibrate", calibrate), (0.51, "Stop", pause), (0.73, "Continue", resume)]:
        button = Button(fig.add_axes([left, 0.045, 0.19, 0.065]), label)
        button.on_clicked(callback)
        buttons.append(button)
    fig.canvas.mpl_connect("close_event", lambda _: stop.set())
    animation = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop.set()
        reader.join(timeout=2)
        if ser:
            ser.close()
        if log:
            log.close()


if __name__ == "__main__":
    main()
