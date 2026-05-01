#!/usr/bin/env python3
"""Live plot of ESP32 serial telemetry.

Expected serial lines from the firmware look like:
Plot,Mode:0,Set:5.00,Raw:1234,Volt:1.234,Dist:10.50,DistF:10.20,Err:-0.20,Int:1.10,Out:2.50,Servo:92.00,Kp:10.00,Ki:0.01,Kd:2.00
"""

from __future__ import annotations

import argparse
import collections
import re
import sys
import time
from typing import Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
import serial


SERIAL_RE = re.compile(r"([A-Za-z]+):(-?\d+(?:\.\d+)?)")


class TelemetryBuffer:
    def __init__(self, max_points: int) -> None:
        self.max_points = max_points
        self.t0 = time.monotonic()
        self.time_s = collections.deque(maxlen=max_points)
        self.mode = collections.deque(maxlen=max_points)
        self.setpoint = collections.deque(maxlen=max_points)
        self.raw = collections.deque(maxlen=max_points)
        self.volt = collections.deque(maxlen=max_points)
        self.dist = collections.deque(maxlen=max_points)
        self.distf = collections.deque(maxlen=max_points)
        self.err = collections.deque(maxlen=max_points)
        self.integral = collections.deque(maxlen=max_points)
        self.output = collections.deque(maxlen=max_points)
        self.servo = collections.deque(maxlen=max_points)
        self.kp = collections.deque(maxlen=max_points)
        self.ki = collections.deque(maxlen=max_points)
        self.kd = collections.deque(maxlen=max_points)

    def append(self, fields: dict[str, float]) -> None:
        now = time.monotonic() - self.t0
        self.time_s.append(now)
        self.mode.append(fields.get("Mode", 0.0))
        self.setpoint.append(fields.get("Set", 0.0))
        self.raw.append(fields.get("Raw", 0.0))
        self.volt.append(fields.get("Volt", 0.0))
        self.dist.append(fields.get("Dist", 0.0))
        self.distf.append(fields.get("DistF", 0.0))
        self.err.append(fields.get("Err", 0.0))
        self.integral.append(fields.get("Int", 0.0))
        self.output.append(fields.get("Out", 0.0))
        self.servo.append(fields.get("Servo", 0.0))
        self.kp.append(fields.get("Kp", 0.0))
        self.ki.append(fields.get("Ki", 0.0))
        self.kd.append(fields.get("Kd", 0.0))



def parse_line(line: str) -> dict[str, float] | None:
    line = line.strip()
    if not line.startswith("Plot,"):
        return None

    fields: dict[str, float] = {}
    for key, value in SERIAL_RE.findall(line):
        try:
            fields[key] = float(value)
        except ValueError:
            continue

    return fields if fields else None



def build_plot(ax, x, y, label, color, linewidth=1.5):
    (line,) = ax.plot(x, y, label=label, color=color, linewidth=linewidth)
    return line


def send_command(ser: serial.Serial, command: str, newline: bool = True) -> None:
    payload = command.encode("utf-8")
    if newline:
        payload += b"\n"
    ser.write(payload)
    ser.flush()


def apply_limits(ax, limits: Sequence[float] | None) -> None:
    if limits is None:
        return
    ax.set_ylim(limits[0], limits[1])



def main() -> int:
    parser = argparse.ArgumentParser(description="Live plot ESP32 serial telemetry")
    parser.add_argument("port", help="Serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--points", type=int, default=400, help="Maximum plotted points (default: 400)")
    parser.add_argument("--send", help="Send a message to the ESP32 and exit")
    parser.add_argument("--no-newline", action="store_true", help="Do not append newline when sending a message")
    parser.add_argument("--dist-lim", nargs=2, type=float, metavar=("MIN", "MAX"), default=[0.0, 30.0],
                        help="Y-axis limits for Dist and DistF (default: 0 30)")
    parser.add_argument("--err-lim", nargs=2, type=float, metavar=("MIN", "MAX"), default=[-20.0, 20.0],
                        help="Y-axis limits for Err and Int (default: -20 20)")
    parser.add_argument("--out-lim", nargs=2, type=float, metavar=("MIN", "MAX"), default=[-90.0, 90.0],
                        help="Y-axis limits for Out and Servo (default: -90 90)")
    parser.add_argument("--volt-lim", nargs=2, type=float, metavar=("MIN", "MAX"), default=[0.0, 3.3],
                        help="Y-axis limits for Volt (default: 0 3.3)")
    parser.add_argument("--auto-scale", action="store_true", help="Use automatic y-axis scaling instead of fixed ranges")
    parser.add_argument("--no-controls", action="store_true", help="Hide sliders and manual control buttons")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    if args.send is not None:
        payload = args.send.encode("utf-8")
        if not args.no_newline:
            payload += b"\n"
        ser.write(payload)
        ser.flush()
        print(f"Sent to {args.port}: {args.send}")
        ser.close()
        return 0

    buf = TelemetryBuffer(args.points)

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
    fig.canvas.manager.set_window_title("ESP32 Telemetry Plot")
    fig.subplots_adjust(bottom=0.40 if not args.no_controls else 0.08, top=0.93)
    fig.suptitle("ESP32 ball-balance telemetry")

    lines = {}
    lines["dist"] = build_plot(axes[0], [], [], "Dist", "tab:blue")
    lines["distf"] = build_plot(axes[0], [], [], "DistF", "tab:cyan")
    lines["set"] = build_plot(axes[0], [], [], "Setpoint", "tab:orange", 1.2)
    axes[0].set_ylabel("cm")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)
    apply_limits(axes[0], None if args.auto_scale else args.dist_lim)

    lines["err"] = build_plot(axes[1], [], [], "Err", "tab:red")
    lines["int"] = build_plot(axes[1], [], [], "Int", "tab:purple")
    axes[1].set_ylabel("error")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)
    apply_limits(axes[1], None if args.auto_scale else args.err_lim)

    lines["out"] = build_plot(axes[2], [], [], "Out", "tab:green")
    lines["servo"] = build_plot(axes[2], [], [], "Servo", "tab:brown")
    axes[2].set_ylabel("output / deg")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)
    apply_limits(axes[2], None if args.auto_scale else args.out_lim)

    lines["volt"] = build_plot(axes[3], [], [], "Volt", "tab:gray")
    axes[3].set_ylabel("V")
    axes[3].set_xlabel("time (s)")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.25)
    apply_limits(axes[3], None if args.auto_scale else args.volt_lim)

    status = fig.text(0.01, 0.015, f"Port: {args.port}", fontsize=9)
    gains_text = fig.text(0.50, 0.015, "Set: -- | Kp: -- | Ki: -- | Kd: -- | Servo: --", fontsize=9, ha="center")

    sliders = {}
    slider_guard = {"enabled": True}

    if not args.no_controls:
        slider_axes = {
            "setpoint": fig.add_axes([0.10, 0.28, 0.70, 0.03]),
            "kp": fig.add_axes([0.10, 0.24, 0.70, 0.03]),
            "ki": fig.add_axes([0.10, 0.20, 0.70, 0.03]),
            "kd": fig.add_axes([0.10, 0.16, 0.70, 0.03]),
            "servo": fig.add_axes([0.10, 0.12, 0.70, 0.03]),
        }

        sliders["setpoint"] = Slider(slider_axes["setpoint"], "Set", 0.0, 30.0, valinit=5.0, valstep=0.1)
        sliders["kp"] = Slider(slider_axes["kp"], "Kp", 0.0, 50.0, valinit=10.0, valstep=0.1)
        sliders["ki"] = Slider(slider_axes["ki"], "Ki", 0.0, 5.0, valinit=0.01, valstep=0.01)
        sliders["kd"] = Slider(slider_axes["kd"], "Kd", 0.0, 20.0, valinit=2.0, valstep=0.1)
        sliders["servo"] = Slider(slider_axes["servo"], "Angle", 0.0, 180.0, valinit=90.0, valstep=1.0)

        for slider in sliders.values():
            slider.valtext.set_fontsize(9)

        button_axes = {
            "auto": fig.add_axes([0.84, 0.25, 0.12, 0.05]),
            "manual": fig.add_axes([0.84, 0.19, 0.12, 0.05]),
            "reset": fig.add_axes([0.84, 0.13, 0.12, 0.05]),
        }
        buttons = {
            "auto": Button(button_axes["auto"], "AUTO"),
            "manual": Button(button_axes["manual"], "MANUAL"),
            "reset": Button(button_axes["reset"], "RESET"),
        }

        def send_from_slider(prefix: str, value: float, fmt: str) -> None:
            if not slider_guard["enabled"]:
                return
            try:
                send_command(ser, f"{prefix}:{fmt.format(value)}")
            except serial.SerialException:
                pass

        sliders["setpoint"].on_changed(lambda value: send_from_slider("set", value, "{:.2f}"))
        sliders["kp"].on_changed(lambda value: send_from_slider("kp", value, "{:.2f}"))
        sliders["ki"].on_changed(lambda value: send_from_slider("ki", value, "{:.3f}"))
        sliders["kd"].on_changed(lambda value: send_from_slider("kd", value, "{:.2f}"))
        sliders["servo"].on_changed(lambda value: send_from_slider("angle", value, "{:.0f}"))

        buttons["auto"].on_clicked(lambda _event: send_command(ser, "auto"))
        buttons["manual"].on_clicked(lambda _event: send_command(ser, "manual:on"))
        buttons["reset"].on_clicked(lambda _event: send_command(ser, "reset"))

    def update(_frame):
        updated = False
        while ser.in_waiting:
            raw_line = ser.readline().decode(errors="ignore")
            fields = parse_line(raw_line)
            if fields is None:
                if raw_line.strip():
                    print(f"[ignored] {raw_line.strip()}")
                continue
            buf.append(fields)
            updated = True
            print(f"[parsed] dist={fields.get('Dist', 0):.2f} servo={fields.get('Servo', 0):.0f}")

        if not buf.time_s:
            return tuple(lines.values())

        x = list(buf.time_s)
        lines["dist"].set_data(x, list(buf.dist))
        lines["distf"].set_data(x, list(buf.distf))
        lines["set"].set_data(x, list(buf.setpoint))
        lines["err"].set_data(x, list(buf.err))
        lines["int"].set_data(x, list(buf.integral))
        lines["out"].set_data(x, list(buf.output))
        lines["servo"].set_data(x, list(buf.servo))
        lines["volt"].set_data(x, list(buf.volt))

        # Always autoscale to show data
        for ax in axes:
            ax.relim()
            ax.autoscale_view()

        if updated:
            if sliders and buf.setpoint:
                slider_guard["enabled"] = False
                try:
                    sliders["setpoint"].set_val(buf.setpoint[-1])
                    sliders["kp"].set_val(buf.kp[-1])
                    sliders["ki"].set_val(buf.ki[-1])
                    sliders["kd"].set_val(buf.kd[-1])
                    sliders["servo"].set_val(buf.servo[-1])
                finally:
                    slider_guard["enabled"] = True

            status.set_text(
                f"Port: {args.port}  Mode: {'MAN' if buf.mode and buf.mode[-1] else 'AUTO'}  "
                f"Dist: {buf.dist[-1]:.2f}  DistF: {buf.distf[-1]:.2f}  Err: {buf.err[-1]:.2f}  "
                f"Servo: {buf.servo[-1]:.1f}"
            )
            gains_text.set_text(
                f"Set: {buf.setpoint[-1]:.2f} | Kp: {buf.kp[-1]:.2f} | Ki: {buf.ki[-1]:.3f} | Kd: {buf.kd[-1]:.2f} | Out: {buf.output[-1]:.1f}"
            )

        return tuple(lines.values())

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()
    _ = anim
    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
