#!/usr/bin/env python3
"""Live plot of ESP32 serial telemetry.

Expected serial lines from the firmware look like:
Plot,Mode:0,Set:5.00,Raw:1234,Volt:1.234,Dist:10.50,DistF:10.20,Err:-0.20,Int:1.10,Out:2.50,Servo:92.00,Kp:10.00,Ki:0.01,Kd:2.00
"""

from __future__ import annotations

import argparse
import collections
import math
import os
import re
import sys
import time
from typing import Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
import serial

try:
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover - vision mode is optional
    cv2 = None
    np = None


SERIAL_RE = re.compile(r"([A-Za-z]+):(-?\d+(?:\.\d+)?)")

VISION_DEFAULT_LOW = (35, 80, 70)
VISION_DEFAULT_HIGH = (90, 255, 255)


def vision_available() -> bool:
    return cv2 is not None and np is not None


def parse_triplet(text: str, default: tuple[int, int, int]) -> tuple[int, int, int]:
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3:
        return default
    try:
        return tuple(max(0, int(part)) for part in parts)  # type: ignore[return-value]
    except ValueError:
        return default


def detect_ball(frame, hsv_low, hsv_high):
    if cv2 is None or np is None:
        return None, None

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_low, hsv_high)

    if int(hsv_low[0]) > int(hsv_high[0]):
        low2 = hsv_low.copy()
        low2[0] = 0
        high2 = hsv_high.copy()
        high2[0] = 179
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low2, high2))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < 60.0:
        return None, mask

    (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
    return (int(center_x), int(center_y), int(radius)), mask


class TelemetryBuffer:
    def __init__(self, max_points: int) -> None:
        self.max_points = max_points
        self.t0 = time.monotonic()
        self.time_s = collections.deque(maxlen=max_points)
        self.mode = collections.deque(maxlen=max_points)
        self.source = collections.deque(maxlen=max_points)
        self.setpoint = collections.deque(maxlen=max_points)
        self.raw = collections.deque(maxlen=max_points)
        self.volt = collections.deque(maxlen=max_points)
        self.dist = collections.deque(maxlen=max_points)
        self.distf = collections.deque(maxlen=max_points)
        self.speed = collections.deque(maxlen=max_points)
        self.stuck_avg_speed = collections.deque(maxlen=max_points)
        self.accel = collections.deque(maxlen=max_points)
        self.err = collections.deque(maxlen=max_points)
        self.integral = collections.deque(maxlen=max_points)
        self.output = collections.deque(maxlen=max_points)
        self.servo = collections.deque(maxlen=max_points)
        self.pout = collections.deque(maxlen=max_points)
        self.dout = collections.deque(maxlen=max_points)
        self.iout = collections.deque(maxlen=max_points)
        self.deriv = collections.deque(maxlen=max_points)
        self.kp = collections.deque(maxlen=max_points)
        self.ki = collections.deque(maxlen=max_points)
        self.kd = collections.deque(maxlen=max_points)
        self.neutral = collections.deque(maxlen=max_points)
        self.travel = collections.deque(maxlen=max_points)
        self.direction = collections.deque(maxlen=max_points)
        self.pid_dead = collections.deque(maxlen=max_points)
        self.settle_err = collections.deque(maxlen=max_points)
        self.settle_deriv = collections.deque(maxlen=max_points)
        self.servo_filt = collections.deque(maxlen=max_points)
        self.servo_rate = collections.deque(maxlen=max_points)
        self.breakaway = collections.deque(maxlen=max_points)
        self.dither_amp = collections.deque(maxlen=max_points)
        self.dither_freq = collections.deque(maxlen=max_points)
        self.stuck_speed = collections.deque(maxlen=max_points)
        self.stuck_band = collections.deque(maxlen=max_points)
        self.static = collections.deque(maxlen=max_points)
        self.settle_active = collections.deque(maxlen=max_points)
        self.breakaway_active = collections.deque(maxlen=max_points)
        self.dither_active = collections.deque(maxlen=max_points)
        self.friction_active = collections.deque(maxlen=max_points)
        self.break_deg = collections.deque(maxlen=max_points)
        self.dither_deg = collections.deque(maxlen=max_points)
        self.dither_on = collections.deque(maxlen=max_points)
        self.break_on = collections.deque(maxlen=max_points)
        self.dist_filter_on = collections.deque(maxlen=max_points)
        self.deriv_filter_on = collections.deque(maxlen=max_points)
        self.pid_filter_on = collections.deque(maxlen=max_points)
        self.servo_filter_on = collections.deque(maxlen=max_points)
        self.servo_dead = collections.deque(maxlen=max_points)
        self.sample_ms = collections.deque(maxlen=max_points)
        self.dist_alpha = collections.deque(maxlen=max_points)
        self.speed_alpha = collections.deque(maxlen=max_points)
        self.int_limit = collections.deque(maxlen=max_points)
        self.max_accel = collections.deque(maxlen=max_points)
        self.mass = collections.deque(maxlen=max_points)
        self.gravity = collections.deque(maxlen=max_points)
        self.roll_factor = collections.deque(maxlen=max_points)
        self.friction = collections.deque(maxlen=max_points)
        self.friction_blend = collections.deque(maxlen=max_points)
        self.beam_gain = collections.deque(maxlen=max_points)
        self.curve_a = collections.deque(maxlen=max_points)
        self.curve_exp = collections.deque(maxlen=max_points)
        self.cal_scale = collections.deque(maxlen=max_points)
        self.cal_offset = collections.deque(maxlen=max_points)
        self.ball_radius = collections.deque(maxlen=max_points)
        self.loopcount = collections.deque(maxlen=max_points)
        self.adc_count = collections.deque(maxlen=max_points)
        self.control_count = collections.deque(maxlen=max_points)

    def append(self, fields: dict[str, float]) -> None:
        now = time.monotonic() - self.t0
        self.time_s.append(now)
        self.mode.append(fields.get("Mode", 0.0))
        self.source.append(fields.get("Src", 0.0))
        self.setpoint.append(fields.get("Set", 0.0))
        self.raw.append(fields.get("Raw", 0.0))
        self.volt.append(fields.get("Volt", 0.0))
        self.dist.append(fields.get("Dist", 0.0))
        self.distf.append(fields.get("DistF", 0.0))
        self.speed.append(fields.get("Speed", 0.0))
        self.stuck_avg_speed.append(fields.get("Avg10Speed", fields.get("StuckAvg", 0.0)))
        self.accel.append(fields.get("Accel", 0.0))
        self.err.append(fields.get("Err", 0.0))
        self.integral.append(fields.get("Int", 0.0))
        self.output.append(fields.get("Out", 0.0))
        self.servo.append(fields.get("Servo", 0.0))
        self.pout.append(fields.get("POut", 0.0))
        self.dout.append(fields.get("DOut", 0.0))
        self.iout.append(fields.get("IOut", 0.0))
        self.deriv.append(fields.get("Deriv", 0.0))
        self.kp.append(fields.get("Kp", 0.0))
        self.ki.append(fields.get("Ki", 0.0))
        self.kd.append(fields.get("Kd", 0.0))
        self.neutral.append(fields.get("Neutral", 84.0))
        self.travel.append(fields.get("Travel", 30.0))
        self.direction.append(fields.get("Dir", -1.0))
        self.pid_dead.append(fields.get("PidDead", 0.0))
        self.settle_err.append(fields.get("SettleError", fields.get("SettleErr", 0.8)))
        self.settle_deriv.append(fields.get("SettleSpeed", fields.get("SettleDeriv", 0.5)))
        self.servo_filt.append(fields.get("ServoFilt", 0.0))
        self.servo_rate.append(fields.get("ServoRate", 0.0))
        self.breakaway.append(fields.get("Breakaway", 0.0))
        self.dither_amp.append(fields.get("DitherAmp", 0.0))
        self.dither_freq.append(fields.get("DitherFreq", 0.0))
        self.stuck_speed.append(fields.get("StuckSpeed", 0.0))
        self.stuck_band.append(fields.get("StuckBand", 0.0))
        self.static.append(fields.get("Static", 0.0))
        self.settle_active.append(fields.get("Settle", 0.0))
        self.breakaway_active.append(fields.get("BreakAct", 0.0))
        self.dither_active.append(fields.get("DitherAct", 0.0))
        self.friction_active.append(fields.get("Friction", 0.0))
        self.break_deg.append(fields.get("BreakDeg", 0.0))
        self.dither_deg.append(fields.get("DitherDeg", 0.0))
        self.dither_on.append(fields.get("DitherOn", 1.0))
        self.break_on.append(fields.get("BreakOn", 1.0))
        self.dist_filter_on.append(fields.get("DistFiltOn", 1.0))
        self.deriv_filter_on.append(fields.get("DerivFiltOn", 1.0))
        self.pid_filter_on.append(fields.get("PidFiltOn", 1.0))
        self.servo_filter_on.append(fields.get("ServoFiltOn", 1.0))
        self.servo_dead.append(fields.get("ServoDead", 0.0))
        self.sample_ms.append(fields.get("SampleMs", 1.0))
        self.dist_alpha.append(fields.get("DistAlpha", 0.25))
        self.speed_alpha.append(fields.get("SpeedAlpha", 0.18))
        self.int_limit.append(fields.get("IntLimit", 12.0))
        self.max_accel.append(fields.get("MaxAccel", 1.5))
        self.mass.append(fields.get("Mass", 0.0455))
        self.gravity.append(fields.get("Gravity", 9.81))
        self.roll_factor.append(fields.get("RollFactor", 1.4))
        self.friction.append(fields.get("Friction", 0.003))
        self.friction_blend.append(fields.get("FrictionBlend", 0.03))
        self.beam_gain.append(fields.get("BeamGain", 0.114))
        self.curve_a.append(fields.get("CurveA", 12.08))
        self.curve_exp.append(fields.get("CurveExp", -1.058))
        self.cal_scale.append(fields.get("CalScale", 1.333333))
        self.cal_offset.append(fields.get("CalOffset", -3.333333))
        self.ball_radius.append(fields.get("BallRadius", 1.0))
        self.loopcount.append(fields.get("LoopCount", 0.0))
        self.adc_count.append(fields.get("ADC", 0.0))
        self.control_count.append(fields.get("Control", 0.0))



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


def state_pulses(values, active_level: float):
    return [active_level if value >= 0.5 else math.nan for value in values]


def state_motion(active_values, motion_values, baseline: float):
    return [
        baseline + motion if active >= 0.5 else math.nan
        for active, motion in zip(active_values, motion_values)
    ]


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
    parser.add_argument("--vision-camera", type=int, default=0, help="Camera index used in computer-vision mode")
    parser.add_argument("--vision-span-cm", type=float, default=30.0,
                        help="Physical span in cm mapped across the camera width in vision mode")
    parser.add_argument("--vision-hsv-low", nargs=3, type=int, metavar=("H", "S", "V"), default=list(VISION_DEFAULT_LOW),
                        help="HSV low threshold for vision mode")
    parser.add_argument("--vision-hsv-high", nargs=3, type=int, metavar=("H", "S", "V"), default=list(VISION_DEFAULT_HIGH),
                        help="HSV high threshold for vision mode")
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
    plot_bottom = 0.42 if not args.no_controls else 0.08
    plot_top = 0.93
    fig.subplots_adjust(bottom=plot_bottom, top=plot_top)
    fig.suptitle("ESP32 ball-balance telemetry")

    # Track which subplot is enlarged (None = all visible)
    enlarged_axis = [None]

    lines = {}
    lines["dist"] = build_plot(axes[0], [], [], "Dist", "tab:blue")
    lines["distf"] = build_plot(axes[0], [], [], "DistF", "tab:cyan")
    lines["speed"] = build_plot(axes[0], [], [], "Speed", "tab:purple")
    lines["stuck_avg_speed"] = build_plot(axes[0], [], [], "Avg10Speed", "tab:red", 1.4)
    lines["accel"] = build_plot(axes[0], [], [], "Accel", "tab:green")
    lines["set"] = build_plot(axes[0], [], [], "Setpoint", "tab:orange", 1.2)
    axes[0].set_ylabel("cm")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)
    apply_limits(axes[0], None if args.auto_scale else args.dist_lim)

    lines["err"] = build_plot(axes[1], [], [], "Err", "tab:red")
    lines["int"] = build_plot(axes[1], [], [], "Int", "tab:purple")
    lines["deriv"] = build_plot(axes[1], [], [], "Deriv", "tab:cyan")
    axes[1].set_ylabel("error")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)
    apply_limits(axes[1], None if args.auto_scale else args.err_lim)

    lines["out"] = build_plot(axes[2], [], [], "Out", "tab:green")
    lines["servo"] = build_plot(axes[2], [], [], "Servo", "tab:brown")
    lines["pout"] = build_plot(axes[2], [], [], "POut", "tab:olive")
    lines["dout"] = build_plot(axes[2], [], [], "DOut", "tab:cyan")
    lines["iout"] = build_plot(axes[2], [], [], "IOut", "tab:pink")
    axes[2].set_ylabel("output / deg")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)
    apply_limits(axes[2], None if args.auto_scale else args.out_lim)

    lines["volt"] = build_plot(axes[3], [], [], "Volt", "tab:gray")
    lines["static"] = build_plot(axes[3], [], [], "Static", "tab:blue", 1.2)
    lines["settle_active"] = build_plot(axes[3], [], [], "Settle", "tab:green", 1.2)
    lines["dither_active"] = build_plot(axes[3], [], [], "Dither", "tab:red", 1.4)
    lines["breakaway_active"] = build_plot(axes[3], [], [], "Break", "tab:orange", 1.2)
    lines["friction_active"] = build_plot(axes[3], [], [], "Friction", "tab:purple", 1.2)
    axes[3].set_ylabel("V / state")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.25)
    apply_limits(axes[3], None if args.auto_scale else args.volt_lim)

    status = fig.text(0.01, 0.015, f"Port: {args.port}", fontsize=9)
    gains_text = fig.text(0.50, 0.015, "Set: -- | Kp: -- | Ki: -- | Kd: -- | Servo: --", fontsize=9, ha="center")
    loopcount_text = fig.text(0.99, 0.015, "Loop: -- | ADC: -- | Ctrl: --", fontsize=9, ha="right")
    help_text = fig.text(0.01, 0.96, 
        "Click chart to enlarge  •  Scroll wheel to zoom  •  ↑↓ to pan  •  +/- to zoom when enlarged  •  R to reset", 
        fontsize=8, style="italic", color="darkgray")

    sliders = {}
    slider_guard = {"enabled": True}
    controls_visible = {"value": True}

    if not args.no_controls:
        # Parameter metadata: unit and explanation
        param_info = {
            "setpoint": ("cm", "Desired ball-center distance from sensor. Positive error means the ball is too far right/away."),
            "kp": ("gain", "Proportional gain. Higher = faster response to distance error."),
            "ki": ("gain", "Integral gain. Eliminates steady-state error over time."),
            "kd": ("gain", "Derivative gain. Dampens oscillations and smooths response."),
            "servo": ("°", "Manual servo angle override. Range 54-114° (±30° from neutral)."),
            "neutral": ("°", "Servo neutral angle at zero beam tilt. Default 84°."),
            "travel": ("°", "Maximum servo travel range in each direction from neutral."),
            "dir": ("±1", "Control direction. 1=forward, -1=reverse. Flips command sign."),
            "distalpha": ("0-1", "Distance filter new-sample weight. Higher reacts faster; lower removes more sensor noise."),
            "speedalpha": ("0-1", "Speed filter new-sample weight used by the D term."),
            "intlimit": ("cm s", "Integral clamp. Limits windup while retaining enough authority to remove steady offset."),
            "maxaccel": ("m/s²", "Maximum acceleration the PID may request from the motion model."),
            "servorate": ("°/cycle", "Servo rate limiter. Maximum angle change per control cycle."),
            "servodead": ("°", "Servo deadband. Minimum change before sending a new servo command. Higher reduces chatter; lower reacts sooner."),
            "samplems": ("ms", "ADC sample interval. Samples are collected continuously; PID still runs at a stable 50 Hz."),
            "mass": ("kg", "Ball mass. It affects the force needed to overcome friction."),
            "gravity": ("m/s²", "Gravity used by the rolling-ball motion equation."),
            "rollfactor": ("ratio", "Effective rolling inertia: 1 + I/(m r²). A solid sphere is 1.4."),
            "friction": ("N", "Estimated rolling/static friction force opposing commanded motion."),
            "frictionblend": ("m/s²", "Softens friction compensation around zero acceleration to prevent chatter."),
            "beamgain": ("ratio", "Beam angle divided by servo angle. Geometry estimate is about 0.114."),
            "curvea": ("coefficient", "GP2Y0A41SK0F curve coefficient in distance = A × voltage^exponent."),
            "curveexp": ("exponent", "GP2Y0A41SK0F voltage-to-distance curve exponent."),
            "calscale": ("ratio", "Beam-scale correction applied to the full ball-center distance."),
            "caloffset": ("cm", "Beam-scale offset applied after CalScale."),
            "ballradius": ("cm", "Added sensor-to-surface correction so control uses the ball center."),
            "settleerror": ("cm", "10-sample deadband error threshold. Default 0.8 cm equals the requested 8 mm target."),
            "settlespeed": ("cm/s", "10-sample average-speed threshold used with SettleError."),
        }

        slider_specs = [
            ("setpoint", "Set", 0.0, 30.0, 16.0, 0.1, "%.2f", "set"),
            ("kp", "Kp", 0.0, 20.0, 3.0, 0.05, "%.2f", "kp"),
            ("ki", "Ki", 0.0, 10.0, 0.15, 0.01, "%.2f", "ki"),
            ("kd", "Kd", 0.0, 10.0, 3.5, 0.05, "%.2f", "kd"),
            ("servo", "Angle", 54.0, 114.0, 84.0, 0.1, "%.1f", "angle"),
            ("neutral", "Neutral", 0.0, 180.0, 84.0, 0.1, "%.1f", "neutral"),
            ("travel", "Travel", 1.0, 60.0, 25.0, 0.5, "%.1f", "travel"),
            ("dir", "Dir", -1.0, 1.0, -1.0, 2.0, "%.0f", "dir"),
            ("distalpha", "DistAlpha", 0.01, 1.0, 0.35, 0.01, "%.2f", "distalpha"),
            ("speedalpha", "SpeedAlpha", 0.01, 1.0, 0.15, 0.01, "%.2f", "speedalpha"),
            ("intlimit", "IntLimit", 0.0, 50.0, 12.0, 0.5, "%.1f", "intlimit"),
            ("maxaccel", "MaxAccel", 0.05, 5.0, 1.5, 0.05, "%.2f", "maxaccel"),
            ("servorate", "SrvRate", 0.01, 10.0, 1.5, 0.01, "%.2f", "servorate"),
            ("servodead", "SrvDead", 0.0, 2.0, 0.02, 0.01, "%.2f", "servodead"),
            ("samplems", "SampleMs", 0.25, 20.0, 1.0, 0.25, "%.2f", "samplems"),
            ("mass", "Mass", 0.001, 0.5, 0.0455, 0.001, "%.3f", "mass"),
            ("gravity", "Gravity", 1.0, 15.0, 9.81, 0.01, "%.2f", "gravity"),
            ("rollfactor", "RollFactor", 1.0, 3.0, 1.4, 0.05, "%.2f", "rollfactor"),
            ("friction", "Friction", 0.0, 0.1, 0.0015, 0.0005, "%.4f", "friction"),
            ("frictionblend", "FricBlend", 0.001, 0.5, 0.08, 0.005, "%.3f", "frictionblend"),
            ("beamgain", "BeamGain", 0.01, 0.5, 0.114, 0.001, "%.3f", "beamgain"),
            ("curvea", "CurveA", 5.0, 20.0, 12.08, 0.01, "%.2f", "curvea"),
            ("curveexp", "CurveExp", -2.0, -0.5, -1.058, 0.001, "%.3f", "curveexp"),
            ("calscale", "CalScale", 0.5, 1.5, 1.333333, 0.001, "%.3f", "calscale"),
            ("caloffset", "CalOffset", -10.0, 10.0, -3.333333, 0.01, "%.2f", "caloffset"),
            ("ballradius", "BallRadius", 0.0, 3.0, 0.75, 0.05, "%.2f", "ballradius"),
            ("settleerror", "SettleErr", 0.0, 3.0, 0.8, 0.05, "%.2f", "settleerror"),
            ("settlespeed", "SettleSpeed", 0.0, 5.0, 0.5, 0.05, "%.2f", "settlespeed"),
        ]

        slider_axes = {}
        info_axes = {}
        info_buttons = {}
        left_x = 0.03
        right_x = 0.52
        width = 0.24
        height = 0.020
        top_row = 0.35
        row_gap = 0.027
        cols = [left_x, right_x]
        info_width = 0.018
        info_gap = 0.050
        button_x = 0.945
        button_width = 0.040

        def create_info_popup(key: str) -> None:
            """Display parameter info in a popup window."""
            if key not in param_info:
                return
            unit, explanation = param_info[key]
            
            # Create a simple info window using matplotlib text
            info_window = plt.figure(figsize=(6, 3))
            info_window.suptitle(f"Parameter Info: {key.upper()}", fontsize=12, fontweight="bold")
            
            ax = info_window.add_subplot(111)
            ax.axis("off")
            
            # Display unit and explanation
            info_text = f"Unit: {unit}\n\nExplanation:\n{explanation}"
            ax.text(0.5, 0.5, info_text, ha="center", va="center", fontsize=11, 
                   wrap=True, bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))
            
            plt.tight_layout()
            plt.show(block=False)

        for idx, spec in enumerate(slider_specs):
            key, label, vmin, vmax, valinit, valstep, valfmt, _command = spec
            col = idx % 2
            row = idx // 2
            axis = fig.add_axes([cols[col], top_row - row * row_gap, width, height])
            slider_axes[key] = axis
            sliders[key] = Slider(axis, label, vmin, vmax, valinit=valinit, valstep=valstep, valfmt=valfmt)
            sliders[key].valtext.set_fontsize(9)
            
            info_x = cols[col] + width + info_gap
            info_y = top_row - row * row_gap
            info_axis = fig.add_axes([info_x, info_y, info_width, height])
            info_axes[key] = info_axis
            info_button = Button(info_axis, "?")
            info_button.label.set_fontsize(10)
            info_button.on_clicked(lambda _event, param_key=key: create_info_popup(param_key))
            info_buttons[key] = info_button

        def set_controls_visible(visible: bool) -> None:
            controls_visible["value"] = visible
            for axis in slider_axes.values():
                axis.set_visible(visible)
            for axis in info_axes.values():
                axis.set_visible(visible)
            help_text.set_visible(visible)
            fig.subplots_adjust(bottom=0.42 if visible else 0.10, top=plot_top)
            fig.canvas.draw_idle()

        set_controls_visible(True)

        for slider in sliders.values():
            slider.valtext.set_fontsize(9)

        button_axes = {
            "source": fig.add_axes([button_x, 0.38, button_width, 0.05]),
            "controls": fig.add_axes([button_x, 0.31, button_width, 0.05]),
            "auto": fig.add_axes([button_x, 0.24, button_width, 0.05]),
            "manual": fig.add_axes([button_x, 0.17, button_width, 0.05]),
            "reset": fig.add_axes([button_x, 0.10, button_width, 0.05]),
            "all": fig.add_axes([button_x, 0.03, button_width, 0.05]),
        }
        buttons = {
            "source": Button(button_axes["source"], "SENSOR"),
            "controls": Button(button_axes["controls"], "HIDE"),
            "auto": Button(button_axes["auto"], "AUTO"),
            "manual": Button(button_axes["manual"], "MANUAL"),
            "reset": Button(button_axes["reset"], "RESET"),
            "all": Button(button_axes["all"], "ALL"),
        }

        vision_state = {
            "enabled": False,
            "capture": None,
            "window": "Ball vision source",
            "hsv_low": np.array(args.vision_hsv_low, dtype=np.uint8) if vision_available() else None,
            "hsv_high": np.array(args.vision_hsv_high, dtype=np.uint8) if vision_available() else None,
            "last_sent_cm": None,
        }

        def open_vision_capture() -> bool:
            if not vision_available():
                return False
            if vision_state["capture"] is None:
                capture = cv2.VideoCapture(args.vision_camera)
                if not capture.isOpened():
                    return False
                vision_state["capture"] = capture
            return True

        def close_vision_capture() -> None:
            capture = vision_state["capture"]
            if capture is not None:
                capture.release()
            vision_state["capture"] = None
            if cv2 is not None:
                try:
                    cv2.destroyWindow(vision_state["window"])
                except Exception:
                    pass

        def set_source_mode(enabled: bool) -> None:
            vision_state["enabled"] = enabled
            vision_state["last_sent_cm"] = None
            buttons["source"].label.set_text("CV" if enabled else "SENSOR")
            try:
                send_command(ser, "source:vision" if enabled else "source:sensor")
            except serial.SerialException:
                pass

            if enabled:
                if not open_vision_capture():
                    vision_state["enabled"] = False
                    buttons["source"].label.set_text("SENSOR")
                    try:
                        status.set_text("OpenCV camera unavailable; staying on SENSOR")
                    except Exception:
                        pass
                    return
            else:
                close_vision_capture()

        def send_vision_position(position_cm: float) -> None:
            if vision_state["last_sent_cm"] is not None and abs(position_cm - vision_state["last_sent_cm"]) < 0.05:
                return
            vision_state["last_sent_cm"] = position_cm
            try:
                send_command(ser, f"vision:{position_cm:.2f}")
            except serial.SerialException:
                pass

        def update_vision_source() -> None:
            if not vision_state["enabled"] or vision_state["capture"] is None or cv2 is None or np is None:
                return

            ret, frame = vision_state["capture"].read()
            if not ret:
                send_vision_position(-1.0)
                return

            detection, mask = detect_ball(frame, vision_state["hsv_low"], vision_state["hsv_high"])
            preview = frame.copy()

            if detection is None:
                send_vision_position(-1.0)
                cv2.putText(preview, "Ball: lost", (12, 28), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 0, 255), 2)
            else:
                center_x, center_y, radius = detection
                position_cm = (center_x / max(1, frame.shape[1])) * args.vision_span_cm
                send_vision_position(position_cm)

                cv2.circle(preview, (center_x, center_y), radius, (0, 255, 0), 2)
                cv2.circle(preview, (center_x, center_y), 4, (0, 255, 255), -1)
                cv2.line(preview, (center_x, 0), (center_x, preview.shape[0]), (255, 0, 0), 1)
                cv2.putText(preview, f"Ball: {position_cm:.2f} cm", (12, 28), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(preview, f"HSV low: {tuple(int(v) for v in vision_state['hsv_low'])}",
                        (12, preview.shape[0] - 28), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(preview, f"HSV high: {tuple(int(v) for v in vision_state['hsv_high'])}",
                        (12, preview.shape[0] - 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(vision_state["window"], preview)
            cv2.waitKey(1)

        overview_state = {"fig": None, "axis": None, "lines": {}, "text": None}

        def open_all_in_one_view(_event=None):
            existing_fig = overview_state["fig"]
            if existing_fig is not None and plt.fignum_exists(existing_fig.number):
                try:
                    existing_fig.canvas.manager.show()
                except Exception:
                    pass
                return

            overview_fig, overview_ax = plt.subplots(figsize=(14, 8))
            overview_fig.canvas.manager.set_window_title("ESP32 all-in-one telemetry")
            overview_fig.suptitle("ESP32 all-in-one telemetry")
            overview_fig.subplots_adjust(top=0.90, bottom=0.10, left=0.07, right=0.98)

            overview_lines = {}
            overview_lines["dist"] = build_plot(overview_ax, [], [], "Dist", "tab:blue")
            overview_lines["distf"] = build_plot(overview_ax, [], [], "DistF", "tab:cyan")
            overview_lines["speed"] = build_plot(overview_ax, [], [], "Speed", "tab:purple")
            overview_lines["stuck_avg_speed"] = build_plot(overview_ax, [], [], "StaticAvg", "tab:red", 1.4)
            overview_lines["accel"] = build_plot(overview_ax, [], [], "Accel", "tab:green")
            overview_lines["set"] = build_plot(overview_ax, [], [], "Setpoint", "tab:orange", 1.2)
            overview_lines["err"] = build_plot(overview_ax, [], [], "Err", "tab:red")
            overview_lines["int"] = build_plot(overview_ax, [], [], "Int", "tab:purple")
            overview_lines["deriv"] = build_plot(overview_ax, [], [], "Deriv", "tab:olive")
            overview_lines["pout"] = build_plot(overview_ax, [], [], "POut", "tab:pink")
            overview_lines["dout"] = build_plot(overview_ax, [], [], "DOut", "tab:brown")
            overview_lines["iout"] = build_plot(overview_ax, [], [], "IOut", "tab:gray")
            overview_lines["out"] = build_plot(overview_ax, [], [], "Out", "tab:green")
            overview_lines["servo"] = build_plot(overview_ax, [], [], "Servo", "tab:blue", 1.2)
            overview_lines["volt"] = build_plot(overview_ax, [], [], "Volt", "tab:cyan")
            overview_lines["raw"] = build_plot(overview_ax, [], [], "Raw", "tab:purple")
            overview_lines["loopcount"] = build_plot(overview_ax, [], [], "Loop/s", "tab:orange")
            overview_lines["adc_count"] = build_plot(overview_ax, [], [], "ADC/s", "tab:red")
            overview_lines["control_count"] = build_plot(overview_ax, [], [], "Ctrl/s", "tab:olive")
            overview_lines["static"] = build_plot(overview_ax, [], [], "Static", "black", 1.2)
            overview_lines["settle_active"] = build_plot(overview_ax, [], [], "Settle", "tab:green", 1.2)
            overview_lines["dither_active"] = build_plot(overview_ax, [], [], "Dither", "tab:red", 1.4)
            overview_lines["breakaway_active"] = build_plot(overview_ax, [], [], "Break", "tab:orange", 1.2)
            overview_lines["friction_active"] = build_plot(overview_ax, [], [], "Friction", "tab:purple", 1.2)

            overview_ax.set_ylabel("mixed units")
            overview_ax.set_xlabel("Time (s)")
            overview_ax.legend(loc="upper right", ncol=4, fontsize=8)
            overview_ax.grid(True, alpha=0.25)

            overview_text = overview_fig.text(0.01, 0.01, "", fontsize=9, family="monospace")

            overview_state["fig"] = overview_fig
            overview_state["axis"] = overview_ax
            overview_state["lines"] = overview_lines
            overview_state["text"] = overview_text

            def close_overview(_event):
                overview_state["fig"] = None
                overview_state["axis"] = None
                overview_state["lines"] = {}
                overview_state["text"] = None

            overview_fig.canvas.mpl_connect("close_event", close_overview)

        def update_all_in_one_view() -> None:
            overview_fig = overview_state["fig"]
            overview_axis = overview_state["axis"]
            overview_lines = overview_state["lines"]
            overview_text = overview_state["text"]
            if overview_fig is None or overview_axis is None or not buf.time_s:
                return

            x = list(buf.time_s)
            overview_lines["dist"].set_data(x, list(buf.dist))
            overview_lines["distf"].set_data(x, list(buf.distf))
            overview_lines["speed"].set_data(x, list(buf.speed))
            overview_lines["stuck_avg_speed"].set_data(x, list(buf.stuck_avg_speed))
            overview_lines["accel"].set_data(x, list(buf.accel))
            overview_lines["set"].set_data(x, list(buf.setpoint))

            overview_lines["err"].set_data(x, list(buf.err))
            overview_lines["int"].set_data(x, list(buf.integral))
            overview_lines["deriv"].set_data(x, list(buf.deriv))

            overview_lines["pout"].set_data(x, list(buf.pout))
            overview_lines["dout"].set_data(x, list(buf.dout))
            overview_lines["iout"].set_data(x, list(buf.iout))
            overview_lines["out"].set_data(x, list(buf.output))

            overview_lines["servo"].set_data(x, list(buf.servo))
            overview_lines["volt"].set_data(x, list(buf.volt))
            overview_lines["raw"].set_data(x, list(buf.raw))

            overview_lines["loopcount"].set_data(x, list(buf.loopcount))
            overview_lines["adc_count"].set_data(x, list(buf.adc_count))
            overview_lines["control_count"].set_data(x, list(buf.control_count))
            overview_lines["static"].set_data(x, state_pulses(buf.static, 0.5))
            overview_lines["settle_active"].set_data(x, state_pulses(buf.settle_active, 1.0))
            overview_lines["dither_active"].set_data(x, state_motion(buf.dither_active, buf.dither_deg, 1.5))
            overview_lines["breakaway_active"].set_data(x, state_motion(buf.breakaway_active, buf.break_deg, 2.0))
            overview_lines["friction_active"].set_data(x, state_pulses(buf.friction_active, 2.5))

            overview_axis.relim()
            overview_axis.autoscale_view()

            if overview_text is not None:
                overview_text.set_text(
                    f"Neutral={buf.neutral[-1]:.0f}  Travel={buf.travel[-1]:.0f}  Dir={buf.direction[-1]:.0f}  "
                    f"PidDead={buf.pid_dead[-1]:.2f}  SettleErr={buf.settle_err[-1]:.2f}  "
                    f"SettleDeriv={buf.settle_deriv[-1]:.3f}  ServoFilt={buf.servo_filt[-1]:.3f}  "
                    f"ServoRate={buf.servo_rate[-1]:.2f}  Breakaway={buf.breakaway[-1]:.2f}  "
                    f"Dither={buf.dither_amp[-1]:.2f}@{buf.dither_freq[-1]:.1f}Hz  "
                    f"StuckAvg={buf.stuck_avg_speed[-1]:.3f} Threshold={buf.stuck_speed[-1]:.2f}/{buf.stuck_band[-1]:.2f}  "
                    f"ServoDead={buf.servo_dead[-1]:.2f}  "
                    f"Static={int(buf.static[-1])} Settle={int(buf.settle_active[-1])} "
                    f"Dither={buf.dither_deg[-1]:.3f}deg Break={buf.break_deg[-1]:.3f}deg "
                    f"Friction={int(buf.friction_active[-1])}  "
                    f"On D/B/Dist/Deriv/PID/Srv={int(buf.dither_on[-1])}/{int(buf.break_on[-1])}/"
                    f"{int(buf.dist_filter_on[-1])}/{int(buf.deriv_filter_on[-1])}/"
                    f"{int(buf.pid_filter_on[-1])}/{int(buf.servo_filter_on[-1])}"
                )

            overview_fig.canvas.draw_idle()

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
        sliders["servo"].on_changed(lambda value: send_from_slider("angle", value, "{:.1f}"))
        sliders["neutral"].on_changed(lambda value: send_from_slider("neutral", value, "{:.1f}"))
        sliders["travel"].on_changed(lambda value: send_from_slider("travel", value, "{:.1f}"))
        sliders["dir"].on_changed(lambda value: send_from_slider("dir", value, "{:.0f}"))
        sliders["distalpha"].on_changed(lambda value: send_from_slider("distalpha", value, "{:.2f}"))
        sliders["speedalpha"].on_changed(lambda value: send_from_slider("speedalpha", value, "{:.2f}"))
        sliders["intlimit"].on_changed(lambda value: send_from_slider("intlimit", value, "{:.1f}"))
        sliders["maxaccel"].on_changed(lambda value: send_from_slider("maxaccel", value, "{:.2f}"))
        sliders["servorate"].on_changed(lambda value: send_from_slider("servorate", value, "{:.2f}"))
        sliders["servodead"].on_changed(lambda value: send_from_slider("servodead", value, "{:.2f}"))
        sliders["samplems"].on_changed(lambda value: send_from_slider("samplems", value, "{:.2f}"))
        sliders["mass"].on_changed(lambda value: send_from_slider("mass", value, "{:.4f}"))
        sliders["gravity"].on_changed(lambda value: send_from_slider("gravity", value, "{:.3f}"))
        sliders["rollfactor"].on_changed(lambda value: send_from_slider("rollfactor", value, "{:.3f}"))
        sliders["friction"].on_changed(lambda value: send_from_slider("friction", value, "{:.5f}"))
        sliders["frictionblend"].on_changed(lambda value: send_from_slider("frictionblend", value, "{:.3f}"))
        sliders["beamgain"].on_changed(lambda value: send_from_slider("beamgain", value, "{:.4f}"))
        sliders["curvea"].on_changed(lambda value: send_from_slider("curvea", value, "{:.4f}"))
        sliders["curveexp"].on_changed(lambda value: send_from_slider("curveexp", value, "{:.4f}"))
        sliders["calscale"].on_changed(lambda value: send_from_slider("calscale", value, "{:.5f}"))
        sliders["caloffset"].on_changed(lambda value: send_from_slider("caloffset", value, "{:.3f}"))
        sliders["ballradius"].on_changed(lambda value: send_from_slider("ballradius", value, "{:.2f}"))
        sliders["settleerror"].on_changed(lambda value: send_from_slider("settleerror", value, "{:.2f}"))
        sliders["settlespeed"].on_changed(lambda value: send_from_slider("settlespeed", value, "{:.2f}"))

        def toggle_controls(_event):
            new_state = not controls_visible["value"]
            set_controls_visible(new_state)
            buttons["controls"].label.set_text("HIDE" if new_state else "SHOW")

        def toggle_source(_event):
            set_source_mode(not vision_state["enabled"])

        buttons["controls"].on_clicked(toggle_controls)
        buttons["source"].on_clicked(toggle_source)
        buttons["auto"].on_clicked(lambda _event: send_command(ser, "auto"))
        buttons["manual"].on_clicked(lambda _event: send_command(ser, "manual:on"))
        buttons["reset"].on_clicked(lambda _event: send_command(ser, "reset"))
        buttons["all"].on_clicked(open_all_in_one_view)

    else:
        # When controls are hidden, also hide help text
        help_text.set_visible(False)

    def on_click(event):
        """Toggle enlarged view when clicking on a plot."""
        if event.inaxes is None:
            return
        
        clicked_idx = None
        for idx, ax in enumerate(axes):
            if event.inaxes == ax:
                clicked_idx = idx
                break
        
        if clicked_idx is None:
            return
        
        # Store original positions if not already stored
        if not hasattr(on_click, 'original_positions'):
            on_click.original_positions = [ax.get_position() for ax in axes]
        
        # Toggle: if this axis is enlarged, show all; otherwise, enlarge this one
        if enlarged_axis[0] == clicked_idx:
            # Return to normal 4-subplot view
            enlarged_axis[0] = None
            for idx, ax in enumerate(axes):
                ax.set_visible(True)
                ax.set_position(on_click.original_positions[idx])
                ax.set_xlabel("Time (s)" if idx == len(axes) - 1 else "")
            fig.subplots_adjust(bottom=plot_bottom, top=plot_top, hspace=0.3)
        else:
            # Enlarge this one to fill most of the figure
            enlarged_axis[0] = clicked_idx
            for idx, ax in enumerate(axes):
                ax.set_visible(idx == clicked_idx)
                ax.set_xlabel("Time (s)" if idx == clicked_idx else "")
            # Set position to fill figure (left, bottom, width, height in figure coords 0-1)
            axes[clicked_idx].set_position([0.08, plot_bottom, 0.88, plot_top - plot_bottom])
        
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_click)

    def on_key(event):
        """Handle keyboard shortcuts for axis scaling."""
        if enlarged_axis[0] is None or event.key is None:
            return
        
        ax = axes[enlarged_axis[0]]
        ymin, ymax = ax.get_ylim()
        ymid = (ymin + ymax) / 2
        yrange = ymax - ymin
        
        if event.key in ['+', '=']:
            # Zoom in (narrow range)
            new_range = yrange * 0.8
            ax.set_ylim(ymid - new_range / 2, ymid + new_range / 2)
            fig.canvas.draw_idle()
        elif event.key in ['-', '_']:
            # Zoom out (widen range)
            new_range = yrange * 1.2
            ax.set_ylim(ymid - new_range / 2, ymid + new_range / 2)
            fig.canvas.draw_idle()
        elif event.key == 'r':
            # Reset to auto scale
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw_idle()
        elif event.key in ['up', 'down']:
            # Pan up/down with arrow keys
            pan_amount = yrange * 0.1 if event.key == 'up' else -yrange * 0.1
            ax.set_ylim(ymin + pan_amount, ymax + pan_amount)
            fig.canvas.draw_idle()
    
    def on_scroll(event):
        """Handle mouse wheel zooming on any chart."""
        if event.inaxes is None:
            return
        
        # Find which axis was scrolled on
        clicked_idx = None
        for idx, ax in enumerate(axes):
            if event.inaxes == ax:
                clicked_idx = idx
                break
        
        if clicked_idx is None:
            return
        
        ax = axes[clicked_idx]
        ymin, ymax = ax.get_ylim()
        ymid = (ymin + ymax) / 2
        yrange = ymax - ymin
        
        # Scroll up = zoom in, scroll down = zoom out
        if event.button == 'up':
            new_range = yrange * 0.85
        elif event.button == 'down':
            new_range = yrange * 1.15
        else:
            return
        
        ax.set_ylim(ymid - new_range / 2, ymid + new_range / 2)
        fig.canvas.draw_idle()
    
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    def update(_frame):
        updated = False
        while ser.in_waiting:
            raw_line = ser.readline().decode(errors="ignore")
            fields = parse_line(raw_line)
            if fields is None:
                continue
            buf.append(fields)
            updated = True

        if not args.no_controls:
            update_vision_source()

        if not buf.time_s:
            return tuple(lines.values())

        x = list(buf.time_s)
        lines["dist"].set_data(x, list(buf.dist))
        lines["distf"].set_data(x, list(buf.distf))
        lines["speed"].set_data(x, list(buf.speed))
        lines["stuck_avg_speed"].set_data(x, list(buf.stuck_avg_speed))
        lines["accel"].set_data(x, list(buf.accel))
        lines["set"].set_data(x, list(buf.setpoint))
        lines["err"].set_data(x, list(buf.err))
        lines["int"].set_data(x, list(buf.integral))
        lines["deriv"].set_data(x, list(buf.deriv))
        lines["out"].set_data(x, list(buf.output))
        lines["servo"].set_data(x, list(buf.servo))
        lines["pout"].set_data(x, list(buf.pout))
        lines["dout"].set_data(x, list(buf.dout))
        lines["iout"].set_data(x, list(buf.iout))
        lines["volt"].set_data(x, list(buf.volt))
        lines["static"].set_data(x, state_pulses(buf.static, 0.5))
        lines["settle_active"].set_data(x, state_pulses(buf.settle_active, 1.0))
        lines["dither_active"].set_data(x, state_motion(buf.dither_active, buf.dither_deg, 1.5))
        lines["breakaway_active"].set_data(x, state_motion(buf.breakaway_active, buf.break_deg, 2.0))
        lines["friction_active"].set_data(x, state_pulses(buf.friction_active, 2.5))

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
                    sliders["neutral"].set_val(buf.neutral[-1])
                    sliders["travel"].set_val(buf.travel[-1])
                    sliders["dir"].set_val(buf.direction[-1])
                    sliders["distalpha"].set_val(buf.dist_alpha[-1])
                    sliders["speedalpha"].set_val(buf.speed_alpha[-1])
                    sliders["intlimit"].set_val(buf.int_limit[-1])
                    sliders["maxaccel"].set_val(buf.max_accel[-1])
                    sliders["servorate"].set_val(buf.servo_rate[-1])
                    sliders["servodead"].set_val(buf.servo_dead[-1])
                    sliders["samplems"].set_val(buf.sample_ms[-1])
                    sliders["mass"].set_val(buf.mass[-1])
                    sliders["gravity"].set_val(buf.gravity[-1])
                    sliders["rollfactor"].set_val(buf.roll_factor[-1])
                    sliders["friction"].set_val(buf.friction[-1])
                    sliders["frictionblend"].set_val(buf.friction_blend[-1])
                    sliders["beamgain"].set_val(buf.beam_gain[-1])
                    sliders["curvea"].set_val(buf.curve_a[-1])
                    sliders["curveexp"].set_val(buf.curve_exp[-1])
                    sliders["calscale"].set_val(buf.cal_scale[-1])
                    sliders["caloffset"].set_val(buf.cal_offset[-1])
                    sliders["ballradius"].set_val(buf.ball_radius[-1])
                    sliders["settleerror"].set_val(buf.settle_err[-1])
                    sliders["settlespeed"].set_val(buf.settle_deriv[-1])
                finally:
                    slider_guard["enabled"] = True

            status.set_text(
                f"Port: {args.port}  Mode: {'MAN' if buf.mode and buf.mode[-1] else 'AUTO'}  "
                f"Src: {'CV' if buf.source and buf.source[-1] else 'SENSOR'}  "
                f"Dist: {buf.dist[-1]:.2f}  DistF: {buf.distf[-1]:.2f}  Err: {buf.err[-1]:.2f}  "
                f"Speed: {buf.speed[-1]:.2f}  StaticAvg: {buf.stuck_avg_speed[-1]:.2f}  Servo: {buf.servo[-1]:.1f}  "
                f"Static:{int(buf.static[-1])} Settle:{int(buf.settle_active[-1])} "
                f"Dither:{buf.dither_deg[-1]:.3f}deg Break:{buf.break_deg[-1]:.3f}deg"
            )
            gains_text.set_text(
                f"Set: {buf.setpoint[-1]:.2f} | Kp: {buf.kp[-1]:.2f} | Ki: {buf.ki[-1]:.3f} | Kd: {buf.kd[-1]:.2f} | "
                f"Neutral: {buf.neutral[-1]:.0f} | Travel: {buf.travel[-1]:.0f} | Dir: {buf.direction[-1]:.0f} | "
                f"Out: {buf.output[-1]:.1f} | Friction:{int(buf.friction_active[-1])}"
            )
            loopcount_text.set_text(
                f"Loop/s: {int(buf.loopcount[-1])} | ADC/s: {int(buf.adc_count[-1])} | Ctrl/s: {int(buf.control_count[-1])}"
            )

            update_all_in_one_view()

        return tuple(lines.values())

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()
    _ = anim

    if not args.no_controls and 'vision_state' in locals():
        close_vision_capture()

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
