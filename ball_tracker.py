"""
ball_tracker.py  –  Real-time ball tracking with OpenCV
========================================================
Project: Positieregeling van een Knikkerbaan
Detects a ball via HSV colour tracking, draws overlay graphics,
and prints position / velocity in real time.

Usage
-----
    python ball_tracker.py                  # auto-detect colour on first click
    python ball_tracker.py --camera 1       # use camera index 1
    python ball_tracker.py --calibrate      # re-run colour calibration
    python ball_tracker.py --scale 0.37     # manually set cm/pixel scale

Controls (while running)
------------------------
    c        – re-calibrate colour (click ball in calibration window)
    s        – set scale: click two known points on the beam, enter real distance
    r        – reset trajectory trail
    q / ESC  – quit

Dependencies
------------
    pip install opencv-python numpy
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")   # force X11 – avoids Wayland/Qt null-window crash

import cv2
import numpy as np
import argparse
import time
import math
from collections import deque

# ── Tuneable defaults ────────────────────────────────────────────────────────
DEFAULT_CAMERA      = 0
TRAIL_LENGTH        = 60        # frames to keep in position trail
MIN_BALL_RADIUS_PX  = 5         # ignore detections smaller than this
MAX_BALL_RADIUS_PX  = 120
BLUR_KERNEL         = 11        # must be odd; larger = less noise, slower response
MORPH_KERNEL        = 5

# Default HSV range – will be overridden by tuner or calibration
DEFAULT_HSV_LOW  = np.array([0,   0,   0],  dtype=np.uint8)
DEFAULT_HSV_HIGH = np.array([179, 255, 255], dtype=np.uint8)

# ── Colour palette ────────────────────────────────────────────────────────────
C_CIRCLE    = (0,  220, 255)   # cyan-yellow  – ball outline
C_CENTER    = (0,  255, 120)   # green        – ball centre dot
C_TRAIL     = (80, 180, 255)   # soft orange  – trail
C_TEXT_BG   = (20,  20,  20)   # near-black   – text background
C_TEXT_FG   = (220,220,220)    # light grey   – text
C_SETPOINT  = (60, 180, 255)   # orange-ish   – setpoint line
C_CROSS     = (0,  100, 255)   # bright red   – crosshair
C_VELOCITY  = (120, 255, 80)   # lime green   – velocity arrow


def parse_args():
    p = argparse.ArgumentParser(description="Ball tracker for Knikkerbaan project")
    p.add_argument("--camera",    type=int,   default=DEFAULT_CAMERA)
    p.add_argument("--calibrate", action="store_true", help="Force colour recalibration")
    p.add_argument("--tune",      action="store_true", help="Open live HSV tuner (recommended first step)")
    p.add_argument("--scale",     type=float, default=None,
                   help="cm per pixel (skip interactive calibration)")
    p.add_argument("--width",     type=int,   default=1280)
    p.add_argument("--height",    type=int,   default=720)
    return p.parse_args()


# ── Live HSV tuner ────────────────────────────────────────────────────────────

def hsv_tuner(cap):
    """
    Opens a live tuner window with 6 trackbars (H/S/V low & high).
    Shows the camera feed AND the resulting mask side by side.
    Press SPACE to accept, ESC to cancel.
    Returns (hsv_low, hsv_high) or None on cancel.
    """
    WIN = "HSV Tuner - adjust sliders until ball is WHITE in mask, press SPACE"
    cv2.imshow(WIN, np.zeros((100, 800, 3), dtype=np.uint8))
    cv2.waitKey(200)

    cv2.createTrackbar("H low",  WIN,   0, 179, lambda x: None)
    cv2.createTrackbar("H high", WIN, 179, 179, lambda x: None)
    cv2.createTrackbar("S low",  WIN,   0, 255, lambda x: None)
    cv2.createTrackbar("S high", WIN, 255, 255, lambda x: None)
    cv2.createTrackbar("V low",  WIN,   0, 255, lambda x: None)
    cv2.createTrackbar("V high", WIN, 255, 255, lambda x: None)

    print("\n[tuner] Adjust sliders until ONLY the ball appears white in the mask.")
    print("        Press SPACE to accept, ESC to cancel.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h_lo = cv2.getTrackbarPos("H low",  WIN)
        h_hi = cv2.getTrackbarPos("H high", WIN)
        s_lo = cv2.getTrackbarPos("S low",  WIN)
        s_hi = cv2.getTrackbarPos("S high", WIN)
        v_lo = cv2.getTrackbarPos("V low",  WIN)
        v_hi = cv2.getTrackbarPos("V high", WIN)

        low  = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
        high = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low, high)

        # Morphological cleanup so you see what the detector sees
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        mask_bgr  = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Overlay green contours on camera feed
        overlay = frame.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, (0, 255, 0), 2)

        # Resize both to same height and place side by side
        h, w = frame.shape[:2]
        disp_w = min(w, 640)
        disp_h = int(h * disp_w / w)
        left  = cv2.resize(overlay,   (disp_w, disp_h))
        right = cv2.resize(mask_bgr,  (disp_w, disp_h))

        # Labels
        cv2.putText(left,  "Camera (green=detected)",
                    (10, 25), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(right, "Mask (ball should be WHITE)",
                    (10, 25), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(right, f"HSV [{h_lo}-{h_hi}, {s_lo}-{s_hi}, {v_lo}-{v_hi}]",
                    (10, 50), cv2.FONT_HERSHEY_DUPLEX, 0.5, (200, 200, 200), 1)

        combined = np.hstack([left, right])
        cv2.imshow(WIN, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            cv2.destroyWindow(WIN)
            print(f"[tuner] Accepted HSV: low={low}  high={high}")
            return low, high
        if key == 27:
            cv2.destroyWindow(WIN)
            print("[tuner] Cancelled.")
            return None




def calibrate_colour(frame):
    """Show frame; user clicks on the ball; returns (hsv_low, hsv_high)."""
    clone = frame.copy()
    hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    samples = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            h, s, v = hsv[y, x]
            samples.append((int(h), int(s), int(v)))
            cv2.circle(clone, (x, y), 8, (0, 255, 0), 2)
            cv2.imshow("Calibrate – click ball (5 clicks), then press SPACE", clone)

    win = "Calibrate - click ball 5 times, then press SPACE"
    cv2.putText(clone, "Click ON the ball 5 times, then press SPACE",
                (20, 40), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 200), 2)
    cv2.imshow(win, clone)
    cv2.waitKey(200)  # give Qt time to actually render the window
    cv2.setMouseCallback(win, on_click)  # attach AFTER window exists

    while True:
        key = cv2.waitKey(20) & 0xFF
        if key == ord(' ') and len(samples) >= 1:
            break
        if key == 27:
            break

    cv2.destroyWindow(win)

    if not samples:
        print("[calibrate] No samples – using defaults.")
        return DEFAULT_HSV_LOW.copy(), DEFAULT_HSV_HIGH.copy()

    h_vals = [s[0] for s in samples]
    s_vals = [s[1] for s in samples]
    v_vals = [s[2] for s in samples]

    margin_h, margin_s, margin_v = 18, 60, 60
    low  = np.array([max(0,   min(h_vals) - margin_h),
                     max(0,   min(s_vals) - margin_s),
                     max(0,   min(v_vals) - margin_v)], dtype=np.uint8)
    high = np.array([min(179, max(h_vals) + margin_h),
                     255,
                     255], dtype=np.uint8)

    print(f"[calibrate] HSV range: {low} → {high}")
    return low, high


# ── Scale calibration ─────────────────────────────────────────────────────────

def calibrate_scale(frame):
    """User clicks two points; enters real-world distance; returns cm/pixel."""
    clone  = frame.copy()
    points = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
            points.append((x, y))
            cv2.circle(clone, (x, y), 6, (0, 255, 100), -1)
            if len(points) == 2:
                cv2.line(clone, points[0], points[1], (0, 255, 100), 2)
            cv2.imshow(win, clone)

    win = "Scale - click 2 known points, then press SPACE"
    cv2.putText(clone, "Click 2 points with known real distance, press SPACE",
                (20, 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 200), 2)
    cv2.imshow(win, clone)
    cv2.waitKey(200)  # give Qt time to actually render the window
    cv2.setMouseCallback(win, on_click)  # attach AFTER window exists

    while True:
        key = cv2.waitKey(20) & 0xFF
        if key == ord(' ') and len(points) == 2:
            break
        if key == 27:
            return None

    cv2.destroyWindow(win)

    dx = points[1][0] - points[0][0]
    dy = points[1][1] - points[0][1]
    pixel_dist = math.hypot(dx, dy)

    real_cm = float(input(f"  Pixel distance = {pixel_dist:.1f} px. Enter real distance in cm: "))
    scale   = real_cm / pixel_dist
    print(f"[scale] {scale:.4f} cm/pixel")
    return scale


# ── Ball detection ────────────────────────────────────────────────────────────

def detect_ball(frame, hsv_low, hsv_high):
    """
    Returns (cx, cy, radius) of the largest matching blob, or None.
    """
    blurred = cv2.GaussianBlur(frame, (BLUR_KERNEL, BLUR_KERNEL), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask    = cv2.inRange(hsv, hsv_low, hsv_high)

    # Handle hue wrap-around (e.g. red ball spanning 170–10)
    if hsv_low[0] > hsv_high[0]:
        low2  = hsv_low.copy();  low2[0]  = 0
        high2 = hsv_high.copy(); high2[0] = 179
        mask  = cv2.bitwise_or(mask, cv2.inRange(hsv, low2, high2))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    best = max(contours, key=cv2.contourArea)
    (cx, cy), radius = cv2.minEnclosingCircle(best)

    if not (MIN_BALL_RADIUS_PX <= radius <= MAX_BALL_RADIUS_PX):
        return None, mask

    return (int(cx), int(cy), int(radius)), mask


# ── Overlay drawing ───────────────────────────────────────────────────────────

def draw_overlay(frame, detection, trail, velocity_px,
                 scale_cm_per_px, setpoint_x, fps,
                 hsv_low, hsv_high, beam_y=None):
    h, w = frame.shape[:2]
    out  = frame.copy()

    # ── Beam reference line (horizontal, mid-height if not given) ────────────
    if beam_y is None:
        beam_y = h // 2
    cv2.line(out, (0, beam_y), (w, beam_y), (60, 60, 60), 1)

    # ── Setpoint vertical line ───────────────────────────────────────────────
    if setpoint_x is not None:
        cv2.line(out, (setpoint_x, 0), (setpoint_x, h), C_SETPOINT, 1)
        cv2.putText(out, "SP", (setpoint_x + 5, 20),
                    cv2.FONT_HERSHEY_DUPLEX, 0.5, C_SETPOINT, 1)

    # ── Trail ────────────────────────────────────────────────────────────────
    pts = list(trail)
    for i in range(1, len(pts)):
        if pts[i-1] is None or pts[i] is None:
            continue
        alpha = i / len(pts)
        col   = tuple(int(c * alpha) for c in C_TRAIL)
        thick = max(1, int(alpha * 3))
        cv2.line(out, pts[i-1][:2], pts[i][:2], col, thick)

    # ── Ball circle + centre ─────────────────────────────────────────────────
    if detection is not None:
        cx, cy, r = detection

        # Outer glow (soft)
        cv2.circle(out, (cx, cy), r + 6, (*C_CIRCLE[:2], 60), 2)
        # Main circle
        cv2.circle(out, (cx, cy), r,     C_CIRCLE, 2)
        # Centre crosshair
        cs = 10
        cv2.line(out, (cx - cs, cy), (cx + cs, cy), C_CROSS, 2)
        cv2.line(out, (cx, cy - cs), (cx, cy + cs), C_CROSS, 2)
        cv2.circle(out, (cx, cy), 3, C_CENTER, -1)

        # Velocity arrow
        vx, vy = velocity_px
        speed  = math.hypot(vx, vy)
        if speed > 1.0:
            scale_arrow = min(60, speed * 4)
            ex = int(cx + vx / speed * scale_arrow)
            ey = int(cy + vy / speed * scale_arrow)
            cv2.arrowedLine(out, (cx, cy), (ex, ey), C_VELOCITY, 2, tipLength=0.3)

        # Position label next to ball
        pos_cm = cx * scale_cm_per_px if scale_cm_per_px else None
        spd_cm = speed * scale_cm_per_px * fps if (scale_cm_per_px and fps > 0) else None

        label_parts = [f"r={r}px"]
        if pos_cm is not None:
            label_parts.append(f"x={pos_cm:.1f}cm")
        if spd_cm is not None:
            label_parts.append(f"v={spd_cm:.1f}cm/s")
        label = "  ".join(label_parts)

        lx = cx + r + 8
        ly = cy - 8
        # clamp to frame
        lx = min(lx, w - len(label) * 8)
        cv2.putText(out, label, (lx, ly),
                    cv2.FONT_HERSHEY_DUPLEX, 0.55, C_TEXT_FG, 1, cv2.LINE_AA)

    # ── HUD panel (top-left) ─────────────────────────────────────────────────
    hud_lines = [
        f"FPS: {fps:.1f}",
        f"Scale: {scale_cm_per_px:.4f} cm/px" if scale_cm_per_px else "Scale: not set",
        f"Ball: {'detected' if detection else 'lost'}",
        f"HSV lo: {hsv_low}",
        f"HSV hi: {hsv_high}",
        "t=HSV tuner  c=colour  s=scale  r=reset  q=quit",
    ]
    pad   = 8
    lh    = 20
    panel_w = 340
    panel_h = len(hud_lines) * lh + pad * 2
    cv2.rectangle(out, (0, 0), (panel_w, panel_h), C_TEXT_BG, -1)
    for i, line in enumerate(hud_lines):
        cv2.putText(out, line, (pad, pad + lh * (i + 1) - 4),
                    cv2.FONT_HERSHEY_DUPLEX, 0.45, C_TEXT_FG, 1, cv2.LINE_AA)

    return out


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera}")

    # Grab one frame for calibration
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Camera returned no frame")

    # ── Colour calibration ───────────────────────────────────────────────────
    if args.tune:
        result = hsv_tuner(cap)
        hsv_low  = result[0] if result else DEFAULT_HSV_LOW.copy()
        hsv_high = result[1] if result else DEFAULT_HSV_HIGH.copy()
    elif args.calibrate:
        hsv_low, hsv_high = calibrate_colour(frame)
    else:
        print("[info] Using wide-open HSV range. Press 't' to open live tuner (recommended!)")
        hsv_low, hsv_high = DEFAULT_HSV_LOW.copy(), DEFAULT_HSV_HIGH.copy()

    # ── Scale calibration ────────────────────────────────────────────────────
    if args.scale is not None:
        scale_cm_per_px = args.scale
    else:
        print("[info] No scale set. Press 's' during tracking to calibrate.")
        scale_cm_per_px = None

    # State
    trail      = deque(maxlen=TRAIL_LENGTH)
    velocity   = (0.0, 0.0)
    prev_cx    = None
    setpoint_x = None
    beam_y     = None

    t_prev  = time.time()
    fps     = 30.0
    fps_alpha = 0.05  # EMA smoothing for FPS display

    print("[tracker] Running. Press 'q' to quit.")
    cv2.namedWindow("Ball Tracker - Knikkerbaan")
    cv2.waitKey(1)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[tracker] Frame grab failed – exiting.")
            break

        # ── FPS ──────────────────────────────────────────────────────────────
        t_now = time.time()
        dt    = max(t_now - t_prev, 1e-6)
        fps   = fps_alpha * (1.0 / dt) + (1 - fps_alpha) * fps
        t_prev = t_now

        # ── Detect ───────────────────────────────────────────────────────────
        detection, mask = detect_ball(frame, hsv_low, hsv_high)

        if detection is not None:
            cx, cy, r = detection
            if prev_cx is not None:
                vx = cx - prev_cx[0]
                vy = cy - prev_cx[1]
                velocity = (vx, vy)
            prev_cx = (cx, cy)
            trail.append((cx, cy))

            # Auto-detect beam y as the bottom of the ball
            if beam_y is None:
                beam_y = cy + r
        else:
            trail.append(None)
            velocity = (0.0, 0.0)

        # ── Draw ─────────────────────────────────────────────────────────────
        vis = draw_overlay(frame, detection, trail, velocity,
                           scale_cm_per_px, setpoint_x, fps,
                           hsv_low, hsv_high, beam_y)

        # Debug mask (small, bottom-right corner)
        mh, mw = mask.shape
        thumb_w = 200
        thumb_h = int(mh * thumb_w / mw)
        thumb   = cv2.resize(mask, (thumb_w, thumb_h))
        thumb_c = cv2.cvtColor(thumb, cv2.COLOR_GRAY2BGR)
        fh, fw  = vis.shape[:2]
        vis[fh - thumb_h:fh, fw - thumb_w:fw] = thumb_c
        cv2.putText(vis, "mask", (fw - thumb_w + 4, fh - thumb_h + 14),
                    cv2.FONT_HERSHEY_DUPLEX, 0.4, (200, 200, 200), 1)

        cv2.imshow("Ball Tracker - Knikkerbaan", vis)

        # ── Keyhandler ───────────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('t'):
            result = hsv_tuner(cap)
            if result:
                hsv_low, hsv_high = result[0], result[1]
            ret2, f2 = cap.read()
            if ret2:
                hsv_low, hsv_high = calibrate_colour(f2)
        elif key == ord('s'):
            ret2, f2 = cap.read()
            if ret2:
                result = calibrate_scale(f2)
                if result is not None:
                    scale_cm_per_px = result
        elif key == ord('r'):
            trail.clear()
            velocity = (0.0, 0.0)
            prev_cx  = None
            print("[tracker] Trail reset.")

        # Live telemetry to terminal
        if detection is not None:
            cx, cy, _ = detection
            pos_cm  = cx * scale_cm_per_px if scale_cm_per_px else float('nan')
            vx, vy  = velocity
            spd_cms = math.hypot(vx, vy) * (scale_cm_per_px or 0) * fps
            print(f"\r  x={pos_cm:6.2f} cm   v={spd_cms:6.2f} cm/s   fps={fps:.1f}   ", end="", flush=True)

    cap.release()
    cv2.destroyAllWindows()
    print("\n[tracker] Done.")


if __name__ == "__main__":
    main()