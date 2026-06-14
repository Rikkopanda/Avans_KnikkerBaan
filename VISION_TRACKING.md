# Vision Tracking

`src/main.cpp` runs on the ESP32. `track.cpp` runs on the Linux computer and
sends measured ball positions to the ESP32 over USB serial.

## Build

```bash
make track
```

This requires the OpenCV development package and `pkg-config`. It creates the
desktop executable `./track`; PlatformIO is only used for the ESP32 firmware.

## Run

1. Upload `src/main.cpp` to the ESP32.
2. Close the PlatformIO serial monitor and serial plotter. Only one program can
   own `/dev/ttyUSB0` at a time.
3. Start the tracker:

```bash
./track --serial /dev/ttyUSB0 --camera 0 --beam-cm 30
```

Use the correct serial device and physical beam length. If serial access is
denied, add the Linux user to the `dialout` group and log in again.

## Calibrate

1. Adjust the HSV sliders until only the ball is white in the threshold image.
2. In the original-image window, click the sensor/left endpoint of the beam.
3. Click the far/right endpoint. The first point is `0 cm`; the second point is
   the value supplied by `--beam-cm`.
4. The tracker sends `vision:<position_cm>`. If the ball is lost it sends
   `vision:-1`, preventing stale positions from reaching the controller.

As soon as the ESP32 receives a vision position, vision becomes the only
measurement source. ADC sampling and the Sharp distance conversion are skipped.
If tracking stops for 250 ms, the beam returns toward neutral; the firmware does
not fall back to the distance sensor. Send `source:sensor` explicitly to use the
Sharp sensor again.

Right-click to redefine the beam. Press `q` or Escape to exit.
