# teensy-feedback-drum
a wired drum instrument that play with an acoustic feedback loop and effects


# Teensy AutoCut DSP

4-band notch filter with automatic feedback detection for Teensy 3.x / 4.x + PJRC Audio Shield.

## What it does
- Listens to LINE IN through a 4-band biquad notch filter
- Adds a multi-tap delay with adjustable timing
- **Autocut**: automatically detects up to 4 feedback frequencies using FFT
  and applies notch filters in real time — in a few seconds
- Full CLI via Serial terminal (115200 baud) with real-unit parameters (Hz, Q, ms)

## Hardware
- Teensy 3.x or 4.x
- PJRC Audio Shield (SGTL5000 codec)

## CLI usage

f1freq          → get notch 1 frequency

f1freq 800      → set notch 1 to 800 Hz

f1q 4.5         → set notch 1 Q

dlt 200         → set delay tap to 200 ms

autocut         → run feedback calibration

list            → show all parameters

help            → show commands



## License
GPL v3 — see [LICENSE](LICENSE)

Copyright (c) 2026 Simon Benichou
