# ASFALT
**Accel Systems Line Following Robot**

A high-resolution line follower built on an Arduino Nano (ATmega328P), featuring a 16-sensor U-shaped IR array, custom KiCad PCBs, 3D-printed chassis parts, and a PD/proportional control algorithm with priority-based corner handling.

---

## Features

- **16× TCRT5000 IR sensors** arranged in a U-shape (left arm · front bar · right arm) for wide-field line detection and sharp corner recognition
- **Analog multiplexer** (16-channel) reads all sensors through a single analog pin
- **Priority-based steering logic** — side sensors override PID for aggressive corner turns; front-bar PD handles straight runs and gentle curves
- **Custom PCBs** (KiCad) for the main board and sensor board (V1 & V2)
- **3D-printed parts** — motor mounts (V1 & V2) and LEGO-to-D-shaft wheel adapters
- **DRV8833 dual H-bridge** motor driver with PWM speed control
- **Serial debug output** for live tuning of PID gains and sensor thresholds

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino Nano / ATmega328P |
| Motor driver | DRV8833 |
| IR sensors | 16× TCRT5000 via 16-ch analog mux |
| Display | I²C 0.96″ OLED |
| UI | 4-button menu |

### Sensor Layout (U-shape)

```
  [ L0 L1 L2 L3 ]        ← Left arm  (CH 0–3,  point left)
  [ F4 F5 F6 F7 F8 F9 F10 F11 ]  ← Front bar (CH 4–11, point forward)
  [ R12 R13 R14 R15 ]     ← Right arm (CH 12–15, point right)
```

### Pin Mapping

| Function | Pin |
|---|---|
| Mux S0–S3 | D8, D10, D11, D12 |
| Mux signal | A0 |
| Left motor 1A / 1B | D3 / D5 |
| Right motor 2A / 2B | D6 / D9 |

---

## Repository Structure

```
ASFALT/
├── src/
│   ├── code/
│   │   ├── ASFALT_LNF.ino     # Main firmware
│   │   ├── MuxReader.h        # 16-ch mux library header
│   │   └── MuxReader.cpp      # 16-ch mux library implementation
│   ├── PCB/
│   │   ├── LNF_SENSOR_V1/     # First sensor board (KiCad)
│   │   ├── LNF_SENSOR_V2/     # Revised sensor board (KiCad)
│   │   └── MAIN_V2/           # Main controller board (KiCad)
│   ├── 3dPrint/
│   │   ├── motorMount/
│   │   │   ├── v1/            # Motor mount V1 (.stl, .f3d)
│   │   │   └── v2/            # Motor mount V2 (.stl × 2)
│   │   └── wheels/
│   │       └── LEGO_30x04TIRE_TO_DSHAFT.stl  # LEGO tire → D-shaft adapter
│   ├── bitmap/
│   │   └── AccelSystemsLogo.kicad_mod
│   └── img/
│       └── AsfaltMainSchematic.pdf
```

---

## Getting Started

### Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) 1.8+ or Arduino CLI
- Arduino Nano board package (`ATmega328P`)
- No external libraries needed — `MuxReader` is included in the repo

### Upload

1. Clone the repository:
   ```bash
   git clone https://github.com/AccelRanger/ASFALT.git
   ```
2. Open `src/code/ASFALT_LNF.ino` in Arduino IDE.
3. Ensure `MuxReader.h` and `MuxReader.cpp` are in the same folder.
4. Select **Arduino Nano** and the correct COM port.
5. Upload.

---

## Tuning Guide

All tunable constants are at the top of `ASFALT_LNF.ino`:

| Constant | Default | Effect |
|---|---|---|
| `BASE_SPEED` | 100 | Forward cruise speed (0–255). Raise once steering is stable. |
| `TURN_SPEED` | 20 | Inner wheel speed during sharp corner turns. Lower = tighter turn. |
| `MAX_SPEED` | 220 | Motor ceiling. |
| `KP_FRONT` | 0.05 | Front-bar proportional gain. Raise until oscillation, then back off ~20%. |
| `KD_FRONT` | 0.15 | Front-bar derivative gain. Raise if the robot wiggles on straights. |
| `KP_SIDE` | 60 | Side-sensor turn force (added directly as correction units). Raise if corners are missed. |
| `SIDE_THRESHOLD` | 200 | Min ADC sum on a side arm before it triggers. Lower if corners are missed. |
| `FRONT_THRESHOLD` | 300 | Min ADC sum on front bar to count as on-line. |

### Steering Logic Priority

1. **Side sensor only** — sharp corner detected → hard turn using `KP_SIDE`
2. **Side + front together** — entering a corner → blended correction
3. **Front bar only** — straight or gentle curve → PD control
4. **No sensors** — line lost → motors stop, PID state reset

---

## MuxReader Library

A lightweight library for reading a 16-channel analog multiplexer on ATmega328P.

```cpp
MuxReader mux(pinA, pinB, pinC, pinD, sigPin);
mux.begin();

int raw[16];
mux.readAll(raw);            // fills array with ADC values (0–1023)

int total = mux.readSum();   // 0–16368
int pos   = mux.readPosition(); // 0–15000, center = 7500, or MUX_NO_LINE
```

**Configuration methods:**

| Method | Default | Description |
|---|---|---|
| `setInvert(bool)` | false | Invert readings (black line = low ADC) |
| `setSettleTime(µs)` | 10 | Delay between channel select and ADC read |
| `setNoLineThreshold(int)` | 50 | Minimum sum before `readPosition()` returns `MUX_NO_LINE` |

---

## PCB

KiCad project files are in `src/PCB/`. The main schematic is also available as a PDF at `src/img/AsfaltMainSchematic.pdf`.

| Board | Path | Notes |
|---|---|---|
| Sensor board V1 | `PCB/LNF_SENSOR_V1/` | Original 16-sensor mux board |
| Sensor board V2 | `PCB/LNF_SENSOR_V2/` | Revised layout |
| Main board V2 | `PCB/MAIN_V2/` | Controller + driver integration |

---

## 3D Printing

All printable parts are in `src/3dPrint/`. Standard PLA works fine.

| File | Description |
|---|---|
| `motorMount/v1/ASFALT_LINE_FOLLOWER_MOTOR_MOUNT.stl` | Motor mount V1 |
| `motorMount/v2/ASLNF_MOTOR_HOLDER_MAIN.stl` | Motor mount V2 — main body |
| `motorMount/v2/ASLNF_MOTOR_HOLDER_TOP.stl` | Motor mount V2 — top clamp |
| `wheels/LEGO_30x04TIRE_TO_DSHAFT.stl` | Adapter for LEGO 30×4 tires on D-shaft motors |

---

## License

This project is open source. Hardware designs, firmware, and mechanical files are provided as-is for personal and educational use.

---

*ASFALT — Accel Systems Line Following Robot*
