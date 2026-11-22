# DC Motor Speed Measurement and PID Control (Arduino)

Three Arduino Uno sketches that build up a small closed-loop DC motor drive:
measuring speed with a Hall sensor, driving the motor through an H-bridge,
and closing the loop with a PID controller that can be tuned over the serial port.

| Step | Sketch | What it does |
|------|--------|--------------|
| 1 | [`01_rpm_measurement`](01_rpm_measurement) | Measures RPM from a Hall sensor using an interrupt, debounce and a low-pass filter |
| 2 | [`02_hbridge_pwm`](02_hbridge_pwm) | 2 kHz complementary PWM for a full H-bridge using Timer1, with dead time and forward/stop/reverse control |
| 3 | [`03_pid_speed_control`](03_pid_speed_control) | Closed-loop speed control: Hall-sensor feedback + PID with anti-windup, live tuning over serial |

## Hardware

- Arduino Uno (ATmega328P, 16 MHz)
- Small brushed DC motor with a magnet on the shaft
- Hall-effect sensor module (e.g. MH-series, open-collector output, active LOW)
- Motor driver: H-bridge module or discrete MOSFET bridge

### Connections

| Signal | Arduino pin | Used in |
|--------|-------------|---------|
| Hall sensor output | D2 (INT0) | 01, 03 |
| PWM to motor driver | D9 | 03 |
| Direction (optional) | D8 | 03 |
| H-bridge side A | D10 | 02 |
| H-bridge side B | D6 | 02 |

Connect the grounds of the Arduino, sensor and motor supply together.

## How it works

### 1. RPM measurement
The Hall sensor pulls the line LOW each time the magnet passes, so the sketch
triggers on the FALLING edge. The interrupt stores the time between pulses;
pulses closer than 1 ms are ignored as bounce. RPM is

```
RPM = (1e6 / interval_us) / pulses_per_rev * 60
```

and is smoothed with an exponential filter. If no pulse arrives for 0.8 s the
motor is reported as stopped.

### 2. H-bridge PWM
Timer1 runs in Fast PWM mode with `ICR1 = 1000` and a prescaler of 8, giving
16 MHz / (8 × 1000) = 2 kHz. The compare and overflow interrupts switch the two
bridge sides with a 5 µs dead time so that both sides are never on together.
Send a number 0–10 over serial: below 5 is reverse, 5 is stop, above 5 is forward.

### 3. PID speed control
Every 100 ms the filtered RPM is compared with the setpoint and a PID output is
computed and written as PWM (0–255). The integral term is clamped to avoid
wind-up. Gains and setpoint can be changed while running.

Serial commands (9600 baud, newline-terminated):

```
S:300      set target speed to 300 RPM
Kp:0.8     set proportional gain
Ki:0.6     set integral gain
Kd:0.2     set derivative gain
RESET      clear the integral and derivative state
```

The sketch prints setpoint, measured RPM, error, output and gains every 400 ms.

## Usage

1. Open the sketch folder in the Arduino IDE (each folder contains one `.ino`).
2. Select **Arduino Uno** and the right port, then upload.
3. Open the Serial Monitor at 9600 baud.

Set `pulsesPerRevolution` to the number of magnets on the shaft.

## Author

Subhrojyoti Mandal, Department of EEE, Khulna University of Engineering & Technology (KUET).
GitHub: [@Dark-atoms](https://github.com/Dark-atoms)

## License

MIT, see [LICENSE](LICENSE).
