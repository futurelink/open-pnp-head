Modular CNC controller
---

This project is based on GRBL source code, deeply refactored, rewritten in C++
to make the code easier to understand and support.

Target platform to run this firmware is STM32F103C8T6, although it can be easily ported
to any other 32-bit MCU. Binary size with all available options switched on is about
76Kb.

### Supported hardware features

Mostly all features were derived from GRBL, some of them were added.

- Spindle (including PWM controlled)
- Coolant (Flood and Mist outputs)
- Limit switches (X, Y and Z)
- Probe
- Control inputs (Cycle start, Feed Hold, Reset and Safety door)
- Relay control 5 channels (turn on commands M100-M104, turn off - M110-M114)

### Running this firmware

Shortly - it can be run on a 'Blue Pill', USB is configured as Virtual COM port,
hence it's used to control and send G-Code. Typical speed is 115200.

Another option is to use controller designed for this firmware and it worth it because it has:

- opto-couplers in input and output lines (excluding control port and stepper drivers)
- RS-485 interface to control additional hardware
- 2 types of USB connectors (USB-B and JST embedding into one case with PC)
- 4 channels with MOSFETs to control relays (valves, solenoids) and separate (max. 24V) relay power input
- 1 channel with the same MOSFET but switchable power supply to connect work zone light
- spindle control 0-10V