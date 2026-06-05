# Servo Arm Control System

A complete 5-servo robotic arm control system with three control methods.

## Hardware

- Arduino UNO R3 (main controller)
- 2x Dual-axis Joystick Modules
- 5x MG995 Servo Motors
- Mechanical arm components

## Pin Definition

### Joystick Pins
| Control | Pin |
|---------|-----|
| Joystick 1 X | A0 |
| Joystick 1 Y | A1 |
| Joystick 1 Button | D2 |
| Joystick 2 X | A3 |
| Joystick 2 Y | A2 |
| Joystick 2 Button | D4 |

### Servo Pins
| Servo | Pin | Range |
|-------|-----|-------|
| Servo 0 | D5 | 0-180° |
| Servo 1 | D9 | 0-180° |
| Servo 2 | D10 | 0-180° |
| Servo 3 | D11 | 0-180° |
| Servo 4 | D7 | 0-90° |

## Control Logic

| Control | Pin | Servo | Range | Function |
|---------|-----|-------|-------|---------|
| Joystick 1 X | A0 | Servo 0 | 0-180° | X-axis |
| Joystick 1 Y | A1 | Servo 1 | 0-180° | Y-axis |
| Joystick 1 Button | D2 | Servo 4 | 0-90° | +2°/press |
| Joystick 2 X | A3 | Servo 2 | 0-180° | X-axis |
| Joystick 2 Y | A2 | Servo 3 | 0-180° | Y-axis |
| Joystick 2 Button | D4 | Servo 4 | 0-90° | -2°/press |

## Quick Start

### Arduino Setup
1. Open servo_control.ino in Arduino IDE
2. Select Board: Arduino UNO
3. Select COM port
4. Upload sketch
5. Open Serial Monitor (9600 baud)

### Serial Commands
| Command | Description |
|---------|-------------|
| 0 90 | Set Servo 0 to 90° |
| 1 45 | Set Servo 1 to 45° |
| 2 90 | Set Servo 2 to 90° |
| 3 90 | Set Servo 3 to 90° |
| 4 45 | Set Servo 4 to 45° (max 90°) |
| 90 | Set ALL to 90° |
| s | Show status |
| h | Show help |

### Python GUI
Requirements: Python 3.x, pyserial

Setup:
```bash
pip install pyserial
```

Run:
```bash
python servo_control.py
```

## Safety Warning

### Power Supply
After uploading code, switch from computer USB to external power supply (9V 2A or higher). MG995 servos require adequate power.

### Overload Protection
Do not apply excessive force. Servo stall causes current surge and board restart.

## Support

GitHub: https://github.com/490437477-sys/servo-arm-control
