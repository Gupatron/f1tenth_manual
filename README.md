# STM32 F405 Serial Control (BDShot + Servo PWM)

This repository contains a Python-based control system for an **STM32 F405** microcontroller using **serial communication**. The STM32 firmware implements **BDShot** for controlling **AM32 ESCs** and **PWM output** for controlling a servo motor.

The project is split into two main components:
- **Base** – runs on a local machine and handles user input and control logic.
- **Rover** – runs on a **Jetson Nano** and interfaces with the STM32.

---

## Requirements

- Python 3.x
- `pygame`

Install dependencies with:

```bash
pip install pygame
```
## On Base (PC)
```bash
cd base
python base.py
```


## On Rover (Jetson) 
```bash
cd rover
python3 rover.py
```
