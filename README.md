# Gyro-Gun-controller-
# Pico Gyro Gun Controller 🎮

A custom, dual-mode game controller built with a Raspberry Pi Pico. It features full 250Hz gyro aiming for FPS games and a potentiometer based steering wheel mode for racing games (like Asphalt). 

Acts as a standard USB HID Mouse and Keyboard no special PC drivers required!

## Features
* **FPS Mode (Default):** 
  * Ultra-smooth gyro aiming using an MPU-6500 sensor.
  * Analog joystick mapped to 8-way WASD movement.
  * Jump (Spacebar) on short joystick press, Mode toggle on long press.
  * Trigger (Left Click), Zoom (Right Click), and Reload (R) buttons.
* **Racing Mode (Asphalt):**
  * Potentiometer steering (maps to A/D) with fixed deadzone mapping.
  * Progressive throttle (W) mapped to the Zoom button.
  * Brake (S) and Boost (Spacebar).
* **Hardware Debouncing:** Rock-solid button presses with zero double-clicks.
* **Auto-Calibration:** Gyro automatically calibrates drift when left perfectly still for 1.5 seconds.

## Hardware Required
* 1x Raspberry Pi Pico (RP2040)
* 1x MPU-6500 Gyroscope/Accelerometer module
* 1x Analog Thumbstick module
* 1x 10k Potentiometer (for steering wheel)
* 3x Arcade/Tactile Push Buttons (Trigger, Zoom, Reload)
* 2x LEDs (with 220Ω resistors)

## Wiring Diagram / Pinout

| Component | Pin on Pico |
| :--- | :--- |
| **MPU-6500 SDA** | GP4 |
| **MPU-6500 SCL** | GP5 |
| **Joystick VRy** | GP26 (ADC0) |
| **Joystick VRx** | GP27 (ADC1) |
| **Joystick SW** | GP16 |
| **Trigger Button** | GP6 |
| **Zoom Button**| GP9 |
| **Reload Button**| GP10 |
| **Steering Potentiometer** | GP28 (ADC2) |
| **LED 1** | GP7 |
| **LED 2** | GP8 |

*(Note: Connect all VCC pins to Pico 3V3, and all GND pins to Pico GND).*

## How to Install
1. Download and install the Arduino IDE.
2. Install the **Raspberry Pi Pico/RP2040** board package by Earle F. Philhower in the Boards Manager.
3. Open the `.ino` file.
4. Install the required `Mouse` and `Keyboard` libraries if you haven't already.
5. Select "Raspberry Pi Pico" in the boards menu and hit Upload!

## Usage
* **Calibration:** When you plug the controller in, leave it perfectly still on the desk for 3 seconds while the gyro calibrates.
* **Mode Switching:** Press and hold the joystick button (L3) for 1.2 seconds to toggle between FPS Mode and Racing Mode. The LEDs will flash to confirm.
* **Re-center Mouse:** Hold the Zoom + Reload buttons together for 0.5 seconds to instantly snap the mouse cursor to the center of the screen.

## License
This project is open-source under the MIT License. Feel free to modify and build your own!
