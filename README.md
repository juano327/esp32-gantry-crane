# ESP32 Wi-Fi Gantry Crane (3 axes)

Firmware for a 3-axis gantry crane controlled from a phone or laptop over Wi-Fi. The ESP32 creates its own access point and serves the control page.

## Features
- **Motion:** X, Y and Z axes driven through L298N bridges with PWM.
- **Safe start-up:** outputs are forced low before the power stage is enabled, and the motors stay locked until homing finishes.
- **Homing:** X and Y run in parallel FreeRTOS tasks against limit switches.
- **Hook height:** a VL53L0X laser time-of-flight sensor measures the hook position (82–260 mm) and stops it within ±15 mm of the target.
- **State machine:** init, waiting for homing, ready, moving, test, sensor error and emergency stop. Limit switches trigger the emergency stop at any time.
- **Web API:** commands arrive at `POST /api/v1/control` and are passed to the motor task through a FreeRTOS queue.

## Structure
| File | Role |
|---|---|
| `main/main.c` | Start-up sequence and parallel homing |
| `main/motor_logic.cpp` | State machine, motor control, ToF reading, safety |
| `main/wifi_server.c` | Access point, web page and REST endpoint |
| `components/ESP32_VL53L0X` | VL53L0X driver by Ryotaro Onuki ([kerikun11/ESP32_VL53L0X](https://github.com/kerikun11/ESP32_VL53L0X), MIT) |

Build with ESP-IDF: `idf.py set-target esp32 && idf.py build flash monitor`. Change the access point password in `main/wifi_server.c` before using it outside a lab.

Course project, Mechatronics Engineering, Universidad Nacional de Cuyo.
