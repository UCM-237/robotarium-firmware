# Robotarium Firmware (Arduino) 🤖

Low-level firmware for the Robotarium agents. This repository manages real-time motor execution, high-resolution quadrature encoder processing, sensor fusion, and MQTT communication for Arduino MKR WiFi 1010 and Nano 33 IoT boards.

📂 Repository Structure

1. /Robot

- The production firmware.
- 100Hz Control Loop: Precise timing for PID calculations.
- JSON Telemetry: Sends battery, IMU (Kalman filtered), and wheel speed ($rad/s$) to the Hub.
- Bi-directional MQTT: Responds to movement commands and real-time parameter tuning.

2. /Test

A suite of tools for hardware validation and control tuning:

1. AjusteControlador/: Scripts to characterize the FeedForward linear model ($PWM = A \cdot \omega + B$) and tune PID gains.
- Encoder/ (TestEncoder.ino): Advanced diagnostic for quadrature encoders.
- 4x Decoding Mode: Uses a state machine to read every transition on Channel A and B, quadrupling the pulses per revolution (PPR) for superior low-speed control.
- Direction Detection: Validates if the wiring matches the software-expected polarity.

2. Test/: General diagnostic scripts for the LSM6DS3 IMU and WiFi signal (RSSI).

3. /ArUCoMarkers
Visual resources for robot tracking using the DICT_ARUCO_ORIGINAL dictionary.

⚙️ Configuration & HAL

The system uses a Hardware Abstraction Layer (HAL) defined in robot.cpp/h. Settings are centralized in the configuration headers:
1. Hardware Profile

In robot.h (or config.h), uncomment your specific setup:

C++


#define ARDUINO_TYPE_MKR   // Board: MKR WiFi 1010
#define H_BRIDGE_RED       // Driver: RED or BLACK L298N
#define SMALLBOT           // Chassis: SMALLBOT or BIGBOT
#define ENCODER_CUADRATURA // Enable 4x State Machine feedback


2. Robot Identity & Networking
Update robotID and credentials in arduino_secrets.h:

C++


#define SECRET_SSID "RobotariumUCM"
#define SECRET_PASS "robotario"


📡 Control & Communication

1. Dual-Loop Strategy

The robot maintains precise velocity ($rad/s$) through:

- FeedForward: Predictive PWM based on a pre-calculated motor curve.

- PID Controller: Proportional-Integral corrector with Anti-Windup ($maxIntegralError = 15$) to eliminate steady-state error without saturation.
- MQTT Operation Codes (operation_t)

2. Code

- OP_MOVE_ROBOT	Set linear ($v$) and angular ($w$) velocities.

- OP_TELEMETRY		Broadcasts IMU, Battery, and Wheel frequencies.

- OP_CONF_PID		Real-time update of Kp, Ki, Kd constants.

- OP_MOVE_WHEELS	Direct $rad/s$ setpoints for each motor.

🚀 Installation & Calibration

1. Dependencies: 

Install via Library Manager: WiFiNINA, ArduinoMqttClient, ArduinoJson, SimpleKalmanFilter, MeanFilterLib.

3. Encoder Check: Run TestEncoder.ino and rotate wheels manually. Ensure countsL/R increase when moving forward.

4. Tuning: Use AjusteControlador.ino to find your motor's $A$ and $B$ FeedForward constants.

5. Production: Flash the code in the /Robot folder for full swarm integration.

----------------------------------------------------------------------------------------------------------
UCM Robotarium Project | Faculty of Computer Science | Complutense University of Madrid.
