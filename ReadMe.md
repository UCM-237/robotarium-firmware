#Robotarium Firmware (Arduino) 🤖

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


## 🏗️ System Architecture (Heterogeneous Swarm)

The Robotarium swarm consists of 10 agents with an adaptive control infrastructure based on two hardware profiles defined in `config.h`: **BIGBOT** (large chassis) and **SMALLBOT** (small chassis).



                           ┌────────────────────────────────┐
                           │       Central Hub Server       │
                           └────────────────────────────────┘
                               │                        │
                   📡 (WiFi / MQTT)              📡 (WiFi / MQTT)
                               │                        │
                               ▼                        ▼
                   ┌───────────────────────┐   ┌───────────────────────┐
                   │    Agent - BIGBOT     │   │   Agent - SMALLBOT    │
                   │                       │   │                       │
                   │ ┌───────────────────┐ │   │ ┌───────────────────┐ │
                   │ │   Raspberry Pi    │ │   │ │    Arduino MKR    │ │
                   │ │ (High-Level: v,w) │ │   │ │   (Single-Level:  │ │
                   │ └───────────────────┘ │   │ │    Direct MQTT,   │ │
                   │           │           │   │ │    PID & IMU)     │ │
                   │    🔌 (Serial/UART)   │   │ └───────────────────┘ │
                   │           ▼           │   └───────────┬───────────┘
                   │ ┌───────────────────┐ │               │
                   │ │    Arduino MKR    │ │               │
                   │ │ (Low-Level: PID)  │ │               │
                   │ └───────────────────┘ │               │
                   └───────────┬───────────┘               │
                               │                           │
                               ▼                           ▼
                   ┌───────────────────────────────────────────┐
                   │          Motors, H-Bridges & Encoders     │
                   └───────────────────────────────────────────┘



### 1. BIGBOT Profile (Dual-Level Distributed Control)
Optimized for complex navigation, trajectory planning, and distributed mission algorithms.
* **High-Level Control (Raspberry Pi):** Connects to the central Hub server over Wi-Fi using the MQTT protocol. It receives global telemetry data or high-level missions (e.g., coordinates from an ArUco overhead camera tracking system), runs the primary pathfinding algorithms, and computes the required target linear ($v$) and angular ($w$) velocities. It forwards these setpoints locally to the Arduino via Serial interface (`Serial1`).
* **Low-Level Control (Arduino MKR / Nano IoT):** Functions strictly as a real-time Hardware Abstraction Layer (HAL). It receives binary-packed `appdata` packets from the Raspberry Pi over the UART line. It runs the primary hardware control loop at **100Hz**, processes high-resolution quadrature encoders via external interrupts, and computes FeedForward + PID outputs to regulate motor power.

### 2. SMALLBOT Profile (Single-Level Monolithic Control)
Optimized for low cost, agility, and massive swarm formations.
* **Single-Level Control (Arduino MKR):** Since these units do not carry an onboard Raspberry Pi, the Arduino takes over all responsibilities. It uses its built-in Wi-Fi module (`WiFiNINA`) to connect directly to the central Hub server, subscribing and publishing to the swarm's MQTT channels.
* The low-level firmware manages network communication, deserializes incoming JSON payloads and simultaneously runs the 100Hz motor PID loops on a single execution thread.

---

## 🔌 Serial Communication Protocol (BIGBOT Exclusive)

For agents configured as `BIGBOT`, local data interchange relies on fixed-size binary structures (`appdata`) defined in `common.h`. Any modification to this struct in the Arduino code must be identically replicated within the Raspberry Pi control scripts (Python/C++) to prevent memory padding offsets and packet corruption.

### `appdata` Packet Structure (255 bytes max):
1. **`InitFlag` (uint16_t):** Synchronization flag (e.g., 112) used to detect the valid boundary of an incoming stream.
2. **`id` (uint16_t):** Unique identifier of the robot in the swarm (mapped to `ROBOT_ID`).
3. **`op` (uint16_t):** Operation code (`operation_t`) specifying which motion action or parameter tuning function to trigger.
4. **`len` (uint16_t):** The exact length of active payload data stored inside the buffer.
5. **`data` (unsigned char[]):** Serialized byte array carrying raw parameters (such as floats, longs, or telemetry values).

## 🕹️ Operation Codes (OP_CODES)

The system decodes incoming packets and fires actions based on the numerical `op` identifier defined as an enumeration (`operation_t`) inside `operation.h`. Data payloads traveling inside the `appdata.data` buffer are parsed into raw parameters using byte-to-primitive conversions (`bytesToFloat`, `bytesToDouble`, etc.).

| Code | Operation Enum | Origin/Destination | Description |
| :---: | :--- | :---: | :--- |
| **1** | `OP_HELLO` | RPi ──▶ Arduino | Handshake initialization packet to establish and verify connection between levels. |
| **2** | `OP_MOVE_ROBOT` | RPi ──▶ Arduino | Unifies unicycle kinematics. Receives linear ($v$) and angular ($w$) velocities. |
| **3** | `OP_STOP_ROBOT` | RPi ──▶ Arduino | Forces immediate emergency stop with active braking on both wheels. |
| **4** | `OP_TELEMETRY` | Arduino ──▶ Hub/RPi | Broadcasts filtered IMU values, battery status, and actual wheel angular speeds. |
| **5** | `OP_TURN_ROBOT` | RPi ──▶ Arduino | Commands a precise pure-rotation maneuver around the central axis by a specific angle. |
| **6** | `OP_SILENCE` | RPi ──▶ Arduino | Deactivates or silences the recurring streaming thread of telemetry messages. |
| **7** | `OP_POSITION` | Bi-directional | Updates or requests coordinates/estimated pose ($x, y, \theta$) within the global arena. |
| **8** | `OP_CONF_PID` | Hub/RPi ──▶ Arduino | Modifies Proportional, Integral, and Derivative gains on-the-fly. |
| **9** | `OP_CONF_FF` | Hub/RPi ──▶ Arduino | Modifies FeedForward slope ($A$) and intercept ($B$) constants on-the-fly. |
| **10** | `OP_DONE` | Arduino ──▶ RPi/Hub | Callback flag sent back when a targeted action (e.g., precise turning) is completed. |
| **11** | `OP_MOVE_WHEELS`| RPi ──▶ Arduino | Independent, direct speed inputs in $rad/s$ for left and right wheels separately. |
| **99** | `OP_ERROR` | Arduino ──▶ RPi/Hub | Dispatched if packet buffers desynchronize, checksums fail, or constants saturate. |

---

### Detailed Payload Layout & Data Marshalling

#### 📐 OP_MOVE_ROBOT (Code 2)
Computes independent wheel differential constraints based on the structural physical parameters (`ROBOT_DIAMETER` / `L`) of the active robot profile.
* **Payload Format:** `[ float v ] [ float w ]` *(8 bytes total)*
* **Behavior:** Converts inputs into target angular wheel speeds ($\omega_L, \omega_R$) in $rad/s$. If the resulting speeds fall within the dead-zone, they are automatically constrained to `MINSETPOINT = 5.5 rad/s` to prevent motor stalling.

#### 📊 OP_TELEMETRY (Code 4)
Dispatched back to the high-level system at **100Hz**. Wheel velocities are processed through a 10-sample moving average filter (`MeanFilterLib`) to suppress sensor noise, while IMU data is stabilized using an onboard `SimpleKalmanFilter`.
* **Payload Format (JSON / Binary stream):** Contains raw battery voltage, Kalman-filtered $X, Y, Z$ accelerations/gyroscope readings, and current individual wheel velocities in $rad/s$.

#### ⚙️ OP_CONF_PID (Code 8) & OP_CONF_FF (Code 9)
Enables dynamic wireless tuning of the motion response from the control dashboard without requiring a full code re-flash.
* **OP_CONF_PID Payload:** `[ float Kp ] [ float Ki ] [ float Kd ]` *(12 bytes)*. It updates parameters for both left and right `controler` class objects.
* **OP_CONF_FF Payload:** `[ float A ] [ float B ]` *(8 bytes)*. Modifies the static predictive friction parameters used in the linear FeedForward equation: 
  $$\text{PWM}_{\text{base}} = A \cdot \omega_{\text{setpoint}} + B$$

#### 🔄 OP_TURN_ROBOT (Code 5)
Executes localized zero-radius turning by setting equal but opposite wheel velocities.
* **Payload Format:** `[ float angle_rad ]` *(4 bytes)*
* **Mechanism:** Computes the absolute target displacement arc length:
  $$\text{distance} = |\theta| \cdot \frac{\text{RobotDiameter}}{2}$$
  This distance is automatically converted to hardware encoder units (`targetTicks`) using the resolution constant (`MAX_ENCODER_STEPS`) and a custom `TURN_CORRECTION_FACTOR`. Once counts match, the Arduino drops back to active braking and transmits an `OP_DONE` status packet.

#### 🏎️ OP_MOVE_WHEELS (Code 11)
Bypasses unicycle-level transformation matrixes. Directly forces raw individual $rad/s$ setpoints.
* **Payload Format:** `[ float left_rad_s ] [ float right_rad_s ]` *(8 bytes)*
* **Behavior:** FEedForward acts instantly as the base signal, and the closed-loop PID controller applies correction tracking error adjustments inside the time slice.

## 📡 Wireless MQTT Protocol (SMALLBOT Exclusive)

Unlike the `BIGBOT` profile, which uses an intermediate Raspberry Pi and binary structures, agents configured as `SMALLBOT` handle direct wireless communication using the native `WiFiNINA` and `ArduinoMqttClient` libraries. They interact directly with the central MQTT Broker (Hub) over network topics formatted by their specific `ROBOT_ID`.

Rather than reading binary byte arrays, `SMALLBOT` firmware parses incoming JSON strings and packages outgoing telemetry payloads into string buffers using the `ArduinoJson` library.

### 🗺️ Topic Subscriptions & Publishing

Each `SMALLBOT` subscribes to and publishes on the following MQTT channels:

* **Inbound Control Topic:** `agent/ROBOT_ID/command`
    * *Purpose:* Receives execution directives from the Hub.
* **Outbound Telemetry Topic:** `agent/ROBOT_ID/telemetry`
    * *Purpose:* Broadcasts sensor streams at a fixed period.
* **Outbound Feedback Topic:** `agent/ROBOT_ID/feedback`
    * *Purpose:* Notifies the Hub when asymmetric operations (such as a precise rotation) complete.

---

### 📝 JSON Payload Formats & Mappings

The operations defined in `operation.h` are processed natively on the Arduino by parsing a standard JSON object containing an operational integer identifier (`op`) along with its respective parameter object.

#### 📐 OP_MOVE_ROBOT (Code 2)
Directly updates the global unicycle kinematics targets via Wi-Fi.
* **Incoming Payload Format:**
    ```json
    {
      "op": 2,
      "v": 0.25,
      "w": -0.1
    }
    ```
* **Behavior:** Extracts `v` (linear speed in $m/s$) and `w` (angular speed in $rad/s$). It instantly routes the calculated individual wheel goals down to the PID and FeedForward loop.

#### 🏎️ OP_MOVE_WHEELS (Code 11)
Bypasses chassis geometric equations to command target angular speeds straight to the left and right wheel controllers.
* **Incoming Payload Format:**
    ```json
    {
      "op": 11,
      "wl": 12.5,
      "wr": 12.5
    }
    ```
* **Behavior:** Sets left and right wheel speeds directly in $rad/s$. Values below `MINSETPOINT` (5.5 $rad/s$) are clipped to zero or scaled to avoid physical motor stall dead-zones.

#### 📊 OP_TELEMETRY (Code 4)
Asynchronously serializes internal states and publishes them back to the Hub's telemetry broker pipeline.
* **Outgoing Payload Format:**
    ```json
    {
      "id": 8,
      "battery": 11.8,
      "wl_real": 12.41,
      "wr_real": 12.45,
      "imu": {
        "ax": 0.02,
        "ay": -0.01,
        "az": 0.98,
        "gx": 0.0,
        "gy": 0.1,
        "gz": -0.05
      }
    }
    ```
* **Behavior:** Sends the robot's battery state, smoothed wheel speeds computed from encoder intervals passed through a 10-sample moving average filter (`MeanFilterLib`), and raw IMU readings stabilized through an onboard `SimpleKalmanFilter`.

#### 🔄 OP_TURN_ROBOT (Code 5) & OP_DONE (Code 10)
Handles standalone, precise on-axis target rotation maneuvers.
* **Incoming Request (`OP_TURN_ROBOT`):**
    ```json
    {
      "op": 5,
      "angle": 1.5708
    }
    ```
* **Outgoing Completion Callback (`OP_DONE`):**
    Once the encoder counts match the computed arc length ($\text{ticks} = f(\text{angle}, \text{TURN\_CORRECTION\_FACTOR})$), the `SMALLBOT` drops into active wheel braking and immediately dispatches an acknowledgement packet to the feedback channel:
    ```json
    {
      "status": "success",
      "op": "OP_TURN_ROBOT",
      "payload": {
        "message": "Rotation target reached successfully"
      }
    }
    ```

#### ⚙️ OP_CONF_PID (Code 8)
Enables over-the-air (OTA) calibration of the tracking loops without requiring a USB cable or a firmware re-flash.
* **Incoming Payload Format:**
    ```json
    {
      "op": 8,
      "kp": 5.0,
      "ki": 2.5,
      "kd": 0.0
    }
    ```
* **Behavior:** Modifies the proportional, integral, and derivative parameters inside both left and right `controler` objects on-the-fly, allowing you to instantly observe chattering or tracking improvements.


## ⚙️ Motion Control, Kinematics & Tracking Loops

The firmware implements an independent real-time tracking algorithm running inside a fixed-interval **100Hz (10ms)** timer block (this interval can be defined through the **SAMPLINGTIME** constant). To ensure smooth trajectories and quick step response without stalling or steady-state errors, each motor utilizes a unified **Dual-Loop Strategy** coupling a predictive model with an active error corrector.

### 1. Dual-Loop Architecture
Instead of relying solely on error-driven feedback, the motor velocity controller splits the duty cycle assignment into two parallel processes:

$$\text{PWM}_{\text{Total}} = \text{PWM}_{\text{FeedForward}} + \text{PWM}_{\text{PID}}$$

* **FeedForward Loop (Predictive Model):** Approximates the required voltage base curve based on steady-state motor characterization parameters ($A$ and $B$) configured per-robot inside `config.h`. This bypasses traditional controller delay by instantly supplying the baseline voltage required to hit a desired angular velocity ($\omega_{\text{setpoint}}$):
  $$\text{PWM}_{\text{FeedForward}} = A \cdot \omega_{\text{setpoint}} + B$$

* **PID Loop (Dynamic Corrector):** Computes a Proportional-Integral-Derivative correction signal based on the operational error ($e(t) = \omega_{\text{setpoint}} - \omega_{\text{real}}$). 
  * *Deadband Threshold:* To prevent erratic continuous motor jitter (chattering), correction is dynamically bypassed if the absolute error remains negligible: $|e(t)| < 0.3 \text{ rad/s}$.
  * *Anti-Windup Protection:* To prevent large integral error accumulation if a robot hits an obstacle or gets trapped, the running summation of errors is clamped to a strict saturation bound:
    $$\text{constrain}(\text{cumError}, -14, 14)$$

---

### 2. Differential Kinematics Mapping

When a high-level command is dispatched via `OP_MOVE_ROBOT`, the firmware parses targeted linear ($v$ in $m/s$) and angular ($w$ in $rad/s$) goals. It applies standard differential drive forward/inverse kinematics using the unique physical layout properties of the chassis (`ROBOT_DIAMETER` or track width $L$, and `ROBOT_WHEEL_DIAMETER` or radius $R$):



                   v (Linear Target)  w (Angular Target)
                           \               /
                            ▼             ▼
                 ┌──────────────────────────────────┐
                 │ Inverse Differential Kinematics  │
                 └──────────────────────────────────┘
                          /                \
                         ▼                  ▼
                   Left Wheel          Right Wheel
                 Target (rad/s)       Target (rad/s)


$$\omega_{\text{Left}} = \frac{v - \left( w \cdot \frac{L}{2} \right)}{R}$$

$$\omega_{\text{Right}} = \frac{v + \left( w \cdot \frac{L}{2} \right)}{R}$$

* **Dead-Zone Prevention:** Low setpoint constraints are automatically applied. If an active velocity instruction maps below the mechanical slip boundary (`MINSETPOINT = 5.5 rad/s`), the target is adjusted to clip or force minimum momentum to avoid physical motor stall situations.

---

### 3. Advanced Quadrature Encoder Processing (`ENCODER_CUADRATURA`)

For robots with quadrature encoders macro constant **ENCODER_CUADRATURA** must be defined.

For high-accuracy velocity tracking, the hardware layer handles rotary encoder telemetry using a highly performant **4x State Machine Decoding Mode**. This method is tied directly to external micro-controller pins via hardware interrupts (`attachInterrupt`).

* **4x Quadrature Decoding:** By tracking every logic transition (rising and falling edges) across both **Channel A** and **Channel B**, the firmware quadruples the default hardware Pulses Per Revolution (PPR). This vastly maximizes precision during ultra-low speed maneuvers.

* **Smoothing Filters:** Raw time periods measured between encoder counts ($\Delta t$) can suffer from intermittent hardware jitter. The incoming stream passes through a 10-sample moving average window filter (`MeanFilterLib`) to output clean, noise-free $rad/s$ data arrays to the PID controller and telemetry topics.

If **ENCODER_CUADRATURA** is not defined a non-quadrature encoder is coded with a dobunce filter.

* **Hardware Debounce Engine:** To eliminate false readings induced by high-frequency electrical noise, electromagnetic interference (EMI) from the brush motors, or vibrations, a strict hardware debounce timer limit (`TIMEDEBOUNCE = 12ms`) is evaluated between consecutive interrupt slices.

## ⚙️ Configuration Parameters (`config.h`)

The `config.h` file acts as the central **Hardware Abstraction Layer (HAL)** configuration hub for the entire fleet. Before compiling and uploading the firmware to an agent, you must uncomment its specific `ROBOT_ID`. The file automatically handles conditional compilation profiles based on that ID.

---

### 1. Global & Environment Constants

These parameters apply globally to all robots regardless of their chassis profile:

* **`SAMPLINGTIME` (`0.01`)**: Defines the target sampling interval for the real-time execution loop (10ms / 100Hz).
* **`DEBUG_ENABLED`**: When left uncommented, it opens the hardware serial line (`Serial`) to pipe diagnostic logs to an attached PC. *Note: For production swarm deployments, comment this out to optimize execution speed.*

---

### 2. Robot Profile Defs (`#define` Swaps)

Depending on the assigned `ROBOT_ID`, the file injects the environmental structure variables needed by the compiler:

* **`BIGBOT` / `SMALLBOT`**: 
    * `BIGBOT`: Compiles the codebase to wait for binary `appdata` packets over the serial UART lines connected to an onboard Raspberry Pi.
    * `SMALLBOT`: Compiles the native `WiFiNINA` stack to route telemetry directly to the network broker.
* **`ARDUINO_TYPE_MKR` / `ARDUINO_TYPE_NANO`**: Maps the physical pin registers to either the SAMD21 (MKR WiFi 1010) architecture or the AVR/SAM architecture on the Nano 33 IoT.
* **`H_BRIDGE_BLACK` / `H_BRIDGE_RED`**: Shifts internal motor driver configurations. Different H-bridges in the fleet (such as L298N vs. specific Red PCB boards) change the forward/reverse logic combinations mapped to the motor pins in `robot.cpp`.
* **`ENCODER_CUADRATURA`**: Activates the dual-channel state-machine interrupt engine. This turns on 4x decoding mode rather than single-edge pulse counting.

---

### 3. Kinematic Layout Parameters

These variables alter physical dimensions used by the core equations:

| Parameter | Type | Description |
| :--- | :---: | :--- |
| `ROBOT_WHEEL_DIAMETER` | `double` | The raw physical outer diameter of the traction wheels (in cm, e.g., `6.7`). Used to derive the wheel radius $R$. |
| `ROBOT_DIAMETER` | `double` | The wheel track width ($L$) measured from the center contact point of the left wheel to the right wheel (in cm, e.g., `14.5`). |

---

### 4. Actuator Limits & Dead-Zone Bounds

To protect the low-cost DC motors and provide clean control constraints, the following saturation cut-offs are configured:

* **`MINPWM`** (typically `30` or `100`) & **`MAXPWM`** (`255`): The physical limits bounded for the micro-controller's `analogWrite()` hardware registers. Any PID output exceeding these values is constrained.
* **`MINSETPOINT`** (`5.5` $rad/s$): The minimal allowable setpoint speed. If a kinematic calculation demands a velocity lower than this threshold, the loop drops it to zero or scales it to prevent the wheels from stalling and buzzing under low torque.
* **`VRMIN`** / **`VLMIN`**: Individual minimum experimental physical velocities observed during characterization runs.

---

### 5. Controller Calibration Parameters (FeedForward & PID)

Because every low-cost motor behaves uniquely due to gearbox wear, manufacturing tolerances, and aging, each robot has unique calibration variables:

```cpp
// Example: Specific Wheel Configuration Block inside config.h
#define A_L 9.12      // FeedForward Slope (Left)
#define B_L 18.90     // FeedForward Intercept (Left)
#define KP_L 5.0      // Proportional Gain (Left)
#define KI_L 2.5      // Integral Gain (Left)
#define KD_L 0.0      // Derivative Gain (Left)
```

### 🛠️ Actuator & Controller Calibration (FeedForward & PID)

Because every low-cost motor behaves uniquely due to manufacturing tolerances, gearbox wear, and physical aging, each robot requires individual parameter tuning. These coefficients are isolated under each specific `ROBOT_ID` block.


              ┌──────────────────────────────────────────────┐
              │          Target Setpoint (rad/s)             │
              └──────────────────────┬───────────────────────┘
                                     │
                ┌────────────────────┴────────────────────┐
                ▼                                         ▼
     ┌─────────────────────┐                   ┌─────────────────────┐
     │  FeedForward Loop   │                   │   PID Controller    │
     │ (Predictive Voltage)│                   │ (Reactive Dynamics) │
     │   PWM = A * w + B   │                   │    Kp, Ki, Kd       │
     └──────────┬──────────┘                   └──────────┬──────────┘
                │                                         │
                └────────────────────┬────────────────────┘
                                     ▼
              ┌──────────────────────────────────────────────┐
              │          Combined PWM Signal Out             │
              └──────────────────────────────────────────────┘


#### 🏎️ FeedForward Parameters (`A_L`, `B_L`, `A_R`, `B_R`)
The FeedForward loop acts as the predictive engine of the system. It maps the desired angular velocity ($\omega$) directly to a baseline PWM duty cycle, bypassing traditional controller lag by overcoming physical friction instantly.

* **`A_L` / `A_R` (Slope Coefficients):** The linear scaling factor that relates target angular speed ($rad/s$) to the hardware PWM value. A higher value indicates a motor that requires more power increments to increase its rotational frequency.
* **`B_L` / `B_R` (Intercept / Deadband Offsets):** The minimum baseline PWM voltage command required to break static drivetrain friction and initiate actual wheel movement. 
  > ⚠️ **Note:** Depending on the efficiency of the custom H-Bridge and motor configuration, this value can be negative (e.g., `#define B_R -53.48`) to offset internal voltage drops, or positive (e.g., `#define B_R 19.0`) to kickstart high-friction gearboxes.

---

#### 📈 PID Tuning Coefficients (`KP_L`, `KI_L`, `KD_L`, etc.)
The closed-loop Proportional-Integral-Derivative controller runs simultaneously to cancel out real-world tracking errors ($e(t) = \omega_{\text{setpoint}} - \omega_{\text{real}}$) caused by payload shifts, wheel slippage, or battery voltage drops.

* **`KP_L` / `KP_R` (Proportional Gain):** Determines the immediate reactive force. It multiplies the current error to quickly push the motor toward the target speed. Higher values speed up the system response but can cause chattering or overshooting if set too high.
* **`KI_L` / `KI_R` (Integral Gain):** Eliminates accumulated steady-state tracking errors over time. If a motor struggles to keep up with a constant velocity command, the integral component steadily increases the PWM signal until the error is eliminated.
  * *Safety Guardrail:* The running integral sum is actively clamped using an anti-windup bound inside `controler.cpp` via `constrain(this->cumError, -maxIntegralError, maxIntegralError)` (where `maxIntegralError = 14`) to prevent catastrophic over-saturation if a wheel stalls.
* **`KD_L` / `KD_R` (Derivative Gain):** Provides a dampening factor by reacting to the rate of change of the error. In the default fleet configuration, this parameter is set to `0.0` to maintain stability and prevent encoder sensor noise from rattling the motor commands at a high loop rate (100Hz).

🚀 Installation & Calibration

1. Dependencies: 

Install via Library Manager: WiFiNINA, ArduinoMqttClient, ArduinoJson, SimpleKalmanFilter, MeanFilterLib.

3. Encoder Check: Run TestEncoder.ino and rotate wheels manually. Ensure countsL/R increase when moving forward.

4. Tuning: Use AjusteControlador.ino to find your motor's $A$ and $B$ FeedForward constants.

5. Production: Flash the code in the /Robot folder for full swarm integration.

----------------------------------------------------------------------------------------------------------
UCM Robotarium Project | Faculty of Computer Science | Complutense University of Madrid.

