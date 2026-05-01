# Gimbal_Spoon
**An Active Tremor-Canceling Stabilizer for Assisted Eating**

Developed for **ECE 4180: Embedded Systems Design** at the Georgia Institute of Technology, the Gimbal_Spoon is a high-performance, distributed embedded system designed to counteract hand tremors. By utilizing a 9-DOF IMU and high-speed digital servos, the system maintains a level spoon orientation in real-time to assist users with motor impairments.

## System Architecture
The project employs a **Distributed BLE Client-Server Architecture** to decouple time-critical motor control from high-overhead graphics rendering.

### 1. Control Node (TinyS3)
* **Role:** GATT Server & Primary Controller.
* **Sensing:** BNO08x 9-DOF IMU via **SPI** (3MHz clock).
* **Actuation:** Dual KST X08 Digital Servos via **333Hz PWM**.
* **Logic:** 100Hz interrupt-driven sensor fusion and PID-lite stabilization.
* **User Input:** Hardware-interrupt-driven calibration button (GPIO 3) for reference zeroing.
* **Persistence:** Utilizes **Non-Volatile Storage (NVS)** to save and load calibration offsets across power cycles.

### 2. Telemetry Node (ESP32-C6)
* **Role:** GATT Client & Remote HUD.
* **Communication:** Connects via **BLE** to receive telemetry.
* **Display:** uLCD-144-G2 Graphics Display via **9600 Baud UART**.
* **Processing:** Implements a time-based accumulator to downsample 100Hz BLE data to a 20 FPS (50ms) graphical waveform, preventing UART buffer overflows.
* **User Input:** Digital input toggle (GPIO 1) with hardware interrupt to power the display on/off via physical hardware reset.

---

## Hardware Specifications
| Component | Protocol | Spec / Rate |
| :--- | :--- | :--- |
| **Microcontrollers** | BLE 5.0 | ESP32-S3 (Server) & ESP32-C6 (Client) |
| **IMU (BNO08x)** | SPI | 100Hz Report Rate |
| **Servos (KST X08)** | PWM | 333Hz Update Frequency |
| **Display (uLCD-144)**| UART | 9600 Baud (Software Throttled) |
| **Servo Range** | PWM | 1150µs - 1450µs (1300µs Center) |
| **Buttons** | Interrupt | Hardware ISR with 250-300ms Debounce |

---

## Pin Mapping

### TinyS3 (Controller)
| Function | Pin | Notes |
| :--- | :--- | :--- |
| **SPI SCK / MISO / MOSI**| 36 / 37 / 35 | High-speed IMU Bus |
| **IMU CS / INT / RST** | 34 / 21 / 7 | Interrupt-driven flow |
| **Pitch / Yaw Servos** | 9 / 8 | 333Hz PWM Output |
| **Calibration Button** | 3 | Pull-up, Hardware Interrupt |

### ESP32-C6 (HUD)
| Function | Pin | Notes |
| :--- | :--- | :--- |
| **uLCD RX / TX** | 16 / 17 | 9600 Baud UART |
| **uLCD Reset** | 15 | Physical power-cycle toggle |
| **Display Button** | 1 | ISR Display On/Off Toggle |

---

## Software Features
* **Interrupt-Driven I/O:** Both systems utilize Hardware Interrupt Service Routines (ISRs) for user buttons and sensor data-ready signals to minimize latency jitter.
* **Binary Data Serialization:** Uses `__attribute__((packed))` C-structs for over-the-air transmission to maximize throughput and minimize BLE packet overhead.
* **Zero-Gimbal Lock:** Processes 4D Quaternions from the BNO08x to establish orientation before converting to Euler angles (Pitch/Yaw) for stabilization.
* **Smart Downsampling:** The C6 node intelligently averages high-frequency telemetry into a smooth, human-readable curve on the uLCD using a non-blocking accumulator.

### Motion Magnitude Calculation
The HUD visualizes the absolute tremor intensity using the Root-Sum-Square (RSS) of the axial errors. This provides a single, intuitive metric for tremor magnitude:

$$Magnitude = \sqrt{pitch^2 + yaw^2}$$

---

## 📂 Project Structure
* `/TinyS3_Controller`: Firmware for the main gimbal control and BLE server.
* `/ESP32C6_HUD`: Firmware for the remote display client and telemetry graph.
* `/CAD`: Design files for the gimbal frame and spoon attachment.

## 🔧 Installation & Usage
1.  **TinyS3:** Flash the `/Controller` code. 
    * Required Libraries: `Adafruit_BNO08x`, `ESP32Servo`, `BLEDevice`.
2.  **ESP32-C6:** Flash the `/HUD` code. 
    * Required Library: `Goldelox_Serial_4DLib`.
3.  **Operation:**
    * Power both units. Connection is automatic via BLE Scan/Connect.
    * Hold the spoon level and press the **TinyS3 Button** to calibrate.
    * Use the **C6 Button** to toggle the graphical HUD.

---

**Developed at the Georgia Institute of Technology**
*Robin Liu: School of Electrical Engineering & School of Aerospace Engineering*
*MJ Wallce: School of Electrical Engineering
