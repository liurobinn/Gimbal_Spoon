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
* **Telemetry:** Broadcasts pitch and yaw error as a packed 8-byte binary struct over BLE.

### 2. Telemetry Node (ESP32-C6)
* **Role:** GATT Client & Remote HUD.
* **Communication:** Connects via **BLE** to receive telemetry.
* **Display:** uLCD-144-G2 Graphics Display via **9600 Baud UART**.
* **Processing:** Implements a time-based accumulator to downsample 100Hz BLE data to a 20 FPS (50ms) graphical waveform, preventing UART buffer overflows.
* **User Input:** Digital input toggle (GPIO 1) with hardware interrupt to power the display on/off via a physical hardware reset.

---

## Hardware Specifications
| Component | Protocol | Spec / Rate |
| :--- | :--- | :--- |
| **BNO08x IMU** | SPI | 100Hz Report Rate |
| **KST X08 Servos** | PWM | 333Hz Update Frequency |
| **BLE Stream** | Bluetooth LE | 100Hz @ 8-bytes/packet |
| **uLCD-144-G2** | UART | 9600 Baud / 20 FPS Refresh |
| **Center Range** | PWM | 1150µs - 1450µs (1300µs Center) |
| **Power** | DC | Fully Battery Powered |

---

## Software Features
* **Interrupt-Driven I/O:** Both systems utilize Hardware Interrupt Service Routines (ISRs) for user buttons and sensor data-ready signals to minimize latency jitter.
* **Binary Data Serialization:** Uses `__attribute__((packed))` C-structs for over-the-air transmission to maximize throughput and minimize BLE packet overhead.
* **Zero-Gimbal Lock:** Processes 4D Quaternions from the BNO08x to establish orientation before converting to Euler angles (Pitch/Yaw) for stabilization.
* **Smart Downsampling:** The C6 node intelligently averages high-frequency telemetry into a smooth, human-readable curve on the uLCD using a non-blocking accumulator.

---

## Installation & Usage
1.  **TinyS3:** Flash using the Arduino IDE with `ESP32-S3` board support. Required libraries: `Adafruit_BNO08x`, `ESP32Servo`, `BLEDevice`.
2.  **ESP32-C6:** Flash using the Arduino IDE with `ESP32-C6` board support. Required library: `Goldelox_Serial_4DLib`.
3.  **Operation:**
    * Power on both units. The C6 will automatically scan and connect to the TinyS3.
    * Hold the spoon level and press the **Calibration Button** on the TinyS3.
    * The system will stabilize. Use the **Display Button** on the C6 to view or hide the real-time tremor intensity graph.

---

**Developed at the Georgia Institute of Technology**
*Robin Liu School of Electrical Engineering & School of Aerospace Engineering*
