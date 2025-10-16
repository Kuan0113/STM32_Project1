# Lab 4: Real-Time Color Detection and Control with FreeRTOS

## 📝 Objective

This project demonstrates a fundamental real-time embedded system concept: a **producer-consumer model** implemented using FreeRTOS on an STM32 microcontroller. The system uses an ISL29125 color sensor to detect colors and controls an external LED based on the sensor's readings. The entire process is managed by two concurrent tasks to separate sensing from actuation.

---

## 🛠️ Hardware Requirements

* **Microcontroller:** STM32 Nucleo Development Board (e.g., NUCLEO-L476RG)
* **Sensor:** ISL29125 RGB Color Light Sensor
* **Actuator:** 1x External LED
* **Other:** Breadboard, Jumper Wires, and a USB cable for power and serial communication.

---

## 💻 Software Requirements

* **IDE:** STM32CubeIDE
* **Frameworks/Libraries:**
    * STM32 HAL (Hardware Abstraction Layer)
    * FreeRTOS (via the CMSIS-OS v2 wrapper)
* **Serial Terminal:** A serial terminal program (e.g., PuTTY, Tera Term, VS Code Serial Monitor) to view the output at a baud rate of **115200**.

---

## 🏛️ System Architecture

The application is built on two distinct FreeRTOS tasks that run concurrently, communicating through a shared global variable.



1.  **`ReadRGB` Task (The Producer)**
    * **Priority:** `osPriorityNormal`
    * **Function:** This task is responsible for interfacing with the physical sensor. Every **500 milliseconds**, it performs the following actions:
        1.  Reads the raw 16-bit Red, Green, and Blue color values from the ISL29125 sensor over the I2C bus.
        2.  Scales these values down to an 8-bit range (0-255).
        3.  Passes the RGB values to the `DetectColor` function to classify the color.
        4.  **Produces data** by updating the `detectedColor` global string variable.
        5.  Transmits the raw RGB values and the detected color name over UART for debugging.

2.  **`ControlTask` (The Consumer)**
    * **Priority:** `osPriorityLow`
    * **Function:** This task is responsible for actuation based on the data from the sensor. It runs more frequently, every **200 milliseconds**:
        1.  **Consumes data** by reading the `detectedColor` global string variable.
        2.  Analyzes the string to make a decision. If the color is detected as `"GREEN"` or `"YELLOW"`, it sets the LED state to ON. Otherwise, the state is OFF.
        3.  Calls the `Actuator_SetLED` function to update the physical state of the external LED.

### Inter-Task Communication

Communication between the two tasks is achieved using a simple shared global variable: `char detectedColor[16];`. The `ReadRGB` task writes to this variable, and the `ControlTask` reads from it.

> **Note:** While simple, this method is not thread-safe. See the "Professional Analysis" section for a more robust alternative.

---

## 🚀 Results & Demo

When the program runs, the serial terminal will display a continuous stream of color data from the sensor.

**Example Serial Output:**
```
R=80 G=252 B=95 | Detected: GREEN
R=80 G=252 B=95 | Detected: GREEN
R=245 G=248 B=191 | Detected: YELLOW
R=50 G=70 B=65 | Detected: BLACK
```

Simultaneously, the external LED connected to the `Out_LED_Pin` will:
* **Turn ON** when the sensor detects a green or yellow object.
* **Turn OFF** for all other colors.



---

## 🧐 Professional Analysis & Future Improvements

The current implementation is functional but has areas that could be improved for robustness and efficiency, especially in a production environment.

### 1. Improve Thread Safety with a Queue

* **Problem:** The use of a shared global variable (`detectedColor`) for inter-task communication is not thread-safe and can lead to **race conditions**. It's possible for the `ControlTask` to read the variable while `ReadRGB` is in the middle of writing to it, resulting in corrupted data.
* **Solution:** The standard FreeRTOS solution is to use a **Queue**. A queue is a thread-safe data structure for passing messages between tasks.
    * The `ReadRGB` task would `xQueueSend()` the detected color string to the queue.
    * The `ControlTask` would use `xQueueReceive()` to wait for a new message. This is more efficient as the task remains blocked (consuming zero CPU) until data is available, instead of polling every 200ms.

### 2. Make Color Detection More Robust

* **Problem:** The `DetectColor` function relies on hardcoded "magic numbers" for RGB ranges. This logic is highly sensitive to ambient lighting, object distance, and material reflectivity. A slight change in conditions can easily break the detection.
* **Solution:**
    * **Use the HSV Color Space:** Convert the RGB values to HSV (Hue, Saturation, Value). The **Hue** component is a much more reliable indicator of pure color, regardless of its brightness or intensity. It is far easier to define a stable range for "green" using Hue than with RGB.
    * **Implement a Calibration Routine:** Add a function that runs at startup. The user would place a known green object in front of the sensor, and the program would record the resulting color values to dynamically set the detection thresholds for the current lighting conditions.
