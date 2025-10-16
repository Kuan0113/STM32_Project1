# Lab 5: Thread-Safe Task Communication with FreeRTOS Queues

## 📝 Objective

This project serves as a critical upgrade to the previous lab's architecture. Following the "Week 5 Tasks" guide, the primary objective is to refactor the inter-task communication model from a non-thread-safe global variable to a robust, industry-standard **FreeRTOS Queue**.

This lab accomplishes two main goals as specified in the course material:
1.  **Eliminate Race Conditions:** By using a message queue, we ensure data is passed between tasks in a safe, atomic manner, preventing data corruption.
2.  **Migrate to Native FreeRTOS API:** The project is updated to use native FreeRTOS functions (e.g., `xTaskCreate`, `xQueueSend`, `vTaskDelay`) instead of the CMSIS-OS abstraction layer, providing more direct control and a deeper understanding of the RTOS kernel.

---

## 🛠️ Hardware & Software Requirements

The hardware and software requirements remain identical to Lab 4.

* **Hardware:**
    * STM32 Nucleo Development Board
    * ISL29125 RGB Color Light Sensor
    * External LED
* **Software:**
    * STM32CubeIDE
    * STM32 HAL & Native FreeRTOS Libraries
    * Serial Terminal (Baud Rate: **115200**)

---

## 🏛️ System Architecture Overhaul

The core producer-consumer model is preserved, but the link between the tasks has been fundamentally re-engineered for safety and efficiency.

### Key Architectural Changes

1.  **Data Encapsulation with a Struct:**
    To pass all relevant information as a single message, a `SensorData_t` struct was created. This bundles the raw sensor values and the resulting color classification into one atomic packet.

    ```c
    typedef struct {
        int r;
        int g;
        int b;
        char color[16];
    } SensorData_t;
    ```

2.  **Implementation of a FreeRTOS Queue:**
    A FreeRTOS queue, `sensorQueue`, was created to act as a thread-safe "mailbox." It is configured to hold a single `SensorData_t` item, ensuring the `ControlTask` always works with the most recent sensor reading.

3.  **Adoption of Native FreeRTOS API:**
    All CMSIS-OS calls were replaced with their native FreeRTOS counterparts as instructed. For example:
    * `osThreadNew` → `xTaskCreate`
    * `osMessageQueueNew` → `xQueueCreate`
    * `osDelay` → `vTaskDelay(pdMS_TO_TICKS(ms))`

### New Task Logic

The producer-consumer data flow is now mediated by the queue, which decouples the tasks and ensures safe data transfer.


1.  **`SensorTask` (Producer)**
    * **Priority:** 2 (Higher)
    * **Function:** This task periodically reads from the ISL29125 sensor every 500ms.
        1.  It populates a **local** `SensorData_t` struct with the sensor readings and the detected color.
        2.  It sends a **copy** of this struct to the `sensorQueue` using `xQueueSend()`. If the queue is full, the old data is overwritten, ensuring freshness.
        3.  It prints its findings to the serial port for debugging.

2.  **`ControlTask` (Consumer)**
    * **Priority:** 1 (Lower)
    * **Function:** This task is now event-driven and much more efficient.
        1.  It calls `xQueueReceive()`, which causes the task to **block** (enter a sleeping state, consuming zero CPU) until a message becomes available in the queue.
        2.  When the `SensorTask` sends data, the `ControlTask` unblocks and receives a clean copy of the `SensorData_t` struct.
        3.  It then performs its control logic (turning the LED on/off) based on the `color` field within the received struct.
        4.  It prints a confirmation message, showing the data it received, before looping to wait for the next message.

---

## 🚀 Results & Demo

The system's external behavior is the same as Lab 4, but the internal workings are now robust. The serial output clearly shows the two tasks communicating through the queue.

**Example Serial Output:**
```
// Message sent by SensorTask
R=80 G=252 B=95 | Detected: GREEN
// Message from ControlTask after receiving from the queue
Control: Received GREEN (R:80, G:252, B:95). Trigger: 1

R=245 G=248 B=191 | Detected: YELLOW
Control: Received YELLOW (R:245, G:248, B:191). Trigger: 1
```

---

## 🎓 Professional Analysis & Key Learnings

This lab successfully implements the professional standard for passing data between FreeRTOS tasks.

* **Thread Safety:** The primary goal has been achieved. The queue is managed by the RTOS kernel, which guarantees that data access is atomic. This completely **eliminates the risk of race conditions** and ensures data integrity.

* **Task Decoupling:** The tasks are now highly decoupled. The `SensorTask` simply sends data to a queue without needing to know what task (if any) is listening. This modular design makes the system easier to debug, maintain, and expand in the future.

* **CPU Efficiency:** The `ControlTask` is no longer a "polling" task that wastes CPU cycles by constantly checking a variable. By blocking on `xQueueReceive()`, it becomes an **event-driven task** that only consumes CPU resources when new data is available and there is actual work to be done.

* **Native API Experience:** By working directly with the native FreeRTOS API, we gain a more precise and powerful level of control over the RTOS, which is essential for optimizing complex embedded applications.
