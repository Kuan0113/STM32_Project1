# Final Demo - Event-Driven Color Sorter using FreeRTOS

## 🚀 Project Overview
This is a real-time embedded application for the STM32 platform that demonstrates a modular, event-driven system using FreeRTOS. The project integrates multiple tasks to perform color sensing, control logic, actuator output, and centralized logging.

The core of this design follows the "Week 6" assignment, implementing an efficient logging system where producer tasks (Sensor, Control, Actuator) send text-based messages to a queue. A dedicated, high-priority PrintTask (the "Logging Task") is signaled by a binary semaphore and uses a mutex to safely write these messages to the UART console (our "log file").

## 🛠️ System Architecture
The system is built on four main tasks, each with a distinct responsibility. Communication and synchronization between tasks are managed by FreeRTOS queues, a binary semaphore, and a mutex.

* `SensorTask` (Priority 2): Reads from the ISL29125 color sensor.

* `ControlTask` (Priority 1): Performs all decision-making.

* `ActuatorTask` (Priority 2): Controls the output hardware (LED and Buzzer).

* `PrintTask` (Priority 3): The "Logging Task". Manages all UART console output.

## 📌 Task Descriptions & Responsibilities

* StartReadRGB **(SensorTask)**
  * Priority: 2 (Medium)
  
  * Reads RGB values from the ISL29125 sensor every 500ms.
  
  * Sends the raw `SensorData_t` struct to the `ControlTask` via `sensorQueue`.
  
  * Formats a human-readable string (e.g., "R=... G=... B=...") and sends a pointer to it to the `LogQueue`.
  
  * Gives the `LogSemaphore` to signal the `PrintTask`.

* StartControlTask **(ControlTask)**
  * Priority: 1 (Lowest)
  
  * Waits for sensor data from `sensorQueue` and non-blockingly checks the UART for the `*` character to trigger the user menu.
  
  * Compares the detected color from the sensor with the user's selectedColor.
  
  * Sends an `ActuatorCommand_t` (ON or OFF) to the `actuatorQueue`.
  
  * Sends a log message (e.g., "[CONTROL LOG] Match Found!") to the `LogQueue`.
  
  * Gives the `LogSemaphore` to signal the `PrintTask`.

* StartActuatorTask **(ActuatorTask)**
  * Priority: 2 (Medium)
  
  * Waits indefinitely for commands on the `actuatorQueue`.
  
  * Controls the `Out_LED_Pin` and the buzzer (TIM1 PWM) based on the received command.
  
  * Only sends a log message to the `LogQueue` (and gives `LogSemaphore`) if its state (ON/OFF) changes, to prevent spamming the log.

* StartPrintTask **(Logging Task)**
  * Priority: 3 (Highest)
  
  * This task functions as the "Logging task" described in the lab 6 handout.
  
  * It sleeps by pending indefinitely on the `LogSemaphore`.
  
  * When signaled, it wakes up and completely drains the `LogQueue` in a while loop.
  
  * For each message from the queue, it acquires the `LogMutex`, prints the message to the UART, and then releases the `LogMutex`.
 
  Here is a complete README.md file for your final project, structured like your example and including the code snippets you requested.



## 🔗 Communication and Synchronization
* sensorQueue: (SensorTask → ControlTask) Carries raw SensorData_t structs for processing.

* actuatorQueue: (ControlTask → ActuatorTask) Carries ActuatorCommand_t (ON or OFF) to trigger the actuators.

* LogQueue: (All Tasks → PrintTask) Carries char* (pointers to static text buffers) for logging and display.

* LogSemaphore: (Producers → PrintTask) A binary semaphore used to signal the PrintTask that a new log message is ready in the LogQueue.

* LogMutex: (Used by PrintTask) A true mutex (xSemaphoreCreateMutex()) that ensures only the PrintTask can access the HAL_UART_Transmit function at any given time, preventing garbled output.

## 📋 Key Code Snippets

RTOS Object Creation [`main.c`](https://github.com/Kuan0113/STM32_Project1/blob/main/project_final/project_final/Core/Src/main.c)
This is where the queues, semaphore, and mutex are created before the scheduler starts.

```c
/* Create the semaphores(s) */
/* USER CODE BEGIN RTOS_SEMAPHORES */
// Create the binary semaphore used to signal the PrintTask
LogSemaphore = xSemaphoreCreateBinary();
/* USER CODE END RTOS_SEMAPHORES */

/* Create the mutex(es) */
/* USER CODE BEGIN RTOS_MUTEXES */
// Create the mutex to protect the UART
LogMutex = xSemaphoreCreateMutex();
/* USER CODE END RTOS_MUTEXES */

/* Create the queue(s) */
/* USER CODE BEGIN RTOS_QUEUES */
sensorQueue = xQueueCreate(1, sizeof(SensorData_t));
actuatorQueue = xQueueCreate(5, sizeof(ActuatorCommand_t));
// Create the log queue to hold text-based messages (pointers to strings)
LogQueue = xQueueCreate(15, sizeof(char*));
/* USER CODE END RTOS_QUEUES */
```
The "Logging Task" Pattern (StartPrintTask)
This task shows the core logic of the assignment: wait for a signal, then drain the queue using a mutex.

```c
void StartPrintTask(void *argument)
{
  char *pMessageToPrint;

  for(;;)
  {
    // 1. Wait to be signaled by the LogSemaphore
    if (xSemaphoreTake(LogSemaphore, portMAX_DELAY) == pdTRUE)
    {
      // 2. A signal was received. Drain the LogQueue completely.
      while (xQueueReceive(LogQueue, &pMessageToPrint, 0) == pdTRUE)
      {
        // 3. Acquire the LogMutex to protect the UART
        if (xSemaphoreTake(LogMutex, portMAX_DELAY) == pdTRUE)
        {
          // 4. Write the log message to the "file" (UART)
          HAL_UART_Transmit(&huart2, (uint8_t*)pMessageToPrint, strlen(pMessageToPrint), HAL_MAX_DELAY);
          
          // 5. Release the mutex
          xSemaphoreGive(LogMutex);
        }
      }
    }
  }
}
```

A Producer Task (StartControlTask)
This snippet shows how the ControlTask sends data to both the ActuatorTask and the PrintTask, signaling the PrintTask with the semaphore.

```c
void StartControlTask(void *argument)
{
    // ... (variable declarations) ...
    static char logMsg[128];
    char *pLogMsg = controlLogMsg; 

    for(;;)
    {
        // ... (check for menu and receive from sensorQueue) ...

        if (xStatus == pdPASS) // If sensor data was received
        {
            // Perform matching logic
            if (strcmp(received_data.color, selectedColor) == 0) {
                actuatorCmd = ACTUATOR_CMD_ON;
                snprintf(pLogMsg, 128, "[CONTROL LOG] Match Found! Detected: %s\r\n", received_data.color);
            } else {
                actuatorCmd = ACTUATOR_CMD_OFF;
                snprintf(pLogMsg, 128, "[CONTROL LOG] No Match. Detected: %s\r\n", received_data.color);
            }

            // Send command to ActuatorTask
            xQueueSend(actuatorQueue, &actuatorCmd, 0);

            // Send log message to PrintTask
            xQueueSend(LogQueue, &pLogMsg, 0);
            xSemaphoreGive(LogSemaphore);
        }
         vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
```
## 🖥️ How to Use
Compile and flash the code to the STM32 board.

Connect to the board using a serial terminal (like PuTTY) at 115200 baud.

You will see a continuous stream of sensor readings printed to the console.

The system defaults to matching the color WHITE.

Place a white object over the sensor. You should see the Out_LED_Pin light up and the buzzer turn on.

To change the target color, press the * key in the terminal.

A menu will be printed. Type a number (e.g., 1 for RED) and press Enter.

A confirmation message will be printed, and the system will now only activate the actuators when RED is detected.

## ✅ Demo (Expected Output)
The terminal output will show sensor readings, control logic decisions, and actuator actions all printing in the correct order, managed by the PrintTask.

Plaintext
```
R=148 G=250 B=117 | Detected: UNKNOWN
[CONTROL LOG] No Match. Detected: UNKNOWN
[ACTUATOR LOG] Turning OFF
R=148 G=250 B=117 | Detected: UNKNOWN
[CONTROL LOG] No Match. Detected: UNKNOWN
R=255 G=255 B=255 | Detected: WHITE
[CONTROL LOG] Match Found! Detected: WHITE
[ACTUATOR LOG] Turning ON
R=255 G=255 B=255 | Detected: WHITE
[CONTROL LOG] Match Found! Detected: WHITE
(No actuator log, as state did not change)
R=148 G=250 B=117 | Detected: UNKNOWN
[CONTROL LOG] No Match. Detected: UNKNOWN
[ACTUATOR LOG] Turning OFF

--- SELECT A COLOR ---
1: RED
2: GREEN
3: BLUE
4: YELLOW
5: BLACK
6: WHITE
Enter your choice followed by ENTER: 1
> Selected color set to: RED
```


