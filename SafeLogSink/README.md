# SafeLogSink

A simple, thread-safe logging system for Arduino ESP32 applications.

## Overview

SafeLogSink provides thread-safe logging capabilities that allow multiple tasks to safely output log messages to a single output stream (e.g., Serial) without conflicts or interleaved output.

### Key Features

- **Thread-safe logging**: Safe to use from any task or ISR
- **Buffered output**: Collects characters until line breaks for atomic printing
- **Multiple interfaces**: Global functions or stream-like interface
- **Auto-flushing**: Automatic buffer flush after configurable inactivity period
- **Low overhead**: Minimal impact on application performance

## Basic Concepts

The logger works by:
1. Collecting log messages in a queue
2. Processing them in a dedicated FreeRTOS task
3. Sending them to the configured output stream (default: Serial)

This approach ensures that log messages from different tasks don't get mixed up or corrupted.

## Usage

There are two main ways to use SafeLogSink:

### 1. Global Functions

The simplest approach using global functions:

```cpp
// Basic logging
log("System starting");

// Logging with newline
logln("Initialization complete");

// Formatted logging (printf style)
logf("Temperature: %.2f°C", temperature);
```

### 2. Stream Interface

An alternative approach using the Stream-like interface:

```cpp
// Using the global LogStream instance
LogStream.print("Hello ");
LogStream.println("World");

// Formatted printing
LogStream.printf("Value: %d, State: %s\n", value, stateStr);
```

### 3. Custom Stream Instances

For more advanced use cases, create custom SafeLogStream instances:

```cpp
// Create a dedicated logger for a specific module
SafeLogStream networkLogger;

void networkTask(void* params) {
    networkLogger.println("Network task started");
    // ...
    networkLogger.printf("Connected to %s\n", ssid);
}
```

## Configuration

Edit these definitions in `SafeLogSink.h` to customize behavior:

```cpp
// Change output destination (default: Serial)
#define LOG_OUTPUT Serial

// Queue size for pending messages
#define LOG_QUEUE_SIZE 32

// Maximum message length
#define MAX_LOG_MSG_SIZE 256

// FreeRTOS task priority for log processing
#define LOG_TASK_PRIORITY 1

// Stack size for log task
#define LOG_STACK_SIZE 4096
```

## How It Works

1. Log messages are captured in memory buffers
2. Messages are queued for processing
3. A dedicated FreeRTOS task processes the queue
4. Messages are atomically written to the output device

This prevents log corruption when multiple tasks try to log simultaneously.

## Example

```cpp
#include "SafeLogSink.h"

void setup() {
  Serial.begin(115200);
  delay(100);
  
  logln("System initialization");
  
  // Start tasks
  xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 2, NULL);
  xTaskCreate(networkTask, "NetworkTask", 8192, NULL, 3, NULL);
  
  logln("All tasks started");
}

void sensorTask(void* parameter) {
  SafeLogStream sensorLog;
  
  while(true) {
    float temp = readTemperature();
    sensorLog.printf("Temperature: %.2f°C\n", temp);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void networkTask(void* parameter) {
  while(true) {
    logln("Network checking...");
    // Do work...
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
```