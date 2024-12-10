# MultiLogger - ESP32 FreeRTOS Logging System

The MultiLogger provides a comprehensive, thread-safe logging solution specifically designed for ESP32 FreeRTOS applications. Operating as an independent FreeRTOS task, it processes logs asynchronously without impacting your application's performance. The system combines robust features with careful resource management, making it suitable for both development debugging and production monitoring.

## Architecture Overview

### Core Design Principles
- Operates as a dedicated FreeRTOS task with its own priority and stack
- Uses a message queue for thread-safe log processing
- Static memory allocation for predictable resource usage
- Multiple output support with per-output filtering
- Robust filesystem management for SD card logging
- Color-coded console output for improved readability
- JSON formatting for modern monitoring integration

### Log Level Hierarchy
```
NONE    -> No logging (disabled output)
ALERT   -> Critical errors (red)
WARNING -> Important warnings (yellow)
INFO    -> General information (green)
SYSTEM  -> System status messages (blue)
DEBUG   -> Detailed debugging info (purple)
```
Each level includes all levels above it in severity, allowing for granular control over log output.

## Basic Usage

```cpp
#include "MultiLogger.h"

void setup() {
    // Get logger instance
    auto& logger = MultiLogger::getInstance();

    // Create and add outputs with optional level filtering
    SerialLogger serialLogger(Serial);
    SDLogger sdLogger(SD, "/logs/app.log");
    JsonLogger jsonLogger(websocketStream);

    logger.addOutput(serialLogger);                    // All levels
    logger.addOutput(sdLogger, LogLevel::WARNING);     // Only warnings and alerts
    logger.addOutput(jsonLogger);

    // Optional: Configure global logger behavior
    MultiLogger::Config config;
    config.queueSize = 8;               // Message queue depth
    config.taskPriority = 1;           // Logger task priority
    config.filter = LogLevel::SYSTEM;   // Global log filter level
    logger.setConfig(config);

    // Initialize the logging system
    logger.begin();

    // Start logging
    LOG_SYSTEM("System initialized");
    LOG_INFO("Configuration loaded");
    LOG_DEBUG("Debug mode active");
}
```

## Output Formats

### Console Output (Color-coded)
```
// Format pattern: timestamp [LEVEL] message (taskName@source:line)
2024-01-10 08:15:30.123 [DEBUG] Temperature: 25.6°C (TempTask@sensor.cpp:45)
2024-01-10 08:15:30.456 [ALERT] Memory low: 10KB free (MainTask@memory.cpp:123)
```
Colors are automatically applied based on log level for improved visual scanning:
- `DEBUG`: Purple
- `INFO`: Green
- `SYSTEM`: Blue
- `WARNING`: Yellow
- `ALERT`: Red

### SD Card Output (CSV)
```csv
timestamp,level,task,message,location
2024-01-10 08:15:30.123,DEBUG,TempTask,Temperature: 25.6°C,sensor.cpp:45
2024-01-10 08:15:30.456,ALERT,MainTask,Memory low: 10KB free,memory.cpp:123
```

### JSON Output (WebSocket)
```json
{
    "timestamp": 1704881730,
    "uptime": 123456,
    "level": "DEBUG",
    "message": "Temperature: 25.6°C",
    "location": "sensor.cpp:45",
    "task": "TempTask"
}
```

## SD Card Management

The SDLogger provides robust file management capabilities:

### Basic Configuration
```cpp
// Single log file
SDLogger sdLogger(SD, "/logs/app.log");
```

### Advanced File Management
```cpp
// Split logs by level
SDLogger::Config config;
config.splitByLevel = true;
SDLogger sdLogger(SD, "/logs/app.log", config);

// Creates:
// /logs/app_alert.log
// /logs/app_warning.log
// /logs/app_info.log
// /logs/app_system.log
// /logs/app_debug.log
```

### Automatic Log Rotation
```cpp
// Monitor file sizes
auto files = sdLogger.getFilesInfo();
for (const auto& file : files) {
    if (file.size > MAX_LOG_SIZE) {
        // Rotates to: app_20240110_123030.log (file_date_time.log)
        sdLogger.rotate();
        break;
    }
}
```

Key features:
- Automatic error recovery on write failures
- Periodic flush to prevent data loss
- Timestamped backup files on rotation
- Per-level file separation for easier analysis

## Time Management

The system supports different time sources through the TimeProvider interface:

### Default Time Provider
Uses system uptime with millis() overflow protection (64-bit timestamp):
```cpp
// Automatically used if no custom provider is set
// Provides uptime in HH:MM:SS.mmm format
```

### RTC Integration
```cpp
class RTCTimeProvider : public TimeProvider {
public:
    RTCTimeProvider(RTC_DS3231& rtc_) : rtc(rtc_) {}
    
    uint32_t getTimestamp() override { 
        return rtc.getEpoch(); 
    }
    uint64_t getMillis() override { 
        return esp_timer_get_time() / 1000; 
    }
private:
    RTC_DS3231& rtc;
};

// Usage (static allocation)
RTCTimeProvider rtcTime(rtc);
logger.setTimeProvider(rtcTime);
```

## FreeRTOS Integration

### Task Management
- Logger runs as an independent FreeRTOS task
- Configurable priority (default: 1 - low priority)
- 4KB stack allocation
- Non-blocking operation through message queue

### Queue Configuration
```cpp
MultiLogger::Config config;
config.queueSize = 8;                      // Number of pending messages
config.queueTimeout = pdMS_TO_TICKS(100);  // Wait time for queue full
logger.setConfig(config);
```

### Performance Considerations
- Log calls are non-blocking (return immediately)
- Messages are processed asynchronously
- Queue overflow protection drops messages rather than blocking
- Per-output filtering reduces processing overhead

## System Monitoring

### Resource Statistics
```cpp
// Log current system state
logger::logSystemStats();
// Output: "Heap: 123456 bytes (120000 min) Stack: 2048 CPU: 45.2°C"

// Periodic monitoring task
void monitorTask(void* parameter) {
    while (true) {
        logger::logSystemStats();
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
```

## Memory Usage

### Static Allocation
- Message structure: ~550 bytes per entry
- Format buffers: ~1KB total
  - Timestamp buffer: 32 bytes
  - Format buffer: 512 bytes
- Queue memory: queueSize × sizeof(LogMessage)
- Task stack: 4KB

### Memory Safety
- No dynamic allocation in core paths
- Buffer overflow protection
- String truncation safety
- Reusable static buffers

## Best Practices

1. Task Priority
   - Keep logger priority low for non-critical systems
   - Raise priority if log messages are time-sensitive
   - Monitor message queue usage to adjust priority

2. Queue Size
   - Start with small queue (8-16 messages)
   - Increase if seeing message drops
   - Monitor queue high water mark

3. SD Card Usage
   - Enable log rotation for long-term logging
   - Use level splitting for easier debugging
   - Monitor file sizes periodically
   - Consider filesystem space management

4. Memory Management
   - Adjust message queue size based on available memory
   - Monitor stack usage in the logger task
   - Use appropriate log levels to reduce processing overhead

This logger system provides a robust foundation for application monitoring and debugging while maintaining system stability and performance. Its flexible architecture allows for customization to meet specific application needs while providing safe defaults for general use.