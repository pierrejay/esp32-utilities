# MultiLogger - ESP32 FreeRTOS Logging System

A thread-safe logging system for ESP32 with multiple outputs and smart file management.

## Basic Usage

```cpp
#include "MultiLogger.h"

// Get logger instance
auto& logger = MultiLogger::getInstance();

// Create and add outputs
SerialLogger serialLogger(Serial);
logger.addOutput(serialLogger);

// Initialize
logger.begin();

// Start logging
LOG_SYSTEM("System started");
LOG_DEBUG("Temperature: %.1f°C", value);
LOG_WARNING("Battery low: %d%%", battery);
LOG_ALERT("System error: %s", error);
```

## Output Types

### Console Output (with colors)
```cpp
SerialLogger serialLogger(Serial);
logger.addOutput(serialLogger);
// Output: 2024-01-10 08:15:30.123 [DEBUG] Message (file.cpp:123)
```

### SD Card Logging
```cpp
// Single file
SDLogger sdLogger(SD, "/logs/app.log");

// Or split by level
SDLogger::Config config;
config.splitByLevel = true;
SDLogger sdLogger(SD, "/logs/app.log", config);
// Creates: app_alert.log, app_warning.log, app_debug.log, app_system.log

// Log rotation
if (sdLogger.getFilesInfo()[0].size > MAX_SIZE) {
    sdLogger.rotate();  // Creates timestamped backup
}
```

### JSON Output (for WebSocket)
```cpp
JsonLogger jsonLogger(websocketStream);
logger.addOutput(jsonLogger);
// Output: {"timestamp":1704881730,"level":"DEBUG","message":"Test","location":"main.cpp:45"}
```

## Time Management

```cpp
// Custom time provider (e.g., with RTC)
class RTCTimeProvider : public TimeProvider {
public:
    RTCTimeProvider(RTC_DS3231& rtc_) : rtc(rtc_) {}
    
    uint32_t getTimestamp() override { return rtc.getEpoch(); }
    uint64_t getMillis() override { return esp_timer_get_time() / 1000; }
private:
    RTC_DS3231& rtc;
};

// Usage
RTCTimeProvider rtcTime(rtc);  // Static/stack allocation
logger.setTimeProvider(rtcTime);
```

## Log Filtering

```cpp
// Configure what gets logged
logger.setLogFilter(LogFilter::ALL);      // Log everything
logger.setLogFilter(LogFilter::ALERTS);   // Only ALERT and WARNING
logger.setLogFilter(LogFilter::NONE);     // Disable logging
```

## Advanced Configuration

```cpp
MultiLogger::Config config;
config.queueSize = 32;                     // Message queue size
config.taskPriority = 1;                   // Logger task priority
config.queueTimeout = pdMS_TO_TICKS(100);  // Queue write timeout

logger.setConfig(config);
```

## Memory Usage

- Static buffers: ~1KB (format + timestamp)
- Queue: 8 messages × ~550 bytes = ~5KB
- Task stack: 4KB
- No dynamic memory allocation for core components

## Design Notes

- Thread-safe using FreeRTOS queue
- Non-blocking log calls with configurable timeout
- Automatic SD card error recovery
- Color-coded console output
- Static time provider allocation
- CSV format for SD logging
- JSON format for WebSocket or MQTT streaming