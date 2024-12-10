# MultiLogger - ESP32 FreeRTOS Logging System

The MultiLogger provides a comprehensive, thread-safe logging system specifically designed for ESP32 FreeRTOS applications. It processes logs asynchronously through a dedicated FreeRTOS task, optimizing application performance while offering robust logging capabilities.

## Core Features

- FreeRTOS task-based asynchronous logging
- Multiple output support (Serial, SD card, JSON) with per-output filtering
- Thread-safe operation via message queue
- Color-coded console output
- Flexible timestamp management with uptime tracking
- Device ID support for multi-device deployments
- Static memory allocation for predictable resource usage

## Log Level Hierarchy

```
NONE    -> No logging (disabled output)
ERROR   -> Critical errors (red)
WARNING -> Important warnings (yellow)
INFO    -> General information (green)
SYSTEM  -> System status messages (blue)  
DEBUG   -> Detailed debugging info (pink)
```

Each level includes all levels above it in severity when used as a filter.

## Basic Usage

```cpp
#include "MultiLogger.h"

void setup() {
    // Get logger instance with optional device ID and filter level
    auto& logger = MultiLogger::getInstance("DEVICE_001", LogLevel::DEBUG);

    // Create and add outputs
    SerialLogger serialLogger(Serial);
    SDLogger sdLogger(SD, "/logs/app.log");
    logger.addOutput(serialLogger);                   
    logger.addOutput(sdLogger, LogLevel::WARNING);  // Only WARNING and ERROR
    
    // Initialize the logging system
    logger.begin();

    // Start logging
    LOG_SYSTEM("System initialized");
    LOG_INFO("Configuration loaded");
    LOG_DEBUG("Debug mode active");
}
```

## Configuration Management

### Instance Creation and Parameters

```cpp
// Method 1: Default initialization
auto& logger = MultiLogger::getInstance();

// Method 2: With device ID only
auto& logger = MultiLogger::getInstance("DEVICE_001");

// Method 3: With device ID and filter level
auto& logger = MultiLogger::getInstance("DEVICE_001", LogLevel::DEBUG);

// Runtime parameter modification
logger.setDeviceId("DEVICE_002");
logger.setLogFilter(LogLevel::INFO);

// Parameter retrieval
const char* deviceId = logger.getDeviceId();
LogLevel filter = logger.getLogFilter();
```

### Fixed Configuration Values

The following parameters are hardcoded for optimal performance:

```cpp
static constexpr size_t QUEUE_SIZE = 8;
static constexpr TickType_t QUEUE_TIMEOUT = pdMS_TO_TICKS(100);
static constexpr UBaseType_t TASK_PRIORITY = 1;
```

## Output Formats

### Console Output (SerialLogger)

```
// With RTC/timestamp available:
2024-01-10 08:15:30.123 [DEBUG] Temperature: 25.6°C (TempTask@sensor.cpp:45)

// Without RTC (uptime only):
1234567 [DEBUG] Temperature: 25.6°C (TempTask@sensor.cpp:45)
```

SerialLogger features:
- Color coding by log level
- Automatic client connection detection
- Logs only sent when client is connected
- Dynamic timestamp/uptime display based on time provider

### SD Card Output (SDLogger)

```csv
timestamp,uptime,level,task@location,message,deviceid
1704881730,123456,DEBUG,TempTask@sensor.cpp:45,Temperature: 25.6°C,DEVICE_001
```

SDLogger features:
- CSV format for easy parsing
- Uptime in seconds
- Raw timestamp (epoch) or 0 if unavailable
- Optional device ID field
- File management capabilities

### JSON Output (JsonLogger)

The JsonLogger is built around a callback system that provides maximum flexibility in how and where logs are processed and delivered. It abstracts the log output behind a simple function call, allowing for seamless integration with any JSON-capable system.

#### Base Design

```cpp
// Callback function type
using JsonCallback = std::function<void(JsonObject&)>;

// Base class with simple constructor
class JsonLogger : public ILogOutput {
public:
    explicit JsonLogger(JsonCallback callback) 
        : callback_(callback) {}
};
```

The logger simply needs to be provided with a callback function that will be called with the formatted log as a JsonObject. This design allows for:
- Custom log processing
- Multiple output destinations
- Integration with any transport layer
- Real-time log transformation
- Conditional routing
- Buffer management

#### Standard Output Format
```json
{
    "timestamp": 1704881730,    // Unix timestamp or 0 if no RTC
    "uptime": 123456789,        // System uptime in milliseconds
    "level": "DEBUG",           // Log level
    "message": "Sensor reading: 25.6°C",
    "location": "sensor.cpp:45", // Source file and line
    "task": "TempTask",         // FreeRTOS task name
    "deviceid": "DEVICE_001"    // Optional device identifier
}
```

#### Implementation Examples

You can easily create custom implementations for any use case:

```cpp
// WebSocket output through APIServer
class WSLogger : public JsonLogger {
public:
    WSLogger(APIServer& apiServer) 
        : JsonLogger([&apiServer](JsonObject& obj) {
            apiServer.broadcast("system/logs", obj);
          }) {}
};

// MQTT publication
class MQTTLogger : public JsonLogger {
public:
    MQTTLogger(PubSubClient& mqtt, const char* topic) 
        : JsonLogger([&mqtt, topic](JsonObject& obj) {
            char buffer[512];
            serializeJson(obj, buffer);
            mqtt.publish(topic, buffer);
          }) {}
};

// HTTP POST requests
class HTTPLogger : public JsonLogger {
public:
    HTTPLogger(const char* host, uint16_t port, const char* endpoint) 
        : JsonLogger([=](JsonObject& obj) {
            // Implement HTTP POST request with log data
        }) {}
};
```

Each implementation is treated as a standard ILogOutput and can be added to the MultiLogger:
```cpp
auto& logger = MultiLogger::getInstance();
logger.addOutput(wsLogger);
logger.addOutput(mqttLogger);
logger.addOutput(httpLogger);
```

Key features:
- Callback-based output for maximum flexibility
- Full JsonObject access for custom processing
- Integration with any transport protocol
- Uptime in milliseconds (uint64_t precision)
- Raw timestamp (epoch) or 0 if unavailable
- Standard JSON format for easy parsing

The JsonLogger's design makes it particularly suitable for:
- Real-time monitoring systems
- Distributed logging architectures
- Cloud integration
- Data analytics pipelines
- Debug interfaces

## Time Management

### Default Time Provider
```cpp
// Automatically used if no custom provider is set
class DefaultTime : public TimeProvider {
    uint32_t getTimestamp() override { 
        return 0;  // No RTC available
    }
    
    uint64_t getMillis() override {
        // Handles millis() overflow
        uint32_t currentMillis = millis();
        if (currentMillis < lastMillis_) {
            overflows_++;
        }
        lastMillis_ = currentMillis;
        return (overflows_ << 32) + currentMillis;
    }
private:
    uint32_t lastMillis_ = 0;
    uint64_t overflows_ = 0;
};
```

### Custom RTC Integration
```cpp
class RTCTimeProvider : public TimeProvider {
public:
    RTCTimeProvider(RTC_DS3231& rtc) : rtc_(rtc) {}
    
    uint32_t getTimestamp() override { 
        return rtc_.getEpoch();
    }
    
    uint64_t getMillis() override {
        return esp_timer_get_time() / 1000;
    }
    
private:
    RTC_DS3231& rtc_;
};

// Usage
RTCTimeProvider rtcTime(rtc);
logger.setTimeProvider(rtcTime);
```

## System Monitoring

### Resource Statistics

```cpp
// Log current system state with pipe separator
logger::logSystemStats();
// Output: "Heap: 123456 bytes (120000 min) | Stack: 2048 | CPU: 45.2°C"
```

## Implementation Notes

### Memory Management
- Static message queue allocation: `QUEUE_SIZE × sizeof(LogMessage)`
- Fixed buffer sizes for messages and formatting
- No dynamic allocation in logging paths
- Task stack: 4KB

### FreeRTOS Integration
- Independent task with priority 1 (low)
- Non-blocking message queue with timeout
- Queue overflow protection (drops messages)
- Thread-safe operation

### Best Practices
1. Use appropriate log levels to control output volume
2. Monitor client connection state for SerialLogger
3. Implement automatic log rotation for SDLogger
4. Consider JsonLogger callback performance impact
5. Set device ID for multi-device deployments
6. Validate time provider implementation

### Custom Output Implementation

```cpp
class CustomOutput : public ILogOutput {
public:
    void begin() override {
        // Initialize output if needed
    }
    
    void writeLogMessage(const LogMessage& msg) override {
        // Format and write message according to your needs
        // All message fields are available in the LogMessage struct
    }
    
    OutputType getType() const override {
        return OutputType::CUSTOM;
    }
};
```

## Error Handling
- Queue full: Messages dropped with timeout
- SD write failure: Automatic recovery attempt
- Serial disconnection: Messages suppressed
- Time provider failure: Fallback to uptime only

This documentation reflects the latest changes to the MultiLogger system, focusing on clarity, completeness, and practical usage examples.