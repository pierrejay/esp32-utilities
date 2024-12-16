# AsyncWire Library Documentation

## Overview

AsyncWire is an I2C library for ESP32 that solves a major challenge: managing concurrent I2C bus access in a multitasking environment. 

### Key Features
- Thread-safe I2C access from multiple tasks
- Non-blocking I2C operations
- Full compatibility with existing Wire code
- Detailed error reporting
- Atomic execution of operation sequences

## Core Concepts

### 1. Operation Types
AsyncWire defines four fundamental I2C operations that form the building blocks of any I2C communication:

```cpp
enum Type { 
    BEGIN,    // Start a transmission
    WRITE,    // Write data
    END,      // End transmission
    REQUEST   // Request data from device
}
```

### 2. Builder Pattern
The library uses a builder pattern through the `WireSequenceBuilder` class. This provides a fluent interface to:
- Construct operation sequences
- Execute them atomically
- Handle results through callbacks

```cpp
// Basic operation sequence
WireSequenceBuilder()
    .beginTransmission(0x42)
    .write(0x10)
    .endTransmission()
    .execute(callback);

// Using helper methods
WireSequenceBuilder()
    .writeRegister(0x42, 0x10, 0x55)  // Write register
    .readRegister(0x42, 0x20, 2)      // Read register
    .execute(callback);
```

### 3. Error Handling System

The library provides comprehensive error reporting at two levels:

#### Individual Operation Results (WireOperationResult)
```cpp
struct WireOperationResult {
    enum Error {
        SUCCESS = 0,
        NACK_ADDRESS = 2,  // Device not found
        NACK_DATA = 3,     // Data not acknowledged
        BUS_ERROR = 4,     // Bus error
        TIMEOUT = 5        // Operation timeout
    } error;
    
    size_t bytesTransferred;      // Number of bytes
    std::vector<uint8_t> buffer;  // Read data
    
    bool isOk() const;
    String getErrorString() const;
};
```

#### Complete Sequence Results (WireSequenceResult)
```cpp
struct WireSequenceResult {
    std::vector<WireOperationResult> operations;
    
    bool isSuccessful() const;      // All operations succeeded?
    WireOperationResult& final();   // Last operation result
    String getErrorDetails() const; // Detailed error info
};
```

## Getting Started

### 1. Configuration

AsyncWire is highly configurable to match your hardware setup:

```cpp
void setup() {
    Serial.begin(115200);
    
    // 1. Default Wire configuration
    AsyncWire::configure(Wire);
    
    // 2. Custom pins on default Wire
    AsyncWire::configure(Wire, GPIO_NUM_21, GPIO_NUM_22, 400000);
    
    // 3. Custom Wire instance
    TwoWire Wire1(1);  // Create Wire1
    Wire1.begin(GPIO_NUM_33, GPIO_NUM_32);  // Custom pins
    AsyncWire::configure(Wire1);  // Use Wire1
}
```

### 2. Basic Usage Example

Here's a simple example reading from a temperature sensor:

```cpp
#include <Arduino.h>
#include "AsyncWire.h"

const uint8_t TEMP_SENSOR_ADDR = 0x18;

void setup() {
    Serial.begin(115200);
    AsyncWire::configure(Wire);
}

void loop() {
    WireSequenceBuilder()
        .readRegister(TEMP_SENSOR_ADDR, 0x05, 2)
        .execute([](const WireSequenceResult& result) {
            if (result.isSuccessful()) {
                const auto& data = result.final().buffer;
                float temp = ((data[0] & 0x0F) * 16 + data[1] / 16.0f);
                Serial.printf("Temperature: %.2f°C\n", temp);
            } else {
                Serial.printf("Read failed: %s\n", 
                            result.getErrorDetails().c_str());
            }
        });
    
    delay(1000);
}
```

## Advanced Usage

### 1. Multi-Device Concurrent Access

This example shows how to handle multiple I2C devices from different tasks:

```cpp
#include <Arduino.h>
#include "AsyncWire.h"

// Device addresses
const uint8_t TEMP_SENSOR_ADDR = 0x18;  // MCP9808 temperature sensor
const uint8_t DISPLAY_ADDR = 0x3C;      // OLED display

// Task 1: Read temperature from MCP9808 sensor
void temperatureTask(void* params) {
    while (true) {
        // Read temperature every second
        WireSequenceBuilder()
            .readRegister(TEMP_SENSOR_ADDR, 0x05, 2)
            .execute([](const WireSequenceResult& result) {
                if (result.isSuccessful()) {
                    const auto& data = result.final().buffer;
                    float temp = ((data[0] & 0x0F) * 16 + data[1] / 16.0f);
                    Serial.printf("Temperature: %.2f°C\n", temp);
                }
            });
            
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task 2: Update OLED display
void displayTask(void* params) {
    while (true) {
        WireSequenceBuilder()
            .writeRegister(DISPLAY_ADDR, 0x00, 0xAE)  // Display off
            .writeRegister(DISPLAY_ADDR, 0x00, 0xA6)  // Normal display
            .writeRegister(DISPLAY_ADDR, 0x00, 0xAF)  // Display on
            .execute([](const WireSequenceResult& result) {
                if (!result.isSuccessful()) {
                    Serial.printf("Display update failed: %s\n", 
                                result.getErrorDetails().c_str());
                }
            });
            
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    AsyncWire::configure(Wire);
    
    // Create concurrent tasks
    xTaskCreate(temperatureTask, "Temp", 2048, nullptr, 1, nullptr);
    xTaskCreate(displayTask, "Display", 2048, nullptr, 1, nullptr);
}

void loop() {
    delay(100);  // Keep the loop running
}
```

### 2. Mixing Modern and Legacy Code

AsyncWire provides seamless compatibility with existing Wire code through `BlockingWireProxy`. Here's an example mixing both styles:

```cpp
// Modern sensor using AsyncWire
class TemperatureSensor {
private:
    const uint8_t address;
    
public:
    TemperatureSensor(uint8_t addr) : address(addr) {}
    
    void readTemperature(std::function<void(float)> callback) {
        WireSequenceBuilder()
            .readRegister(address, 0x05, 2)
            .execute([callback](const WireSequenceResult& result) {
                if (result.isSuccessful()) {
                    const auto& data = result.final().buffer;
                    float temp = ((data[0] & 0x0F) * 16 + data[1] / 16.0f);
                    callback(temp);
                }
            });
    }
};

// Legacy sensor using Wire-style code
class HumiditySensor {
private:
    BlockingWireProxy wire;  // Wire-compatible interface
    const uint8_t address;
    
public:
    HumiditySensor(uint8_t addr) : address(addr) {}
    
    float readHumidity() {
        wire.beginTransmission(address);
        wire.write(0x01);  // Humidity register
        wire.endTransmission(false);
        
        wire.requestFrom(address, (uint8_t)2);
        uint16_t raw = wire.read() << 8 | wire.read();
        return raw / 100.0f;
    }
};

// Usage in tasks
void temperatureTask(void* params) {
    TemperatureSensor sensor(0x18);
    while (true) {
        sensor.readTemperature([](float temp) {
            Serial.printf("Temperature: %.1f°C\n", temp);
        });
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void humidityTask(void* params) {
    HumiditySensor sensor(0x40);
    while (true) {
        float humidity = sensor.readHumidity();
        Serial.printf("Humidity: %.1f%%\n", humidity);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```

## Best Practices

### 1. Atomic Operations
Group related operations in a single sequence:
```cpp
// Good: All config happens atomically
WireSequenceBuilder()
    .writeRegister(addr, reg1, val1)
    .writeRegister(addr, reg2, val2)
    .execute(callback);
```

### 2. Error Handling
Always check operation results:
```cpp
.execute([](const WireSequenceResult& result) {
    if (!result.isSuccessful()) {
        Serial.printf("Failed: %s\n", result.getErrorDetails().c_str());
        return;
    }
    // Process success case...
});
```

### 3. Buffer Management
Only use read buffers within callbacks:
```cpp
// Good: Buffer used in callback
WireSequenceBuilder()
    .readRegister(addr, reg, 2)
    .execute([](const WireSequenceResult& result) {
        const auto& buffer = result.final().buffer;
        // Process buffer here
    });
```

## Conclusion

AsyncWire simplifies I2C communication on ESP32 by providing:
- Thread-safe concurrent access
- Non-blocking operations
- Comprehensive error handling
- Legacy code compatibility

The library handles all the complexities of I2C communication in a multi-tasking environment while providing a clean, modern API.