# AsyncWire Library - Technical Documentation

## Table of Contents
1. [Overview](#overview)
2. [Installation](#installation)
3. [Core Concepts](#core-concepts)
4. [API Reference](#api-reference)
5. [Error Handling](#error-handling)
6. [Legacy Wire Support](#legacy-wire-support)
7. [Best Practices](#best-practices)
8. [Advanced Usage](#advanced-usage)
9. [Memory Management](#memory-management)
10. [Troubleshooting](#troubleshooting)

## Overview

AsyncWire is an I2C communication library for ESP32, designed to provide thread-safe, non-blocking operations while maintaining compatibility with the legacy Wire library.

### Key Features
- Thread-safe I2C bus access
- Non-blocking operations with callback support
- Fluent API for complex I2C sequences
- Built-in error handling and diagnostics
- Legacy Wire compatibility layer
- Buffer management with memory safety
- FreeRTOS task integration

### Prerequisites
- ESP32 platform
- Arduino framework
- FreeRTOS (included with ESP32 Arduino core)
- C++11 or later

## Installation

### Using PlatformIO
```ini
# platformio.ini
dependencies:
  AsyncWire=
    https://github.com/your-repo/AsyncWire.git#v1.0.0
```

### Manual Installation
1. Download the library
2. Place it in your Arduino libraries folder
3. Include in your project:
```cpp
#include <AsyncWire.h>
```

## Core Concepts

### Operation Sequencing
Operations in AsyncWire are built using sequences. A sequence is a series of I2C operations that will be executed atomically.

```cpp
// Example of a sequence
WireSequenceBuilder()
    .beginTransmission(0x42)    // Start with device address
    .write(0x10)               // Write register address
    .endTransmission(false)    // Repeated start
    .requestFrom(0x42, 2)      // Read 2 bytes
    .execute([](const SequenceResult& result) {
        // Handle result
    });
```

### Result Handling

Results are provided through two main structures:

#### WireResult
Represents the result of a single I2C operation.

```cpp
struct WireResult {
    enum Error {
        SUCCESS = 0,
        NACK_ADDRESS = 2,
        NACK_DATA = 3,
        BUS_ERROR = 4,
        TIMEOUT = 5
    } error;
    
    size_t bytesTransferred;
    std::vector<uint8_t> buffer;

    bool isOk() const;
    String getErrorString() const;
};
```

#### SequenceResult
Represents the result of an entire sequence of operations.

```cpp
struct SequenceResult {
    std::vector<WireResult> operations;
    
    bool isSuccessful() const;
    WireResult& final();
    String getErrorDetails() const;
};
```

## API Reference

### WireSequenceBuilder Methods

#### Basic I2C Operations

```cpp
WireSequenceBuilder& beginTransmission(uint8_t address)
```
Initiates an I2C transmission.
- `address`: 7-bit I2C device address
- Returns: Builder reference for chaining

```cpp
WireSequenceBuilder& write(uint8_t data)
WireSequenceBuilder& write(const uint8_t* data, size_t length)
```
Writes data to the I2C bus.
- `data`: Single byte or pointer to data buffer
- `length`: Number of bytes to write
- Returns: Builder reference for chaining

```cpp
WireSequenceBuilder& endTransmission(bool sendStop = true)
```
Ends the current transmission.
- `sendStop`: Whether to send STOP condition
- Returns: Builder reference for chaining

```cpp
WireSequenceBuilder& requestFrom(uint8_t address, size_t length, bool sendStop = true)
```
Requests data from a device.
- `address`: Device address
- `length`: Number of bytes to request (max 32)
- `sendStop`: Whether to send STOP condition
- Returns: Builder reference for chaining

#### High-Level Operations

```cpp
WireSequenceBuilder& writeRegister(uint8_t address, uint8_t reg, uint8_t value)
```
Writes a single byte to a device register.
- `address`: Device address
- `reg`: Register address
- `value`: Value to write
- Returns: Builder reference for chaining

```cpp
WireSequenceBuilder& readRegister(uint8_t address, uint8_t reg, size_t length = 1)
```
Reads from a device register.
- `address`: Device address
- `reg`: Register address
- `length`: Number of bytes to read
- Returns: Builder reference for chaining

#### Execution

```cpp
void execute(CompletionCallback callback)
```
Executes the sequence.
- `callback`: Function to call when sequence completes
- Callback signature: `void(const SequenceResult&)`

### Processing Results

#### Checking Operation Success

```cpp
// Check entire sequence
if (result.isSuccessful()) {
    // All operations succeeded
}

// Check specific operation
if (result.operations[0].isOk()) {
    // First operation succeeded
}

// Get data from read operation
const auto& readResult = result.final();
if (readResult.isOk()) {
    // Access data through readResult.data
    // Length available in readResult.bytesTransferred
}
```

#### Error Handling Detail

```cpp
void handleResult(const SequenceResult& result) {
    if (!result.isSuccessful()) {
        // Get detailed error info
        Serial.println(result.getErrorDetails());
        
        // Analyze specific operations
        for (const auto& op : result.operations) {
            if (!op.isOk()) {
                Serial.printf("Error: %s\n", op.getErrorString());
                Serial.printf("Bytes transferred: %d\n", op.bytesTransferred);
            }
        }
    }
}
```

## Legacy Wire Support

### BlockingWireProxy

Provides a Wire-compatible interface:

```cpp
class BlockingWireProxy {
public:
    void beginTransmission(uint8_t address);
    size_t write(uint8_t data);
    size_t write(const uint8_t* data, size_t length);
    uint8_t endTransmission(bool sendStop = true);
    size_t requestFrom(uint8_t address, size_t length, bool sendStop = true);
    int available();
    int read();
};
```

### Migration Examples

#### Original Wire Code
```cpp
Wire.beginTransmission(0x42);
Wire.write(0x10);
Wire.write(data, length);
Wire.endTransmission();
```

#### Using BlockingWireProxy
```cpp
BlockingWireProxy wire;
wire.beginTransmission(0x42);
wire.write(0x10);
wire.write(data, length);
wire.endTransmission();
```

#### Modern Async Version
```cpp
WireSequenceBuilder()
    .beginTransmission(0x42)
    .write(0x10)
    .write(data, length)
    .endTransmission()
    .execute([](const SequenceResult& result) {
        if (result.isSuccessful()) {
            // Operation completed successfully
        }
    });
```

## Best Practices

### Sequence Organization

DO:
```cpp
// Group related operations
WireSequenceBuilder()
    .writeRegister(address, CONFIG_REG, config)
    .readRegister(address, STATUS_REG)
    .execute([](const SequenceResult& result) {
        // Handle both operations together
    });
```

DON'T:
```cpp
// Don't split related operations
WireSequenceBuilder()
    .writeRegister(address, CONFIG_REG, config)
    .execute([](const SequenceResult& result) {
        WireSequenceBuilder()
            .readRegister(address, STATUS_REG)
            .execute([](const SequenceResult& result) {
                // Handling split across callbacks
            });
    });
```

### Error Handling

DO:
```cpp
void handleI2CResult(const SequenceResult& result) {
    if (!result.isSuccessful()) {
        log_e("I2C Error: %s", result.getErrorDetails().c_str());
        return;
    }
    
    const auto& readResult = result.final();
    if (readResult.bytesTransferred < expected_length) {
        log_w("Incomplete read: %d/%d bytes", 
              readResult.bytesTransferred, expected_length);
    }
}
```

DON'T:
```cpp
void handleI2CResult(const SequenceResult& result) {
    // Don't ignore errors
    const auto& data = result.final().data;
    processData(data); // Might crash if operation failed
}
```

### Memory Management

DO:
```cpp
// Data is safely stored in result.buffer
void processData(const SequenceResult& result) {
    const auto& readResult = result.final();
    std::vector<uint8_t> localCopy(
        readResult.data,
        readResult.data + readResult.bytesTransferred
    );
    // Can now safely use localCopy even after callback returns
}
```

DON'T:
```cpp
// Don't store raw pointers to result data
uint8_t* stored_data;
void processData(const SequenceResult& result) {
    stored_data = const_cast<uint8_t*>(result.final().data);
    // Dangerous! Data might be invalidated
}
```

## Advanced Usage

### Custom Device Classes

```cpp
class I2CDevice {
protected:
    const uint8_t address;
    
    template<typename T>
    void readRegister(uint8_t reg, T& value) {
        static_assert(std::is_trivially_copyable<T>::value,
                     "T must be trivially copyable");
        
        WireSequenceBuilder()
            .readRegister(address, reg, sizeof(T))
            .execute([&value](const SequenceResult& result) {
                if (result.isSuccessful()) {
                    memcpy(&value, result.final().data, sizeof(T));
                }
            });
    }
    
    template<typename T>
    void writeRegister(uint8_t reg, const T& value) {
        static_assert(std::is_trivially_copyable<T>::value,
                     "T must be trivially copyable");
        
        const uint8_t* data = reinterpret_cast<const uint8_t*>(&value);
        WireSequenceBuilder()
            .writeRegister(address, reg, data, sizeof(T))
            .execute([](const SequenceResult& result) {
                // Handle result if needed
            });
    }
};

// Example implementation
class BME280 : private I2CDevice {
public:
    struct CalibrationData {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        // ... other calibration data
    };
    
    void readCalibration() {
        readRegister(0x88, calibData);
    }
    
private:
    CalibrationData calibData;
};
```

### Timeout Handling

```cpp
class TimeoutManager {
    static constexpr uint32_t DEFAULT_TIMEOUT = 1000; // ms
    
    void startOperation() {
        WireSequenceBuilder()
            .readRegister(device_addr, status_reg)
            .execute([this, start = millis()](const SequenceResult& result) {
                if (millis() - start > DEFAULT_TIMEOUT) {
                    handleTimeout();
                    return;
                }
                // Process result
            });
    }
};
```

## Memory Management

### Buffer Limits

The library enforces a strict limit on I2C transfer sizes:
```
static constexpr size_t MAX_I2C_BUFFER_SIZE = 32;
```

This limit applies to:
- Data reads (requestFrom)
- Write operations (write)
- Register sequences

Any attempt to exceed this will result in:
- A BUS_ERROR for reads
- Silent truncation for writes

### Recommendations for Large Transfers

For transfers exceeding MAX_I2C_BUFFER_SIZE:
```cpp
// ✅ Split into multiple sequences
void readLargeData(uint8_t address, uint8_t startReg, size_t totalSize) {
    size_t offset = 0;
    while (offset < totalSize) {
        size_t chunk = std::min(MAX_I2C_BUFFER_SIZE, totalSize - offset);
        WireSequenceBuilder()
            .writeRegister(address, startReg + offset)
            .requestFrom(address, chunk)
            .execute([offset, chunk](const SequenceResult& result) {
                processDataChunk(result.final().data, chunk, offset);
            });
        offset += chunk;
    }
}

// ❌ Do not attempt to read more than MAX_I2C_BUFFER_SIZE
WireSequenceBuilder()
    .requestFrom(address, 64)  // BUS_ERROR
    .execute(callback);
```

### Buffer Lifecycle

1. Data Reception
   - Buffer is allocated in WireResult
   - Size is determined by requestFrom length
   - Data remains valid during callback execution

2. Data Storage
   - Copy data if needed beyond callback
   - Use std::vector for dynamic storage
   - Consider fixed buffers for constrained systems

### Memory Safety Examples

```cpp
// Safe data handling
class SafeHandler {
    std::vector<uint8_t> stored_data;
    
    void handleData(const SequenceResult& result) {
        const auto& read_result = result.final();
        stored_data.assign(
            read_result.data,
            read_result.data + read_result.bytesTransferred
        );
        // stored_data is now safe to use anytime
    }
};
```

## Troubleshooting

### Common Issues

1. Operation Timeouts
```cpp
// Monitor operation timing
unsigned long start = millis();
WireSequenceBuilder()
    .readRegister(address, reg)
    .execute([start](const SequenceResult& result) {
        unsigned long duration = millis() - start;
        if (duration > 100) { // 100ms threshold
            log_w("Slow I2C operation: %dms", duration);
        }
    });
```

2. Buffer Overflows
```cpp
// Check transfer sizes
if (result.final().bytesTransferred != expected_size) {
    log_e("Transfer size mismatch: got %d, expected %d",
          result.final().bytesTransferred, expected_size);
}
```

3. Bus Errors
```cpp
// Detailed error logging
void logI2CError(const SequenceResult& result) {
    if (!result.isSuccessful()) {
        log_e("I2C Error: %s", result.getErrorDetails().c_str());
        log_e("Failed operation details:");
        for (size_t i = 0; i < result.operations.size(); i++) {
            const auto& op = result.operations[i];
            if (!op.isOk()) {
                log_e("Operation %d: %s, transferred %d bytes",
                      i, op.getErrorString(), op.bytesTransferred);
            }
        }
    }
}
```

### Performance Optimization

1. Minimize Transaction Count
```cpp
// Good: Single transaction for multiple registers
WireSequenceBuilder()
    .writeRegister(addr, START_REG)
    .requestFrom(addr, 4) // Read multiple registers
    .execute(callback);

// Bad: Multiple transactions
WireSequenceBuilder()
    .readRegister(addr, REG1)
    .execute([](const SequenceResult& r1) {
        WireSequenceBuilder()
            .readRegister(addr, REG2)
            .execute(callback);
    });
```

2. Optimize Buffer Usage
```cpp
// Pre-allocate buffers for frequent operations
static std::vector<uint8_t> shared_buffer;
shared_buffer.reserve(MAX_I2C_BUFFER_SIZE);
```

This documentation will continue to be updated with more examples, use cases, and best practices as users provide feedback and common patterns emerge.