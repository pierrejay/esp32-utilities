# AsyncSerial Documentation

## Introduction

`AsyncSerial` is a library designed to manage asynchronous and multi-proxy communication over a single hardware serial port on Arduino-compatible platforms. The primary goals are:

- Provide a **drop-in replacement for the Serial API**, ensuring existing code using `Serial` can be easily adapted.
- Allow **multiple logical streams** (`SerialProxy`) to share a single physical UART, each with its own buffers and configuration.
- Offer cooperative, non-blocking behavior for operations like `flush()` to prevent system stalls and improve responsiveness in multi-tasking or RTOS environments.

## Core Concepts

### Drop-in Serial Replacement
AsyncSerial is designed to be a direct replacement for Arduino's Serial interface:
- Each `SerialProxy` implements the standard `Stream` interface
- Identical method signatures (`print`, `println`, `write`, `read`, `available`, etc.)
- Compatible with existing libraries that use Serial communication
- Same behavior for string formatting and special characters handling

### Multiple Proxies
- Up to 8 independent logical serial streams sharing a single physical UART
- Each proxy operates independently with its own:
  - TX/RX buffers
  - Inter-message delays
  - Flush behavior

### Thread Safety
Non-blocking collaboration between threads through `CooperativeLock`, ideal for:
- RTOS environments (such as Arduino ESP32 with FreeRTOS)
- Cooperative multitasking
- Interrupt-driven architectures

## Dual Port Type Support (HardwareSerial & HWCDC)

### Overview

AsyncSerial provides a unified interface for both HardwareSerial and HWCDC (USB CDC) ports through modern C++ template metaprogramming techniques.

### Implementation

#### SerialPort Wrapper

```cpp
class SerialPort {
    enum class PortType { CDC, HARDWARE };
    Stream* _port;      // Common base for operations
    PortType _type;     // Runtime type information
};
```

Key features:
- Type-safe wrapper around different serial port implementations
- Common Stream interface for shared operations
- Specialized handling for port-specific operations

#### Compile-Time Port Detection

```cpp
template<typename T>
static PortType getPortType() {
    if constexpr (std::is_same_v<T, HWCDC>) {
        return PortType::CDC;
    } else {
        return PortType::HARDWARE;
    }
}
```

#### Type-Safe Construction

```cpp
template<typename T>
SerialPort(T& serial) : _port(&serial), _type(getPortType<T>()) {
    static_assert(std::is_base_of_v<Stream, T>, 
                  "Serial port type must derive from Stream");
}
```

#### Port-Specific Method Implementation

```cpp
void begin(unsigned long baud) {
    if (_type == PortType::CDC) {
        static_cast<HWCDC*>(_port)->begin();
    } else {
        static_cast<HardwareSerial*>(_port)->begin(baud);
    }
}
```

### Usage

```cpp
// Hardware Serial
AsyncSerial uartSerial(Serial1);
uartSerial.begin(9600);

// USB CDC
AsyncSerial usbSerial(Serial);
usbSerial.begin(115200);
```

## Key Components

### 1. AsyncSerial
The main driver class managing the physical UART:
- Multiplexes data between physical UART and logical proxies
- Maintains a state machine for UART operations
- Handles priority and timing

### 2. SerialProxy
A virtual serial port implementing the `Stream` interface:
```cpp
AsyncSerial serial(Serial);
serial.begin(115200);

SerialProxy<1024> console;  // 1KB buffer
SerialProxy<256> protocol;  // 256B buffer

serial.registerProxy(&console);
serial.registerProxy(&protocol);

// Use exactly like Serial:
console.println("Debug message");
protocol.print("AT+COMMAND\r\n");
```

### 3. RingBuffer & CooperativeLock
Thread-safe mechanisms ensuring non-blocking resource access
- `RingBuffer` between `AsyncSerial` and proxies (replicates the existing Serial ring buffer to each proxy)
- `CooperativeLock` to protect the `flush()` operation (ensures each proxy can flush its own buffer + ensures its content is actually sent to the physical serial, without race condition)

# AsyncSerial Documentation

## Drop-in Serial Compatibility

### Fully Compatible Methods
SerialProxy implements all standard Serial/Stream methods:
```cpp
// Basic I/O
int available()
int read()
int peek()
size_t write(uint8_t)
size_t write(const uint8_t*, size_t)
void flush()
int availableForWrite()

// String operations
String readString()
String readStringUntil(char terminator)
size_t readBytes(uint8_t *buffer, size_t length)
size_t readBytesUntil(char terminator, uint8_t *buffer, size_t length)

// Configuration
void setTimeout(unsigned long timeout)
unsigned long getTimeout()
void begin(unsigned long baud)
void begin(unsigned long baud, uint16_t config)
void end()
```

### Placeholder Methods
Some methods are implemented as no-ops for compatibility but don't provide functionality:
```cpp
bool find(char *target)                 // Always returns false
bool find(uint8_t *target, size_t)      // Always returns false
bool findUntil(char*, char*)            // Always returns false
bool findUntil(uint8_t*, size_t, char*, size_t) // Always returns false
float parseFloat()                      // Always returns 0.0
long parseInt()                         // Always returns 0
long parseInt(char skipChar)            // Always returns 0
```

### Key Differences from Serial

1. **Configuration Methods**
```cpp
// These don't actually configure the hardware when called on the proxy.
// Configure the hardware via AsyncSerial instance's begin()/end().
proxy.begin(9600);    // No effect on physical port config
proxy.end();          // No effect on physical port config
```

2. **Blocking Operations**
```cpp
// These methods block but maintain system responsiveness by calling poll()
String response = proxy.readStringUntil('\n');  // Blocks with timeout
size_t count = proxy.readBytes(buffer, size);   // Blocks with timeout
```

3. **Buffer Management**
```cpp
// Each proxy has its own fixed-size buffer
SerialProxy<1024> proxy;  // Buffer size must be defined at compile time
```

## Thread Safety Considerations

### Thread-Safe Operations
`flush()` is protected by `CooperativeLock`.  
`poll()` is safe to call frequently from a dedicated task.

### Non-Thread-Safe Operations
```cpp
// Must be called only during initialization phase (e.g., in setup())
asyncSerial.begin()
asyncSerial.end()
asyncSerial.registerProxy()

// Must be synchronized if called from multiple threads concurrently (but all the point of this implementation is to allow one individual proxy per thread, so if you need to share a proxy between threads, you're probably doing something wrong)
proxy.write()
proxy.read()
proxy.available()
```

### Initialization Pattern
```cpp
AsyncSerial hwSerial(Serial1);
SerialProxy<1024> debugProxy;
SerialProxy<512> protocolProxy;

void setup() {
    // 1. Initialize AsyncSerial first (non-thread-safe)
    hwSerial.begin(115200);
    
    // 2. Register all proxies (non-thread-safe)
    hwSerial.registerProxy(&debugProxy);
    hwSerial.registerProxy(&protocolProxy);
    
    // 3. After this, thread-safe operations can begin
    // If using RTOS, create tasks now
}
```

### RTOS Usage Notes

Below is an example for Arduino ESP32 using FreeRTOS. We create a polling task to regularly call `poll()`, and another task to handle console input. We pass the `SerialProxy` instance to the task via `void* parameter` and cast it back inside the task.

```cpp
#include <Arduino.h>
#include "AsyncSerial.h"
#include <FreeRTOS.h>
#include <task.h>

AsyncSerial hwSerial(Serial1);
SerialProxy<1024> consoleProxy;

void pollingTask(void* parameter) {
    for (;;) {
        hwSerial.poll(); // Keep AsyncSerial state machine running
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void consoleTask(void* parameter) {
    SerialProxy<1024>& console = *((SerialProxy<1024>*)parameter);
    for (;;) {
        if (console.available()) {
            String line = console.readStringUntil('\n');
            console.println("Received: " + line);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    hwSerial.begin(115200);
    hwSerial.registerProxy(&consoleProxy);

    // Create RTOS tasks
    xTaskCreate(pollingTask, "PollingTask", 2048, NULL, 2, NULL);
    xTaskCreate(consoleTask, "ConsoleTask", 2048, &consoleProxy, 1, NULL);
}

void loop() {
    // Not used in RTOS environment
}
```

## State Machine

The library operates in four states:

1. **IDLE**: Searching for work
   - Checks for incoming Serial data
   - Looks for proxies with pending transmissions

2. **READ**: Processing incoming data
   - Reads data in chunks
   - Distributes to all proxies

3. **WRITE**: Sending data
   - Respects inter-message delays
   - Handles one proxy at a time

4. **FLUSH**: Ensuring transmission
   - Blocks until complete
   - Enforces timeout

## Integration Examples

### With Existing Libraries
```cpp
// Using with libraries that expect Serial
class ModbusPort : public SerialProxy<512> {
public:
    ModbusPort() : SerialProxy<512>(10) {}  // 10ms delay
};

ModbusPort modbusSerial;
ModbusMaster node(1, modbusSerial);  // Works with ModbusMaster library

// Initialize AsyncSerial and register modbusSerial similarly
```

### In RTOS Environment
As shown above, you can create tasks for polling and for handling data on a given proxy. Each task receives the proxy pointer, casts it, and uses it as needed.

## Best Practices

1. **Buffer Sizing**
   - Use power of 2 sizes for optimal memory alignment
   - Match size to expected data patterns:
     - Large for debug logs (1024-2048 bytes)
     - Small for status updates (128-256 bytes)
     - Medium for protocols (512 bytes)

2. **Inter-message Delays**
   - Use smaller delays for real-time data (1-5ms)
   - Use larger delays for human-readable output (20-50ms)
   - Consider protocol timing requirements

3. **Polling Frequency**
   - Call `poll()` in the main loop or a dedicated RTOS task
   - Balance responsiveness vs CPU usage
   - Consider using timer interrupts for consistent polling

4. **Error Handling**
   - Check buffer overflow conditions
   - Implement timeout handling for flush operations
   - Monitor proxy registration success

## Performance Considerations

1. **Memory Usage**
   - Each proxy consumes: 2 * BUFFER_SIZE + overhead
   - Static allocation prevents fragmentation
   - Configure buffer sizes based on available RAM

2. **CPU Usage**
   - State machine design minimizes busy-waiting
   - Chunk-based operations reduce interrupt overhead
   - Cooperative locking avoids priority inversion

3. **Latency**
   - Inter-message delays affect throughput
   - Polling frequency impacts responsiveness
   - Multiple proxies share bandwidth

## Appendix - `RingBuffer`

### Overview
The `RingBuffer` is a templated circular buffer implementation providing efficient, fixed-size FIFO operations:
- Template parameters for type and size
- No dynamic memory allocation
- Thread-safe for single producer/single consumer
- Overflow protection

### Implementation Details
```cpp
template<typename T, size_t SIZE>
class RingBuffer {
    T _buffer[SIZE];
    size_t _readIndex;
    size_t _writeIndex;
    size_t _count;
}
```

### Key Operations
```cpp
bool write(const T& data);
bool write(const T* data, size_t length);
bool read(T& data);
bool peek(T& data);
```

### Usage Example
```cpp
RingBuffer<uint8_t, 256> buffer;
uint8_t data = 0x42;
buffer.write(data);

uint8_t bulk[4] = {1, 2, 3, 4};
buffer.write(bulk, 4);

uint8_t readData;
if (buffer.read(readData)) {
    // process readData
}
```

### Memory Efficiency
- Fixed size at compile time
- No fragmentation
- Simple and predictable

### Integration with AsyncSerial
Each `SerialProxy` uses two `RingBuffer`s internally for TX and RX, providing independent buffering and efficient handling of data.

## Appendix - `CooperativeLock`

### Overview
The `CooperativeLock` is a synchronization primitive designed for cooperative multitasking. Unlike traditional mutex implementations, it:
- Never blocks completely
- Allows useful work to be done while waiting

### Core Implementation
```cpp
template<typename T>
class CooperativeLock {
public:
    using PollCallback = std::function<void()>;
    
    bool acquire(T* owner, PollCallback poll) {
        T* expected = nullptr;
        while (!_owner.compare_exchange_weak(expected, owner)) {
            expected = nullptr;
            poll();  // Do useful work (e.g., call asyncSerial.poll())
        }
        return true;
    }

    void release(T* owner) {
        T* expected = owner;
        _owner.compare_exchange_strong(expected, nullptr);
    }

    bool isOwnedBy(T* owner) const {
        return _owner.load() == owner;
    }

private:
    std::atomic<T*> _owner;
};
```

### Key Features

1. **Type Safety**
// Lock can only be used with the specified type
CooperativeLock<SerialProxyBase> proxyLock;    // For proxies
CooperativeLock<Task> taskLock;                // For tasks

2. **Atomic Operations**
- Uses `std::atomic` for thread-safe operations
- `compare_exchange_weak` for lock attempts
- `compare_exchange_strong` for release
- `load` for ownership checks

3. **Cooperative Nature**
```cpp
// Instead of blocking, poll is called
lock.acquire(&proxy, [&]() {
    hwSerial.poll();  // Keep system moving while waiting
});
```

### Usage in AsyncSerial

#### 1. Flush Protection
```cpp
bool AsyncSerial::flush(SerialProxyBase* proxy) {
    // Acquire lock and poll while waiting
    _flushLock.acquire(proxy, [this]() { poll(); });

    _state = State::FLUSH;
    // ... perform flush ...

    // Release the lock
    _flushLock.release(proxy);
    return true;
}
```

#### 2. Custom Integration Example
```cpp
class CustomProxy : public SerialProxyBase {
private:
    CooperativeLock<CustomProxy> _txLock;

    bool sendMessage(const String& msg) {
        // Try to acquire lock with custom work
        _txLock.acquire(this, [this]() {
            // Perform useful work while waiting for lock
            processRxBuffer();
        });
        bool result = doSendMessage(msg);
        _txLock.release(this);
        return result;
    }
};
```

### Best Practices

#### **Keep Critical Sections Short**
   - Acquire, do minimal work, release quickly
```cpp

// Good - Short critical section
lock.acquire(owner, callback);
doQuickOperation();
lock.release(owner);

// Bad - Long critical section
lock.acquire(owner, callback);
doLengthyOperation();  // Others wait too long
lock.release(owner);
```

#### **Useful Work in Callbacks**
   - Avoid busy-waiting
```cpp
// Good - Productive waiting
lock.acquire(&proxy, []() {
    processIncoming();
    updateStatus();
    AsyncSerial::getInstance().poll();
});

// Bad - Busy waiting
lock.acquire(&proxy, []() {
    delayMicroseconds(10);  // Just wastes CPU
});
```

#### **Always Release**
   - Ensure lock is released after use (RAII patterns recommended)
```cpp
// Good - RAII-style usage
class LockGuard {
    CooperativeLock<T>& _lock;
    T* _owner;

public:
    LockGuard(CooperativeLock<T>& lock, T* owner, 
              std::function<void()> poll) 
        : _lock(lock), _owner(owner) {
        _lock.acquire(owner, poll);
    }
    ~LockGuard() {
        _lock.release(_owner);
    }
};
```

### Performance Considerations
1. **Memory Impact**
   - Single atomic pointer per lock
   - No additional memory allocation
   - Cache-friendly single-word storage

2. **CPU Usage**
   - No spinlock busy-waiting
   - Useful work during contention
   - Lock-free atomic operations

3. **Contention Handling**
   - No priority inversion
   - No thread blocking
   - Fair access through polling

### Common Use Cases
- Resource protection (one writer at a time)
- State synchronization without blocking