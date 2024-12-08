# AsyncSerial Documentation

This document provides a comprehensive overview of the **AsyncSerial** library, which enables **asynchronous, thread-safe serial communication**. The library is designed to support **multiple logical streams** over a single physical UART using **proxies**, while maintaining full compatibility with Arduino's Serial interface.

## Core Concepts

### Drop-in Serial Replacement
AsyncSerial is designed to be a direct replacement for Arduino's Serial interface:
- Each SerialProxy implements the standard `Stream` interface
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
- RTOS environments
- Cooperative multitasking
- Interrupt-driven architectures

---

## Key Components

### 1. AsyncSerial
The main driver class managing the physical UART:
- Multiplexes data between physical UART and logical proxies
- Maintains a state machine for UART operations
- Handles priority and timing

### 2. SerialProxy
A virtual serial port implementing the `Stream` interface:
```cpp
SerialProxy<1024> console;  // 1KB buffer
SerialProxy<256> protocol;  // 256B buffer

// Use exactly like Serial:
console.println("Debug message");
protocol.print("AT+COMMAND\r\n");
```

### 3. RingBuffer & CooperativeLock
Thread-safe mechanisms ensuring non-blocking resource access
- RingBuffer between AsyncSerial and proxies (replicates the existing Serial ring buffer to each proxy)
- CooperativeLock to protect the flush() operation (ensures each proxy can flush its own buffer + ensures its content is actually sent to the physical serial, without race condition)

## Proxy Usage Examples

### Basic Usage (Drop-in Replacement)
```cpp
// Instead of Serial.println():
debugProxy.println("System started");  

// Instead of Serial.available() and Serial.read():
if (protocolProxy.available()) {
    char c = protocolProxy.read();
}

// Instead of Serial.readStringUntil():
String response = protocolProxy.readStringUntil('\n');
```

### Advanced Usage
```cpp
// Custom buffer sizes for different needs
SerialProxy<2048> debugProxy;    // Large buffer for debug logs
SerialProxy<128> statusProxy;    // Small buffer for status updates
SerialProxy<512> protocolProxy;  // Medium buffer for protocol

// Independent delays
SerialProxy<1024> fastProxy(1);    // 1ms between messages
SerialProxy<1024> slowProxy(50);   // 50ms between messages
```

## Buffer Management and Flushing

### Buffer Types
Each proxy maintains two independent ring buffers:
- TX buffer for outgoing data
- RX buffer for incoming data

### Flushing Behavior
Both proxy flush and direct flush are blocking operations with timeout protection:

1. **Proxy-Level Flush** (SerialProxy::flush)
```cpp
// Blocking flush via AsyncSerial
debugProxy.flush();  // Blocks until complete or timeout
```

2. **Direct Flush** (AsyncSerial::flush)
```cpp
// Blocking flush with timeout
bool success = AsyncSerial::getInstance().flush(&debugProxy);
if (!success) {
    // Handle timeout
}
```

Key characteristics:
- Both methods are blocking operations
- Both include timeout protection (default 1000ms)
- Both ensure all data is transmitted or timeout occurs
- Both use CooperativeLock to manage access
- While blocking, they continue to call poll() to maintain system responsiveness
- Return false if timeout occurs, true if flush completes successfully

Typical use cases:
```cpp
// When you need to ensure all data is sent:
debugProxy.println("Critical message");
if (!AsyncSerial::getInstance().flush(&debugProxy)) {
    // Handle timeout - data may not have been fully sent
}

// Or using proxy's flush method (identical behavior):
debugProxy.println("Critical message");
debugProxy.flush();  // Will block until complete or timeout
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
```

### In RTOS Environment
```cpp
void debugTask(void* parameter) {
    SerialProxy<1024>& debug = *((SerialProxy<1024>*)parameter);
    for (;;) {
        debug.println("Task running...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void protocolTask(void* parameter) {
    SerialProxy<512>& protocol = *((SerialProxy<512>*)parameter);
    for (;;) {
        if (protocol.available()) {
            processProtocolData(protocol);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

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
   - Call `poll()` in main loop or high-priority task
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
The `RingBuffer` is a templated circular buffer implementation that provides efficient, fixed-size FIFO (First-In-First-Out) operations. Key characteristics:
- Template parameters for type and size
- No dynamic memory allocation
- Thread-safe for single producer/single consumer
- Overflow protection

### Implementation Details
```cpp
template<typename T, size_t SIZE>
class RingBuffer {
    T _buffer[SIZE];      // Fixed-size array
    size_t _readIndex;    // Next position to read
    size_t _writeIndex;   // Next position to write
    size_t _count;        // Current number of items
}
```

### Key Operations

#### Writing Data
```cpp
// Single item write
bool write(const T& data);

// Bulk write
bool write(const T* data, size_t length);
```
- Returns `false` if buffer would overflow
- Writing maintains FIFO order
- O(1) for single writes, O(n) for bulk writes

#### Reading Data
```cpp
// Destructive read
bool read(T& data);

// Non-destructive read
bool peek(T& data);
```
- Returns `false` if buffer is empty
- Reading preserves FIFO order
- Both operations are O(1)

### Usage Example
```cpp
RingBuffer<uint8_t, 256> buffer;  // 256-byte buffer

// Writing data
uint8_t data = 0x42;
if (!buffer.write(data)) {
    // Handle buffer full condition
}

// Bulk writing
uint8_t bulk[4] = {1, 2, 3, 4};
if (!buffer.write(bulk, 4)) {
    // Handle insufficient space
}

// Reading data
uint8_t readData;
if (buffer.read(readData)) {
    // Process readData
}

// Peeking at next item
uint8_t peekData;
if (buffer.peek(peekData)) {
    // Examine peekData without removing it
}
```

### Memory Efficiency
- Size must be defined at compile time
- Memory usage is exactly `sizeof(T) * SIZE + 3 * sizeof(size_t)`
- No memory fragmentation
- Indexes wrap around using modulo arithmetic

### Best Practices

1. **Size Selection**
```cpp
// Good - Power of 2 size for efficient modulo
RingBuffer<uint8_t, 256> goodBuffer;   // 256 = 2^8
RingBuffer<uint8_t, 512> alsoGood;     // 512 = 2^9

// Avoid - Non-power of 2 size
RingBuffer<uint8_t, 250> lessEfficient;  // Modulo is more expensive
```

2. **Overflow Handling**
```cpp
// Always check write operations
uint8_t data[8];
if (!buffer.write(data, sizeof(data))) {
    // 1. Skip data
    // 2. Wait and retry
    // 3. Flush buffer first
    // 4. Signal error condition
}
```

3. **Empty Condition Handling**
```cpp
// Check before each read
uint8_t data;
while (buffer.peek(data)) {  // Non-destructive check
    if (isValid(data)) {
        buffer.read(data);    // Actually remove the data
        process(data);
    } else {
        buffer.read(data);    // Discard invalid data
    }
}
```

### Integration with AsyncSerial

The RingBuffer is used internally by each SerialProxy for both TX and RX:
```cpp
template<size_t BUFFER_SIZE = 1024>
class SerialProxy : public SerialProxyBase {
private:
    RingBuffer<uint8_t, BUFFER_SIZE> _rxBuffer;  // Incoming data
    RingBuffer<uint8_t, BUFFER_SIZE> _txBuffer;  // Outgoing data
    //...
};
```

This provides:
- Independent buffers for each direction
- Configurable size per proxy
- Overflow protection
- Efficient memory usage

[Previous sections remain unchanged]

## Appendix - `CooperativeLock`

### Overview
The `CooperativeLock` is a lightweight synchronization primitive designed for cooperative multitasking. Unlike traditional mutex implementations, it:
- Never blocks completely
- Allows work to be done while waiting
- Uses atomic operations for thread safety
- Is template-based for type safety

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
            poll();  // Do useful work while waiting
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

#### 1. Type Safety
```cpp
// Lock can only be used with the specified type
CooperativeLock<SerialProxyBase> proxyLock;    // For proxies
CooperativeLock<Task> taskLock;                // For tasks
```

#### 2. Atomic Operations
- Uses `std::atomic` for thread-safe operations
- `compare_exchange_weak` for lock attempts
- `compare_exchange_strong` for release
- `load` for ownership checks

#### 3. Cooperative Nature
```cpp
// Instead of blocking:
lock.acquire(&proxy, []() {
    // Do useful work while waiting:
    AsyncSerial::getInstance().poll();
});
```

### Usage in AsyncSerial

#### 1. Flush Protection
```cpp
// In AsyncSerial::flush
bool AsyncSerial::flush(SerialProxyBase* proxy) {
    // Take the lock, keep polling while waiting
    _flushLock.acquire(proxy, [this]() { 
        poll(); 
    });

    // Lock acquired, do the flush operation
    _state = State::FLUSH;
    
    // ... flush operation ...

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
            // Process incoming data while waiting
            processRxBuffer();
            // Keep serial operations moving
            AsyncSerial::getInstance().poll();
        });
        
        // Lock acquired, send message
        bool result = doSendMessage(msg);
        
        _txLock.release(this);
        return result;
    }
};
```

### Best Practices

#### 1. Keep Critical Sections Short
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

#### 2. Useful Work in Callbacks
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

#### 3. Always Release
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

1. **Resource Protection**
```cpp
CooperativeLock<Resource> resourceLock;

void useResource(Resource* resource) {
    resourceLock.acquire(resource, []() {
        // Work while waiting
        doOtherStuff();
    });
    // Resource is now protected
    resource->use();
    resourceLock.release(resource);
}
```

2. **State Synchronization**
```cpp
CooperativeLock<State> stateLock;

void updateState(State* state) {
    stateLock.acquire(state, []() {
        // Keep system responsive
        AsyncSerial::getInstance().poll();
    });
    state->update();
    stateLock.release(state);
}
```
