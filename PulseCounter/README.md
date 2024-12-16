# ESP32 Pulse Counter Library

A high-performance pulse counting library for ESP32 microcontrollers that leverages the hardware `PCNT` (Pulse Counter) module. Supports multiple simultaneous counters, hardware-based debouncing, and provides accurate frequency measurements with minimal CPU overhead.

## Features

- Support for up to 8 simultaneous counters (N.B. ESP32-S2/S3 only have 4 PCNTs)
- Real-time frequency measurement with adjustable sample time
- Continuous pulse counting with 32-bit total counter
- Automatic overflow handling
- Configurable hardware debounce filtering
- Activity detection for reliable frequency measurement
- Global, timer-based polling system with microsecond precision
- Thread-safe operation

## Core Concepts

### Multiple Counter Support
- Up to 8 independent pulse counters can run simultaneously
- Each counter requires a unique PCNT unit (PCNT_UNIT_0 through PCNT_UNIT_7)
- All counters share a single global polling timer

### Hardware Filtering
- Built-in debounce filter using ESP32's hardware capabilities
- Filter length specified in APB_CLK cycles (80MHz on standard ESP32)
- Example: filterLength of 800 creates a 10µs debounce filter

### Global Polling System
- Single esp_timer handles all counter updates
- Configurable polling interval in microseconds
- More efficient than individual FreeRTOS tasks
- Thread-safe operation for all counters

### Sample Time
- Configurable measurement window for frequency calculation
- Independent from global polling interval
- Affects only frequency calculation, not counting accuracy
- Default is 100ms

## Usage

### Basic Setup

```cpp
#include "ESP32PulseLib.h"

// Initialize counter on GPIO 5, using PCNT_UNIT_0
PulseCounter counter(5);

void setup() {
    // Initialize the counter
    if (!counter.begin()) {
        Serial.println("Failed to initialize pulse counter!");
        return;
    }
    
    // Start global polling (default = 1ms interval)
    PulseCounter::startPolling();
}

void loop() {
    // No need to call poll() - handled by global timer
    float freq = counter.getFrequency();
    uint32_t count = counter.getCount();
}
```

### Multiple Counters

```cpp
// Create two counters on different pins and PCNT units
PulseCounter counter1(5, PCNT_UNIT_0);  // GPIO 5, PCNT unit 0
PulseCounter counter2(18, PCNT_UNIT_1); // GPIO 18, PCNT unit 1

void setup() {
    counter1.begin();
    counter2.begin();
    
    // Start global polling
    PulseCounter::startPolling();
}

void loop() {
    float freq1 = counter1.getFrequency();
    float freq2 = counter2.getFrequency();
}
```

### Hardware Filtering

```cpp
// Create counter with 10µs debounce filter
PulseCounter counter(5, PCNT_UNIT_0, 100, 800);  // 800 cycles = 10µs at 80MHz

// For mechanical switches, use longer filter
PulseCounter switchCounter(12, PCNT_UNIT_1, 100, 8000);  // 100µs debounce
```

## API Reference

### Constructor
```cpp
PulseCounter(
    uint8_t pin,                          // GPIO pin number
    pcnt_unit_t pcntUnit = PCNT_UNIT_0,  // PCNT unit (0-7)
    uint32_t sampleTimeMs = 100,          // Sample time for frequency
    uint16_t filterLength = 0             // Hardware filter length (APB cycles)
)
```

### Static Methods

#### `static void startPollingGlobal()`
Starts the global polling timer for all counters.

#### `static void stopPollingGlobal()`
Stops the global polling timer.

### Instance Methods

#### `bool begin()`
Initializes the pulse counter. Returns `true` if successful.

#### `float getFrequency()`
Returns current frequency in Hz. Returns 0 if no signal detected.

#### `uint32_t getCount()`
Returns total number of pulses counted since last reset.

#### `void resetCount()`
Resets both the frequency measurement and total pulse count.

#### `void setSampleTime(uint32_t newSampleTimeMs)`
Changes the sample time for frequency calculations.

#### `uint32_t getSampleTime()`
Returns current sample time in milliseconds.

## Implementation Details

### Global Polling System
- Uses esp_timer for precise timing
- Automatically polls all active counters
- Thread-safe operation with portENTER_CRITICAL
- Minimal overhead per counter
The global polling system uses a fixed 1 ms interval to update all counters. This value is sufficient for the majority of use cases, ensuring accurate pulse counting and frequency measurement without overloading the CPU.
At a 1 ms interval, the library can handle signals with up to 3 MHz pulse frequency (30,000 pulses per interval) reliably using ESP32's hardware PCNT capabilities.
However, you can override this value by providing an argument when starting polling if you need higher frequencies, or to further decrease CPU load in case of lower frequencies :
`static void startPollingGlobal(uint32_t intervalUs = 1000)`

### Counter Management
- Static array manages up to 8 counter instances
- Automatic counter registration and cleanup
- Protected by RTOS mutex for thread safety
- Efficient polling in ISR context

### Frequency Calculation
- Based on pulse count delta over sample time
- Uses esp_timer_get_time() for microsecond precision
- Includes signal presence detection
- Updates only when valid signal detected

## Performance & Limitations

### Limitations
- Maximum of 8 simultaneous counters (ESP32 hardware limit)
- Single channel operation (PCNT_CHANNEL_0 only)
- Global polling interval affects all counters
- High-frequency signals may require very short polling intervals

### Best Practices
1. Select proper sample time:
   - Longer for stable readings
   - Shorter for faster updates
   - Consider signal frequency

2. Configure hardware filter:
   - Mechanical switches: 8000-16000 cycles (100-200µs)
   - Electronic signals: 0-800 cycles (0-10µs)
   - High-speed signals: Disable filter (0)

### Performance
The library has been tested on ESP32-S3 and provides exceptional accuracy across a wide frequency range. The hardware-based implementation ensures reliable operation even under heavy system load.