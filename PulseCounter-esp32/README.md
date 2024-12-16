# ESP32 Pulse Counter Library

A high-performance pulse counting library leveraging ESP32's built-in PCNT (Pulse Counter) hardware module. The library provides accurate pulse counting and frequency measurement capabilities with minimal CPU overhead.

## Features

- Hardware-based pulse counting using ESP32's PCNT module
- Real-time frequency measurement with configurable sample time
- Continuous pulse counting with 32-bit total counter
- Automatic overflow handling
- Activity detection for reliable frequency measurement
- Zero CPU overhead for pulse counting operations
- Minimal processing requirements in main loop
- Single edge counting (rising edge only) - can be modified by editing the `pcnt_config_t` object

## Core Concepts

### Sample Time
The library uses a configurable sample time (in milliseconds) to calculate frequency:
- Shorter sample times (e.g., 5ms) provide faster updates but may be less stable
- Longer sample times (e.g., 100ms) provide more stable readings
- The sample time affects only frequency calculation, not pulse counting accuracy
- Default sample time is 100ms

### Frequency Measurement
- Calculated by counting pulses over the sample time window
- Automatically detects signal presence/absence
- Returns 0 Hz when no signal is detected
- Updates only when valid signal is present
- Maintains last valid reading until timeout

### Total Counter
- 32-bit counter can accumulate up to 4,294,967,295 pulses
- Hardware counter automatically manages overflow
- Counter can be reset at any time
- Continues counting regardless of frequency calculation

## Usage

### Basic Setup

```cpp
#include "ESP32PulseLib.h"

// Initialize counter on GPIO pin 5 with default 100ms sample time
PulseCounter counter(5);

void setup() {
    // Initialize the pulse counter
    if (!counter.begin()) {
        Serial.println("Failed to initialize pulse counter!");
        return;
    }
}

void loop() {
    // IMPORTANT: Must call poll() regularly to update measurements
    counter.poll();
    
    // Read current frequency in Hz
    float freq = counter.getFrequency();
    
    // Read total pulse count since last reset
    uint32_t count = counter.getCount();
}
```

### Advanced Usage

```cpp
// Create counter with 5ms sample time for faster updates
PulseCounter counter(GPIO_NUM_5, 5);  

void setup() {
    counter.begin();
}

void loop() {
    // CRITICAL: poll() must be called frequently enough to prevent overflow
    // With a 5ms sample time, call at least every 2-3ms to be safe
    counter.poll();
    
    if (counter.getFrequency() > 0) {
        // Process active signal
    }
    
    // Reset counter if needed
    if (someCondition) {
        counter.resetCount();
    }
    
    // Dynamically adjust sample time
    counter.setSampleTime(newSampleTime);
}
```

## API Reference

### Constructor
```cpp
PulseCounter(uint8_t pin, uint32_t sampleTimeMs = 100)
```
- `pin`: GPIO pin to use for pulse counting
- `sampleTimeMs`: Sample time for frequency calculations (default: 100ms)

### Core Methods

#### `bool begin()`
Initializes the pulse counter. Returns `true` if successful.

#### `void poll()`
Updates frequency and count measurements. Must be called regularly.

**Important**: Call frequency depends on sample time:
- Call at least 2x faster than sample time to prevent overflow
- Example: with 5ms sample time, call every 2-3ms
- Blocking operations between polls may cause missed pulses
- Consider running in a dedicated task for critical applications

#### `float getFrequency()`
Returns current frequency in Hz. Returns 0 if no signal detected.

#### `uint32_t getCount()`
Returns total number of pulses counted since last reset.

#### `void resetCount()`
Resets both the frequency measurement and total pulse count.

### Configuration Methods

#### `void setSampleTime(uint32_t newSampleTimeMs)`
Changes the sample time for frequency calculations.
- Shorter times: faster updates but less stable
- Longer times: more stable but slower updates
- Takes effect at next sample window

#### `uint32_t getSampleTime()`
Returns current sample time in milliseconds.

## Implementation Details

### Frequency Calculation
- Based on pulse count delta over sample time
- Uses hardware timer for precise timing
- Automatically handles counter overflow
- Includes signal presence detection
- Updates only when valid signal detected

### Counter Management
- Uses ESP32 PCNT hardware module
- Zero CPU overhead for counting
- 16-bit hardware counter with overflow handling
- Expanded to 32-bit in software
- Automatic rollover management

### Activity Detection
- Monitors pulse presence continuously
- Requires at least one pulse per 2x sample time
- Automatically zeros frequency when signal lost
- Prevents false readings during signal absence

## Performance Notes

The library has been extensively tested and provides exceptional accuracy across a wide frequency range, up to at least 5 MHz. Performance is maintained even under heavy system load due to the hardware-based implementation.