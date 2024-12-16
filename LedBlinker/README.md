# LedBlinker Library

A simple and flexible Arduino library for controlling LED blinking patterns with FreeRTOS support.

## Features

- Simple pin configuration with support for both active-high and active-low LEDs
- Multiple blinking modes:
  - Continuous blinking at a specified frequency
  - Fixed number of blinks
  - Repeating sequences with configurable periods
- Zero-frequency mode for static LED state
- Non-blocking operation using millis()
- FreeRTOS support with automatic update task

## Usage

### Basic Setup

```cpp
// Create a LedBlinker instance (active-high LED on pin 13)
LedBlinker led(13, true);

void setup() {
    // Initialize the LED
    led.begin();
}

void loop() {
    // Must be called in loop if auto-update is disabled
    led.update();
}
```

### Auto-Update Mode

```cpp
void setup() {
    led.begin();
    // Enable automatic updates using FreeRTOS task
    led.enableAutoUpdate();
    // No need to call update() in loop anymore
}
```

### Blinking Modes

#### Continuous Blinking

```cpp
// Blink continuously at 2Hz
led.setMode(2);
```

#### Fixed Number of Blinks

```cpp
// Blink 3 times at 5Hz
led.setMode(3, 5);
```

#### Repeating Sequence

```cpp
// Blink 2 times at 3Hz, repeat every 2000ms (2 seconds)
led.setMode(2, 3, 2000);
```

## Constructor Parameters

- `pin`: The Arduino pin number connected to the LED
- `activeHigh`: Set to `true` for LEDs connected to VCC (active-high), `false` for LEDs connected to GND (active-low). Default is `false`.

## Methods

### `void begin()`

Initializes the LED pin. Must be called in `setup()`.

### `void enableAutoUpdate(bool enable = true)`

Enables or disables automatic LED state updates using a FreeRTOS task.
- When enabled, no need to call `update()` in loop
- Task runs with priority 1 and 1024 bytes stack size
- 10ms delay between updates

### `void setMode(uint16_t freq)`

Sets continuous blinking mode.
- `freq`: Frequency in Hz (0 stops blinking)

### `void setMode(uint8_t blinks, uint16_t freq)`

Sets single sequence blinking mode.
- `blinks`: Number of blinks to perform
- `freq`: Blinking frequency in Hz

### `void setMode(uint8_t blinks, uint16_t freq, unsigned long period)`

Sets repeating sequence blinking mode.
- `blinks`: Number of blinks in each sequence
- `freq`: Blinking frequency in Hz
- `period`: Time between sequence starts in milliseconds

### `void update()`

Updates LED state. Must be called regularly in `loop()` if auto-update is disabled.

## Example with Auto-Update

```cpp
LedBlinker statusLed(13, true);

void setup() {
    statusLed.begin();
    // Enable automatic updates
    statusLed.enableAutoUpdate();
    // Blink 3 times at 2Hz, repeat every 5 seconds
    statusLed.setMode(3, 2, 5000);
}

void loop() {
    // No need to call update() here
    // Do other tasks...
}
```