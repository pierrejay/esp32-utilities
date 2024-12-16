# SimpleTimer

A lightweight and versatile timer class for ESP32. This implementation provides a clean and efficient way to handle timing operations with microsecond precision while offering a simple millisecond-based interface.

## Key Features

### Flexible Usage
- Use as local variable: timer resets each time the scope is entered
- Use as static local: state persists between function calls
- Use as global: state shared across the program
- No special setup needed - just declare and use!

### Technical Features
- Uses ESP32's high-precision timer (esp_timer_get_time)
- Independent timer instances
- Pause/Resume capabilities
- 64-bit overflow protection
- Memory efficient implementation (24 bytes per instance)
- Microsecond precision internally, millisecond interface externally

## Example

```cpp
// As regular local (resets each call)
void function1() {
    Timer t1;
    if(t1.done(1000)) {
        Serial.println("Timer expired!");
    }
}

// As static local (persists between calls)
void function2() {
    static Timer t2;
    if(t2.done(1000)) {
        Serial.println("Triggers every second!");
    }
}

// As global (shared state)
Timer globalTimer;
void function3() {
    if(globalTimer.done(1000)) {
        Serial.println("Shared timing!");
    }
}
```

## API Reference

### Core Methods
- `elapsed()`: Returns elapsed time since start/reset in milliseconds
- `done(duration)`: Returns true if specified duration has elapsed and automatically resets
- `check(duration)`: Like done() but without auto-reset
- `reset()`: Manually resets the timer
- `pause()`: Temporarily stops the timer
- `resume()`: Resumes a paused timer

### Technical Notes
- Uses `uint64_t` internally for microsecond precision
- Returns `unsigned long` (milliseconds) for all time values
- First instantiation initializes the timer
- Paused timers maintain their elapsed time
- Handles 64-bit timer overflow correctly

## Implementation
The timer uses ESP32's esp_timer_get_time() for microsecond precision while presenting a simple millisecond-based interface. Each instance requires only 24 bytes of memory (two 64-bit timestamps and a boolean flag).