# SimpleTimer

A lightweight, persistent and versatile timer class for embedded systems and IoT applications. This timer implementation provides a clean and efficient way to handle timing operations with automatic state persistence.

## Key Features

### Unique State Persistence
The most distinctive feature is its effortless persistence handling:
- No global variables needed
- No complex static declarations
- No need to think about where to define your timers
- Just create and use timers anywhere in your code - they'll maintain their state automatically

### Technical Features
- Hardware agnostic implementation (works on any platform)
- Independent timer instances
- Pause/Resume capabilities
- Overflow protection
- Memory efficient implementation
- *Millisecond precision (microsecond precision possible with minor modifications)*

## Example

```cpp
void loop() {
    Timer t1;          // Create timer on-the-fly (persistence is automatic)
    
    if(t1.done(1000)) {         // Check if 1000 ms elapsed
        Serial.println("Timer expired!");
    }
    
    Serial.println(t1.elapsed()); // Get elapsed time
    
    t1.pause();       // Pause the timer
    delay(500);       // Do something while paused
    t1.resume();      // Resume the timer
}
```

## API Reference

### Core Methods
- `elapsed()`: Returns elapsed time since start/reset in milliseconds
- `done(duration)`: Returns true if specified duration has elapsed and automatically resets
- `check(duration)`: Similar to done() but without auto-reset
- `reset()`: Manually resets the timer
- `pause()`: Temporarily stops the timer
- `resume()`: Resumes a paused timer

### Technical Notes
- Uses `unsigned long` for duration values
- First `check()` or `done()` call initializes the timer
- Paused timers maintain their elapsed time
- Handles timer overflow correctly