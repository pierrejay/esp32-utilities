#pragma once

/**
 * @class Timer
 * @brief A simple, persistent, and versatile timer class for embedded systems.
 *
 * This class provides a lightweight timer implementation with straightforward usage
 * and additional features like pause/resume capabilities. Each `Timer` instance maintains 
 * its own state, allowing multiple independent timers if desired. By declaring the timer 
 * as a static local variable, its state persists across function calls.
 *
 * ## Key Features
 * - **Simple API**: Methods are easy to use and follow the KISS principle.
 * - **Pause/Resume**: Ability to temporarily halt the timer and resume later.
 * - **Overflow Protection**: Automatically handles time wrap-around safely.
 * - **Flexible Usage**: Instantiable as global, static local, or regular local variables, 
 *   giving you control over when and how the timer state persists.
 *
 * ## Usage Example
 * ```cpp
 * void loop() {
 *     static Timer t1; // Timer persists between loop() calls
 *     
 *     if (t1.done(1000)) { // Check if 1000 ms have elapsed since last reset
 *         Serial.println("Timer expired!");
 *     }
 *     
 *     Serial.println(t1.elapsed()); // Print elapsed time in ms
 *     
 *     t1.pause();    // Pause the timer
 *     delay(500);     // Do some work while paused
 *     t1.resume();   // Resume counting
 * }
 * ```
 *
 * ## Methods
 * - **elapsed()**: Returns elapsed time since start or last reset (in milliseconds).
 * - **done(duration)**: Checks if the specified duration (in ms) has elapsed. 
 *   If yes, returns true and resets the timer to start counting again.
 * - **check(duration)**: Like `done()`, but does not reset the timer.
 * - **reset()**: Manually resets the timer's reference start time.
 * - **pause()**: Halts the timer, preserving the elapsed time so far.
 * - **resume()**: Resumes the timer from the point it was paused.
 *
 * ## Notes
 * - Use `unsigned long` for durations to match the timer's operations.
 * - If the timer hasn't started yet, the first call to `check()` or `done()` 
 *   initializes it.
 * - While paused, the elapsed time is fixed until `resume()` is called.
 * - Internally uses microseconds (`esp_timer_get_time()`), but `elapsed()` returns milliseconds.
 *   This approach provides high resolution while offering millisecond granularity to the user.
 * - If you need platform-agnostic behavior, consider adapting the timing source (e.g. using `millis()`).
 *
 * @author Pierre
 * @date 2024
 */
#include <Arduino.h>
#include <climits>
#include "esp_timer.h"

class Timer {
private:
    uint64_t start;    
    uint64_t paused;   
    bool isPaused;     

    uint64_t delta(uint64_t current, uint64_t previous) {
        return (current >= previous) ? 
               current - previous : 
               (UINT64_MAX - previous) + current + 1;
    }

public:
    Timer() : start(esp_timer_get_time()), paused(0), isPaused(false) {}

    // Returns elapsed time in milliseconds
    unsigned long elapsed() {
        if (isPaused) {
            return delta(paused, start) / 1000;  // µs to ms
        }
        return delta(esp_timer_get_time(), start) / 1000;
    }

    // Tests if duration elapsed and auto-resets if true
    bool done(unsigned long duration) {
        if (check(duration)) {
            reset();
            return true;
        }
        return false;
    }

    // Tests if duration elapsed without reset
    bool check(unsigned long duration) {
        return elapsed() >= duration;
    }

    // Resets the timer
    void reset() {
        start = esp_timer_get_time();
        if (isPaused) {
            paused = start;
        }
    }

    // Pauses the timer
    void pause() {
        if (!isPaused) {
            paused = esp_timer_get_time();
            isPaused = true;
        }
    }

    // Resumes the timer
    void resume() {
        if (isPaused) {
            start += delta(esp_timer_get_time(), paused);
            isPaused = false;
        }
    }
};