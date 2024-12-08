#pragma once

/**
 * @class Timer
 * @brief A simple, persistent and versatile timer class for embedded systems.
 *
 * This class provides a lightweight timer implementation with automatic persistence
 * and additional features like pause/resume capabilities. Each timer instance is 
 * independent and maintains its state between function calls without explicit static
 * declaration.
 *
 * ## Key Features
 * - **Independent Timers**: Each Timer instance operates independently
 * - **Automatic Persistence**: State persists between function calls
 * - **Simple API**: Single-word methods following KISS principle
 * - **Pause/Resume**: Ability to temporarily halt the timer
 * - **Overflow Protection**: Handles overflow correctly
 *
 * ## Usage Example
 * ```cpp
 * void loop() {
 *     Timer t1;          // Create timer on-the-fly (persistence is automatic)
 *     
 *     if(t1.done(1000)) {         // Check if 1000 ms elapsed
 *         Serial.println("Timer expired!");
 *     }
 *     
 *     Serial.println(t1.elapsed()); // Get elapsed time
 *     
 *     t1.pause();       // Pause the timer
 *     delay(500);       // Do something while paused
 *     t1.resume();      // Resume the timer
 * }
 * ```
 *
 * ## Methods
 * - **elapsed()**: Returns elapsed time since start/reset in milliseconds
 * - **done(duration)**: Returns true if specified duration has elapsed and resets
 * - **check(duration)**: Like done() but without auto-reset
 * - **reset()**: Manually resets the timer
 * - **pause()**: Temporarily stops the timer
 * - **resume()**: Resumes a paused timer
 *
 * ## Implementation Details
 * - Uses static variables for automatic persistence
 * - Handles overflow protection
 * - Paused state preserved between function calls
 * - Memory efficient - only uses static memory when needed
 *
 * ## Notes
 * - Always use unsigned long for durations to match timer
 * - First check() or done() call initializes the timer
 * - Paused timers maintain their elapsed time
 *
 * @author Pierre Jay
 * @date 2024
 */

#include <climits>
#include "esp_timer.h"

class Timer {
private:
    struct State {
        uint64_t start;    // Start timestamp
        uint64_t paused;   // Timestamp when paused
        bool isPaused;         // Pause state
        bool isFirstRun;       // First run state
        
        State() : start(0), paused(0), isPaused(false), isFirstRun(true) {}
    };

    State& getState() {
        static State state;
        return state;
    }

    State* state;  // Pointer to persistent state

    // Handles overflow
    uint64_t delta(uint64_t current, uint64_t previous) {
        return (current >= previous) ? 
               current - previous : 
               (UINT64_MAX - previous) + current + 1;
    }

public:
    Timer() : state(&getState()) {}

    // Returns elapsed time in milliseconds
    unsigned long elapsed() {
        if (state->isFirstRun) {
            state->start = esp_timer_get_time();
            state->isFirstRun = false;
            return 0;
        }

        if (state->isPaused) {
            return delta(state->paused, state->start) / 1000; // Convert µs to ms
        }

        return delta(esp_timer_get_time(), state->start) / 1000; // Convert µs to ms
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
        state->start = esp_timer_get_time();
        state->isFirstRun = false;
        if (state->isPaused) {
            state->paused = state->start;
        }
    }

    // Pauses the timer
    void pause() {
        if (!state->isPaused && !state->isFirstRun) {
            state->paused = esp_timer_get_time();
            state->isPaused = true;
        }
    }

    // Resumes the timer
    void resume() {
        if (state->isPaused) {
            state->start += delta(esp_timer_get_time(), state->paused);
            state->isPaused = false;
        }
    }
};