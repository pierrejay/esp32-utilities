// ESP32PulseLib.h
#ifndef ESP32_PULSE_COUNTER_H
#define ESP32_PULSE_COUNTER_H

#include <Arduino.h>
#include "driver/pcnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class PulseCounter {
public:
    static constexpr size_t MAX_COUNTERS = 8;  // Limit for ESP32
    static constexpr uint32_t DEFAULT_MIN_SAMPLE_TIME_MS = 100;

    PulseCounter(uint8_t pin, pcnt_unit_t pcntUnit = PCNT_UNIT_0, uint32_t sampleTimeMs = 100, uint16_t filterLength = 0)
        : inputPin(pin)
        , pcntUnit(pcntUnit)
        , sampleTimeMs(sampleTimeMs)
        , filterLength(filterLength)  // Filter duration in APB_CLK cycles
        , lastCount(0)
        , lastFreqTime(0)
        , totalCount(0)
        , frequency(0.0f)
        , lastPulseTime(0)
        , frequencyValid(false)
        , lastCountForFreq(0)
    {}
    
    ~PulseCounter() {
        portENTER_CRITICAL(&counterMutex);
        removeCounter(this);
        if (counterCount == 0 && timerHandle != nullptr) {
            esp_timer_stop(timerHandle);
            esp_timer_delete(timerHandle);
            timerHandle = nullptr;
        }
        portEXIT_CRITICAL(&counterMutex);
    }

    bool begin() {
        pcnt_config_t config = {
            .pulse_gpio_num = inputPin,
            .ctrl_gpio_num = PCNT_PIN_NOT_USED,
            .lctrl_mode = PCNT_MODE_KEEP,
            .hctrl_mode = PCNT_MODE_KEEP,
            .pos_mode = PCNT_COUNT_INC,   // Count only positive fronts
            .neg_mode = PCNT_COUNT_DIS,   // Disable counting on negative fronts
            .counter_h_lim = 32767,       // Maximum high limit for int16_t
            .counter_l_lim = 0,           // Never count in negative
            .unit = pcntUnit,  // Use instance-specific unit
            .channel = PCNT_CHANNEL_0,
        };
        
        esp_err_t err = pcnt_unit_config(&config);
        if (err != ESP_OK) {
            log_e("Failed to configure PCNT unit %d", pcntUnit);
            return false;
        }

        if (filterLength > 0) {
            err = pcnt_set_filter_value(pcntUnit, filterLength);
            if (err != ESP_OK) return false;
            
            err = pcnt_filter_enable(pcntUnit);
            if (err != ESP_OK) return false;
        }
        
        pcnt_counter_pause(pcntUnit);
        pcnt_counter_clear(pcntUnit);
        pcnt_counter_resume(pcntUnit);

        portENTER_CRITICAL(&counterMutex);
        addCounter(this);
        portEXIT_CRITICAL(&counterMutex);

        updateMinSampleTime();
        
        return true;
    }

    static void startPolling(uint32_t intervalUs = 1000) {
        if (timerHandle != nullptr || counterCount == 0) return;

        // Create and start the pulse capture timer
        esp_timer_create_args_t pulseTimerArgs = {
            .callback = &PulseCounter::globalPollCallback,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "pulse_poll_timer"
        };

        esp_timer_create(&pulseTimerArgs, &timerHandle);
        esp_timer_start_periodic(timerHandle, intervalUs);

        // Create and start the frequency calculation task
        if (freqTaskHandle == nullptr) {
            xTaskCreate(
                FreqCalcTask,
                "FreqCalcTask",
                2048,
                nullptr,
                1,
                &freqTaskHandle
            );
        }
    }

    static void stopPolling() {
        if (timerHandle != nullptr) {
            esp_timer_stop(timerHandle);
            esp_timer_delete(timerHandle);
            timerHandle = nullptr;
        }
        if (freqTaskHandle != nullptr) {
            vTaskDelete(freqTaskHandle);
            freqTaskHandle = nullptr;
        }
    }

    float getFrequency() const { 
        return frequencyValid ? frequency : 0.0f; 
    }

    uint32_t getCount() const { return totalCount; }
    
    void resetCount() {
        portENTER_CRITICAL(&counterMutex);
        pcnt_counter_clear(pcntUnit);  // Use instance-specific unit
        lastCount = 0;
        totalCount = 0;
        frequency = 0.0f;
        lastFreqTime = esp_timer_get_time() / 1000;
        lastCountForFreq = 0;
        portEXIT_CRITICAL(&counterMutex);
    }
    
    void setSampleTime(uint32_t newSampleTimeMs) {
        sampleTimeMs = newSampleTimeMs;
        updateMinSampleTime();
    }
    
    uint32_t getSampleTime() const {
        return sampleTimeMs;
    }
    
private:
    static PulseCounter* counters[MAX_COUNTERS];
    static size_t counterCount;
    static portMUX_TYPE counterMutex;
    static esp_timer_handle_t timerHandle;
    static TaskHandle_t freqTaskHandle;
    static uint32_t minSampleTimeMs;
    volatile uint16_t lastCount;
    volatile uint32_t totalCount;
    volatile float frequency;
    volatile uint32_t lastFreqTime;
    volatile uint32_t lastPulseTime;
    volatile bool frequencyValid;
    volatile uint32_t lastCountForFreq;
    const uint8_t inputPin;
    const pcnt_unit_t pcntUnit;  // Store PCNT unit per instance
    uint32_t sampleTimeMs;
    const uint16_t filterLength;  // 0 = no filter


    static void addCounter(PulseCounter* counter) {
        if (counterCount < MAX_COUNTERS) {
            counters[counterCount++] = counter;
        } else {
            log_e("Failed to add counter: maximum limit reached");
        }
    }

    static void removeCounter(PulseCounter* counter) {
        portENTER_CRITICAL(&counterMutex);
        for (size_t i = 0; i < counterCount; ++i) {
            if (counters[i] == counter) {
                for (size_t j = i; j < counterCount - 1; ++j) {
                    counters[j] = counters[j + 1];
                }
                --counterCount;
                break;
            }
        }
        portEXIT_CRITICAL(&counterMutex);
        
        updateMinSampleTime();
    }

    inline void pollPulses() {
        int16_t rawCount;
        pcnt_get_counter_value(pcntUnit, &rawCount);
        uint16_t currentCount = (uint16_t)rawCount;

        uint16_t deltaPulses = (currentCount < lastCount)
            ? currentCount + (32767 - lastCount)
            : currentCount - lastCount;

        totalCount = totalCount + deltaPulses;
        lastCount = currentCount;

        if (currentCount >= 30000) {
            pcnt_counter_clear(pcntUnit);
            lastCount = 0;
        }
    }

    void updateFrequency() {
        uint32_t currentTime = esp_timer_get_time() / 1000;  // Conversion µs -> ms
        uint32_t deltaTime = currentTime - lastFreqTime;
        
        if (deltaTime >= sampleTimeMs) {
            uint32_t deltaPulses = totalCount - lastCountForFreq;
            
            if(deltaPulses > 0) {
                lastPulseTime = currentTime;
                frequencyValid = true;
            } else if(currentTime - lastPulseTime > (2 * sampleTimeMs)) {
                frequencyValid = false;
                frequency = 0;
            }

            if(frequencyValid) {
                frequency = (float)deltaPulses * 1000.0f / deltaTime;
            }
            
            lastCountForFreq = totalCount;
            lastFreqTime = currentTime;
        }
    }

    // Update the minimum sample time for all counters. 
    // Should be called when a new counter is added, removed or when the sample time is changed for any counter.
    static void updateMinSampleTime() {
        portENTER_CRITICAL(&counterMutex);
        uint32_t newMinSampleTime = UINT32_MAX;
        for (size_t i = 0; i < counterCount; ++i) {
            if (counters[i]->sampleTimeMs < newMinSampleTime) {
                newMinSampleTime = counters[i]->sampleTimeMs;
            }
        }
        minSampleTimeMs = (newMinSampleTime == UINT32_MAX) ? DEFAULT_MIN_SAMPLE_TIME_MS : newMinSampleTime;
        portEXIT_CRITICAL(&counterMutex);
    }

    static void IRAM_ATTR globalPollCallback(void* arg) {
        portENTER_CRITICAL_ISR(&counterMutex);
        for (size_t i = 0; i < counterCount; ++i) {
            counters[i]->pollPulses();
        }
        portEXIT_CRITICAL_ISR(&counterMutex);
    }

    static void FreqCalcTask(void* pvParameters) {
        while (true) {
            uint32_t currentTime = esp_timer_get_time() / 1000;  // Conversion µs -> ms

            portENTER_CRITICAL(&counterMutex);
            for (size_t i = 0; i < counterCount; ++i) {
                PulseCounter* counter = counters[i];
                uint32_t deltaTime = currentTime - counter->lastFreqTime;
                
                if (deltaTime >= counter->sampleTimeMs) {
                    uint32_t deltaPulses = counter->totalCount - counter->lastCountForFreq;
                    
                    if(deltaPulses > 0) {
                        counter->lastPulseTime = currentTime;
                        counter->frequencyValid = true;
                    } else if(currentTime - counter->lastPulseTime > (2 * counter->sampleTimeMs)) {
                        counter->frequencyValid = false;
                        counter->frequency = 0;
                    }

                    if(counter->frequencyValid) {
                        counter->frequency = (float)deltaPulses * 1000.0f / deltaTime;
                    }
                    
                    counter->lastCountForFreq = counter->totalCount;
                    counter->lastFreqTime = currentTime;
                }
            }
            portEXIT_CRITICAL(&counterMutex);
            
            vTaskDelay(pdMS_TO_TICKS(minSampleTimeMs / 2));
        }
    }

};

// Static member initialization
PulseCounter* PulseCounter::counters[MAX_COUNTERS] = {};
size_t PulseCounter::counterCount = 0;
portMUX_TYPE PulseCounter::counterMutex = portMUX_INITIALIZER_UNLOCKED;
esp_timer_handle_t PulseCounter::timerHandle = nullptr;
TaskHandle_t PulseCounter::freqTaskHandle = nullptr;
uint32_t PulseCounter::minSampleTimeMs = DEFAULT_MIN_SAMPLE_TIME_MS;

#endif // ESP32_PULSE_COUNTER_H