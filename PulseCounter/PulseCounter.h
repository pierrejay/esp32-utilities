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
    static constexpr size_t MAX_COUNTERS = 8;  // Limite raisonnable pour ESP32

    PulseCounter(uint8_t pin, pcnt_unit_t pcntUnit = PCNT_UNIT_0, uint32_t sampleTimeMs = 100, uint16_t filterLength = 0)
        : inputPin(pin)
        , pcntUnit(pcntUnit)
        , sampleTimeMs(sampleTimeMs)
        , filterLength(filterLength)  // Durée du filtre en cycles APB_CLK
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
            .pos_mode = PCNT_COUNT_INC,   // Compte uniquement les fronts positifs
            .neg_mode = PCNT_COUNT_DIS,   // Désactive le comptage sur front négatif
            .counter_h_lim = 32767,       // Limite haute max pour int16_t
            .counter_l_lim = 0,           // On ne compte jamais en négatif
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
        
        return true;
    }

    static void startPolling(uint32_t intervalUs = 1000) {
        if (timerHandle != nullptr || counterCount == 0) return;

        esp_timer_create_args_t timerArgs = {
            .callback = &PulseCounter::globalPollCallback,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "pulse_poll_timer"
        };

        esp_timer_create(&timerArgs, &timerHandle);
        esp_timer_start_periodic(timerHandle, intervalUs);
    }

    static void stopPolling() {
        if (timerHandle != nullptr) {
            esp_timer_stop(timerHandle);
            esp_timer_delete(timerHandle);
            timerHandle = nullptr;
        }
    }

    void poll() {
        int16_t rawCount;
        pcnt_get_counter_value(pcntUnit, &rawCount);  // Use instance-specific unit
        uint16_t currentCount = (uint16_t)rawCount;

        uint16_t deltaPulses = (currentCount < lastCount)
            ? currentCount + (32767 - lastCount)
            : currentCount - lastCount;

        totalCount += deltaPulses;
        lastCount = currentCount;
        
        uint32_t currentTime = esp_timer_get_time() / 1000;  // Conversion µs -> ms
        uint32_t deltaTime = currentTime - lastFreqTime;
        
        if (deltaTime >= sampleTimeMs) {
            if(deltaPulses > 0) {
                lastPulseTime = currentTime;
                frequencyValid = true;
            } else if(currentTime - lastPulseTime > (2 * sampleTimeMs)) {
                frequencyValid = false;
                frequency = 0;
            }

            if(frequencyValid) {
                frequency = (float)(totalCount - lastCountForFreq) * 1000.0f / deltaTime;
            }
            
            lastCountForFreq = totalCount;
            lastFreqTime = currentTime;
        }

        if (currentCount >= 30000) {
            pcnt_counter_clear(pcntUnit);
            lastCount = 0;
        }
    }

    float getFrequency() const { 
        return frequencyValid ? frequency : 0.0f; 
    }

    uint32_t getCount() const { return totalCount; }  // Changed to unsigned
    
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
    }
    
    uint32_t getSampleTime() const {
        return sampleTimeMs;
    }

    // Used for tests
    void waitForFirstPulse() {
        int16_t lastValue;      // Types imposés par l'API PCNT
        int16_t currentValue;
        
        // Lire la valeur initiale
        pcnt_get_counter_value(pcntUnit, &lastValue);
        
        // Attendre un changement
        do {
            pcnt_get_counter_value(pcntUnit, &currentValue);
        } while (currentValue == lastValue);
    }

    // Used for tests
    uint32_t waitForPulses(uint32_t targetPulses) {
        // Désactiver le polling pendant le test pour éviter les interférences
        stopPolling();
        
        resetCount();
        waitForFirstPulse();
        resetCount();
        
        uint32_t startTime = esp_timer_get_time();
        while (totalCount < targetPulses) {
            poll();
        }
        return esp_timer_get_time() - startTime;
    }
    
private:
    static PulseCounter* counters[MAX_COUNTERS];
    static size_t counterCount;
    static portMUX_TYPE counterMutex;
    static esp_timer_handle_t timerHandle;

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
    }

    static void IRAM_ATTR globalPollCallback(void* arg) {
        portENTER_CRITICAL_ISR(&counterMutex);
        for (size_t i = 0; i < counterCount; ++i) {
            counters[i]->poll();
        }
        portEXIT_CRITICAL_ISR(&counterMutex);
    }

    const uint8_t inputPin;
    const pcnt_unit_t pcntUnit;  // Store PCNT unit per instance
    uint32_t sampleTimeMs;
    volatile uint16_t lastCount;
    volatile uint32_t totalCount;
    volatile float frequency;
    volatile uint32_t lastFreqTime;
    volatile uint32_t lastPulseTime;
    volatile bool frequencyValid;
    volatile uint32_t lastCountForFreq;
    const uint16_t filterLength;  // 0 = pas de filtre
    static const pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
};

// Initialisation des membres statiques
PulseCounter* PulseCounter::counters[MAX_COUNTERS] = {};
size_t PulseCounter::counterCount = 0;
portMUX_TYPE PulseCounter::counterMutex = portMUX_INITIALIZER_UNLOCKED;
esp_timer_handle_t PulseCounter::timerHandle = nullptr;

#endif // ESP32_PULSE_COUNTER_H