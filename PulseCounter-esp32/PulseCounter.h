// ESP32PulseLib.h
#ifndef ESP32_PULSE_COUNTER_H
#define ESP32_PULSE_COUNTER_H

#include <Arduino.h>
#include "driver/pcnt.h"

class PulseCounter {
public:
    PulseCounter(uint8_t pin, uint32_t sampleTimeMs = 100)
        : inputPin(pin)
        , sampleTimeMs(sampleTimeMs)
        , lastSampleTime(0)
        , lastCount(0)
        , lastFreqCount(0)
        , lastFreqTime(0)
        , totalCount(0)
        , frequency(0.0f)
        , lastPulseTime(0)
        , frequencyValid(false)
        , lastCountForFreq(0)  // Initialisation de lastCountForFreq
    {}
    
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
            .unit = PCNT_UNIT,
            .channel = PCNT_CHANNEL_0,
        };
        
        esp_err_t err = pcnt_unit_config(&config);
        if (err != ESP_OK) return false;
        
        pcnt_counter_pause(PCNT_UNIT);
        pcnt_counter_clear(PCNT_UNIT);
        pcnt_counter_resume(PCNT_UNIT);
        
        lastSampleTime = millis();
        return true;
    }
    
    void poll() {
        int16_t rawCount;  // Type imposé par l'API PCNT
        pcnt_get_counter_value(PCNT_UNIT, &rawCount);
        uint16_t currentCount = (uint16_t)rawCount;  // Conversion après lecture
        
        // Gestion du comptage total avec overflow (déplacé en premier)
        uint16_t deltaPulses;
        if (currentCount < lastCount) {
            deltaPulses = currentCount + (32767 - lastCount);
        } else {
            deltaPulses = currentCount - lastCount;
        }
        totalCount += deltaPulses;
        lastCount = currentCount;
        
        uint32_t currentTime = millis();
        uint32_t deltaTime = currentTime - lastFreqTime;
        
        if (deltaTime >= sampleTimeMs) {
            // Détection activité
            if(deltaPulses > 0) {  // Utilise deltaPulses au lieu de comparer currentCount
                lastPulseTime = currentTime;
                frequencyValid = true;
            } else if(currentTime - lastPulseTime > (2 * sampleTimeMs)) {
                frequencyValid = false;
                frequency = 0;
            }

            // Calcul fréquence si valide
            if(frequencyValid) {
                // Plus besoin de gérer l'overflow ici car déjà fait
                frequency = (float)(totalCount - lastCountForFreq) * 1000.0f / deltaTime;
            }
            
            lastCountForFreq = totalCount;  // Stocke le total plutôt que la valeur courante
            lastFreqTime = currentTime;
        }
        
        // Reset du compteur PCNT avant overflow
        if (currentCount > 30000) {
            pcnt_counter_clear(PCNT_UNIT);
            lastCount = 0;
        }
    }
    
    float getFrequency() const { 
        return frequencyValid ? frequency : 0.0f; 
    }

    uint32_t getCount() const { return totalCount; }  // Changed to unsigned
    
    void resetCount() {
        pcnt_counter_clear(PCNT_UNIT);
        lastCount = 0;
        lastFreqCount = 0;
        lastFreqTime = millis();
        totalCount = 0;
        frequency = 0.0f;
        lastCountForFreq = 0;  // Réinitialisation de lastCountForFreq
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
        pcnt_get_counter_value(PCNT_UNIT, &lastValue);
        
        // Attendre un changement
        do {
            pcnt_get_counter_value(PCNT_UNIT, &currentValue);
        } while (currentValue == lastValue);
    }

    // Used for tests
    uint32_t waitForPulses(uint32_t targetPulses) {
        resetCount();
        
        // Attendre le premier front montant et le jeter
        waitForFirstPulse();
        resetCount();
        
        uint32_t startTime = micros();
        while(getCount() < targetPulses) {
            poll();
        }
        
        uint32_t duration = micros() - startTime;
        return duration;
    }
    
private:
    static const pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
    const uint8_t inputPin;
    uint32_t sampleTimeMs;
    uint32_t lastSampleTime;
    uint16_t lastCount;        // Changed to unsigned
    uint16_t lastFreqCount;    // Changed to unsigned
    uint32_t lastFreqTime;
    uint32_t totalCount;       // Can count up to 4,294,967,295 pulses
    float frequency;
    uint32_t lastPulseTime;
    bool frequencyValid;
    uint32_t lastCountForFreq;  // Changé: utilise le total pour la fréquence
};

#endif // ESP32_PULSE_COUNTER_H