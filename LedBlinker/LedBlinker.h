#ifndef LED_BLINKER_H
#define LED_BLINKER_H

#include <Arduino.h>

class LedBlinker {
public:
    // Constructeur
    LedBlinker(uint8_t pin, bool activeHigh = false)
        : _pin(pin)
        , _activeHigh(activeHigh)
        , _isOn(false)
        , _blinkCount(0)
        , _freq(0)
        , _period(0)
        , _currentBlinks(0)
        , _lastToggle(0)
        , _sequenceStart(0)
        , _continuous(false)
        , _initialized(false)
    {
    }
    
    // Initialisation (à appeler dans setup())
    void begin() {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, _activeHigh ? LOW : HIGH);
        _initialized = true;
    }
    
    // Configuration des modes de clignotement
    void setMode(uint16_t freq) {
        if (!_initialized) return;
        _freq = freq;
        _blinkCount = 0;
        _period = 0;
        _continuous = true;
        _currentBlinks = 0;
        _lastToggle = millis();
        _sequenceStart = _lastToggle;
        update();
    }
    
    void setMode(uint8_t blinks, uint16_t freq) {
        if (!_initialized) return;
        _blinkCount = blinks;
        _freq = freq;
        _period = 0;
        _continuous = false;
        _currentBlinks = 0;
        _lastToggle = millis();
        _sequenceStart = _lastToggle;
        update();
    }
    
    void setMode(uint8_t blinks, uint16_t freq, unsigned long period) {
        if (!_initialized) return;
        _blinkCount = blinks;
        _freq = freq;
        _period = period;
        _continuous = false;
        _currentBlinks = 0;
        _lastToggle = millis();
        _sequenceStart = _lastToggle;
        update();
    }
    
    // Mise à jour de l'état de la LED
    void update() {
        if (!_initialized) return;
        
        unsigned long currentTime = millis();
        unsigned long halfPeriod = 500UL / _freq;  // Demi-période en ms
        
        // Gestion de la période de répétition
        if (_period > 0) {
            unsigned long elapsed = currentTime - _sequenceStart;
            if (elapsed >= _period) {
                _sequenceStart = currentTime;
                _currentBlinks = 0;
                _lastToggle = currentTime;
                _isOn = false;
                toggleLed();
                return;
            }
        }
        
        // Pas de clignotement si fréquence nulle
        if (_freq == 0) {
            return;
        }
        
        // Vérification du temps écoulé pour le changement d'état
        if (currentTime - _lastToggle >= halfPeriod) {
            // En mode séquentiel, on vérifie si on a atteint le nombre de clignotements
            if (!_continuous && _currentBlinks >= _blinkCount * 2) {
                if (_period == 0) {
                    digitalWrite(_pin, _activeHigh ? LOW : HIGH);
                    return;
                }
            } else {
                toggleLed();
                _lastToggle = currentTime;
                if (!_continuous) {
                    _currentBlinks++;
                }
            }
        }
    }

private:
    const uint8_t _pin;           // Broche de la LED
    const bool _activeHigh;       // true si LED active à HIGH, false si active à LOW
    bool _initialized;            // Flag d'initialisation
    
    bool _isOn;                   // État actuel de la LED
    uint8_t _blinkCount;          // Nombre de clignotements souhaités
    uint16_t _freq;              // Fréquence de clignotement en Hz
    unsigned long _period;        // Période de répétition en ms
    
    uint8_t _currentBlinks;       // Compteur de clignotements effectués
    unsigned long _lastToggle;    // Dernier changement d'état
    unsigned long _sequenceStart; // Début de la séquence actuelle
    
    bool _continuous;             // Mode continu (true) ou séquence (false)
    
    void toggleLed() {
        _isOn = !_isOn;
        digitalWrite(_pin, _isOn ? (_activeHigh ? HIGH : LOW) : (_activeHigh ? LOW : HIGH));
    }
};

#endif
