#ifndef ASYNCSERIAL_H
#define ASYNCSERIAL_H

#include <Arduino.h>
#include <vector>
#include <atomic>
#include <functional>

using Bytes = std::vector<uint8_t>;
static constexpr size_t MAX_PROXIES = 8;

// ============================== RingBuffer ==============================
template<typename T, size_t SIZE>
class RingBuffer {
public:
    RingBuffer() : _readIndex(0), _writeIndex(0), _count(0) {}

    bool write(const T& data) {
        if (_count >= SIZE) return false;
        _buffer[_writeIndex] = data;
        _writeIndex = (_writeIndex + 1) % SIZE;
        _count++;
        return true;
    }

    bool write(const T* data, size_t length) {
        if (_count + length > SIZE) return false;
        for (size_t i = 0; i < length; i++) {
            _buffer[_writeIndex] = data[i];
            _writeIndex = (_writeIndex + 1) % SIZE;
        }
        _count += length;
        return true;
    }

    bool read(T& data) {
        if (_count == 0) return false;
        data = _buffer[_readIndex];
        _readIndex = (_readIndex + 1) % SIZE;
        _count--;
        return true;
    }

    bool peek(T& data) const {
        if (_count == 0) return false;
        data = _buffer[_readIndex];
        return true;
    }

    size_t available() const { return _count; }
    void clear() { _readIndex = _writeIndex = _count = 0; }

private:
    T _buffer[SIZE];
    size_t _readIndex;
    size_t _writeIndex;
    size_t _count;
};

// ============================== SerialProxyBase ==============================
class SerialProxyBase : public Stream {
public:
    virtual ~SerialProxyBase() = default;
    virtual uint32_t getInterMessageDelay() const = 0;
    virtual bool pushToRx(uint8_t data) = 0;
    virtual bool readFromTx(uint8_t& data) = 0;
    virtual size_t txAvailable() const = 0;
};

// ============================== SerialProxy ==============================
template<size_t BUFFER_SIZE = 1024>
class SerialProxy : public SerialProxyBase {
public:
    explicit SerialProxy(uint32_t interMessageDelay = 5)
        : _rxBuffer(), _txBuffer(), _interMessageDelay(interMessageDelay) {}

    uint32_t getInterMessageDelay() const override { return _interMessageDelay; }
    bool pushToRx(uint8_t data) override { return _rxBuffer.write(data); }
    bool readFromTx(uint8_t& data) override { return _txBuffer.read(data); }
    size_t txAvailable() const override { return _txBuffer.available(); }

    int available() override { return _rxBuffer.available(); }
    int read() override {
        uint8_t data;
        return _rxBuffer.read(data) ? data : -1;
    }
    int peek() override {
        uint8_t data;
        return _rxBuffer.peek(data) ? data : -1;
    }
    size_t write(uint8_t data) override { return _txBuffer.write(data) ? 1 : 0; }

    bool write(const Bytes& data) { return _txBuffer.write(data.data(), data.size()); }
    bool write(const String& data) { return _txBuffer.write((const uint8_t*)data.c_str(), data.length()); }

    void flush() override { AsyncSerial::getInstance().flush(this); }

    // Méthodes de configuration
    void begin(unsigned long baud) {
        // Ne fait rien, l'initialisation doit être faite via AsyncSerial::getInstance().begin()
    }
    void begin(unsigned long baud, uint16_t config) {
        // Ne fait rien, l'initialisation doit être faite via AsyncSerial::getInstance().begin()
    }
    void end() {
        // Ne fait rien, la terminaison doit être faite via AsyncSerial::getInstance().end()
    }

    // Méthodes d'écriture supplémentaires pour la compatibilité
    size_t write(const uint8_t *buffer, size_t size) override { 
        return _txBuffer.write(buffer, size) ? size : 0; 
    }

    // Méthodes de configuration du timeout
    void setTimeout(unsigned long timeout) { _timeout = timeout; }
    unsigned long getTimeout() const { return _timeout; }

    // Méthode manquante pour la compatibilité avec Stream
    int availableForWrite() override {
        return BUFFER_SIZE - _txBuffer.available();
    }

    // Méthodes de lecture en bloc avec timeout
    size_t readBytes(uint8_t *buffer, size_t length) {
        size_t count = 0;
        unsigned long startMillis = millis();
        
        while (count < length) {
            if (millis() - startMillis > _timeout) break;
            
            if (available()) {
                buffer[count++] = read();
                startMillis = millis(); // Reset timeout on successful read
            }
            AsyncSerial::getInstance().poll();
        }
        return count;
    }

    size_t readBytesUntil(char terminator, uint8_t *buffer, size_t length) {
        if (length < 1) return 0;
        
        size_t count = 0;
        unsigned long startMillis = millis();
        
        while (count < length - 1) {
            if (millis() - startMillis > _timeout) break;
            
            if (available()) {
                uint8_t c = read();
                if (c == terminator) break;
                buffer[count++] = c;
                startMillis = millis();
            }
            AsyncSerial::getInstance().poll();
        }
        buffer[count] = 0; // Null-terminate
        return count;
    }

    String readString() {
        String ret;
        unsigned long startMillis = millis();
        
        while (true) {
            if (millis() - startMillis > _timeout) break;
            
            if (available()) {
                ret += (char)read();
                startMillis = millis();
            }
            AsyncSerial::getInstance().poll();
        }
        return ret;
    }

    String readStringUntil(char terminator) {
        String ret;
        unsigned long startMillis = millis();
        
        while (true) {
            if (millis() - startMillis > _timeout) break;
            
            if (available()) {
                char c = read();
                if (c == terminator) break;
                ret += c;
                startMillis = millis();
            }
            AsyncSerial::getInstance().poll();
        }
        return ret;
    }

    // Méthodes inutiles mais implémentées pour la compatibilité
    bool find(char *target) { return false; }
    bool find(uint8_t *target, size_t length) { return false; }
    bool findUntil(char *target, char *terminator) { return false; }
    bool findUntil(uint8_t *target, size_t targetLen, char *terminator, size_t termLen) { return false; }
    float parseFloat() { return 0.0f; }
    long parseInt() { return 0L; }
    long parseInt(char skipChar) { return 0L; }

private:
    RingBuffer<uint8_t, BUFFER_SIZE> _rxBuffer;
    RingBuffer<uint8_t, BUFFER_SIZE> _txBuffer;
    const uint32_t _interMessageDelay;
    unsigned long _timeout = 1000; // Timeout par défaut de 1 seconde
};

// ============================== CooperativeLock ==============================
template<typename T>
class CooperativeLock {
public:
    using PollCallback = std::function<void()>;

    CooperativeLock() : _owner(nullptr) {}

    bool acquire(T* owner, PollCallback poll) {
        T* expected = nullptr;
        while (!_owner.compare_exchange_weak(expected, owner)) {
            expected = nullptr;
            poll();
        }
        return true;
    }

    void release(T* owner) {
        T* expected = owner;
        _owner.compare_exchange_strong(expected, nullptr);
    }

    bool isOwnedBy(T* owner) const { return _owner.load() == owner; }
    T* getOwner() const { return _owner.load(); }

private:
    std::atomic<T*> _owner;
};

// ============================== AsyncSerial ==============================
/**
 * @class AsyncSerial
 * @brief Singleton class managing asynchronous access to the hardware serial port.
 * 
 * This class implements the Singleton pattern to ensure only one instance
 * manages the hardware serial port. Access it via AsyncSerial::getInstance().
 * 
 * Thread-safety notes:
 * - getInstance(): Thread-safe (C++11 static initialization)
 * - flush(): Thread-safe (protected by CooperativeLock)
 * - poll(): Thread-safe when used as intended
 * - begin()/end(): Not thread-safe, should be called once at startup/shutdown
 * - registerProxy(): Not thread-safe, should be called once per proxy at initialization
 */
class AsyncSerial {
public:
    static AsyncSerial& getInstance() {
        static AsyncSerial instance;
        return instance;
    }

    bool registerProxy(SerialProxyBase* proxy) {
        if (_proxyCount >= MAX_PROXIES) return false;
        _proxies[_proxyCount++] = {proxy, 0, false};
        return true;
    }

   bool flush(SerialProxyBase* proxy) {
        // Prendre immédiatement le lock
        _flushLock.acquire(proxy, [this]() { poll(); });

        // Attendre que l'état WRITE termine
        while (_state == State::WRITE || _state == State::READ) {
            poll();  // Continue de traiter l'état WRITE/READ jusqu'à ce qu'il se termine
        }

        // Passe au mode FLUSH
        _state = State::FLUSH;

        // Bloque jusqu'à ce que le mode FLUSH soit terminé ou un timeout survienne
        unsigned long startTime = millis();
        while (_state == State::FLUSH) {
            poll();
            if (millis() - startTime >= SERIAL_TIMEOUT) {
                _flushLock.release(proxy);
                return false;  // Indique l'échec du flush
            }
        }

        // Libérer le lock après le flush
        _flushLock.release(proxy);
        return true;  // Flush réussi
    }

    void poll() {
        unsigned long now = millis();

        switch (_state) {
            case State::IDLE:
                // Transition vers READ si des données série sont disponibles
                if (Serial.available()) {
                    _state = State::READ;
                }
                // Transition vers WRITE si un proxy a des données prêtes
                else {
                    for (size_t i = 0; i < _proxyCount; i++) {
                        if (_proxies[i].proxy->txAvailable() > 0) {
                            _proxies[i].isActive = true;  // Active ce proxy
                            _state = State::WRITE;
                            break;
                        }
                    }
                    if (_state == State::IDLE) delay(1);  // Libère le CPU si aucun proxy actif
                }
                break;

            case State::READ: {
                size_t bytesRead = 0;
                while (Serial.available() && bytesRead < RX_CHUNK_SIZE) {
                    uint8_t data = Serial.read();
                    for (size_t i = 0; i < _proxyCount; i++) {
                        _proxies[i].proxy->pushToRx(data);
                    }
                    bytesRead++;
                }
                _state = Serial.available() ? State::READ : State::IDLE;
                break;
            }

            case State::WRITE:
                for (size_t i = 0; i < _proxyCount; i++) {
                    auto& state = _proxies[i];
                    if (!state.isActive) continue;

                    // Respecte le délai inter-message
                    if (millis() - state.lastTxTime < state.proxy->getInterMessageDelay()) {
                        break;  // Attends avant d'envoyer un autre chunk
                    }

                    // Envoie un chunk
                    sendChunk(state.proxy);

                    // Met à jour le délai inter-message après l'envoi
                    state.lastTxTime = millis();

                    // Si le buffer est vide, désactive le proxy
                    if (state.proxy->txAvailable() == 0) {
                        state.isActive = false;
                        _state = State::IDLE;  // Reviens à IDLE une fois terminé
                        return;
                    }
                    break;  // Un seul proxy actif à la fois
                }
                break;


            case State::FLUSH: {
                SerialProxyBase* proxy = _flushLock.getOwner();
                unsigned long startTime = millis();

                Serial.flush();  // S'assure que le buffer série est vide avant de commencer

                // Vider le buffer du proxy chunk par chunk
                while (proxy->txAvailable() > 0) {
                    sendChunk(proxy);

                    // Vérifie si le timeout est atteint
                    if (millis() - startTime >= SERIAL_TIMEOUT) {
                        break;
                    }
                }

                Serial.flush();  // S'assure que le dernier chunk est bien transmis

                // Vérifie si le buffer du proxy est vide
                if (proxy->txAvailable() == 0) {
                    _state = State::IDLE;
                } 
                break;
            }


        }
    }

    void begin(unsigned long baud) {
        Serial.begin(baud);
    }

    void end() {
        Serial.end();
    }

private:
    AsyncSerial() = default;

    static constexpr size_t RX_CHUNK_SIZE = 256;
    static constexpr unsigned long SERIAL_TIMEOUT = 1000;

    enum class State { IDLE, READ, WRITE, FLUSH };

    struct ProxyState {
        SerialProxyBase* proxy;
        unsigned long lastTxTime;
        bool isActive;  // Indique si le proxy est actuellement actif
    };

    State _state = State::IDLE;
    CooperativeLock<SerialProxyBase> _flushLock;
    ProxyState _proxies[MAX_PROXIES];
    size_t _proxyCount = 0;

    void sendChunk(SerialProxyBase* proxy) {
        size_t availableSpace = Serial.availableForWrite();
        if (availableSpace == 0) return;  // Rien à envoyer si le buffer série est plein

        size_t toSend = min(availableSpace, proxy->txAvailable());

        uint8_t chunk[toSend];
        size_t bytesRead = 0;

        while (bytesRead < toSend) {
            if (!proxy->readFromTx(chunk[bytesRead])) break;
            bytesRead++;
        }

        if (bytesRead > 0) {
            Serial.write(chunk, bytesRead);
        }
    }

};

#endif // ASYNCSERIAL_H