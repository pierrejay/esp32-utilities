#ifndef ASYNCSERIAL_H
#define ASYNCSERIAL_H

#include <Arduino.h>
#include <vector>
#include <atomic>
#include <functional>
#include <type_traits>

using Bytes = std::vector<uint8_t>;
static constexpr size_t MAX_PROXIES = 8;

// ============================== RingBuffer ==============================
template<typename T, size_t SIZE>
class RingBuffer {
public:
    RingBuffer() : _readIndex(0), _writeIndex(0), _count(0) {}

    /**
     * @brief Write a single element to the buffer
     * @param data The element to write
     * @return True if the element was successfully written, false if the buffer is full
     */
    bool write(const T& data) {
        if (_count >= SIZE) return false;
        _buffer[_writeIndex] = data;
        _writeIndex = (_writeIndex + 1) % SIZE;
        _count++;
        return true;
    }

    /**
     * @brief Write multiple elements to the buffer
     * @param data Pointer to the data array
     * @param length Number of elements to write
     * @return True if all elements were successfully written, false if the buffer does not have enough space
     */
    bool write(const T* data, size_t length) {
        if (_count + length > SIZE) return false;
        for (size_t i = 0; i < length; i++) {
            _buffer[_writeIndex] = data[i];
            _writeIndex = (_writeIndex + 1) % SIZE;
        }
        _count += length;
        return true;
    }

    /**
     * @brief Read a single element from the buffer
     * @param data Reference to store the read element
     * @return True if an element was successfully read, false if the buffer is empty
     */
    bool read(T& data) {
        if (_count == 0) return false;
        data = _buffer[_readIndex];
        _readIndex = (_readIndex + 1) % SIZE;
        _count--;
        return true;
    }

    /**
     * @brief Peek at the next element in the buffer without removing it
     * @param data Reference to store the peeked element
     * @return True if an element was successfully peeked, false if the buffer is empty
     */
    bool peek(T& data) const {
        if (_count == 0) return false;
        data = _buffer[_readIndex];
        return true;
    }

    /**
     * @brief Get the number of elements currently in the buffer
     * @return The number of elements in the buffer
     */
    size_t available() const { return _count; }

    /**
     * @brief Clear the buffer
     */
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

    // New method to associate with an AsyncSerial instance
    void setAsyncSerial(AsyncSerial* async) { _async = async; }

protected:
    AsyncSerial* _async = nullptr;
};

// ============================== SerialProxy ==============================
template<size_t BUFFER_SIZE = 1024>
class SerialProxy : public SerialProxyBase {
public:
    explicit SerialProxy(uint32_t interMessageDelay = 5)
        : _rxBuffer(), _txBuffer(), _interMessageDelay(interMessageDelay) {}

    /**
     * @brief Get the inter-message delay
     * @return The inter-message delay in milliseconds
     */
    uint32_t getInterMessageDelay() const override { return _interMessageDelay; }

    /**
     * @brief Push data to the receive buffer
     * @param data The data to push
     * @return True if the data was successfully pushed, false if the buffer is full
     */
    bool pushToRx(uint8_t data) override { return _rxBuffer.write(data); }

    /**
     * @brief Read data from the transmit buffer
     * @param data Reference to store the read data
     * @return True if data was successfully read, false if the buffer is empty
     */
    bool readFromTx(uint8_t& data) override { return _txBuffer.read(data); }

    /**
     * @brief Get the number of bytes available for writing
     * @return The number of bytes available for writing
     */
    size_t txAvailable() const override { return _txBuffer.available(); }

    /**
     * @brief Flush the transmit buffer
     */
    void flush() override {
        if (_async) _async->flush(this);
    }

    // Stream methods for drop-in compatibility
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
    size_t write(const uint8_t *buffer, size_t size) override { return _txBuffer.write(buffer, size) ? size : 0; }
    void setTimeout(unsigned long timeout) { _timeout = timeout; }
    unsigned long getTimeout() const { return _timeout; }
    int availableForWrite() override {
        return BUFFER_SIZE - _txBuffer.available();
    }

    // Unused methods but implemented for compatibility
    bool find(char *target) { return false; }
    bool find(uint8_t *target, size_t length) { return false; }
    bool findUntil(char *target, char *terminator) { return false; }
    bool findUntil(uint8_t *target, size_t targetLen, char *terminator, size_t termLen) { return false; }
    float parseFloat() { return 0.0f; }
    long parseInt() { return 0L; }
    long parseInt(char skipChar) { return 0L; }

    // Configuration methods
    void begin(unsigned long baud) {
        // Do nothing, initialization must be done on the AsyncSerial instance before registering this proxy
    }
    void begin(unsigned long baud, uint16_t config) {
        // Do nothing, initialization must be done on the AsyncSerial instance before registering this proxy
    }
    void end() {
        // Do nothing, termination must be done on the AsyncSerial instance
    }

    // Extended methods to allow bulk (Bytes or String) reading/writing

    /**
     * @brief Read bytes from the serial port with a timeout
     * @param buffer Pointer to the buffer to store the read bytes
     * @param length Maximum number of bytes to read
     * @return Number of bytes actually read
     */
    size_t readBytes(uint8_t *buffer, size_t length) {
        size_t count = 0;
        unsigned long startMillis = millis();
        
        while (count < length) {
            if (millis() - startMillis > _timeout) break;
            
            if (available()) {
                buffer[count++] = read();
                startMillis = millis();
            }
            if (_async) _async->poll();
        }
        return count;
    }

    /**
     * @brief Read bytes from the serial port until a terminator is encountered
     * @param terminator The character to look for
     * @param buffer Pointer to the buffer to store the read bytes
     * @param length Maximum number of bytes to read
     * @return Number of bytes actually read
     */
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
            if (_async) _async->poll();
        }
        buffer[count] = 0; // Null-terminate
        return count;
    }

    /**
     * @brief Read a string from the serial port until a timeout occurs
     * @return The string read from the serial port
     */
    String readString() {
        String ret;
        unsigned long startMillis = millis();
        
        while (true) {
            if (millis() - startMillis > _timeout) break;
            
            if (available()) {
                ret += (char)read();
                startMillis = millis();
            }
            if (_async) _async->poll();
        }
        return ret;
    }

    /**
     * @brief Read a string from the serial port until a terminator is encountered
     * @param terminator The character to look for
     * @return The string read from the serial port
     */
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
            if (_async) _async->poll();
        }
        return ret;
    }

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

    /**
     * @brief Acquire the lock
     * @param owner The owner of the lock
     * @param poll The poll callback to call while waiting for the lock
     * @return True if the lock was acquired, false if it was already owned
     */
    bool acquire(T* owner, PollCallback poll) {
        T* expected = nullptr;
        while (!_owner.compare_exchange_weak(expected, owner)) {
            expected = nullptr;
            poll();
        }
        return true;
    }

    /**
     * @brief Release the lock
     * @param owner The owner of the lock
     */
    void release(T* owner) {
        T* expected = owner;
        _owner.compare_exchange_strong(expected, nullptr);
    }

    /**
     * @brief Check if the lock is owned by a specific owner
     * @param owner The owner to check
     * @return True if the lock is owned by the specified owner, false otherwise
     */
    bool isOwnedBy(T* owner) const { return _owner.load() == owner; }

    /**
     * @brief Get the owner of the lock
     * @return The owner of the lock
     */
    T* getOwner() const { return _owner.load(); }

private:
    std::atomic<T*> _owner;
};

// ============================== AsyncSerial ==============================
/**
 * @class AsyncSerial
 * @brief Class managing asynchronous access to a serial port.
 * 
 * This class provides asynchronous access to a serial port (USB CDC or UART)
 * with multiple proxies support.
 * 
 * Usage:
 * 1. Create an AsyncSerial instance for your port
 * 2. Call begin() on the instance
 * 3. Create and register proxies using registerProxy()
 * 4. Call poll() regularly in your loop
 * 
 * Thread-safety notes:
 * - flush(): Thread-safe (protected by CooperativeLock)
 * - poll(): Thread-safe when used as intended
 * - begin()/end(): Not thread-safe, should be called once at startup/shutdown
 * - registerProxy(): Not thread-safe, should be called once per proxy
 */
class AsyncSerial {
public:
    template<typename T>
    explicit AsyncSerial(T& serial) : _port(serial) {
        static_assert(std::is_base_of_v<Stream, T>, 
                     "Serial port type must derive from Stream");
    }

    /**
     * @brief Register a proxy to the serial port
     * @param proxy The proxy to register
     * @return True if the proxy was successfully registered, false if the maximum number of proxies is reached
     */
    bool registerProxy(SerialProxyBase* proxy) {
        if (_proxyCount >= MAX_PROXIES) return false;
        proxy->setAsyncSerial(this);  // Set this instance as the proxy's AsyncSerial
        _proxies[_proxyCount++] = {proxy, 0, false};
        return true;
    }

    /**
     * @brief Flush the serial port
     * @param proxy The proxy to flush
     * @return True if the flush was successful, false if a timeout occurred
     */
    bool flush(SerialProxyBase* proxy) {
        // Immediately acquire the lock
        _flushLock.acquire(proxy, [this]() { poll(); });

        // Wait for the WRITE state to finish
        while (_state == State::WRITE || _state == State::READ) {
            poll();  // Continue de traiter l'état WRITE/READ jusqu'à ce qu'il se termine
        }

        // Switch to FLUSH mode
        _state = State::FLUSH;

        // Block until FLUSH mode is finished or a timeout occurs
        unsigned long startTime = millis();
        while (_state == State::FLUSH) {
            poll();
            if (millis() - startTime >= SERIAL_TIMEOUT) {
                _flushLock.release(proxy);
                return false;  // Indique l'échec du flush
            }
        }

        // Release the lock after flushing
        _flushLock.release(proxy);
        return true;  // Flush réussi
    }

    /**
     * @brief Process the AsyncSerial state machine to handle reading and writing operations
     */
    void poll() {
        unsigned long now = millis();

        switch (_state) {
            case State::IDLE:
                // Transition to READ if serial data is available
                if (_port.available()) {
                    _state = State::READ;
                }
                // Transition to WRITE if a proxy has data ready
                else {
                    for (size_t i = 0; i < _proxyCount; i++) {
                        if (_proxies[i].proxy->txAvailable() > 0) {
                            _proxies[i].isActive = true;  // Indicate that this proxy is currently active
                            _state = State::WRITE;
                            break;
                        }
                    }
                    if (_state == State::IDLE) delay(1);  // Free the CPU if no active proxy
                }
                break;

            case State::READ: {
                size_t bytesRead = 0;
                while (_port.available() && bytesRead < RX_CHUNK_SIZE) {
                    uint8_t data = _port.read();
                    for (size_t i = 0; i < _proxyCount; i++) {
                        _proxies[i].proxy->pushToRx(data);
                    }
                    bytesRead++;
                }
                _state = _port.available() ? State::READ : State::IDLE;
                break;
            }

            case State::WRITE:
                for (size_t i = 0; i < _proxyCount; i++) {
                    auto& state = _proxies[i];
                    if (!state.isActive) continue;

                    // Respect the inter-message delay
                    if (millis() - state.lastTxTime < state.proxy->getInterMessageDelay()) {
                        break;  // Wait before sending another chunk
                    }

                    // Send a chunk
                    sendChunk(state.proxy);

                    // Update the inter-message delay after sending
                    state.lastTxTime = millis();

                    // If the buffer is empty, deactivate the proxy
                    if (state.proxy->txAvailable() == 0) {
                        state.isActive = false;
                        _state = State::IDLE;  // Return to IDLE once finished
                        return;
                    }
                    break;  // Only one proxy active at a time
                }
                break;


            case State::FLUSH: {
                SerialProxyBase* proxy = _flushLock.getOwner();
                unsigned long startTime = millis();

                _port.flush();

                // Empty the proxy buffer chunk by chunk
                while (proxy->txAvailable() > 0) {
                    sendChunk(proxy);

                    if (millis() - startTime >= SERIAL_TIMEOUT) {
                        break;
                    }
                }

                _port.flush();

                if (proxy->txAvailable() == 0) {
                    _state = State::IDLE;
                } 
                break;
            }


        }
    }

    /**
     * @brief Initialize the serial port
     * @param baud The baud rate to use
     */
    template<typename... Args>
    void begin(Args... args) {
        beginImpl(std::forward<Args>(args)...);
    }

    /**
     * @brief Terminate the serial port
     */
    void end() {
        _port.end();
    }

private:
    class SerialPort {
    public:
        template<typename T>
        SerialPort(T& serial) : _port(&serial), _type(getPortType<T>()) {
            static_assert(std::is_base_of_v<Stream, T>, 
                         "Serial port type must derive from Stream");
        }

        // Méthodes spécifiques qui diffèrent selon le type
        void begin(unsigned long baud) {
            if (_type == PortType::CDC) {
                static_cast<HWCDC*>(_port)->begin();
            } else {
                static_cast<HardwareSerial*>(_port)->begin(baud);
            }
        }

        void begin(unsigned long baud, uint32_t config) {
            if (_type == PortType::CDC) {
                static_cast<HWCDC*>(_port)->begin();
            } else {
                static_cast<HardwareSerial*>(_port)->begin(baud, config);
            }
        }

        void begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin) {
            if (_type == PortType::CDC) {
                static_cast<HWCDC*>(_port)->begin();
            } else {
                static_cast<HardwareSerial*>(_port)->begin(baud, config, rxPin, txPin);
            }
        }

        void end() {
            if (_type == PortType::CDC) {
                static_cast<HWCDC*>(_port)->end();
            } else {
                static_cast<HardwareSerial*>(_port)->end();
            }
        }

        // Méthodes communes venant de Stream
        int available() { return _port->available(); }
        int read() { return _port->read(); }
        size_t write(uint8_t data) { return _port->write(data); }
        size_t write(const uint8_t* buffer, size_t size) { return _port->write(buffer, size); }
        int availableForWrite() { return _port->availableForWrite(); }
        void flush() { return _port->flush(); }

    private:
        enum class PortType { CDC, HARDWARE };
        Stream* _port;  // Base commune pour les méthodes de Stream
        PortType _type; // Pour différencier CDC/Hardware pour les méthodes spécifiques

        template<typename T>
        static PortType getPortType() {
            if constexpr (std::is_same_v<T, HWCDC>) {
                return PortType::CDC;
            } else {
                return PortType::HARDWARE;
            }
        }
    };

    SerialPort _port;

    static constexpr size_t RX_CHUNK_SIZE = 256;
    static constexpr unsigned long SERIAL_TIMEOUT = 1000;

    enum class State { IDLE, READ, WRITE, FLUSH };

    /**
     * @brief Structure to hold the state of a proxy
     */
    struct ProxyState {
        SerialProxyBase* proxy;
        unsigned long lastTxTime;
        bool isActive;  // Indicate that this proxy is currently active
    };

    State _state = State::IDLE;
    CooperativeLock<SerialProxyBase> _flushLock;
    ProxyState _proxies[MAX_PROXIES];
    size_t _proxyCount = 0;

    /**
     * @brief Send a chunk of data to the serial port
     * @param proxy The proxy to send the data from
     */
    void sendChunk(SerialProxyBase* proxy) {
        size_t availableSpace = _port.availableForWrite();
        if (availableSpace == 0) return;

        size_t toSend = min(availableSpace, proxy->txAvailable());

        uint8_t chunk[toSend];
        size_t bytesRead = 0;

        while (bytesRead < toSend) {
            if (!proxy->readFromTx(chunk[bytesRead])) break;
            bytesRead++;
        }

        if (bytesRead > 0) {
            _port.write(chunk, bytesRead);
        }
    }

    // Implémentations de begin selon les arguments
    void beginImpl(unsigned long baud) {
        _port.begin(baud);
    }

    void beginImpl(unsigned long baud, uint32_t config) {
        _port.begin(baud, config);
    }

    void beginImpl(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin) {
        _port.begin(baud, config, rxPin, txPin);
    }

};

#endif // ASYNCSERIAL_H
