#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <functional>
#include <algorithm>


static constexpr size_t MAX_I2C_BUFFER_SIZE = 32;


struct WireOperationResult {
    enum Error {
        SUCCESS = 0,
        NACK_ADDRESS = 2,
        NACK_DATA = 3,
        BUS_ERROR = 4,
        TIMEOUT = 5
    } error = SUCCESS;
    
    size_t bytesTransferred = 0;
    std::vector<uint8_t> buffer;
    
    const uint8_t* data() const { return buffer.data(); }
    
    bool isOk() const { return error == SUCCESS; }
    String getErrorString() const {
        switch (error) {
            case SUCCESS: return "Success";
            case NACK_ADDRESS: return "Address NACK";
            case NACK_DATA: return "Data NACK";
            case BUS_ERROR: return "Bus Error";
            case TIMEOUT: return "Timeout";
            default: return "Unknown Error";
        }
    }
};

struct WireSequenceResult {
    std::vector<WireOperationResult> operations;
    
    WireOperationResult& final() { return operations.back(); }
    const WireOperationResult& final() const { return operations.back(); }
    
    bool isSuccessful() const {
        return std::all_of(operations.begin(), operations.end(),
            [](const WireOperationResult& r) { return r.isOk(); });
    }
    
    String getErrorDetails() const {
        for (size_t i = 0; i < operations.size(); i++) {
            if (!operations[i].isOk()) {
                return "Operation " + String(i) + " failed: " + 
                       operations[i].getErrorString();
            }
        }
        return "Success";
    }
};

class AsyncWire {
public:
    struct WireOperation {
        enum Type { BEGIN, WRITE, END, REQUEST } type;
        uint8_t address;
        const uint8_t* data;
        size_t length;
        bool sendStop;
    };

    using WireSequence = std::vector<WireOperation>;
    using WireCallback = std::function<void(const WireSequenceResult&)>;

    static AsyncWire& instance() {
        static AsyncWire instance;
        return instance;
    }

    static void configure(TwoWire& wire, int sda = -1, int scl = -1, uint32_t frequency = 100000) {
        auto& instance = AsyncWire::instance();
        instance.wire = &wire;
        if (sda != -1 && scl != -1) {
            wire.begin(sda, scl, frequency);
        } else {
            wire.begin();
        }
        instance.wireInitialized = true;
    }

    bool isInitialized() const {
        return wireInitialized;
    }

private:
    friend class WireSequenceBuilder;  // Ajout de cette ligne

    struct QueueItem {
        WireSequence sequence;
        WireCallback callback;
        WireSequenceResult result;
    };

    QueueHandle_t operationQueue;
    TwoWire* wire;  // Changé en pointeur
    TaskHandle_t wireTask;
    static constexpr size_t QUEUE_SIZE = 32;
    static constexpr size_t STACK_SIZE = 4096;
    bool wireInitialized;

    AsyncWire() : wire(&Wire), wireInitialized(false) {
        operationQueue = xQueueCreate(QUEUE_SIZE, sizeof(QueueItem));
        xTaskCreate(taskFunction, "Wire", STACK_SIZE, this, 3, &wireTask);
    }

    static void taskFunction(void* params) {
        AsyncWire* self = (AsyncWire*)params;
        self->processQueue();
    }

    void processQueue() {
        QueueItem item;
        while (true) {
            if (xQueueReceive(operationQueue, &item, portMAX_DELAY)) {
                bool success = true;
                for (size_t i = 0; i < item.sequence.size() && success; i++) {
                    item.result.operations[i] = executeOperation(item.sequence[i]);
                    success = item.result.operations[i].isOk();
                }
                
                if (item.callback) {
                    item.callback(item.result);
                }
            }
        }
    }

    static size_t readToBuffer(TwoWire& wire, std::vector<uint8_t>& buffer, size_t length) {
        length = std::min(length, MAX_I2C_BUFFER_SIZE);
        buffer.resize(length);
        
        size_t bytesRead = 0;
        for (size_t i = 0; i < length && wire.available(); i++) {
            buffer[i] = wire.read();
            bytesRead++;
        }
        
        return bytesRead;
    }

    WireOperationResult executeOperation(const WireOperation& op) {
        if (!wireInitialized) {
            WireOperationResult result;
            result.error = WireOperationResult::BUS_ERROR;
            return result;
        }

        WireOperationResult result;
        
        switch (op.type) {
            case WireOperation::BEGIN:
                wire->beginTransmission(op.address);
                break;
                
            case WireOperation::WRITE:
                result.bytesTransferred = wire->write(op.data, op.length);
                break;
                
            case WireOperation::END:
                result.error = (WireOperationResult::Error)wire->endTransmission(op.sendStop);
                break;
                
            case WireOperation::REQUEST:
                if (op.length > MAX_I2C_BUFFER_SIZE) {
                    result.error = WireOperationResult::BUS_ERROR;
                    return result;
                }
                
                result.bytesTransferred = wire->requestFrom(op.address, op.length, op.sendStop);
                if (result.bytesTransferred > 0) {
                    result.bytesTransferred = readToBuffer(*wire, result.buffer, result.bytesTransferred);
                }
                break;
        }
        
        return result;
    }

protected:
    void executeSequence(const WireSequence& sequence, WireCallback callback) {
        QueueItem item{
            .sequence = sequence,
            .callback = callback,
            .result = WireSequenceResult{std::vector<WireOperationResult>(sequence.size())}
        };
        xQueueSend(operationQueue, &item, portMAX_DELAY);
    }
};

// Interface bloquante compatible Wire
class BlockingWireProxy {
private:
    WireSequenceResult lastResult;
    SemaphoreHandle_t semaphore;

public:
    BlockingWireProxy() {
        semaphore = xSemaphoreCreateBinary();
    }
    
    void beginTransmission(uint8_t address) {
        WireSequenceBuilder()
            .beginTransmission(address)
            .execute([this](const WireSequenceResult& result) {
                lastResult = result;
                xSemaphoreGive(semaphore);
            });
        
        xSemaphoreTake(semaphore, portMAX_DELAY);
    }
    
    size_t write(const uint8_t* data, size_t length) {
        WireSequenceBuilder()
            .write(data, length)
            .execute([this](const WireSequenceResult& result) {
                lastResult = result;
                xSemaphoreGive(semaphore);
            });
        
        xSemaphoreTake(semaphore, portMAX_DELAY);
        return lastResult.final().bytesTransferred;
    }
    
    size_t write(uint8_t byte) {
        return write(&byte, 1);
    }
    
    uint8_t endTransmission(bool sendStop = true) {
        WireSequenceBuilder()
            .endTransmission(sendStop)
            .execute([this](const WireSequenceResult& result) {
                lastResult = result;
                xSemaphoreGive(semaphore);
            });
        
        xSemaphoreTake(semaphore, portMAX_DELAY);
        return static_cast<uint8_t>(lastResult.final().error);
    }
    
    size_t requestFrom(uint8_t address, size_t length, bool sendStop = true) {
        WireSequenceBuilder()
            .requestFrom(address, length, sendStop)
            .execute([this](const WireSequenceResult& result) {
                lastResult = result;
                xSemaphoreGive(semaphore);
            });
        
        xSemaphoreTake(semaphore, portMAX_DELAY);
        return lastResult.final().bytesTransferred;
    }
};

// Builder pour construire les séquences
class WireSequenceBuilder {
public:
    WireSequenceBuilder& beginTransmission(uint8_t address) {
        sequence.push_back({
            .type = AsyncWire::WireOperation::BEGIN,
            .address = address
        });
        return *this;
    }

    WireSequenceBuilder& write(const uint8_t* data, size_t length) {
        sequence.push_back({
            .type = AsyncWire::WireOperation::WRITE,
            .data = data,
            .length = length
        });
        return *this;
    }

    WireSequenceBuilder& write(uint8_t byte) {
        return write(&byte, 1);
    }

    WireSequenceBuilder& endTransmission(bool sendStop = true) {
        sequence.push_back({
            .type = AsyncWire::WireOperation::END,
            .sendStop = sendStop
        });
        return *this;
    }

    WireSequenceBuilder& requestFrom(uint8_t address, size_t length, bool sendStop = true) {
        sequence.push_back({
            .type = AsyncWire::WireOperation::REQUEST,
            .address = address,
            .length = length,
            .sendStop = sendStop
        });
        return *this;
    }

    // Helpers métier
    WireSequenceBuilder& writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
        return beginTransmission(address)
               .write(reg)
               .write(value)
               .endTransmission();
    }

    WireSequenceBuilder& readRegister(uint8_t address, uint8_t reg, size_t length = 1) {
        return beginTransmission(address)
               .write(reg)
               .endTransmission()
               .requestFrom(address, length);
    }

    void execute(AsyncWire::WireCallback callback) {
        AsyncWire::instance().executeSequence(sequence, callback);
    }

private:
    AsyncWire::WireSequence sequence;
};