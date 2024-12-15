#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <functional>
#include <algorithm>

// Ajout après les includes
static constexpr size_t MAX_I2C_BUFFER_SIZE = 32;

// Résultats
struct WireResult {
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

struct SequenceResult {
    std::vector<WireResult> operations;
    
    WireResult& final() { return operations.back(); }
    const WireResult& final() const { return operations.back(); }
    
    bool isSuccessful() const {
        return std::all_of(operations.begin(), operations.end(),
            [](const WireResult& r) { return r.isOk(); });
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

// Gestionnaire centralisé
class AsyncWire {
public:
    struct Operation {
        enum Type { BEGIN, WRITE, END, REQUEST } type;
        uint8_t address;
        const uint8_t* data;
        size_t length;
        bool sendStop;
    };

    using Sequence = std::vector<Operation>;
    using CompletionCallback = std::function<void(const SequenceResult&)>;

    static AsyncWire& instance() {
        static AsyncWire instance;
        return instance;
    }

    void executeSequence(const Sequence& sequence, CompletionCallback callback) {
        QueueItem item{
            .sequence = sequence,
            .callback = callback,
            .result = SequenceResult{std::vector<WireResult>(sequence.size())}
        };
        xQueueSend(operationQueue, &item, portMAX_DELAY);
    }

private:
    struct QueueItem {
        Sequence sequence;
        CompletionCallback callback;
        SequenceResult result;
    };

    QueueHandle_t operationQueue;
    TwoWire& wire;
    TaskHandle_t wireTask;
    static constexpr size_t QUEUE_SIZE = 32;
    static constexpr size_t STACK_SIZE = 4096;

    AsyncWire(TwoWire& w = Wire) : wire(w) {
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

    WireResult executeOperation(const Operation& op) {
        WireResult result;
        
        switch (op.type) {
            case Operation::BEGIN:
                wire.beginTransmission(op.address);
                break;
                
            case Operation::WRITE:
                result.bytesTransferred = wire.write(op.data, op.length);
                break;
                
            case Operation::END:
                result.error = (WireResult::Error)wire.endTransmission(op.sendStop);
                break;
                
            case Operation::REQUEST:
                if (op.length > MAX_I2C_BUFFER_SIZE) {
                    result.error = WireResult::BUS_ERROR;
                    return result;
                }
                
                result.bytesTransferred = wire.requestFrom(op.address, op.length, op.sendStop);
                if (result.bytesTransferred > 0) {
                    result.bytesTransferred = readToBuffer(wire, result.buffer, result.bytesTransferred);
                }
                break;
        }
        
        return result;
    }
};

// Interface non-bloquante
class AsyncWireProxy {
public:
    void beginTransmission(uint8_t address, std::function<void(const SequenceResult&)> callback) {
        WireSequenceBuilder()
            .beginTransmission(address)
            .execute(callback);
    }
    
    void write(const uint8_t* data, size_t length, 
               std::function<void(const SequenceResult&)> callback) {
        WireSequenceBuilder()
            .write(data, length)
            .execute(callback);
    }
    
    void write(uint8_t byte, std::function<void(const SequenceResult&)> callback) {
        write(&byte, 1, callback);
    }
    
    void endTransmission(bool sendStop, 
                        std::function<void(const SequenceResult&)> callback) {
        WireSequenceBuilder()
            .endTransmission(sendStop)
            .execute(callback);
    }
    
    void requestFrom(uint8_t address, size_t length, bool sendStop,
                    std::function<void(const SequenceResult&)> callback) {
        WireSequenceBuilder()
            .requestFrom(address, length, sendStop)
            .execute(callback);
    }
};

// Interface bloquante compatible Wire
class BlockingWireProxy {
private:
    SequenceResult lastResult;
    SemaphoreHandle_t semaphore;

public:
    BlockingWireProxy() {
        semaphore = xSemaphoreCreateBinary();
    }
    
    void beginTransmission(uint8_t address) {
        WireSequenceBuilder()
            .beginTransmission(address)
            .execute([this](const SequenceResult& result) {
                lastResult = result;
                xSemaphoreGive(semaphore);
            });
        
        xSemaphoreTake(semaphore, portMAX_DELAY);
    }
    
    size_t write(const uint8_t* data, size_t length) {
        WireSequenceBuilder()
            .write(data, length)
            .execute([this](const SequenceResult& result) {
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
            .execute([this](const SequenceResult& result) {
                lastResult = result;
                xSemaphoreGive(semaphore);
            });
        
        xSemaphoreTake(semaphore, portMAX_DELAY);
        return static_cast<uint8_t>(lastResult.final().error);
    }
    
    size_t requestFrom(uint8_t address, size_t length, bool sendStop = true) {
        WireSequenceBuilder()
            .requestFrom(address, length, sendStop)
            .execute([this](const SequenceResult& result) {
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
            .type = AsyncWire::Operation::BEGIN,
            .address = address
        });
        return *this;
    }

    WireSequenceBuilder& write(const uint8_t* data, size_t length) {
        sequence.push_back({
            .type = AsyncWire::Operation::WRITE,
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
            .type = AsyncWire::Operation::END,
            .sendStop = sendStop
        });
        return *this;
    }

    WireSequenceBuilder& requestFrom(uint8_t address, size_t length, bool sendStop = true) {
        sequence.push_back({
            .type = AsyncWire::Operation::REQUEST,
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

    void execute(AsyncWire::CompletionCallback callback) {
        AsyncWire::instance().executeSequence(sequence, callback);
    }

private:
    AsyncWire::Sequence sequence;
};