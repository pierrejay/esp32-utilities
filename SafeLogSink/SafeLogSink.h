// SafeLogSink.h
#ifndef SAFE_LOG_SINK_H
#define SAFE_LOG_SINK_H

#include <Arduino.h>
#include <Stream.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Configuration
// Change this definition to use a different output stream
#define LOG_OUTPUT Serial

// Definitions for the logger queue/task
#define LOG_QUEUE_SIZE 32
#define MAX_LOG_MSG_SIZE 256
#define LOG_TASK_PRIORITY 1
#define LOG_STACK_SIZE 4096

// Structure of a log message
typedef struct {
    char message[MAX_LOG_MSG_SIZE];
} LogMessage;

// Forward declaration
class SafeLogStream;

class StreamManager {
private:
    static constexpr size_t MAX_STREAMS = 10;
    static constexpr uint32_t FLUSH_INTERVAL = 100; // Flush each buffer after 100ms of inactivity (buffer pending)
    static SafeLogStream* streams[MAX_STREAMS];
    static uint32_t lastWriteTimes[MAX_STREAMS];  // 0 = inactif, >0 = timestamp dernière écriture
    static size_t streamCount;

public:
    // Returns the index assigned to the stream
    static size_t registerStream(SafeLogStream* stream) {
        if (streamCount < MAX_STREAMS) {
            size_t idx = streamCount++;
            streams[idx] = stream;
            lastWriteTimes[idx] = 0;  // Inactive at start
            return idx;
        }
        return -1;
    }

    static void notifyWrite(size_t streamIndex) {
        if (streamIndex < streamCount) {
            lastWriteTimes[streamIndex] = millis();
        }
    }

    static void notifyFlush(size_t streamIndex) {
        if (streamIndex < streamCount) {
            lastWriteTimes[streamIndex] = 0;  // Buffer inactive
        }
    }

    static void checkAndFlushStreams();
};

// Initialization of static variables
SafeLogStream* StreamManager::streams[MAX_STREAMS] = {};
uint32_t StreamManager::lastWriteTimes[MAX_STREAMS] = {};
size_t StreamManager::streamCount = 0;

class SafeLogSink {
private:
    static QueueHandle_t logQueue;
    static TaskHandle_t logTaskHandle;
    static bool initialized;
    static portMUX_TYPE logMux;  // For critical section
    
    // Task that reads the queue and writes the logs
    static void logTask(void* parameter) {
        LogMessage msg;
        const TickType_t checkInterval = pdMS_TO_TICKS(10);  // Check every 10ms max
        
        while (true) {
            // Wait at most 10ms if the queue is empty
            if (xQueueReceive(logQueue, &msg, checkInterval) == pdTRUE) {
                // Enter critical section to ensure atomic writing
                taskENTER_CRITICAL(&logMux);
                LOG_OUTPUT.print(msg.message);
                taskEXIT_CRITICAL(&logMux);
            }
            // Check all active streams
            StreamManager::checkAndFlushStreams();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

public:
    // Automatic initialization
    static void begin() {
        if (!initialized) {
            // Create the message queue
            logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));
            
            // Create the logging task
            xTaskCreate(
                logTask,
                "LogTask",
                LOG_STACK_SIZE,
                NULL,
                LOG_TASK_PRIORITY,
                &logTaskHandle
            );
            
            initialized = true;
        }
    }
    
    // Send a message to the queue
    static void sendToQueue(const char* message) {
        if (!initialized) {
            begin();
        }
        
        LogMessage msg;
        strncpy(msg.message, message, MAX_LOG_MSG_SIZE - 1);
        msg.message[MAX_LOG_MSG_SIZE - 1] = '\0';
        
        xQueueSend(logQueue, &msg, 0); // Send without waiting if queue is full
    }
};

// Stream class for logging
class SafeLogStream : public Stream {
private:
    friend class StreamManager;  // For StreamManager to access our private members
    
    // Temporary buffer to accumulate characters
    static constexpr size_t BUFFER_SIZE = 256;
    char buffer[BUFFER_SIZE];
    size_t bufferIndex = 0;
    uint32_t lastWriteTime = 0;
    size_t streamIndex;  // Index in the StreamManager array

public:
    SafeLogStream() : bufferIndex(0) {
        memset(buffer, 0, BUFFER_SIZE);
        lastWriteTime = millis();
        streamIndex = StreamManager::registerStream(this);
    }
    
    ~SafeLogStream() {
        flushBuffer();  // Final flush
    }
    
    // Required methods for Stream
    virtual size_t write(uint8_t c) override {
        lastWriteTime = millis();
        
        // If the buffer is full or if it's a newline, flush first
        if (bufferIndex >= BUFFER_SIZE - 1 || c == '\n') {
            flushBuffer();
        }
        
        // Now we can add the character to the buffer
        if (bufferIndex < BUFFER_SIZE - 1) {
            buffer[bufferIndex++] = c;
            StreamManager::notifyWrite(streamIndex);
        }
        
        return 1;
    }

    void flushBuffer() {
        if (bufferIndex > 0) {
            buffer[bufferIndex] = '\0';
            
            // If the last character is not a \n, add it
            char finalBuffer[MAX_LOG_MSG_SIZE];
            snprintf(finalBuffer, sizeof(finalBuffer), "%s%s", 
                buffer, 
                (buffer[bufferIndex-1] != '\n') ? "\n" : "");
            
            SafeLogSink::sendToQueue(finalBuffer);
            bufferIndex = 0;
            StreamManager::notifyFlush(streamIndex);
        }
    }

    // Optimized writing for blocks of data
    virtual size_t write(const uint8_t *buffer, size_t size) override {
        for (size_t i = 0; i < size; i++) {
            write(buffer[i]);
        }
        return size;
    }
    
    // Required methods for Stream (not used for logging)
    virtual int available() override { return 0; }
    virtual int read() override { return -1; }
    virtual int peek() override { return -1; }
    
    // Explicit flush
    virtual void flush() override {
        flushBuffer();
    }
    
    // Additional methods to facilitate usage
    size_t printf(const char *format, ...) {
        // First flush the existing buffer
        flushBuffer();
        
        char localBuffer[BUFFER_SIZE];
        va_list args;
        va_start(args, format);
        size_t len = vsnprintf(localBuffer, BUFFER_SIZE, format, args);
        va_end(args);
        
        // Use sendToQueue directly to avoid recursion
        SafeLogSink::sendToQueue(localBuffer);
        return len;
    }

    // Overload of println to use our system
    size_t println(const char* message = "") {
        return printf("%s\n", message);
    }

    size_t println(const String &s) {
        return printf("%s\n", s.c_str());
    }

    // Overloads of print to use our system
    size_t print(const char* message) {
        return printf("%s", message);
    }

    size_t print(const String &s) {
        return printf("%s", s.c_str());
    }

};

// Initialization of static variables
QueueHandle_t SafeLogSink::logQueue = NULL;
TaskHandle_t SafeLogSink::logTaskHandle = NULL;
bool SafeLogSink::initialized = false;
portMUX_TYPE SafeLogSink::logMux = portMUX_INITIALIZER_UNLOCKED;

// Default instance for compatibility with the global API
static SafeLogStream _defaultLogStream;
Stream& LogStream = _defaultLogStream;

// The global functions use the default instance
void log(const char* message) {
    SafeLogSink::sendToQueue(message);
}

void logln(const char* message = "") {
    char buffer[MAX_LOG_MSG_SIZE];
    snprintf(buffer, sizeof(buffer), "%s\n", message);
    SafeLogSink::sendToQueue(buffer);
}

void logf(const char* format, ...) {
    char buffer[MAX_LOG_MSG_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    SafeLogSink::sendToQueue(buffer);
}

// Now we can define checkAndFlushStreams because SafeLogStream is complete
void StreamManager::checkAndFlushStreams() {
    uint32_t currentTime = millis();
    for (size_t i = 0; i < streamCount; i++) {
        if (lastWriteTimes[i] > 0 && 
            (currentTime - lastWriteTimes[i]) >= FLUSH_INTERVAL) {
            streams[i]->flushBuffer();
        }
    }
}

#endif // SAFE_LOG_SINK_H