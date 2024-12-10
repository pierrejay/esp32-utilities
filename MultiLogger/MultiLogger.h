#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string>
#include <vector>
#include <memory>
#include "FS.h"
#include <inttypes.h>

//##############################################################################
//                            MACROS for logging
//##############################################################################

#define LOG_ERROR(...)   MultiLogger::getInstance().log(logger::LogLevel::ERROR, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARNING(...) MultiLogger::getInstance().log(logger::LogLevel::WARNING, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...)    MultiLogger::getInstance().log(logger::LogLevel::INFO, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_SYSTEM(...) MultiLogger::getInstance().log(logger::LogLevel::SYSTEM, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_DEBUG(...)   MultiLogger::getInstance().log(logger::LogLevel::DEBUG, __FILE__, __LINE__, __VA_ARGS__)


namespace logger {

    static constexpr size_t MAX_UINT64_CHARS = 20;  // Taille max d'un uint64_t en caractères
    static constexpr size_t MIN_TIMESTAMP_BUFFER = MAX_UINT64_CHARS + 1;  // +1 pour le \0

//##############################################################################
//                                Structures
//##############################################################################

/**
 * @brief Log levels in order of decreasing severity
 * Each level includes all levels above it in severity when used as a filter
 */
enum class LogLevel {
    NONE,       // Pas de log (désactive la sortie)
    ERROR,      // Rouge - Uniquement les erreurs
    WARNING,    // Jaune - ERROR + WARNING
    INFO,       // Vert  - ERROR + WARNING + INFO
    SYSTEM,     // Bleu  - Tous sauf DEBUG
    DEBUG       // Rose  - Tous les logs
};

// Fonction helper pour convertir LogLevel en string
inline const char* toString(LogLevel level) {
    switch (level) {
        case LogLevel::NONE:    return "NONE";
        case LogLevel::ERROR:   return "ERROR";
        case LogLevel::WARNING: return "WARNING";
        case LogLevel::INFO:    return "INFO";
        case LogLevel::SYSTEM:  return "SYSTEM";
        case LogLevel::DEBUG:   return "DEBUG";
        default:                return "UNKNOWN";
    }
}

enum class OutputFormat {
    ASCII,
    CSV
};

enum class OutputType {
    CONSOLE,
    SD,
    JSON
};


struct LogMessage {
    LogLevel level;
    char message[256];
    char file[256];
    int line;
    uint32_t timestamp;     // Timestamp Unix en secondes
    uint64_t uptime;        // Temps depuis le démarrage en millisecondes
    char taskName[16];
    const char* deviceId;   // Identifiant du dispositif (peut être nullptr)
};


//##############################################################################
//                             Time Provider
//##############################################################################

/**
 * @brief Time provider interface for timestamp generation
 * 
 * Provides both Unix timestamp and system uptime in milliseconds.
 * Implementations should handle their specific RTC or time source.
 */
class TimeProvider {
public:
    virtual ~TimeProvider() = default;
    virtual uint32_t getTimestamp() = 0;
    virtual uint64_t getMillis() = 0;
};

/**
 * @brief Default time provider using system millis()
 * 
 * Handles millis() overflow by tracking rollovers to maintain
 * accurate 64-bit uptime counter. Does not provide real timestamp.
 */
class DefaultTime : public TimeProvider {
private:
    uint32_t lastMillis_ = 0;    // Dernière valeur de millis() vue
    uint64_t overflows_ = 0;     // Nombre d'overflows détectés
    
public:
    uint32_t getTimestamp() override { 
        return 0;  // Pas de RTC, retourne 0
    }

    uint64_t getMillis() override {
        uint32_t currentMillis = millis();
        
        // Détection de l'overflow
        if (currentMillis < lastMillis_) {
            overflows_++;  // On a fait un tour complet
        }
        lastMillis_ = currentMillis;
        
        return (overflows_ << 32) + currentMillis;
    }
};

//##############################################################################
//                        Log Output (abstract class)
//##############################################################################

/**
 * @brief Interface for log output
 * 
 * Simplified interface following SOLID principles, focusing on the core
 * responsibility of writing log messages.
 */
class ILogOutput {
public:
    virtual ~ILogOutput() = default;
    virtual void begin() = 0;  // Initialisation spécifique si nécessaire
    virtual void writeLogMessage(const LogMessage& msg) = 0;
    virtual OutputType getType() const = 0;

    inline void setLogLevel(LogLevel level) {
        level_ = level;
    }

    LogLevel getLogLevel() const {
        return level_;
    }

protected:
    LogLevel level_ = LogLevel::DEBUG;
};


//##############################################################################
//                          Serial Logger (console output)
//##############################################################################

/**
 * @brief Serial output
 * 
 * Handles serial output with color coding for different log levels.
 */
class SerialLogger : public ILogOutput {
public:
    explicit SerialLogger(HardwareSerial& serial, uint32_t baud = 115200) 
        : serial_(serial), baud_(baud), lastCheckTime_(0), isConnected_(false) {}

    void begin() override {
        return;
    }

    void writeLogMessage(const LogMessage& msg) override {
        if (!checkConnection()) return;

        char timestamp[32];
        if (msg.timestamp > 0) {
            // Validation du timestamp
            if (msg.timestamp > 0x7FFFFFFF) {  // Max valid Unix timestamp
                goto use_uptime;
            }

            time_t t = msg.timestamp;
            struct tm* timeinfo = gmtime(&t);
            if (!timeinfo) {
                goto use_uptime;
            }

            int written = snprintf(timestamp, sizeof(timestamp), 
                    "%04d-%02d-%02d %02d:%02d:%02d.%03lu",
                    timeinfo->tm_year + 1900,
                    timeinfo->tm_mon + 1,
                    timeinfo->tm_mday,
                    timeinfo->tm_hour,
                    timeinfo->tm_min,
                    timeinfo->tm_sec,
                    static_cast<unsigned long>(msg.uptime % 1000));

            if (written <= 0 || written >= sizeof(timestamp)) {
                goto use_uptime;
            }
        } else {
        use_uptime:
            snprintf(timestamp, sizeof(timestamp), "%" PRIu64, msg.uptime);
        }

        setColor(msg.level);
        serial_.print(timestamp);
        serial_.print(" [");
        serial_.print(logger::toString(msg.level));
        serial_.print("] ");
        
        resetColor();
        serial_.print(msg.message);
        serial_.print(" (");
        serial_.print(msg.taskName);
        serial_.print("@");
        serial_.print(msg.file);
        serial_.print(":");
        serial_.print(msg.line);
        serial_.println(")");
    }

    OutputType getType() const override { return OutputType::CONSOLE; }

private:
    HardwareSerial& serial_;
    uint32_t baud_;
    unsigned long lastCheckTime_;
    bool isConnected_;
    static constexpr unsigned long CHECK_INTERVAL = 1000;

    void setColor(LogLevel level) {
        if (!checkConnection()) return;
        switch (level) {
            case LogLevel::ERROR:   serial_.print("\033[31m"); break;
            case LogLevel::WARNING: serial_.print("\033[33m"); break;
            case LogLevel::INFO:    serial_.print("\033[32m"); break;
            case LogLevel::SYSTEM:  serial_.print("\033[34m"); break;
            case LogLevel::DEBUG:   serial_.print("\033[35m"); break;
            default:               serial_.print("\033[37m"); break;
        }
    }

    void resetColor() {
        if (!checkConnection()) return;
        serial_.print("\033[37m");
    }

    bool checkConnection() {
        unsigned long currentTime = millis();
        if (currentTime - lastCheckTime_ >= CHECK_INTERVAL) {
            #ifdef ESP32
                isConnected_ = Serial.availableForWrite() > 0;
            #else
                isConnected_ = serial_.availableForWrite() > 0;
            #endif
            lastCheckTime_ = currentTime;
        }
        return isConnected_;
    }

    void formatTimestamp(uint32_t timestamp, uint64_t ms, char* buffer, size_t bufferSize) {
        if (timestamp > 0) {
            time_t t = timestamp;
            struct tm* timeinfo = gmtime(&t);
            snprintf(buffer, bufferSize, 
                    "%04d-%02d-%02d %02d:%02d:%02d.%03lu",
                    timeinfo->tm_year + 1900,
                    timeinfo->tm_mon + 1,
                    timeinfo->tm_mday,
                    timeinfo->tm_hour,
                    timeinfo->tm_min,
                    timeinfo->tm_sec,
                    static_cast<unsigned long>(ms % 1000));
        } else {
            uint64_t totalMs = ms;
            uint32_t hours = totalMs / 3600000;
            uint32_t minutes = (totalMs % 3600000) / 60000;
            uint32_t seconds = (totalMs % 60000) / 1000;
            uint32_t millis = totalMs % 1000;
            
            snprintf(buffer, bufferSize, "%02lu:%02lu:%02lu.%03lu", 
                    static_cast<unsigned long>(hours), 
                    static_cast<unsigned long>(minutes), 
                    static_cast<unsigned long>(seconds), 
                    static_cast<unsigned long>(millis));
        }
    }
};


//##############################################################################
//                         SD Logger (filesystem output)
//##############################################################################

/**
 * @brief SD output
 * 
 * Handles SD output with file rotation and level splitting.
 */
class SDLogger : public ILogOutput {
public:
    // Structure pour les infos de fichier
    struct FileInfo {
        size_t size;
        const char* path;
    };

    SDLogger(fs::FS& fs, const char* path, bool splitByLevel = false) 
        : fs_(fs), basePath_(path), splitByLevel_(splitByLevel), writeCount_(0) {}

    void begin() override {
        if (!splitByLevel_) {
            // Mode normal : un seul fichier
            file_ = fs_.open(basePath_, "a");
            return;
        }
        // Mode fichiers séparés : on n'ouvre rien pour l'instant
        // Les fichiers seront ouverts à la demande
    }

    void writeLogMessage(const LogMessage& msg) override {
        char buffer[512];
        int written;
        if (msg.deviceId) {
            written = snprintf(buffer, sizeof(buffer),
                "%lu,%" PRIu64 ",%s,%s@%s:%d,%s,%s",
                static_cast<unsigned long>(msg.timestamp),
                msg.uptime,
                logger::toString(msg.level),
                msg.taskName,
                msg.file,
                msg.line,
                msg.message,
                msg.deviceId);
        } else {
            written = snprintf(buffer, sizeof(buffer),
                "%lu,%" PRIu64 ",%s,%s@%s:%d,%s",
                static_cast<unsigned long>(msg.timestamp),
                msg.uptime,
                logger::toString(msg.level),
                msg.taskName,
                msg.file,
                msg.line,
                msg.message);
        }

        if (written > 0 && written < sizeof(buffer)) {
            if (!splitByLevel_) {
                writeToFile(file_, buffer);
            } else {
                fs::File& file = getFileForLevel(msg.level);
                writeToFile(file, buffer);
            }
        }
    }

    OutputType getType() const override { return OutputType::SD; }

    // Obtenir la taille des fichiers
    std::vector<FileInfo> getFilesInfo() const {
        std::vector<FileInfo> infos;
        
        if (!splitByLevel_) {
            if (file_) {
                infos.push_back({file_.size(), file_.path()});
            }
            return infos;
        }

        // En mode split, récupérer les infos de tous les fichiers
        for (const auto& file : levelFiles_) {
            if (file) {
                infos.push_back({file.size(), file.path()});
            }
        }
        return infos;
    }

    // Créer un nouveau fichier en archivant l'existant
    bool rotate() {
        if (!splitByLevel_) {
            return rotateFile(file_, basePath_);
        }

        // En mode split, rotation de tous les fichiers
        bool success = true;
        for (size_t i = 0; i < levelFiles_.size(); i++) {
            if (levelFiles_[i]) {
                const char* path = levelFiles_[i].path();
                success &= rotateFile(levelFiles_[i], path);
                // Réouvrir le fichier pour continuer à logger
                levelFiles_[i] = fs_.open(path, "a");
            }
        }
        return success;
    }

private:
    fs::FS& fs_;
    const char* basePath_;
    bool splitByLevel_;
    fs::File file_;  // Fichier unique en mode normal
    std::array<fs::File, 4> levelFiles_;  // Un fichier par niveau en mode séparé
    size_t writeCount_;
    static constexpr size_t FLUSH_INTERVAL = 10;

    fs::File& getFileForLevel(LogLevel level) {
        size_t idx = static_cast<size_t>(level);
        if (!levelFiles_[idx]) {
            // Construire le nom du fichier
            char path[256];
            const char* base = basePath_;
            
            // Trouver où insérer le suffixe (avant l'extension)
            const char* dot = strrchr(base, '.');
            if (dot) {
                size_t baseLen = dot - base;
                snprintf(path, sizeof(path), "%.*s_%s%s", 
                        static_cast<int>(baseLen), base,
                        getLevelSuffix(level), dot);
            } else {
                snprintf(path, sizeof(path), "%s_%s", 
                        base, getLevelSuffix(level));
            }
            
            levelFiles_[idx] = fs_.open(path, "a");
        }
        return levelFiles_[idx];
    }

    const char* getLevelSuffix(LogLevel level) {
        static char buffer[32];
        const char* str = logger::toString(level);
        
        size_t i = 0;
        while (str[i] && i < sizeof(buffer) - 1) {
            buffer[i] = tolower(str[i]);
            i++;
        }
        buffer[i] = '\0';
        
        return buffer;
    }

    void writeToFile(fs::File& file, const char* message) {
        if (!file) return;
        
        if (!file.println(message)) {
            file.close();
            file = fs_.open(file.path(), "a");
            if (file) {
                file.println(message);
            }
        }
        
        if (++writeCount_ >= FLUSH_INTERVAL) {
            file.flush();
            writeCount_ = 0;
        }
    }

    bool rotateFile(File& file, const char* originalPath) {
        if (!file) return false;

        // Construire le nom du fichier d'archive
        char archivePath[256];
        const char* dot = strrchr(originalPath, '.');
        if (dot) {
            size_t baseLen = dot - originalPath;
            snprintf(archivePath, sizeof(archivePath), 
                    "%.*s_%lu%s", 
                    static_cast<int>(baseLen),
                    originalPath,
                    static_cast<unsigned long>(time(nullptr)),
                    dot);
        } else {
            snprintf(archivePath, sizeof(archivePath), 
                    "%s_%lu",
                    originalPath,
                    static_cast<unsigned long>(time(nullptr)));
        }

        // Fermer le fichier actuel
        file.close();

        // Renommer le fichier existant
        if (!fs_.rename(originalPath, archivePath)) {
            // En cas d'échec, réouvrir le fichier original
            file = fs_.open(originalPath, "a");
            return false;
        }

        // Créer un nouveau fichier
        file = fs_.open(originalPath, "a");
        return file;
    }
};


//##############################################################################
//                       JSON Logger (JSON stream output)
//##############################################################################

/**
 * @brief JSON output
 * 
 * Handles JSON output to a Print stream.
 */
class JsonLogger : public ILogOutput {
public:
    using JsonCallback = std::function<void(JsonObject&)>;

    explicit JsonLogger(JsonCallback callback) 
        : callback_(callback) {}

    void begin() override {}

    void writeLogMessage(const LogMessage& msg) override {
        StaticJsonDocument<512> doc;  // Taille suffisante pour un message de log
        JsonObject obj = doc.to<JsonObject>();

        // Ajout des champs obligatoires
        obj["timestamp"] = msg.timestamp;
        obj["uptime"] = msg.uptime;
        obj["level"] = logger::toString(msg.level);
        obj["message"] = msg.message;
        obj["location"] = String(msg.file) + ":" + String(msg.line);
        obj["task"] = msg.taskName;

        // Ajout conditionnel du deviceId
        if (msg.deviceId) {
            obj["deviceid"] = msg.deviceId;
        }

        // Appel du callback
        callback_(obj);
    }

    OutputType getType() const override { return OutputType::JSON; }

private:
    JsonCallback callback_;
};


//##############################################################################
//                            System stats helpers
//##############################################################################

inline String getSystemStats() {
    char buffer[256];
    #ifdef ESP32
        snprintf(buffer, sizeof(buffer),
                "Heap: %zu bytes (%zu min) | Stack: %lu | CPU: %.1f°C",
                ESP.getFreeHeap(),
                ESP.getMinFreeHeap(),
                uxTaskGetStackHighWaterMark(nullptr),
                temperatureRead());
    #else
        snprintf(buffer, sizeof(buffer), "System stats not available");
    #endif
    return String(buffer);
}


inline void logSystemStats() {
    LOG_SYSTEM(logger::getSystemStats().c_str());
}

} // namespace logger


//##############################################################################
//                            MultiLogger class
//##############################################################################

/**
 * @brief Thread-safe logging system with multiple outputs and filtering capabilities
 * 
 * Singleton class that manages logging with the following features:
 * - Multiple output support (Serial, SD card, JSON)
 * - Per-output log filtering
 * - FreeRTOS task-safe implementation
 * - Timestamp support with RTC integration
 * - Configurable queue size and task priority
 */
class MultiLogger {
public:
    using LogLevel = logger::LogLevel;
    using LogMessage = logger::LogMessage;
    using OutputType = logger::OutputType;
    using TimeProvider = logger::TimeProvider;
    using DefaultTime = logger::DefaultTime;

    // Constructeur avec paramètres optionnels
    static MultiLogger& getInstance(const char* deviceId = nullptr, LogLevel filter = LogLevel::DEBUG) {
        static MultiLogger instance(deviceId, filter);
        return instance;
    }

    // Getters/Setters pour les paramètres configurables
    void setDeviceId(const char* deviceId) {
        deviceId_ = deviceId;
    }

    const char* getDeviceId() const {
        return deviceId_;
    }

    void setLogFilter(LogLevel filter) {
        filter_ = filter;
    }

    LogLevel getLogFilter() const {
        return filter_;
    }

    void addOutput(logger::ILogOutput& output) {
        outputs_.push_back(&output);
    }

    void addOutput(logger::ILogOutput& output, LogLevel filter) {
        output.setLogLevel(filter);
        outputs_.push_back(&output);
    }

    bool begin() {
        if (logQueue_ != nullptr) return true;

        logQueue_ = xQueueCreate(QUEUE_SIZE, sizeof(LogMessage));
        if (logQueue_ == nullptr) return false;

        BaseType_t result = xTaskCreate(
            loggerTask,
            "Logger",
            4096,
            this,
            TASK_PRIORITY,
            &loggerTaskHandle_
        );

        if (result != pdPASS) {
            vQueueDelete(logQueue_);
            logQueue_ = nullptr;
            return false;
        }

        for (auto* output : outputs_) {
            output->begin();
        }

        return true;
    }

    bool log(LogLevel level, const char* file, int line, const char* message) {
        // Check first if at least one output will accept this level
        bool anyOutputWillAccept = false;
        for (auto* output : outputs_) {
            if (isLevelEnabled(level, output->getLogLevel())) {
                anyOutputWillAccept = true;
                break;
            }
        }

        // If no output will accept this level, don't put it in the queue
        if (!anyOutputWillAccept) {
            return true;  // Success because it's a normal filter
        }

        if (logQueue_ == nullptr || file == nullptr || message == nullptr) {
            return false;
        }

        LogMessage msg = {
            .level = level,
            .line = line,
            .timestamp = timeProvider_.current->getTimestamp(),
            .uptime = timeProvider_.current->getMillis(),
            .deviceId = deviceId_
        };

        // Copie sécurisée des chaînes avec le nom de fichier déjà extrait
        strncpy(msg.file, getFileName(file), MAX_FILENAME - 1);
        msg.file[MAX_FILENAME - 1] = '\0';

        strncpy(msg.message, message, MAX_MESSAGE - 1);
        msg.message[MAX_MESSAGE - 1] = '\0';

        // Nom de la tâche courante
        TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
        const char* taskName = currentTask ? pcTaskGetName(currentTask) : nullptr;
        if (taskName) {
            strncpy(msg.taskName, taskName, sizeof(msg.taskName) - 1);
            msg.taskName[sizeof(msg.taskName) - 1] = '\0';
        } else {
            strcpy(msg.taskName, "unknown");
        }

        return xQueueSend(logQueue_, &msg, QUEUE_TIMEOUT) == pdTRUE;
    }

    inline void setTimeProvider(TimeProvider& provider) {
        timeProvider_.current = &provider;
    }

    inline bool isLevelEnabled(LogLevel level) const {
        return isLevelEnabled(level, filter_);
    }

    bool isLevelEnabled(LogLevel messageLevel, LogLevel filterLevel) const {
        if (filterLevel == LogLevel::NONE) return false;
        return messageLevel <= filterLevel;  // Les niveaux sont ordonnés du plus critique au moins critique
    }

    ~MultiLogger() {
        // Wait for all messages to be processed
        if (logQueue_) {
            const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);  // 1 seconde max
            TickType_t xTimeWaited = 0;
            
            while (uxQueueMessagesWaiting(logQueue_) > 0 && xTimeWaited < xMaxBlockTime) {
                vTaskDelay(pdMS_TO_TICKS(10));
                xTimeWaited += pdMS_TO_TICKS(10);
            }

            // Clean up resources
            vQueueDelete(logQueue_);
            logQueue_ = nullptr;
        }

        if (loggerTaskHandle_) {
            vTaskDelete(loggerTaskHandle_);
            loggerTaskHandle_ = nullptr;
        }

        // Clear buffers
        memset(formatBuffer_, 0, FORMAT_BUFFER);
        memset(timestampBuffer_, 0, TIMESTAMP_BUFFER_SIZE);
    }

private:
    // Configuration hardcodée
    static constexpr size_t MAX_FILENAME = 64;
    static constexpr size_t MAX_MESSAGE = 256;
    static constexpr size_t MAX_TASKNAME = 16;
    static constexpr size_t FORMAT_BUFFER = 512;
    static constexpr size_t TIMESTAMP_BUFFER_SIZE = 32;
    
    // Paramètres FreeRTOS hardcodés
    static constexpr size_t QUEUE_SIZE = 8;
    static constexpr TickType_t QUEUE_TIMEOUT = pdMS_TO_TICKS(100);
    static constexpr UBaseType_t TASK_PRIORITY = 1;

    char formatBuffer_[FORMAT_BUFFER];
    char timestampBuffer_[TIMESTAMP_BUFFER_SIZE];

    QueueHandle_t logQueue_;
    TaskHandle_t loggerTaskHandle_;
    std::vector<logger::ILogOutput*> outputs_;

    /**
     * @brief Static time provider
     */
    struct StaticTimeProvider {
        DefaultTime defaultTime;     // Maintenant DefaultTime est défini
        TimeProvider* current;
        
        StaticTimeProvider() : defaultTime(), current(&defaultTime) {}
    };

    StaticTimeProvider timeProvider_;

    const char* deviceId_;
    LogLevel filter_;

    MultiLogger(const char* deviceId = nullptr, LogLevel filter = LogLevel::DEBUG) 
        : logQueue_(nullptr)
        , loggerTaskHandle_(nullptr)
        , deviceId_(deviceId)
        , filter_(filter)
        , timeProvider_()
    {
    }

    /**
     * @brief Logger task
     * 
     * @param parameter The parameter to pass to the task
     */
    static void loggerTask(void* parameter) {
        MultiLogger* logger = static_cast<MultiLogger*>(parameter);
        LogMessage msg;

        while (true) {
            if (xQueueReceive(logger->logQueue_, &msg, portMAX_DELAY) == pdTRUE) {
                logger->processLogMessage(msg);
            }
        }
    }

    /**
     * @brief Get the file name from a full path
     * 
     * @param fullPath The full path to the file
     * @return The file name
     */
   inline const char* getFileName(const char* fullPath) {
        const char* fileName = strrchr(fullPath, '/');
        if (fileName) {
            return fileName + 1;
        }
        
        fileName = strrchr(fullPath, '\\');
        if (fileName) {
            return fileName + 1;
        }
        
        return fullPath;
    }

    /**
     * @brief Process a log message
     * 
     * @param msg The log message to process
     */
    void processLogMessage(const LogMessage& msg) {
        for (auto* output : outputs_) {
            if (isLevelEnabled(msg.level, output->getLogLevel())) {
                output->writeLogMessage(msg);
            }
        }
    }

    MultiLogger(const MultiLogger&) = delete;
    MultiLogger& operator=(const MultiLogger&) = delete;
};
