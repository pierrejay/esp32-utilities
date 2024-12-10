#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string>
#include <vector>
#include <memory>
#include "FS.h"

#define LOG_ALERT(...)   MultiLogger::getInstance().log(logger::LogLevel::ALERT, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARNING(...) MultiLogger::getInstance().log(logger::LogLevel::WARNING, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_DEBUG(...) MultiLogger::getInstance().log(logger::LogLevel::DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_SYSTEM(...)  MultiLogger::getInstance().log(logger::LogLevel::SYSTEM, __FILE__, __LINE__, __VA_ARGS__)


namespace logger {

enum class LogLevel {
    ALERT,      // Rouge
    WARNING,    // Jaune
    DEBUG,    // Vert
    SYSTEM      // Bleu
};

enum class LogFilter {
    NONE,       // Aucun log
    ALERTS,     // Alertes + warnings
    ALL         // Tous les logs
};

enum class OutputFormat {
    ASCII,
    CSV
};

enum class OutputType {
    CONSOLE,
    SD,
    JSON
};

// Interface pour le RTC
class TimeProvider {
public:
    virtual ~TimeProvider() = default;
    virtual uint32_t getTimestamp() = 0;  // Retourne le timestamp Unix en secondes
    virtual uint64_t getMillis() = 0;     // Retourne les millisecondes (64 bits)
};

// Implémentation par défaut utilisant millis()
class DefaultTime : public TimeProvider {
private:
    uint32_t lastMillis_ = 0;    // Dernière valeur de millis() vue
    uint64_t overflows_ = 0;     // Nombre d'overflows détectés
    
public:
    uint32_t getTimestamp() override { return 0; }

    uint64_t getMillis() override {
        uint32_t currentMillis = millis();
        
        // Dtection de l'overflow
        if (currentMillis < lastMillis_) {
            overflows_++;  // On a fait un tour complet
        }
        lastMillis_ = currentMillis;
        
        return (overflows_ << 32) + currentMillis;
    }
};

struct LogMessage {
    LogLevel level;
    char message[256];
    char file[256];
    int line;
    uint32_t timestamp;     // Timestamp Unix en secondes
    uint64_t uptime;        // Temps depuis le démarrage en millisecondes
    char taskName[16];
};

// Interface de sortie pour le logger
class ILogOutput {
public:
    virtual ~ILogOutput() = default;
    virtual void begin() = 0;
    virtual void print(const char* message) = 0;
    virtual void println(const char* message) = 0;
    virtual OutputType getType() const = 0;

    inline void setLogFilter(LogFilter filter) {
        logFilter_ = filter;
    }

    LogFilter getLogFilter() const {
        return logFilter_;
    }

protected:
    LogFilter logFilter_ = LogFilter::ALL;  // Par défaut, tout logger
};

// Sortie vers un port série
class SerialLogger : public ILogOutput {
public:
    explicit SerialLogger(HardwareSerial& serial, uint32_t baud = 115200) 
        : serial_(serial), baud_(baud) {}

    void begin() override {
        serial_.begin(baud_);
    }

    void print(const char* message) override {
        serial_.print(message);
    }

    void println(const char* message) override {
        serial_.println(message);
    }

    void setColor(LogLevel level) {
        switch (level) {
            case LogLevel::ALERT:
                serial_.print("\033[31m"); // Rouge
                break;
            case LogLevel::WARNING:
                serial_.print("\033[33m"); // Jaune
                break;
            case LogLevel::SYSTEM:
                serial_.print("\033[34m"); // Bleu
                break;
            case LogLevel::DEBUG:
                serial_.print("\033[32m"); // Vert
                break;
            default:
                serial_.print("\033[37m"); // Blanc
                break;
        }
    }

    void resetColor() {
        serial_.print("\033[37m"); // Retour au blanc
    }

    OutputType getType() const override { return OutputType::CONSOLE; }

private:
    HardwareSerial& serial_;
    uint32_t baud_;
};

// Sortie vers un fichier sur SD
class SDLogger : public ILogOutput {
public:
    struct Config {
        bool splitByLevel = false;  // Si true, crée un fichier par niveau
        const char* extension = nullptr;  // Extension à utiliser (par défaut prend celle du path)
    };

    // Structure pour les infos de fichier
    struct FileInfo {
        size_t size;
        const char* path;
    };

    SDLogger(fs::FS& fs, const char* path, const Config& config = Config()) 
        : fs_(fs), basePath_(path), config_(config), writeCount_(0) {
        // Extraire l'extension si non spécifiée
        if (!config_.extension) {
            const char* dot = strrchr(path, '.');
            if (dot) {
                config_.extension = dot;  // Utilise l'extension du fichier
            } else {
                config_.extension = "";   // Pas d'extension
            }
        }
    }

    void begin() override {
        if (!config_.splitByLevel) {
            // Mode normal : un seul fichier
            file_ = fs_.open(basePath_, "a");
            return;
        }

        // Mode fichiers séparés : on n'ouvre rien pour l'instant
        // Les fichiers seront ouverts à la demande
    }

    void println(const char* message) override {
        if (!config_.splitByLevel) {
            writeLine(file_, message);
            return;
        }

        // Déterminer le niveau à partir du message CSV
        LogLevel level = detectLevel(message);
        fs::File& file = getFileForLevel(level);
        writeLine(file, message);
    }

    OutputType getType() const override { return OutputType::SD; }

    // Obtenir la taille des fichiers
    std::vector<FileInfo> getFilesInfo() const {
        std::vector<FileInfo> infos;
        
        if (!config_.splitByLevel) {
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
        if (!config_.splitByLevel) {
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
    Config config_;
    fs::File file_;  // Fichier unique en mode normal
    std::array<fs::File, 4> levelFiles_;  // Un fichier par niveau en mode séparé
    size_t writeCount_;
    static constexpr size_t FLUSH_INTERVAL = 10;

    LogLevel detectLevel(const char* message) {
        // Le message est au format CSV, le niveau est le 2e champ
        const char* comma = strchr(message, ',');
        if (!comma) return LogLevel::DEBUG;
        comma++;
        if (strncmp(comma, "ALERT", 5) == 0) return LogLevel::ALERT;
        if (strncmp(comma, "WARNING", 7) == 0) return LogLevel::WARNING;
        if (strncmp(comma, "SYSTEM", 6) == 0) return LogLevel::SYSTEM;
        return LogLevel::DEBUG;
    }

    fs::File& getFileForLevel(LogLevel level) {
        size_t idx = static_cast<size_t>(level);
        if (!levelFiles_[idx]) {
            // Construire le nom du fichier
            char path[256];
            const char* base = basePath_;
            const char* ext = config_.extension;
            
            // Trouver où insérer le suffixe (avant l'extension)
            const char* dot = strrchr(base, '.');
            if (dot) {
                size_t baseLen = dot - base;
                snprintf(path, sizeof(path), "%.*s_%s%s", 
                        static_cast<int>(baseLen), base,
                        getLevelSuffix(level), ext);
            } else {
                snprintf(path, sizeof(path), "%s_%s%s", 
                        base, getLevelSuffix(level), ext);
            }
            
            levelFiles_[idx] = fs_.open(path, "a");
        }
        return levelFiles_[idx];
    }

    const char* getLevelSuffix(LogLevel level) {
        switch (level) {
            case LogLevel::ALERT: return "alert";
            case LogLevel::WARNING: return "warning";
            case LogLevel::DEBUG: return "debug";
            case LogLevel::SYSTEM: return "system";
            default: return "unknown";
        }
    }

    void writeLine(fs::File& file, const char* message) {
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

// Sortie au format JSON vers un Stream
class JsonLogger : public ILogOutput {
public:
    explicit JsonLogger(Print& output) 
        : output_(output) {}

    void begin() override {}
    void print(const char* message) override {}
    void println(const char* message) override {}

    void sendLogEvent(const LogMessage& msg) {
        char location[256];
        
        // Début du JSON directement avec les données
        output_.print("{");
        
        // Timestamp Unix
        output_.print("\"timestamp\":");
        output_.print(msg.timestamp);
        output_.print(",");
        
        // Uptime en millisecondes
        output_.print("\"uptime\":");
        output_.print(msg.uptime);
        output_.print(",");
        
        // Level
        output_.print("\"level\":\"");
        output_.print(getLevelString(msg.level));
        output_.print("\",");
        
        // Message (avec échappement)
        output_.print("\"message\":\"");
        printEscaped(msg.message);
        output_.print("\",");
        
        // Location (file:line)
        snprintf(location, sizeof(location), "%s:%d", msg.file, msg.line);
        output_.print("\"location\":\"");
        printEscaped(location);
        output_.print("\",");
        
        // Task
        output_.print("\"task\":\"");
        printEscaped(msg.taskName);
        output_.print("\"");
        
        // Fin du JSON
        output_.println("}");
    }

    OutputType getType() const override { return OutputType::JSON; }

private:
    Print& output_;

    // Fonction utilitaire pour échapper les caractères spéciaux JSON
    inline void printEscaped(const char* str) {
        if (!str) {
            output_.print("null");  // Gestion des pointeurs null
            return;
        }

        while (*str) {
            unsigned char c = *str++;  // unsigned pour gérer correctement les caractères étendus
            
            // Caractères de contrôle JSON standard
            switch (c) {
                case '\"': output_.print("\\\""); break;  // Guillemets
                case '\\': output_.print("\\\\"); break;  // Backslash
                case '\b': output_.print("\\b"); break;   // Backspace
                case '\f': output_.print("\\f"); break;   // Form feed
                case '\n': output_.print("\\n"); break;   // Nouvelle ligne
                case '\r': output_.print("\\r"); break;   // Retour chariot
                case '\t': output_.print("\\t"); break;   // Tab
                case '/': output_.print("\\/"); break;    // Slash (optionnel mais recommandé)
                
                default:
                    // Caractères de contrôle et caractères Unicode étendus
                    if (c < 0x20 || c == 0x7F) {
                        // Caractères de contrôle en notation \uXXXX
                        char hex[7];
                        snprintf(hex, sizeof(hex), "\\u%04x", c);
                        output_.print(hex);
                    }
                    // Caractères Unicode étendus (>127)
                    else if (c > 0x7E) {
                        char hex[7];
                        snprintf(hex, sizeof(hex), "\\u%04x", c);
                        output_.print(hex);
                    }
                    // Caractères ASCII imprimables normaux
                    else {
                        output_.print((char)c);
                    }
            }
        }
    }

    inline const char* getLevelString(LogLevel level) {
        switch (level) {
            case LogLevel::ALERT:   return "ALERT";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::DEBUG:   return "DEBUG";
            case LogLevel::SYSTEM:  return "SYSTEM";
            default:                return "UNKNOWN";
        }
    }
};

inline String getSystemStats() {
    char buffer[256];
    #ifdef ESP32
        snprintf(buffer, sizeof(buffer),
                "Heap: %zu bytes (%zu min) Stack: %lu CPU: %.1f°C",
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

class MultiLogger {
public:
    using LogLevel = logger::LogLevel;
    using LogFilter = logger::LogFilter;
    using LogMessage = logger::LogMessage;
    using OutputType = logger::OutputType;
    using TimeProvider = logger::TimeProvider;
    using DefaultTime = logger::DefaultTime;  // Ajout de l'alias manquant

    struct Config {
        TickType_t queueTimeout = pdMS_TO_TICKS(100);
        UBaseType_t taskPriority = 1;
        size_t queueSize = 8;
        LogLevel minLevel = LogLevel::DEBUG;  // Niveau minimum de log
    };

    inline void setConfig(const Config& config) {
        config_ = config;
    }

    static MultiLogger& getInstance() {
        static MultiLogger instance;
        return instance;
    }

    void addOutput(logger::ILogOutput& output) {
        outputs_.push_back(&output);
    }

    bool begin() {
        if (logQueue_ != nullptr) return true;

        logQueue_ = xQueueCreate(config_.queueSize, sizeof(LogMessage));
        if (logQueue_ == nullptr) return false;

        BaseType_t result = xTaskCreate(
            loggerTask,
            "Logger",
            4096,
            this,
            config_.taskPriority,
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
        if (!isLevelEnabled(level)) return true;
        if (logQueue_ == nullptr || file == nullptr || message == nullptr) {
            return false;
        }

        LogMessage msg = {
            .level = level,
            .line = line,
            .timestamp = timeProvider_.current->getTimestamp(),
            .uptime = timeProvider_.current->getMillis()
        };

        // Copie sécurisée des chaînes
        strncpy(msg.file, getFileName(file), MAX_FILENAME - 1);
        msg.file[MAX_FILENAME - 1] = '\0';

        strncpy(msg.message, message, MAX_MESSAGE - 1);
        msg.message[MAX_MESSAGE - 1] = '\0';

        // Gestion sécurisée du nom de tâche
        TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
        const char* taskName = currentTask ? pcTaskGetName(currentTask) : nullptr;
        if (taskName) {
            strncpy(msg.taskName, taskName, sizeof(msg.taskName) - 1);
            msg.taskName[sizeof(msg.taskName) - 1] = '\0';
        } else {
            strcpy(msg.taskName, "unknown");
        }

        if (xQueueSend(logQueue_, &msg, config_.queueTimeout) != pdTRUE) {
            return false;
        }

        return true;
    }

    inline void setTimeProvider(TimeProvider& provider) {
        timeProvider_.current = &provider;
    }

    inline void setLogFilter(LogFilter filter) {
        logFilter_ = filter;
    }

    inline bool isLevelEnabled(LogLevel level) const {
        return isLevelEnabled(level, logFilter_);
    }

    bool isLevelEnabled(LogLevel level, LogFilter filter) const {
        switch (filter) {
            case LogFilter::NONE:
                return false;
            case LogFilter::ALERTS:
                return level == LogLevel::ALERT || level == LogLevel::WARNING;
            case LogFilter::ALL:
            default:
                return true;
        }
    }

    ~MultiLogger() {
        // Attendre que tous les messages soient traités
        if (logQueue_) {
            const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);  // 1 seconde max
            TickType_t xTimeWaited = 0;
            
            while (uxQueueMessagesWaiting(logQueue_) > 0 && xTimeWaited < xMaxBlockTime) {
                vTaskDelay(pdMS_TO_TICKS(10));
                xTimeWaited += pdMS_TO_TICKS(10);
            }

            // Nettoyage des ressources
            vQueueDelete(logQueue_);
            logQueue_ = nullptr;
        }

        if (loggerTaskHandle_) {
            vTaskDelete(loggerTaskHandle_);
            loggerTaskHandle_ = nullptr;
        }

        // Vider les buffers
        memset(formatBuffer_, 0, FORMAT_BUFFER);
        memset(timestampBuffer_, 0, TIMESTAMP_BUFFER_SIZE);
    }

private:
    // Constantes pour les tailles de buffer
    static constexpr size_t MAX_FILENAME = 64;
    static constexpr size_t MAX_MESSAGE = 128;
    static constexpr size_t MAX_TASKNAME = 16;
    static constexpr size_t FORMAT_BUFFER = 512;
    static constexpr size_t TIMESTAMP_BUFFER_SIZE = 32;

    // Buffers réutilisables
    char formatBuffer_[FORMAT_BUFFER];
    char timestampBuffer_[TIMESTAMP_BUFFER_SIZE];

    QueueHandle_t logQueue_;
    TaskHandle_t loggerTaskHandle_;
    std::vector<logger::ILogOutput*> outputs_;
    Config config_;
    LogFilter logFilter_ = LogFilter::ALL;  // Par défaut, tout logger

    struct StaticTimeProvider {
        DefaultTime defaultTime;     // Maintenant DefaultTime est défini
        TimeProvider* current;
        
        StaticTimeProvider() : defaultTime(), current(&defaultTime) {}
    };

    StaticTimeProvider timeProvider_;

    MultiLogger() 
        : logQueue_(nullptr)
        , loggerTaskHandle_(nullptr)
        , timeProvider_()  // Initialise avec DefaultTime
    {
    }

    static void loggerTask(void* parameter) {
        MultiLogger* logger = static_cast<MultiLogger*>(parameter);
        LogMessage msg;

        while (true) {
            if (xQueueReceive(logger->logQueue_, &msg, portMAX_DELAY) == pdTRUE) {
                logger->processLogMessage(msg);
            }
        }
    }

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

    void processLogMessage(const LogMessage& msg) {
        formatTimestamp(msg.timestamp, msg.uptime, timestampBuffer_, sizeof(timestampBuffer_));
        
        for (auto* output : outputs_) {
            // Vérifier si ce niveau doit être loggé pour cette sortie
            if (!isLevelEnabled(msg.level, output->getLogFilter())) {
                continue;
            }

            switch (output->getType()) {
                case logger::OutputType::JSON:
                    static_cast<logger::JsonLogger*>(output)->sendLogEvent(msg);
                    break;
                    
                case logger::OutputType::CONSOLE:
                    formatAndSendSerial(
                        static_cast<logger::SerialLogger*>(output),
                        msg,
                        timestampBuffer_
                    );
                    break;
                    
                default:
                    formatStandardOutput(output, msg, timestampBuffer_);
                    break;
            }
        }
    }

    void formatAndSendSerial(logger::SerialLogger* output, 
                           const LogMessage& msg,
                           const char* timestamp) {
        output->setColor(msg.level);
        output->print(timestamp);
        output->print(" [");
        output->print(getLevelString(msg.level));
        output->print("] ");
        
        output->resetColor();
        output->print(msg.message);
        output->print(" (");
        output->print(getFileName(msg.file));
        output->print(":");
        output->print(std::to_string(msg.line).c_str());
        output->println(")");
    }

    void formatTimestamp(uint32_t timestamp, uint64_t ms, char* buffer, size_t bufferSize) {
        if (timeProvider_.current->getTimestamp() > 0) {
            time_t t = timestamp;
            struct tm* timeinfo = gmtime(&t);
            int written = snprintf(buffer, bufferSize, 
                    "%04d-%02d-%02d %02d:%02d:%02d.%03lu",
                    timeinfo->tm_year + 1900,
                    timeinfo->tm_mon + 1,
                    timeinfo->tm_mday,
                    timeinfo->tm_hour,
                    timeinfo->tm_min,
                    timeinfo->tm_sec,
                    static_cast<unsigned long>(ms % 1000));
            
            if (written < 0 || written >= bufferSize) {
                strncpy(buffer, "timestamp error", bufferSize - 1);
                buffer[bufferSize - 1] = '\0';
            }
        } else {
            // Utiliser directement l'uptime en millisecondes
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
    
    inline const char* getLevelString(LogLevel level) {
        switch (level) {
            case LogLevel::ALERT:   return "ALERT";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::SYSTEM:  return "SYSTEM";
            default:                return "UNKNOWN";
        }
    }
    
    void formatStandardOutput(logger::ILogOutput* output, 
                            const LogMessage& msg,
                            const char* timestamp) {
        // Format selon le type de sortie
        if (output->getType() == OutputType::SD) {
            // Format CSV pour SD
            snprintf(formatBuffer_, sizeof(formatBuffer_),
                    "%s,%s,%s,%s,%s:%d",
                    timestamp,
                    getLevelString(msg.level),
                    msg.taskName,
                    msg.message,
                    getFileName(msg.file),
                    msg.line);
        } else {
            // Format ASCII pour les autres
            snprintf(formatBuffer_, sizeof(formatBuffer_), 
                    "%s [%s] %s (%s:%d)",
                    timestamp,
                    getLevelString(msg.level),
                    msg.message,
                    getFileName(msg.file),
                    msg.line);
        }
        output->println(formatBuffer_);
    }
    
    MultiLogger(const MultiLogger&) = delete;
    MultiLogger& operator=(const MultiLogger&) = delete;
};
