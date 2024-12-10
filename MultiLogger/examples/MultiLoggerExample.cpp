#include "../MultiLogger.h"

// Simulation d'une tâche de surveillance de batterie
void batteryTask(void* parameter) {
    int batteryLevel = 100;
    
    while (true) {
        if (batteryLevel < 20) {
            LOG_WARNING("Niveau batterie critique: %d%%", batteryLevel);
        } else if (batteryLevel < 50) {
            LOG_DEBUG("Batterie en dessous de 50%%: %d%%", batteryLevel);
        } else {
            LOG_SYSTEM("Batterie OK: %d%%", batteryLevel);
        }
        
        batteryLevel -= 10;  // Simulation de décharge
        if (batteryLevel < 0) batteryLevel = 100;
        
        vTaskDelay(pdMS_TO_TICKS(2000));  // Attente de 2 secondes
    }
}

// Simulation d'une tâche de surveillance de température
void temperatureTask(void* parameter) {
    float temperature = 25.0;
    bool isHeating = true;
    
    while (true) {
        if (temperature > 80.0) {
            LOG_ALERT("TEMPÉRATURE CRITIQUE: %.1f°C - ARRÊT IMMÉDIAT!", temperature);
        } else if (temperature > 60.0) {
            LOG_WARNING("Température élevée: %.1f°C", temperature);
        } else {
            LOG_SYSTEM("Température normale: %.1f°C", temperature);
        }
        
        // Simulation variation température
        if (isHeating) {
            temperature += 5.0;
            if (temperature > 85.0) isHeating = false;
        } else {
            temperature -= 5.0;
            if (temperature < 25.0) isHeating = true;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1500));  // Attente de 1.5 secondes
    }
}

// Simulation d'une tâche de traitement
void processTask(void* parameter) {
    int processCount = 0;
    
    while (true) {
        processCount++;
        LOG_DEBUG("Traitement #%d en cours...", processCount);
        
        // Simulation d'une erreur aléatoire
        if (processCount % 10 == 0) {
            LOG_ALERT("Erreur lors du traitement #%d!", processCount);
        } else if (processCount % 5 == 0) {
            LOG_WARNING("Traitement #%d plus long que prévu", processCount);
        } else {
            LOG_DEBUG("Traitement #%d terminé avec succès", processCount);
        }
        
        vTaskDelay(pdMS_TO_TICKS(3000));  // Attente de 3 secondes
    }
}

void setup() {
    // Le logger est automatiquement initialisé grâce au Singleton
    
    LOG_SYSTEM("=== Démarrage du système ===");
    LOG_DEBUG("Création des tâches...");
    
    // Création des tâches
    xTaskCreate(
        batteryTask,
        "Battery",
        2048,
        nullptr,
        1,
        nullptr
    );
    
    xTaskCreate(
        temperatureTask,
        "Temperature",
        2048,
        nullptr,
        1,
        nullptr
    );
    
    xTaskCreate(
        processTask,
        "Process",
        2048,
        nullptr,
        1,
        nullptr
    );
    
    LOG_DEBUG("Toutes les tâches sont créées et démarrées");
    LOG_SYSTEM("=== Système prêt ===");
}

void loop() {
    // La boucle principale reste vide car tout est géré par les tâches
    vTaskDelay(portMAX_DELAY);
} 