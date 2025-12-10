#include <Arduino.h> 
#include <WiFi.h> 
#include <HTTPClient.h> 

// *** NOTA IMPORTANTE ***
// Aunque la lógica de red y botón es funcional,
// este código requiere que los archivos del framework Modest IoT (como EventHandler.h)
// estén presentes en tu proyecto para la definición de 'struct Event'.
#include "ModestIoT.h"      

// --- CONFIGURACIÓN WIFI (Wokwi) ---
const char* ssid = "Wokwi-GUEST"; 
const char* password = ""; 

// --- CONFIGURACIÓN DE LA API REST (Beeceptor) ---
const char* host = "rest-api-test-wokwi-jz.free.beeceptor.com";
const char* api_endpoint = "/data-value"; 

// --- CONFIGURACIÓN DE PINES Y DEBOUNCE ---
// ¡Asegúrate de cablear el botón a D4 (GPIO 4)!
#define BUTTON_PIN 26
#define DEBOUNCE_DELAY_MS 200 

// --- VARIABLES GLOBALES DE ESTADO PARA EL DEBOUNCE Y EVENTOS ---
unsigned long lastDebounceTime = 0; 
int lastButtonState = HIGH;         
// Definición simple del evento (requiere EventHandler.h)
const Event BUTTON_PRESSED_EVENT = Event(0); 

// *******************************************************************
// * FUNCIONES *
// *******************************************************************

/**
 * @brief Conecta el ESP32 a la red WiFi.
 */
void connectToWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    
    Serial.print("Conectando a WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nWiFi Conectado!");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.localIP());
}

/**
 * @brief Envía la petición HTTP POST con el estado del botón.
 */
void postButtonState() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi desconectado. Intentando reconectar...");
        connectToWiFi();
        if (WiFi.status() != WL_CONNECTED) return;
    }

    HTTPClient http;
    String serverPath = "http://" + String(host) + String(api_endpoint);
    
    http.begin(serverPath.c_str()); 
    http.addHeader("Content-Type", "application/json");

    String httpRequestData = "{\"event\":\"BUTTON_PRESSED_POLLED\", \"timestamp\":\"" + String(millis()) + "\"}";

    Serial.print("-> POST: ");
    Serial.println(serverPath);
    
    int httpResponseCode = http.POST(httpRequestData);

    if (httpResponseCode > 0) {
        Serial.printf("<- HTTP Response: %d - %s\n", httpResponseCode, http.getString().c_str());
    } else {
        Serial.printf("<- Error POST, Code: %d\n", httpResponseCode);
    }

    http.end();
}


// *******************************************************************
// * ARDUINO SETUP Y LOOP *
// *******************************************************************

void setup() {
    Serial.begin(115200);
    
    // 1. Configuración del pin del botón (INPUT_PULLUP)
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // 2. Conexión WiFi inicial
    connectToWiFi();
    
    Serial.println("Sistema listo. Monitoreando botón mediante Polling en GPIO 4.");
}

void loop() {
    // 1. Manejo de pulsación de botón (Polling con Debounce)
    
    // Lectura del pin actual
    int reading = digitalRead(BUTTON_PIN);
    
    // Comprobar si el estado ha cambiado
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    // Si ha pasado el tiempo de debounce y el estado es BAJO (Botón presionado)
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
        if (reading == LOW) {
            
            Serial.println(">>> Botón Presionado Confirmado! Enviando API POST.");
            
            // Simula la acción del evento (POST HTTP)
            postButtonState(); 
            
            // Esperamos a que el botón sea soltado (bloqueo para evitar POSTs múltiples)
            while (digitalRead(BUTTON_PIN) == LOW) {
                delay(10);
            }
            
            // Actualizamos el tiempo de debounce después de soltar
            lastDebounceTime = millis();
        }
    }
    
    // Guardar el estado actual para la siguiente iteración
    lastButtonState = reading;
    
    // 2. Revisión de conexión
    if (WiFi.status() != WL_CONNECTED) {
        delay(100);
        connectToWiFi();
    }

    delay(10); 
}