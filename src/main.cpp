/**
 * @file sketch.ino
 * @brief Ember Mug 3 IoT Device Prototype. Implements temperature control, 
 * ACTIVE/STAND_BY modes, and JSON status updates for the final exam.
 * * NOTA: La conexión WiFi se simula para evitar bloqueo en el setup().
 * * @author Jose Zarate
 * @date December 10, 2025
 * @version 1.0
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h> // Library for JSON formatting [cite: 84]
#include <OneWire.h>     // Library for DS18B20 [cite: 85]
#include <DallasTemperature.h> // Library for DS18B20 [cite: 86]

// --- CONSTANTES Y PINOUT ---
#define SERIAL_BAUD 115200

// Pines de Actuadores y Sensores (Utilizados en el diagrama)
const int HEATER_RELAY_PIN = 2;   // D2 - Heater Relay Module [cite: 26]
const int TEMP_SENSOR_PIN = 4;    // D4 - Data Pin for DS18B20 [cite: 24]
const int BUTTON_PLUS_PIN = 13;   // D13 - Button for increment (+) [cite: 24]
const int BUTTON_MINUS_PIN = 12;  // D12 - Button for decrement (-) [cite: 24]
const int TRIG_PIN = 25;          // D25 - HC-SR04 Trigger [cite: 25]
const int ECHO_PIN = 26;          // D26 - HC-SR04 Echo [cite: 25]

// Pines del LED Bar Graph (10 barras) [cite: 24]
const int LED_BAR_PINS[] = {15, 16, 17, 5, 18, 19, 21, 22, 23, 27}; 
const int NUM_BARS = 10;

// Constantes de Operación (Valores del enunciado)
const float MIN_IDEAL_TEMP = 45.0; // Temperatura mínima ideal [cite: 29]
const float MAX_IDEAL_TEMP = 63.5; // Temperatura máxima ideal [cite: 29]
const float TEMP_INCREMENT = 0.5;  // Incremento/decremento por botón [cite: 29]
const int STANDBY_DISTANCE_CM = 18; // 18 cm o más para STAND_BY [cite: 38]
const long UPDATE_INTERVAL_MS = 5000; // Intervalo de actualización JSON (5 segundos) [cite: 43]

// --- VARIABLES GLOBALES DE ESTADO ---
float idealTemperature = MIN_IDEAL_TEMP;
float currentTemperature = 0.0;
float waterLevelDistance = 0.0; 
enum OperationMode { STAND_BY, ACTIVE };
OperationMode operationMode = STAND_BY;
const String SIMULATED_MAC_ADDRESS = "A0:20:A6:01:23:45"; // MAC Address simulada 

// Variables de timing y debounce
volatile unsigned long lastButtonPressTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 200;
unsigned long lastUpdateTime = 0;

// --- INSTANCIACIÓN DE LIBRERÍAS ---
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

// *******************************************************************
// * SENSOR Y ACTUADOR FUNCTIONS *
// *******************************************************************

float readCurrentTemperature() {
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    return (tempC != DEVICE_DISCONNECTED_C) ? tempC : 0.0;
}

float readWaterLevelDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    return duration * 0.0343 / 2;
}

void setHeaterState(int state) {
    digitalWrite(HEATER_RELAY_PIN, state);
}

void updateLedBarGraph() {
    float range = MAX_IDEAL_TEMP - MIN_IDEAL_TEMP;
    float tempDifference = idealTemperature - MIN_IDEAL_TEMP; 
    
    // Cálculo de barras a encender (0 a 10) [cite: 30]
    int barsToLight = round(tempDifference * ((float)NUM_BARS / range));
    barsToLight = constrain(barsToLight, 0, NUM_BARS); 

    for (int i = 0; i < NUM_BARS; i++) {
        digitalWrite(LED_BAR_PINS[i], (i < barsToLight) ? HIGH : LOW);
    }
}


// *******************************************************************
// * DEVICE LOGIC AND CONTROL *
// *******************************************************************

void applyTemperatureRules() {
    currentTemperature = readCurrentTemperature();

    // Comprobar si la bebida está en el nivel ideal [cite: 35]
    if (abs(currentTemperature - idealTemperature) <= TEMP_INCREMENT / 2.0) {
        // En ideal: Apagar calentador (si está encendido, para mantener)
        setHeaterState(LOW); 
    } else if (currentTemperature < idealTemperature) {
        // Debajo del ideal: Encender calentador.
        setHeaterState(HIGH); 
    } else {
        // Por encima del ideal: Apagar calentador [cite: 36]
        setHeaterState(LOW); 
    }
}

void checkWaterLevelAndMode() {
    waterLevelDistance = readWaterLevelDistance();

    if (waterLevelDistance >= STANDBY_DISTANCE_CM) {
        // No hay líquido (18 cm o más) -> STAND_BY [cite: 38]
        if (operationMode != STAND_BY) {
            Serial.println("Operation Mode: STAND_BY (No liquid detected)");
            operationMode = STAND_BY;
            setHeaterState(LOW); // Dejar de calentar y verificar reglas [cite: 38]
        }
    } else {
        // Hay líquido -> ACTIVE [cite: 39]
        if (operationMode != ACTIVE) {
            Serial.println("Operation Mode: ACTIVE (Liquid detected)");
            operationMode = ACTIVE;
        }
    }
}

/**
 * @brief Envía el estado de operación del dispositivo a la consola en formato JSON cada 5 segundos. [cite: 43]
 */
void sendStatusUpdate() {
    if (millis() - lastUpdateTime >= UPDATE_INTERVAL_MS) {
        lastUpdateTime = millis();
        
        StaticJsonDocument<256> doc;
        
        // Estructura JSON requerida [cite: 44, 45]
        doc["deviceMacAddress"] = SIMULATED_MAC_ADDRESS;
        doc["operationMode"] = (operationMode == ACTIVE) ? "ACTIVE" : "STAND_BY";
        doc["idealTemperature"] = idealTemperature;
        doc["currentTemperature"] = currentTemperature;
        doc["waterLevelDistance"] = waterLevelDistance;
        doc["createdAt"] = millis(); 
        
        Serial.print("Status Update: ");
        serializeJson(doc, Serial);
        Serial.println();
    }
}


// *******************************************************************
// * ISRs (INTERRUPT SERVICE ROUTINES) *
// *******************************************************************

void IRAM_ATTR buttonPlusISR() {
    unsigned long now = millis();
    if (now - lastButtonPressTime > DEBOUNCE_DELAY_MS) {
        idealTemperature = min(idealTemperature + TEMP_INCREMENT, MAX_IDEAL_TEMP);
        updateLedBarGraph();
        Serial.printf("Ideal Temperature set to: %.1f°C\n", idealTemperature);
        lastButtonPressTime = now;
    }
}

void IRAM_ATTR buttonMinusISR() {
    unsigned long now = millis();
    if (now - lastButtonPressTime > DEBOUNCE_DELAY_MS) {
        idealTemperature = max(idealTemperature - TEMP_INCREMENT, MIN_IDEAL_TEMP);
        updateLedBarGraph();
        Serial.printf("Ideal Temperature set to: %.1f°C\n", idealTemperature);
        lastButtonPressTime = now;
    }
}


// *******************************************************************
// * ARDUINO SETUP AND LOOP *
// *******************************************************************

void setup() {
    Serial.begin(SERIAL_BAUD);

    // Información inicial requerida [cite: 57]
    Serial.println("\n--------------------------------------------------");
    Serial.println(" Ember Mug 3 - IoT Device Prototype Initialization");
    Serial.println(" Developer Member: Jose Zarate (Developer Member of Ember, inc.)");
    Serial.println("--------------------------------------------------\n");

    // Configuración de Pines
    pinMode(HEATER_RELAY_PIN, OUTPUT);
    setHeaterState(LOW); 
    
    for (int pin : LED_BAR_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    sensors.begin();

    // SIMULACIÓN DE CONEXIÓN WIFI
    // La línea que originalmente iniciaba la conexión (WiFi.begin) se elimina para evitar el bloqueo.
    Serial.println("WiFi: Conexión simulada exitosamente. MAC: " + SIMULATED_MAC_ADDRESS);
    
    // Configuración de Interrupciones para botones
    pinMode(BUTTON_PLUS_PIN, INPUT_PULLUP);
    pinMode(BUTTON_MINUS_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PLUS_PIN), buttonPlusISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_MINUS_PIN), buttonMinusISR, FALLING);
    
    // Estado inicial
    updateLedBarGraph(); 
    Serial.printf("Initial Ideal Temperature: %.1f°C\n", idealTemperature);
}

void loop() {
    // 1. Verificar el nivel de líquido y actualizar el modo de operación
    checkWaterLevelAndMode();
    
    // 2. Ejecutar la lógica de control solo en modo ACTIVO
    if (operationMode == ACTIVE) {
        applyTemperatureRules();
    } else {
        setHeaterState(LOW); 
    }
    
    // 3. Enviar actualización de estado JSON
    sendStatusUpdate();
}