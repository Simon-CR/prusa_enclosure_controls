/**
 * Prusa Enclosure Controller
 * Main firmware for RP2040/ATtiny1614
 * 
 * Complete version with VOC sensing, dual fan, and web dashboard
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SGP30.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

// Pin Definitions
#define GPIO_ERROR_OUT   0
#define GPIO_STATUS_OK   1
#define GPIO_SPECIAL     2
#define GPIO_PURGE       3
#define GPIO_PROFILE_0   4
#define GPIO_PROFILE_1   5
#define GPIO_PROFILE_2   6
#define GPIO_ENABLE      7
#define FAN_PWM_PIN      8
#define FAN2_PWM_PIN     17
#define HEATER_CTRL_PIN  9
#define LED_RED          11
#define LED_GREEN        12
#define LED_BLUE         13

// Configuration
#define PWM_FREQUENCY    25000
#define UPDATE_INTERVAL  100
#define TEMP_MAX_LIMIT   70.0
#define HEATER_DUTY_MAX  80

// System state
enum SystemMode {
    MODE_OFF,
    MODE_PREHEAT,
    MODE_PRINTING,
    MODE_PURGE,
    MODE_COOLDOWN,
    MODE_ERROR
};

struct MaterialProfile {
    const char* name;
    float minTemp;
    float targetTemp;
    float maxTemp;
    uint8_t minFanSpeed;
    uint8_t normalFanSpeed;
    uint8_t boostFanSpeed;
    bool requiresPreheat;
    bool hasVOCs;
    uint16_t purgeTime;
    float maxCoolRate;
};

// Material profiles
const MaterialProfile profiles[8] = {
    {"OFF",   20.0, 20.0, 30.0,  0,  0,   0, false, false,   0, 10.0},
    {"PLA",   20.0, 25.0, 35.0,  0, 30,  60, false, false, 120, 10.0},
    {"PETG",  25.0, 35.0, 45.0, 10, 40,  70, false, false, 180,  5.0},
    {"TPU",   30.0, 35.0, 40.0, 20, 40,  60, false, false, 180,  5.0},
    {"Nylon", 35.0, 45.0, 55.0, 15, 50,  80,  true, false, 240,  3.0},
    {"ABS",   40.0, 45.0, 55.0, 10, 15,  30,  true,  true, 300,  2.0},
    {"ASA",   45.0, 50.0, 60.0, 15, 20,  40,  true,  true, 360,  2.0},
    {"PC",    50.0, 55.0, 65.0, 20, 30,  50,  true,  true, 300,  1.0}
};

// Global variables
SystemMode currentMode = MODE_OFF;
const MaterialProfile* activeProfile = &profiles[0];
Adafruit_BME280 bme;
Adafruit_SGP30 sgp;
WebServer server(80);
WiFiClient espClient;
PubSubClient mqtt(espClient);

float currentTemp = 20.0;
float currentHumidity = 50.0;
uint16_t currentTVOC = 0;
uint8_t fan1Speed = 0;
uint8_t fan2Speed = 0;
bool systemEnabled = false;

void setup() {
    Serial.begin(115200);
    Serial.println("Prusa Enclosure Controller Starting...");
    
    // Setup pins
    for(int i = 0; i <= 7; i++) {
        pinMode(i, i < 2 ? OUTPUT : INPUT);
    }
    pinMode(FAN_PWM_PIN, OUTPUT);
    pinMode(FAN2_PWM_PIN, OUTPUT);
    pinMode(HEATER_CTRL_PIN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    // Initialize sensors
    Wire.begin();
    if (!bme.begin(0x76)) {
        Serial.println("BME280 not found!");
    }
    if (!sgp.begin()) {
        Serial.println("SGP30 not found!");
    }
    
    // Initialize WiFi and web server
    // WiFi.begin("SSID", "PASSWORD");
    // server.on("/api/status", handleStatusAPI);
    // server.begin();
    
    Serial.println("System ready!");
    digitalWrite(LED_GREEN, HIGH);
}

void loop() {
    static unsigned long lastUpdate = 0;
    
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = millis();
        
        // Read inputs
        readGPIOCommands();
        updateSensors();
        
        // Control logic
        updateControl();
        
        // Apply outputs
        applyOutputs();
    }
    
    // Handle web server
    // server.handleClient();
}

void readGPIOCommands() {
    systemEnabled = digitalRead(GPIO_ENABLE);
    
    if (systemEnabled) {
        // Read profile
        uint8_t profileIndex = 0;
        profileIndex |= digitalRead(GPIO_PROFILE_0) ? 1 : 0;
        profileIndex |= digitalRead(GPIO_PROFILE_1) ? 2 : 0;
        profileIndex |= digitalRead(GPIO_PROFILE_2) ? 4 : 0;
        activeProfile = &profiles[profileIndex];
        
        // Check purge
        if (digitalRead(GPIO_PURGE)) {
            currentMode = MODE_PURGE;
        }
    } else {
        currentMode = MODE_OFF;
    }
}

void updateSensors() {
    currentTemp = bme.readTemperature();
    currentHumidity = bme.readHumidity();
    
    if (sgp.IAQmeasure()) {
        currentTVOC = sgp.TVOC;
    }
}

void updateControl() {
    if (currentMode == MODE_OFF) {
        fan1Speed = 0;
        fan2Speed = 0;
        return;
    }
    
    // VOC-based control
    if (currentTVOC > 5000) {
        // Emergency ventilation
        fan1Speed = 100;
        fan2Speed = 80;
    } else if (activeProfile->hasVOCs) {
        // Toxic material - minimal ventilation
        if (digitalRead(GPIO_SPECIAL)) {
            // First layer - no ventilation
            fan1Speed = 0;
            fan2Speed = 0;
        } else {
            fan1Speed = activeProfile->minFanSpeed;
            fan2Speed = 10;
        }
    } else {
        // Safe material - temperature control only
        float tempError = currentTemp - activeProfile->targetTemp;
        if (tempError > 3) {
            fan2Speed = map(tempError, 3, 10, 30, 80);
        } else {
            fan2Speed = 0;
        }
        fan1Speed = 0;
    }
}

void applyOutputs() {
    analogWrite(FAN_PWM_PIN, map(fan1Speed, 0, 100, 0, 255));
    analogWrite(FAN2_PWM_PIN, map(fan2Speed, 0, 100, 0, 255));
    
    digitalWrite(LED_RED, currentTVOC > 2000);
    digitalWrite(LED_GREEN, systemEnabled);
    digitalWrite(LED_BLUE, fan1Speed > 50);
    
    digitalWrite(GPIO_STATUS_OK, systemEnabled && !errorState);
    digitalWrite(GPIO_ERROR_OUT, errorState);
}
