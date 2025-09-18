/**
 * Prusa Enclosure Controller - Production Version
 * Aligned with Prusa Core ONE enclosure implementation
 * For RP2040/ATtiny1614 with VOC sensing and dual fan control
 * 
 * Author: Simon-CR
 * Date: 2025-01-18
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SGP30.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// ============================================================================
// Pin Definitions - Hackerboard Interface
// ============================================================================
#define GPIO_ERROR_OUT   0  // To printer: error state
#define GPIO_STATUS_OK   1  // To printer: chamber ready for print
#define GPIO_SPECIAL     2  // From printer: first layer mode
#define GPIO_PURGE       3  // From printer: purge command
#define GPIO_PROFILE_0   4  // From printer: profile bit 0
#define GPIO_PROFILE_1   5  // From printer: profile bit 1
#define GPIO_PROFILE_2   6  // From printer: profile bit 2
#define GPIO_ENABLE      7  // From printer: system enable

// Control outputs
#define FAN1_PWM_PIN     8  // Prusa filtration system control
#define FAN2_PWM_PIN     17 // Internal circulation fan
#define HEATER_CTRL_PIN  9  // Heater on/off
#define HEATER_PWM_PIN   10 // Heater PWM (for PTC)

// Status LEDs
#define LED_RED          11 // Error/heating
#define LED_GREEN        12 // Ready/OK
#define LED_BLUE         13 // Cooling/purge active

// I2C pins (RP2040 defaults)
#define I2C_SDA          14
#define I2C_SCL          15

// Extension board monitor
#define EXT_FAN_MON      16 // Monitor original fan request

// ============================================================================
// Configuration Constants
// ============================================================================
#define PWM_FREQUENCY    25000  // 25kHz for fan PWM
#define UPDATE_INTERVAL  100    // Main loop update (ms)
#define SENSOR_INTERVAL  1000   // Sensor read interval (ms)
#define MQTT_INTERVAL    10000  // MQTT publish interval (ms)
#define TEMP_AVG_SAMPLES 10     // Temperature averaging
#define VOC_HISTORY_SIZE 10     // VOC history for rate calculation

// Safety limits
#define TEMP_MAX_LIMIT   70.0   // Absolute maximum temperature
#define TEMP_MIN_LIMIT   10.0   // Absolute minimum (sensor fail)
#define HEATER_DUTY_MAX  80     // Maximum heater duty cycle (%)
#define HEATER_TIMEOUT   1200000 // Max heater runtime (20 min)
#define PREHEAT_TIMEOUT  1800000 // Max preheat time (30 min)
#define PURGE_MAX_TIME   600000  // Max purge time (10 min)

// VOC thresholds (ppb) - Based on real-world testing
#define VOC_BASELINE     400    // Clean air baseline
#define VOC_ELEVATED     1000   // Noticeable odor
#define VOC_HIGH         2000   // Ventilation needed
#define VOC_DANGEROUS    5000   // Immediate action
#define VOC_EMERGENCY    10000  // System shutdown

// ============================================================================
// System State Definitions
// ============================================================================
enum SystemMode {
    MODE_OFF,
    MODE_PREHEAT,    // Waiting for chamber temperature
    MODE_READY,      // At temperature, waiting for print
    MODE_PRINTING,   // Actively printing
    MODE_PURGE,      // Post-print VOC purge
    MODE_COOLDOWN,   // Controlled cooldown
    MODE_ERROR       // Error state
};

// Material profiles - Aligned with Prusa Core ONE specifications
struct MaterialProfile {
    const char* name;
    float waitTemp;          // Temperature to wait for before print (M191 equivalent)
    float targetTemp;        // Target chamber temperature (M141 equivalent)
    float maxTemp;           // Maximum safe temperature
    uint8_t minFanSpeed;     // Minimum fan for VOCs (Prusa uses 10-20%)
    uint8_t normalFanSpeed;  // Normal operation fan
    uint8_t boostFanSpeed;   // Cooling boost fan speed
    uint8_t firstLayers;     // Number of layers to protect from drafts
    bool requiresPreheat;    // Needs chamber preheating
    bool hasVOCs;           // Material produces toxic fumes
    uint16_t purgeTime;      // Post-print purge duration (seconds)
    float maxCoolRate;       // Maximum cooling rate (°C/min) to prevent warping
};

// Profiles matching Prusa's actual Core ONE implementation
const MaterialProfile profiles[8] = {
    // Profile 0: System OFF
    {"OFF",     20.0, 20.0, 30.0,  0,  0,   0,  0, false, false,   0, 10.0},
    
    // Profile 1: PLA - Keep cool to prevent heat creep
    {"PLA",     20.0, 25.0, 30.0,  0, 50, 100,  1, false, false, 120, 10.0},
    
    // Profile 2: PETG - Mild chamber temperature
    {"PETG",    25.0, 30.0, 40.0, 10, 30,  50,  2, false, false, 180,  5.0},
    
    // Profile 3: TPU - Similar to PETG
    {"TPU",     25.0, 30.0, 35.0, 10, 30,  40,  2, false, false, 180,  5.0},
    
    // Profile 4: Nylon - Needs dry, warm air
    {"Nylon",   35.0, 40.0, 50.0, 10, 30,  50,  3,  true, false, 240,  3.0},
    
    // Profile 5: ABS - Prusa's 40°C chamber spec
    {"ABS",     35.0, 40.0, 50.0, 10, 15,  20,  4,  true,  true, 300,  2.0},
    
    // Profile 6: ASA - Slightly higher than ABS
    {"ASA",     38.0, 45.0, 55.0, 10, 15,  20,  5,  true,  true, 300,  2.0},
    
    // Profile 7: PC - Highest temperature requirement
    {"PC",      45.0, 50.0, 60.0,  5, 10,  15,  6,  true,  true, 300,  1.0}
};

// ============================================================================
// Global Variables
// ============================================================================

// System state
SystemMode currentMode = MODE_OFF;
SystemMode previousMode = MODE_OFF;
const MaterialProfile* activeProfile = &profiles[0];
bool systemEnabled = false;
bool errorState = false;
bool chamberReady = false;
uint8_t currentLayer = 0;

// Sensors
Adafruit_BME280 bme;
Adafruit_SGP30 sgp;
bool bmePresent = false;
bool sgpPresent = false;

// Temperature control
float currentTemp = 20.0;
float currentHumidity = 50.0;
float currentPressure = 1013.0;
float tempHistory[TEMP_AVG_SAMPLES];
uint8_t tempHistoryIndex = 0;
float tempRateOfChange = 0.0;

// VOC monitoring
uint16_t currentTVOC = 0;
uint16_t currentCO2eq = 400;
uint16_t vocHistory[VOC_HISTORY_SIZE];
uint8_t vocHistoryIndex = 0;
float vocRateOfChange = 0.0;
uint16_t vocBaseline = 400;

// Fan control
uint8_t fan1Speed = 0;  // Prusa filtration system
uint8_t fan2Speed = 0;  // Internal circulation
uint8_t targetFan1Speed = 0;
uint8_t targetFan2Speed = 0;

// Heater control
bool heaterEnabled = false;
uint8_t heaterDuty = 0;
unsigned long heaterStartTime = 0;
unsigned long preheatStartTime = 0;

// Purge control
bool purgeActive = false;
unsigned long purgeStartTime = 0;

// Timing
unsigned long lastUpdate = 0;
unsigned long lastSensorRead = 0;
unsigned long lastSecond = 0;
unsigned long systemStartTime = 0;

// PID controller for temperature
float pidSetpoint = 20.0;
float pidError = 0.0;
float pidLastError = 0.0;
float pidIntegral = 0.0;
float pidDerivative = 0.0;
float pidOutput = 0.0;

// PID tuning (conservative for chamber control)
const float PID_KP = 2.5;
const float PID_KI = 0.1;
const float PID_KD = 1.0;

// Network
WebServer server(80);
WiFiClient espClient;
PubSubClient mqtt(espClient);

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("Prusa Enclosure Controller v2.0");
    Serial.println("Aligned with Core ONE Implementation");
    Serial.println("========================================\n");
    
    // Initialize EEPROM
    EEPROM.begin(512);
    
    // Setup pins
    setupPins();
    
    // Initialize sensors
    setupSensors();
    
    // Initialize temperature history
    for (int i = 0; i < TEMP_AVG_SAMPLES; i++) {
        tempHistory[i] = 20.0;
    }
    
    // Initialize VOC history
    for (int i = 0; i < VOC_HISTORY_SIZE; i++) {
        vocHistory[i] = 400;
    }
    
    // Load saved baselines if available
    loadBaselines();
    
    // Setup network (optional)
    // setupNetwork();
    
    systemStartTime = millis();
    Serial.println("System initialization complete.\n");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    unsigned long now = millis();
    
    // Fast update cycle (100ms)
    if (now - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = now;
        
        // Read GPIO commands from printer
        readGPIOCommands();
        
        // Update control logic
        updateControl();
        
        // Apply outputs with ramping
        applyOutputs();
        
        // Update status indicators
        updateStatusLEDs();
    }
    
    // Sensor update cycle (1 second)
    if (now - lastSensorRead >= SENSOR_INTERVAL) {
        lastSensorRead = now;
        
        // Read sensors
        updateSensors();
        
        // Calculate rates of change
        updateRates();
        
        // Safety checks
        performSafetyChecks();
        
        // Debug output
        printStatus();
    }
    
    // Handle web server
    // server.handleClient();
    
    // Handle MQTT
    // if (mqtt.connected()) {
    //     mqtt.loop();
    //     publishMQTTStatus();
    // }
}

// ============================================================================
// Pin Configuration
// ============================================================================

void setupPins() {
    Serial.println("Configuring GPIO pins...");
    
    // Hackerboard interface
    pinMode(GPIO_ERROR_OUT, OUTPUT);
    pinMode(GPIO_STATUS_OK, OUTPUT);
    pinMode(GPIO_SPECIAL, INPUT);
    pinMode(GPIO_PURGE, INPUT);
    pinMode(GPIO_PROFILE_0, INPUT);
    pinMode(GPIO_PROFILE_1, INPUT);
    pinMode(GPIO_PROFILE_2, INPUT);
    pinMode(GPIO_ENABLE, INPUT);
    
    // Set safe defaults
    digitalWrite(GPIO_ERROR_OUT, LOW);
    digitalWrite(GPIO_STATUS_OK, LOW);
    
    // Control outputs
    pinMode(FAN1_PWM_PIN, OUTPUT);
    pinMode(FAN2_PWM_PIN, OUTPUT);
    pinMode(HEATER_CTRL_PIN, OUTPUT);
    pinMode(HEATER_PWM_PIN, OUTPUT);
    
    // Status LEDs
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    // Extension board monitor
    pinMode(EXT_FAN_MON, INPUT);
    
    // Configure PWM frequency
    #ifdef ESP32
    ledcSetup(0, PWM_FREQUENCY, 8);
    ledcSetup(1, PWM_FREQUENCY, 8);
    ledcAttachPin(FAN1_PWM_PIN, 0);
    ledcAttachPin(FAN2_PWM_PIN, 1);
    #else
    analogWriteFreq(PWM_FREQUENCY);
    #endif
    
    Serial.println("GPIO configuration complete.");
}

// ============================================================================
// Sensor Setup
// ============================================================================

void setupSensors() {
    Serial.println("Initializing sensors...");
    
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Initialize BME280
    if (bme.begin(0x76) || bme.begin(0x77)) {
        bmePresent = true;
        Serial.println("✓ BME280 detected");
        
        // Configure for indoor navigation
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                       Adafruit_BME280::SAMPLING_X2,   // temp
                       Adafruit_BME280::SAMPLING_X16,  // pressure
                       Adafruit_BME280::SAMPLING_X1,   // humidity
                       Adafruit_BME280::FILTER_X4,
                       Adafruit_BME280::STANDBY_MS_125);
    } else {
        Serial.println("✗ BME280 not found - temperature control limited!");
        errorState = true;
    }
    
    // Initialize SGP30
    if (sgp.begin()) {
        sgpPresent = true;
        Serial.println("✓ SGP30 VOC sensor detected");
        
        // Initialize baseline
        if (vocBaseline > 0 && vocBaseline < 60000) {
            sgp.setIAQBaseline(0x8E68, vocBaseline);
        }
    } else {
        Serial.println("✗ SGP30 not found - VOC monitoring disabled");
        // Not critical - can run without VOC sensor
    }
}

// ============================================================================
// GPIO Command Reading
// ============================================================================

void readGPIOCommands() {
    // Read system enable
    bool newEnable = digitalRead(GPIO_ENABLE);
    
    if (newEnable != systemEnabled) {
        systemEnabled = newEnable;
        
        if (!systemEnabled) {
            // System disabled - immediate safe shutdown
            Serial.println("System DISABLED by G-code");
            currentMode = MODE_OFF;
            chamberReady = false;
            digitalWrite(GPIO_STATUS_OK, LOW);
        } else {
            Serial.println("System ENABLED by G-code");
            currentMode = MODE_PREHEAT;
            preheatStartTime = millis();
        }
    }
    
    if (!systemEnabled) {
        return;  // Skip other commands when disabled
    }
    
    // Read material profile (3 bits)
    uint8_t profileIndex = 0;
    if (digitalRead(GPIO_PROFILE_0)) profileIndex |= 1;
    if (digitalRead(GPIO_PROFILE_1)) profileIndex |= 2;
    if (digitalRead(GPIO_PROFILE_2)) profileIndex |= 4;
    
    // Update profile if changed
    if (activeProfile != &profiles[profileIndex]) {
        activeProfile = &profiles[profileIndex];
        Serial.print("Profile changed: ");
        Serial.println(activeProfile->name);
        
        // Reset chamber ready flag for new profile
        chamberReady = false;
        
        // Start preheat if required
        if (activeProfile->requiresPreheat && currentMode != MODE_PREHEAT) {
            currentMode = MODE_PREHEAT;
            preheatStartTime = millis();
            Serial.println("Starting preheat sequence...");
        }
    }
    
    // Read special mode (first layer protection)
    if (digitalRead(GPIO_SPECIAL)) {
        // First layer mode active
        currentLayer = 1;  // Reset to first layer
    }
    
    // Read purge command
    if (digitalRead(GPIO_PURGE) && !purgeActive) {
        Serial.println("PURGE command received");
        startPurge();
    }
}

// ============================================================================
// Sensor Updates
// ============================================================================

void updateSensors() {
    if (bmePresent) {
        float newTemp = bme.readTemperature();
        
        // Sanity check
        if (newTemp > TEMP_MIN_LIMIT && newTemp < TEMP_MAX_LIMIT) {
            currentTemp = newTemp;
            currentHumidity = bme.readHumidity();
            currentPressure = bme.readPressure() / 100.0F;
            
            // Update rolling average
            tempHistory[tempHistoryIndex] = currentTemp;
            tempHistoryIndex = (tempHistoryIndex + 1) % TEMP_AVG_SAMPLES;
        }
    }
    
    if (sgpPresent) {
        if (sgp.IAQmeasure()) {
            currentTVOC = sgp.TVOC;
            currentCO2eq = sgp.eCO2;
            
            // Update VOC history
            vocHistory[vocHistoryIndex] = currentTVOC;
            vocHistoryIndex = (vocHistoryIndex + 1) % VOC_HISTORY_SIZE;
        }
    }
}

// ============================================================================
// Rate Calculations
// ============================================================================

void updateRates() {
    static float lastTemp = currentTemp;
    static uint16_t lastVOC = currentTVOC;
    
    // Temperature rate of change (°C/min)
    tempRateOfChange = (currentTemp - lastTemp) * 60.0;
    lastTemp = currentTemp;
    
    // VOC rate of change (ppb/min)
    vocRateOfChange = (float)(currentTVOC - lastVOC) * 60.0;
    lastVOC = currentTVOC;
}

// ============================================================================
// Main Control Logic
// ============================================================================

void updateControl() {
    switch (currentMode) {
        case MODE_OFF:
            controlOff();
            break;
            
        case MODE_PREHEAT:
            controlPreheat();
            break;
            
        case MODE_READY:
            controlReady();
            break;
            
        case MODE_PRINTING:
            controlPrinting();
            break;
            
        case MODE_PURGE:
            controlPurge();
            break;
            
        case MODE_COOLDOWN:
            controlCooldown();
            break;
            
        case MODE_ERROR:
            controlError();
            break;
    }
}

// ============================================================================
// Control Mode Implementations
// ============================================================================

void controlOff() {
    // System off - all outputs disabled
    targetFan1Speed = 0;
    targetFan2Speed = 0;
    heaterDuty = 0;
    heaterEnabled = false;
}

void controlPreheat() {
    // Preheat mode - reach chamber temperature before allowing print
    
    // Check for timeout
    if (millis() - preheatStartTime > PREHEAT_TIMEOUT) {
        Serial.println("ERROR: Preheat timeout!");
        currentMode = MODE_ERROR;
        return;
    }
    
    float targetTemp = activeProfile->waitTemp;
    float tempError = targetTemp - currentTemp;
    
    // Aggressive heating to reach temperature quickly
    if (tempError > 5.0) {
        // Far from target - maximum heating
        heaterDuty = HEATER_DUTY_MAX;
        targetFan1Speed = 0;  // No ventilation
        targetFan2Speed = 20; // Light circulation for even heating
        
    } else if (tempError > 2.0) {
        // Getting closer - moderate heating
        heaterDuty = 60;
        targetFan1Speed = 0;
        targetFan2Speed = 30;
        
    } else if (tempError > 0.5) {
        // Almost there - gentle heating
        heaterDuty = 30;
        targetFan1Speed = 0;
        targetFan2Speed = 20;
        
    } else {
        // Temperature reached!
        Serial.print("Chamber ready at ");
        Serial.print(currentTemp);
        Serial.println("°C");
        
        chamberReady = true;
        digitalWrite(GPIO_STATUS_OK, HIGH);  // Signal to printer
        currentMode = MODE_READY;
        
        // Beep pattern for ready (if buzzer connected)
        // tone(BUZZER_PIN, 1000, 100);
    }
    
    heaterEnabled = (heaterDuty > 0);
}

void controlReady() {
    // At temperature, waiting for print to start
    // Maintain temperature with PID control
    
    updatePID(activeProfile->targetTemp);
    
    if (pidOutput > 0) {
        heaterDuty = constrain(pidOutput, 0, HEATER_DUTY_MAX);
        heaterEnabled = true;
    } else {
        heaterDuty = 0;
        heaterEnabled = false;
    }
    
    // Minimal ventilation to maintain temperature
    targetFan1Speed = 0;
    targetFan2Speed = 10;
    
    // Check if we've lost temperature
    if (currentTemp < activeProfile->waitTemp - 3.0) {
        Serial.println("Temperature dropped - returning to preheat");
        currentMode = MODE_PREHEAT;
        chamberReady = false;
        digitalWrite(GPIO_STATUS_OK, LOW);
    }
}

void controlPrinting() {
    // Active printing - balance temperature, VOCs, and layer protection
    
    float tempError = activeProfile->targetTemp - currentTemp;
    
    // Layer counting (simplified - would need actual layer info from printer)
    static unsigned long lastLayerTime = 0;
    if (millis() - lastLayerTime > 60000) {  // Assume ~1 min per layer
        currentLayer++;
        lastLayerTime = millis();
    }
    
    // First layer protection - CRITICAL for adhesion
    bool firstLayerProtection = (currentLayer <= activeProfile->firstLayers);
    
    if (firstLayerProtection) {
        // NO ventilation during first layers
        Serial.print("First layer protection active (layer ");
        Serial.print(currentLayer);
        Serial.print("/");
        Serial.print(activeProfile->firstLayers);
        Serial.println(")");
        
        targetFan1Speed = 0;
        targetFan2Speed = 0;
        
        // But check for dangerous VOC levels
        if (sgpPresent && currentTVOC > VOC_DANGEROUS) {
            // Override for safety
            targetFan1Speed = 10;  // Absolute minimum
            Serial.println("WARNING: VOC override during first layer!");
        }
        
    } else {
        // Normal printing - manage temperature and VOCs
        
        // VOC-based filtration control
        if (activeProfile->hasVOCs || currentTVOC > VOC_HIGH) {
            // Need filtration
            if (currentTVOC > VOC_EMERGENCY) {
                targetFan1Speed = 100;  // Emergency
            } else if (currentTVOC > VOC_DANGEROUS) {
                targetFan1Speed = 50;   // High
            } else if (currentTVOC > VOC_HIGH) {
                targetFan1Speed = 30;   // Moderate
            } else {
                targetFan1Speed = activeProfile->minFanSpeed;  // Minimum
            }
        } else {
            // No VOC concern
            targetFan1Speed = 0;
        }
        
        // Temperature-based circulation control
        if (tempError > 3.0) {
            // Too cold
            targetFan2Speed = 0;
            heaterDuty = 60;
        } else if (tempError > 0) {
            // Slightly cold
            targetFan2Speed = 10;
            heaterDuty = 30;
        } else if (tempError > -3.0) {
            // Good temperature
            targetFan2Speed = 20;
            heaterDuty = 0;
        } else {
            // Too hot
            targetFan2Speed = 50;
            heaterDuty = 0;
            // Increase filtration for cooling
            targetFan1Speed = max(targetFan1Speed, 40);
        }
    }
    
    // PID refinement
    updatePID(activeProfile->targetTemp);
    if (pidOutput > 0 && heaterDuty == 0) {
        heaterDuty = constrain(pidOutput, 0, HEATER_DUTY_MAX);
    }
    
    heaterEnabled = (heaterDuty > 0);
}

void controlPurge() {
    // Post-print VOC purge cycle
    
    unsigned long purgeElapsed = (millis() - purgeStartTime) / 1000;  // seconds
    
    if (purgeElapsed < activeProfile->purgeTime) {
        // Active purging
        targetFan1Speed = 100;  // Maximum filtration
        targetFan2Speed = 80;   // High circulation
        heaterDuty = 0;         // No heating
        heaterEnabled = false;
        
        // Status update
        if (purgeElapsed % 10 == 0) {
            Serial.print("Purging: ");
            Serial.print(purgeElapsed);
            Serial.print("/");
            Serial.print(activeProfile->purgeTime);
            Serial.println(" seconds");
        }
        
    } else {
        // Purge complete
        Serial.println("Purge cycle complete");
        purgeActive = false;
        currentMode = MODE_COOLDOWN;
    }
}

void controlCooldown() {
    // Controlled cooldown to prevent warping
    
    float coolRate = abs(tempRateOfChange);
    
    if (coolRate > activeProfile->maxCoolRate) {
        // Cooling too fast - slow it down
        targetFan1Speed = 0;
        targetFan2Speed = 0;
        Serial.print("Slowing cooldown: ");
        Serial.print(coolRate);
        Serial.println("°C/min");
        
    } else {
        // Controlled cooling
        targetFan1Speed = 20;
        targetFan2Speed = 30;
    }
    
    heaterDuty = 0;
    heaterEnabled = false;
    
    // Exit cooldown when safe
    if (currentTemp < 35.0 && currentTVOC < VOC_ELEVATED) {
        Serial.println("Cooldown complete - system safe");
        currentMode = MODE_OFF;
    }
}

void controlError() {
    // Error state - safety mode
    targetFan1Speed = 100;  // Maximum cooling
    targetFan2Speed = 100;
    heaterDuty = 0;
    heaterEnabled = false;
    
    digitalWrite(GPIO_ERROR_OUT, HIGH);
    
    // Flash red LED
    digitalWrite(LED_RED, (millis() / 500) % 2);
    
    // Try to auto-recover
    if (currentTemp < TEMP_MAX_LIMIT - 10 && !errorState) {
        Serial.println("Attempting recovery from error state...");
        currentMode = MODE_OFF;
        digitalWrite(GPIO_ERROR_OUT, LOW);
    }
}

// ============================================================================
// PID Controller
// ============================================================================

void updatePID(float setpoint) {
    float avgTemp = getAverageTemperature();
    
    pidSetpoint = setpoint;
    pidError = pidSetpoint - avgTemp;
    
    // Proportional
    float P = PID_KP * pidError;
    
    // Integral with anti-windup
    pidIntegral += pidError;
    pidIntegral = constrain(pidIntegral, -100, 100);
    float I = PID_KI * pidIntegral;
    
    // Derivative
    pidDerivative = pidError - pidLastError;
    float D = PID_KD * pidDerivative;
    pidLastError = pidError;
    
    // Calculate output
    pidOutput = P + I + D;
    pidOutput = constrain(pidOutput, 0, 100);
}

float getAverageTemperature() {
    float sum = 0;
    for (int i = 0; i < TEMP_AVG_SAMPLES; i++) {
        sum += tempHistory[i];
    }
    return sum / TEMP_AVG_SAMPLES;
}

// ============================================================================
// Output Control
// ============================================================================

void applyOutputs() {
    // Smooth fan speed changes to prevent abrupt changes
    static unsigned long lastFanUpdate = 0;
    
    if (millis() - lastFanUpdate > 100) {  // Update every 100ms
        lastFanUpdate = millis();
        
        // Ramp fan speeds
        if (fan1Speed < targetFan1Speed) {
            fan1Speed = min(fan1Speed + 5, targetFan1Speed);
        } else if (fan1Speed > targetFan1Speed) {
            fan1Speed = max(fan1Speed - 5, targetFan1Speed);
        }
        
        if (fan2Speed < targetFan2Speed) {
            fan2Speed = min(fan2Speed + 5, targetFan2Speed);
        } else if (fan2Speed > targetFan2Speed) {
            fan2Speed = max(fan2Speed - 5, targetFan2Speed);
        }
    }
    
    // Apply PWM outputs
    #ifdef ESP32
    ledcWrite(0, map(fan1Speed, 0, 100, 0, 255));
    ledcWrite(1, map(fan2Speed, 0, 100, 0, 255));
    #else
    analogWrite(FAN1_PWM_PIN, map(fan1Speed, 0, 100, 0, 255));
    analogWrite(FAN2_PWM_PIN, map(fan2Speed, 0, 100, 0, 255));
    #endif
    
    // Heater control
    if (heaterEnabled && heaterDuty > 0) {
        digitalWrite(HEATER_CTRL_PIN, HIGH);
        analogWrite(HEATER_PWM_PIN, map(heaterDuty, 0, 100, 0, 255));
    } else {
        digitalWrite(HEATER_CTRL_PIN, LOW);
        analogWrite(HEATER_PWM_PIN, 0);
    }
}

// ============================================================================
// Status LEDs
// ============================================================================

void updateStatusLEDs() {
    // Red LED: Error or Heating
    if (currentMode == MODE_ERROR) {
        digitalWrite(LED_RED, (millis() / 250) % 2);  // Fast flash
    } else if (heaterEnabled) {
        digitalWrite(LED_RED, (millis() / 1000) % 2);  // Slow flash
    } else {
        digitalWrite(LED_RED, LOW);
    }
    
    // Green LED: System ready
    if (chamberReady) {
        digitalWrite(LED_GREEN, HIGH);  // Solid when ready
    } else if (systemEnabled) {
        digitalWrite(LED_GREEN, (millis() / 500) % 2);  // Flash when active
    } else {
        digitalWrite(LED_GREEN, LOW);
    }
    
    // Blue LED: Fans active
    if (fan1Speed > 50 || fan2Speed > 50) {
        digitalWrite(LED_BLUE, HIGH);
    } else if (fan1Speed > 0 || fan2Speed > 0) {
        digitalWrite(LED_BLUE, (millis() / 500) % 2);
    } else {
        digitalWrite(LED_BLUE, LOW);
    }
}

// ============================================================================
// Safety Checks
// ============================================================================

void performSafetyChecks() {
    // Over-temperature protection
    if (currentTemp > TEMP_MAX_LIMIT) {
        Serial.println("EMERGENCY: Over-temperature!");
        emergencyStop();
        currentMode = MODE_ERROR;
        errorState = true;
        return;
    }
    
    // Under-temperature (sensor failure)
    if (currentTemp < TEMP_MIN_LIMIT) {
        Serial.println("EMERGENCY: Sensor failure!");
        emergencyStop();
        currentMode = MODE_ERROR;
        errorState = true;
        return;
    }
    
    // Heater timeout
    if (heaterEnabled) {
        if (millis() - heaterStartTime > HEATER_TIMEOUT) {
            Serial.println("Safety: Heater timeout");
            heaterEnabled = false;
            heaterDuty = 0;
        }
    } else {
        heaterStartTime = millis();
    }
    
    // VOC emergency levels
    if (currentTVOC > VOC_EMERGENCY) {
        Serial.println("EMERGENCY: VOC levels critical!");
        targetFan1Speed = 100;
        targetFan2Speed = 100;
        // Don't stop print, but maximize ventilation
    }
    
    // Thermal runaway detection
    static unsigned long heatingCheckTime = 0;
    static float heatingCheckTemp = 0;
    
    if (heaterDuty > 50 && currentMode == MODE_PREHEAT) {
        if (millis() - heatingCheckTime > 120000) {  // Check every 2 minutes
            if (currentTemp - heatingCheckTemp < 2.0) {
                Serial.println("EMERGENCY: Thermal runaway detected!");
                emergencyStop();
                currentMode = MODE_ERROR;
                errorState = true;
            }
            heatingCheckTime = millis();
            heatingCheckTemp = currentTemp;
        }
    }
}

void emergencyStop() {
    // Immediate safety shutdown
    heaterEnabled = false;
    heaterDuty = 0;
    digitalWrite(HEATER_CTRL_PIN, LOW);
    digitalWrite(HEATER_PWM_PIN, LOW);
    
    targetFan1Speed = 100;
    targetFan2Speed = 100;
    fan1Speed = 100;
    fan2Speed = 100;
    
    digitalWrite(GPIO_ERROR_OUT, HIGH);
    digitalWrite(GPIO_STATUS_OK, LOW);
    
    Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
}

// ============================================================================
// Helper Functions
// ============================================================================

void startPurge() {
    purgeActive = true;
    purgeStartTime = millis();
    previousMode = currentMode;
    currentMode = MODE_PURGE;
    Serial.print("Starting ");
    Serial.print(activeProfile->purgeTime);
    Serial.println(" second purge cycle");
}

void loadBaselines() {
    // Load VOC baseline from EEPROM
    uint16_t saved = (EEPROM.read(0) << 8) | EEPROM.read(1);
    if (saved > 0 && saved < 60000) {
        vocBaseline = saved;
        Serial.print("Loaded VOC baseline: ");
        Serial.println(vocBaseline);
    }
}

void saveBaselines() {
    // Save VOC baseline to EEPROM
    EEPROM.write(0, vocBaseline >> 8);
    EEPROM.write(1, vocBaseline & 0xFF);
    EEPROM.commit();
}

// ============================================================================
// Debug Output
// ============================================================================

void printStatus() {
    static unsigned long lastPrint = 0;
    
    // Print every 5 seconds
    if (millis() - lastPrint < 5000) return;
    lastPrint = millis();
    
    Serial.print("Mode: ");
    Serial.print(getModeName());
    Serial.print(" | Profile: ");
    Serial.print(activeProfile->name);
    Serial.print(" | Temp: ");
    Serial.print(currentTemp, 1);
    Serial.print("°C (target: ");
    Serial.print(activeProfile->targetTemp, 1);
    Serial.print("°C) | VOC: ");
    Serial.print(currentTVOC);
    Serial.print(" ppb | Fans: ");
    Serial.print(fan1Speed);
    Serial.print("/");
    Serial.print(fan2Speed);
    Serial.print("% | Heater: ");
    Serial.print(heaterDuty);
    Serial.print("% | Layer: ");
    Serial.println(currentLayer);
}

const char* getModeName() {
    switch(currentMode) {
        case MODE_OFF: return "OFF";
        case MODE_PREHEAT: return "PREHEAT";
        case MODE_READY: return "READY";
        case MODE_PRINTING: return "PRINTING";
        case MODE_PURGE: return "PURGE";
        case MODE_COOLDOWN: return "COOLDOWN";
        case MODE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}