#include <lvgl.h>
#include <TFT_eSPI.h>
#include <TAMC_GT911.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <time.h>
#include "esp_task_wdt.h"  // Watchdog control
#include "ui.h"
#include <Adafruit_SHT4x.h>

// Touch I2C pins
#define TOUCH_SDA    33
#define TOUCH_SCL    32
#define TOUCH_INT    -1 //21
#define TOUCH_RST    25

// SHT45 Sensor I2C pins
//#define SHT45_SDA 21 //16
//#define SHT45_SCL 22 //17

// Backlight control pins
#define TFT_BL_PIN 27       // TFT backlight PWM pin
#define LUX_SENSOR_PIN 34   // GT36516 light sensor (analog input)

// PWM settings for backlight
#define BL_PWM_CHANNEL 0
#define BL_PWM_FREQ 5000
#define BL_PWM_RESOLUTION 8  // 0-255

// Screen saver settings
#define SCREENSAVER_TIMEOUT 60000  // 2 minutes (120000 ms) 60,000ms 1 minute
#define SCREENSAVER_BRIGHTNESS 20   // Very dim when screensaver active

// Brightness thresholds
#define LUX_SAMPLE_INTERVAL 5000  // Check every 5 seconds
#define LUX_DARK_THRESHOLD 500    // Below this = dim mode
#define LUX_BRIGHT_THRESHOLD 2000 // Above this = bright mode
#define BRIGHTNESS_MIN 30         // Minimum brightness (0-255)
#define BRIGHTNESS_MAX 255        // Maximum brightness (0-255)

#define TOUCH_W_RES  480 
#define TOUCH_H_RES  320

static const uint16_t screenWidth  = TOUCH_W_RES;
static const uint16_t screenHeight = TOUCH_H_RES;

// WiFi credentials
const char* ssid = "SSID";
const char* password = "PASSKEY";

// Initialize I2C for SHT45 on Wire1
//TwoWire I2C_SHT = TwoWire(1);
//Adafruit_SHT4x sht4 = Adafruit_SHT4x();

TAMC_GT911 tp = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, screenWidth, screenHeight);

// MQTT Broker settings
const char* mqtt_server = "BROKER IP";
const int mqtt_port = 1883;
const char* mqtt_user = "USERNAME";
const char* mqtt_password = "PASSWORD";
const char* mqtt_client_id = "Thermostat";

// MQTT Topics
#define TOPIC_HEAT "hvac/relay/heat"
#define TOPIC_COOL "hvac/relay/cool"
#define TOPIC_FAN "hvac/relay/fan"
#define TOPIC_HUMID "hvac/relay/humidifier"
#define TOPIC_STATUS "hvac/thermostat/status"
#define TOPIC_THERM_LWT "hvac/thermostat/lwt" //added 
#define TOPIC_RELAY_STATUS "hvac/relay/status"
// 1. ADD NEW MQTT TOPICS (add these with your other topic definitions)
#define TOPIC_REMOTE_TARGET "hvac/thermostat/remote/target"
#define TOPIC_REMOTE_MODE "hvac/thermostat/remote/mode"

// Individual sensor topics for easy monitoring
#define TOPIC_CURRENT_TEMP "hvac/thermostat/temperature"
#define TOPIC_TARGET_TEMP "hvac/thermostat/target"
#define TOPIC_HUMIDITY "hvac/thermostat/humidity"
#define TOPIC_MODE "hvac/thermostat/mode"
#define TOPIC_BRIGHTNESS "hvac/thermostat/brightness"
#define TOPIC_SENSOR_TEMP "hvac/sensor/temperature"
#define TOPIC_SENSOR_HUMIDITY "hvac/sensor/humidity"
#define TOPIC_SENSOR_STATUS "hvac/sensor/status"

// OpenWeather API
const char* apiKey = "API-KEY";
const char* cityId = "5416357";

// NTP Time Server
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -25200;
const int daylightOffset_sec = 3600;

// Display & Touch
TFT_eSPI tft = TFT_eSPI();
static lv_disp_draw_buf_t draw_buf;
static lv_color_t lv_buf[screenWidth * 20];

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Thermostat variables
float currentTemp = 0.0;
float currentHumidity = 0.0;
float targetTemp = 68.0;
float hysteresis = 1.0;

// Relay states
bool heatState = false;
bool coolState = false;
bool fanState = false;
bool humidifierState = false;

// HVAC Mode
enum HVACMode { MODE_OFF, MODE_HEAT, MODE_COOL, MODE_AUTO, MODE_FAN };
HVACMode currentMode = MODE_OFF;

// Weather data
float outsideTemp = 0.0;
float outsideHumidity = 0.0;
String weatherDescription = "";

// Brightness control
unsigned long lastLuxRead = 0;
int currentBrightness = BRIGHTNESS_MAX;
int targetBrightness = BRIGHTNESS_MAX;
bool autobrightness = true;  // Auto mode by default

// Screen saver control
unsigned long lastTouchTime = 0;
bool screensaverActive = false;
int brightnessBeforeScreensaver = BRIGHTNESS_MAX;
lv_obj_t * screensaver_screen = NULL;
lv_obj_t * screensaver_time_label = NULL;
unsigned long lastScreensaverUpdate = 0;
const unsigned long screensaverUpdateInterval = 1000; // Update every second


// Timers
unsigned long lastSensorRead = 0;
unsigned long lastWeatherUpdate = 0;
unsigned long lastMQTTUpdate = 0;
unsigned long lastUIUpdate = 0;
unsigned long lastLoopTime = 0;  // For monitoring loop speed

const unsigned long sensorInterval = 2000;
const unsigned long weatherInterval = 600000;
const unsigned long mqttInterval = 5000;
const unsigned long uiUpdateInterval = 500;

// Flag to know if UI is ready
bool uiReady = false;

// Function prototypes
void setupWiFi();
void setupMQTT();
void setupTime();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
//void readSensor();
void updateWeather();
void controlHVAC();
void publishRelayStates();
void updateAllUI();
void updateHomeScreen();
void updateTempScreen();
void updateOutsideScreen();
void updateSystemScreen();
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void set_temp_val(lv_event_t * e);
void mode_changed(lv_event_t * e);
void brightness_override(lv_event_t * e);
void updateBrightness();
int readLightLevel();
void checkScreensaver();
void activateScreensaver();
void deactivateScreensaver();

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  uint32_t size = w * h;

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t*)color_p, size, true); 
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  tp.read();
  if (tp.isTouched) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = tp.points[0].x;
    data->point.y = tp.points[0].y;
    
    // Track touch activity for screensaver
    lastTouchTime = millis();
    
    // Wake from screensaver if active
    if (screensaverActive) {
      deactivateScreensaver();
    }
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // ===== CRITICAL: Configure Watchdog First =====
  Serial.println("\n[WATCHDOG] Configuring watchdog timer...");
  
  // ESP32 Arduino Core 3.x+ uses new API
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,        // 30 seconds
    .idle_core_mask = 0,        // Don't watch idle tasks
    .trigger_panic = false      // Don't panic, just reset
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  
  Serial.println("[WATCHDOG] Watchdog set to 30 seconds");
  
  Serial.println("\n==========================================");
  Serial.println("ESP32 Smart Thermostat - Starting...");
  Serial.println("SHT45 Sensor + Auto Brightness Control");
  Serial.println("==========================================\n");

  // ===== STEP 1: Initialize Hardware FIRST =====
  Serial.println("[1/13] Initializing TFT Display...");
  tft.begin();
  tft.setRotation(1);
  
  // Test display with color flash
  tft.fillScreen(TFT_RED);
  esp_task_wdt_reset();  // Feed watchdog
  delay(300);
  tft.fillScreen(TFT_GREEN);
  esp_task_wdt_reset();
  delay(300);
  tft.fillScreen(TFT_BLUE);
  esp_task_wdt_reset();
  delay(300);
  tft.fillScreen(TFT_BLACK);
  Serial.println("       ✓ TFT Display working (480x320)");

  // ===== STEP 1.5: Initialize Backlight PWM =====
  Serial.println("[1.5/13] Initializing Backlight Control...");
  pinMode(TFT_BL_PIN, OUTPUT);
  
  // ESP32 Arduino Core 3.x uses new API
  ledcAttach(TFT_BL_PIN, BL_PWM_FREQ, BL_PWM_RESOLUTION);
  ledcWrite(TFT_BL_PIN, BRIGHTNESS_MAX);  // Start at full brightness
  Serial.println("       ✓ Backlight PWM on pin 27");
  
  pinMode(LUX_SENSOR_PIN, INPUT);
  Serial.println("       ✓ Light sensor (GT36516) on pin 34");
  esp_task_wdt_reset();

  // ===== STEP 2: Initialize Touch =====
  Serial.println("[2/13] Initializing Touch Controller...");
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  tp.begin();
  tp.setRotation(2);
  Serial.println("       ✓ GT911 Touch initialized");
  esp_task_wdt_reset();

  // ===== STEP 3: Initialize SHT45 Sensor =====
  //Serial.println("[3/13] Initializing SHT45 Temperature/Humidity Sensor...");
  //I2C_SHT.begin(SHT45_SDA, SHT45_SCL, 400000);  // 400kHz I2C speed
  
  //if (sht4.begin(&I2C_SHT)) {
    //sht4.setPrecision(SHT4X_HIGH_PRECISION);
    //sht4.setHeater(SHT4X_NO_HEATER);
    //Serial.println("       ✓ SHT45 initialized on I2C (SDA=16, SCL=17)");
    //Serial.print("       Serial Number: 0x");
    //Serial.println(sht4.readSerial(), HEX);
 // } else {
    //Serial.println("       ✗ SHT45 not found! Check wiring:");
    //Serial.println("          SDA -> GPIO 16");
   // Serial.println("          SCL -> GPIO 17");
    //Serial.println("          VCC -> 3.3V");
    //Serial.println("          GND -> GND");
 // }
  //esp_task_wdt_reset();

  // ===== STEP 4: Initialize LVGL =====
  Serial.println("[4/13] Initializing LVGL...");
  lv_init();
  Serial.println("       ✓ LVGL core ready");
  esp_task_wdt_reset();

  // ===== STEP 5: Setup LVGL Display Buffer =====
  Serial.println("[5/13] Creating LVGL Display Buffer...");
  lv_disp_draw_buf_init(&draw_buf, lv_buf, NULL, screenWidth * 20);
  Serial.println("       ✓ Buffer created (20 lines)");

  // ===== STEP 6: Register Display Driver =====
  Serial.println("[6/13] Registering Display Driver...");
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  Serial.println("       ✓ Display driver registered");
  esp_task_wdt_reset();

  // ===== STEP 7: Register Touch Driver =====
  Serial.println("[7/13] Registering Touch Driver...");
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  Serial.println("       ✓ Touch driver registered");

  // ===== STEP 8: Load SquareLine UI - CRITICAL! =====
  Serial.println("[8/13] Loading UI from SquareLine Studio...");
  ui_init();
  Serial.println("       ✓ UI loaded!");
  esp_task_wdt_reset();

  // Verify objects exist
  if (ui_tempdial != NULL) {
    Serial.println("       ✓ ui_tempdial exists");
  } else {
    Serial.println("       ✗ WARNING: ui_tempdial is NULL!");
  }
  
  if (ui_modeset != NULL) {
    Serial.println("       ✓ ui_modeset exists");
  } else {
    Serial.println("       ✗ WARNING: ui_modeset is NULL!");
  }
  
  if (ui_brightoverride != NULL) {
    Serial.println("       ✓ ui_brightoverride exists");
  } else {
    Serial.println("       ✗ WARNING: ui_brightoverride is NULL!");
  }

  // ===== STEP 9: Attach Event Handlers =====
  Serial.println("[9/13] Attaching Event Handlers...");
  if (ui_tempdial != NULL) {
    lv_obj_add_event_cb(ui_tempdial, set_temp_val, LV_EVENT_VALUE_CHANGED, NULL);
    lv_arc_set_value(ui_tempdial, (int)targetTemp);
    Serial.println("       ✓ Arc events attached");
  }
  
  if (ui_modeset != NULL) {
    lv_obj_add_event_cb(ui_modeset, mode_changed, LV_EVENT_VALUE_CHANGED, NULL);
    lv_dropdown_set_selected(ui_modeset, MODE_OFF);
    Serial.println("       ✓ Dropdown events attached");
  }
  
  if (ui_brightoverride != NULL) {
    lv_obj_add_event_cb(ui_brightoverride, brightness_override, LV_EVENT_VALUE_CHANGED, NULL);
    lv_slider_set_value(ui_brightoverride, BRIGHTNESS_MAX, LV_ANIM_OFF);
    Serial.println("       ✓ Brightness slider events attached");
  }

  // UI is now ready!
  uiReady = true;
  
  // Force initial screen render
  lv_obj_invalidate(lv_scr_act());
  lv_refr_now(NULL);
  Serial.println("       ✓ Screen refreshed");
  esp_task_wdt_reset();

  // ===== STEP 10: Connect WiFi =====
  Serial.println("[10/13] Connecting to WiFi...");
  setupWiFi();
  esp_task_wdt_reset();

  // ===== STEP 11: Setup Time & MQTT =====
  Serial.println("[11/13] Setting up Time & MQTT...");
  setupTime();
  setupMQTT();
  updateWeather();
  esp_task_wdt_reset();

  // ===== STEP 12: Initial Data Read =====
  Serial.println("[12/13] Reading Initial Sensor Data...");
  //readSensor();
  
  // ===== STEP 13: Initial Brightness Setup =====
  Serial.println("[13/13] Setting Initial Brightness...");
  updateBrightness();
  
  // Update UI with initial data
  if (uiReady) {
    updateAllUI();
    Serial.println("       ✓ UI updated with data");
  }

  Serial.println("\n==========================================");
  Serial.println("✓ SETUP COMPLETE - System Running!");
  Serial.println("==========================================\n");
  
  Serial.print("Current Temp: ");
  Serial.print(currentTemp);
  Serial.print("°F, Target: ");
  Serial.print(targetTemp);
  Serial.println("°F");
  Serial.print("Auto Brightness: ");
  Serial.println(autobrightness ? "ENABLED" : "DISABLED");
  
  esp_task_wdt_reset();
}

void loop() {
  // Monitor loop timing for debugging
  unsigned long loopStart = millis();
  unsigned long loopDuration = loopStart - lastLoopTime;
  
  if (loopDuration > 100) {  // Warn if loop took >100ms
    Serial.print("[WARNING] Slow loop: ");
    Serial.print(loopDuration);
    Serial.println("ms");
  }
  lastLoopTime = loopStart;
  
  // Feed watchdog at the very start of every loop
  esp_task_wdt_reset();
  
  // LVGL timer - keep this quick!
  unsigned long lvglStart = millis();
  lv_timer_handler();
  unsigned long lvglTime = millis() - lvglStart;
  
  if (lvglTime > 50) {  // LVGL taking too long
    Serial.print("[WARNING] LVGL slow: ");
    Serial.print(lvglTime);
    Serial.println("ms");
  }
  
  // Check MQTT connection (non-blocking)
  if (!mqttClient.connected()) {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      reconnectMQTT();
      esp_task_wdt_reset();
    }
  } else {
    mqttClient.loop();
  }
/*/
  // Read sensor 
  if (millis() - lastSensorRead >= sensorInterval) {
    lastSensorRead = millis();
    //readSensor();
    controlHVAC();
    esp_task_wdt_reset();
  }
*/
  static unsigned long lastHVACCheck = 0;
  if (millis() - lastHVACCheck >= 5000) {  // Every 5 seconds
    lastHVACCheck = millis();
    controlHVAC();  // Will use latest sensor data from MQTT
  }
  // Update weather (only every 10 minutes)
  if (millis() - lastWeatherUpdate >= weatherInterval) {
    lastWeatherUpdate = millis();
    Serial.println("[INFO] Fetching weather...");
    updateWeather();
    esp_task_wdt_reset();
  }

  // Publish MQTT status
  if (millis() - lastMQTTUpdate >= mqttInterval) {
    lastMQTTUpdate = millis();
    publishRelayStates();
    esp_task_wdt_reset();
  }

  // Update UI
  if (uiReady && (millis() - lastUIUpdate >= uiUpdateInterval)) {
    lastUIUpdate = millis();
    updateAllUI();
    esp_task_wdt_reset();
  }
  
  // Update backlight brightness based on ambient light (only if auto mode)
  if (autobrightness && (millis() - lastLuxRead >= LUX_SAMPLE_INTERVAL)) {
    lastLuxRead = millis();
    updateBrightness();
  }
  if (ui_autobrightbutton != NULL) {
    lv_obj_add_event_cb(ui_autobrightbutton, enable_auto_brightness, LV_EVENT_CLICKED, NULL);
  }
    // Check for screensaver timeout
  checkScreensaver();
    if (screensaverActive && (millis() - lastScreensaverUpdate >= screensaverUpdateInterval)) {
    lastScreensaverUpdate = millis();
    updateScreensaverClock();
  }
  // Small delay and final watchdog feed
  delay(2);
  esp_task_wdt_reset();
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    // Feed watchdog every few attempts
    if (attempts % 5 == 0) {
      yield();
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n       ✓ WiFi connected!");
    Serial.print("       IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n       ✗ WiFi failed!");
  }
}

void setupTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("       ✓ Time configured");
}

void setupMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(15);  // Short keepalive
  mqttClient.setSocketTimeout(5);  // 5 second timeout
  
  Serial.print("       Connecting to MQTT...");
  
  //String clientId = "Thermostat-";
  //clientId += String(random(0xffff), HEX);
  
  // Single connection attempt
    if (mqttClient.connect(
      mqtt_client_id,
      mqtt_user,
      mqtt_password,
      TOPIC_THERM_LWT, // LWT topic
      0,                    // QoS
      true,                 // retained
      "offline"             // LWT payload
)) {
    mqttClient.publish(TOPIC_THERM_LWT, "online", true);
    Serial.println("connected!");
    mqttClient.subscribe(TOPIC_RELAY_STATUS);
    mqttClient.subscribe(TOPIC_REMOTE_TARGET);
    mqttClient.subscribe(TOPIC_REMOTE_MODE);
    mqttClient.subscribe(TOPIC_SENSOR_TEMP);
    mqttClient.subscribe(TOPIC_SENSOR_HUMIDITY);
    mqttClient.subscribe(TOPIC_SENSOR_STATUS);
    Serial.println("       ✓ MQTT ready");
    Serial.println("       ✓ Subscribed to remote sensor");
    Serial.println("       ✓ MQTT ready");
  } else {
    Serial.print("failed! rc=");
    Serial.println(mqttClient.state());
    Serial.println("       ! MQTT will retry in loop");
  }
}

void reconnectMQTT() {
  // Ultra-simple non-blocking reconnection
  if (mqttClient.connected()) return;
  
  //String clientId = "Thermostat-";
  //clientId += String(random(0xffff), HEX);
  
  // Single quick attempt - don't wait!
  //bool connected = mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password);
  
  if (mqttClient.connect(
      mqtt_client_id,
      mqtt_user,
      mqtt_password,
      TOPIC_THERM_LWT,
      0,
      true,
      "offline"
)) {
    mqttClient.publish(TOPIC_THERM_LWT, "online", true);
    Serial.println("[MQTT] Reconnected!");
    mqttClient.subscribe(TOPIC_RELAY_STATUS);
    mqttClient.subscribe(TOPIC_REMOTE_TARGET);
    mqttClient.subscribe(TOPIC_REMOTE_MODE);
    mqttClient.subscribe(TOPIC_SENSOR_TEMP);
    mqttClient.subscribe(TOPIC_SENSOR_HUMIDITY);
    mqttClient.subscribe(TOPIC_SENSOR_STATUS);
  }
  // If failed, just return - will try again later
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  esp_task_wdt_reset();  // Feed watchdog at start
  
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  String topicStr = String(topic);
  
  Serial.print("[MQTT] Topic: ");
  Serial.print(topicStr);
  Serial.print(" | Message: ");
  Serial.println(message);
  
  esp_task_wdt_reset();  // Feed after string processing
  
  // Handle relay status JSON messages
  if (topicStr == TOPIC_RELAY_STATUS) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (!error) {
      if (doc.containsKey("heat")) heatState = doc["heat"];
      if (doc.containsKey("cool")) coolState = doc["cool"];
      if (doc.containsKey("fan")) fanState = doc["fan"];
      if (doc.containsKey("humidifier")) humidifierState = doc["humidifier"];
      Serial.println("[MQTT] Relay status updated");
    }
  }
  
  // Handle remote target temperature changes
  else if (topicStr == TOPIC_REMOTE_TARGET) {
    float newTarget = message.toFloat();
    
    if (newTarget >= 50.0 && newTarget <= 95.0) {
      targetTemp = newTarget;
      
      Serial.print("[REMOTE] Target temp updated to: ");
      Serial.print(targetTemp);
      Serial.println("°F");
      
      // Update the UI arc to match
      if (ui_tempdial != NULL) {
        lv_arc_set_value(ui_tempdial, (int)targetTemp);
      }
      
      // Update the UI label to match
      if (ui_settemp != NULL) {
        char tempStr[16];
        sprintf(tempStr, "%.0f°F", targetTemp);
        lv_label_set_text(ui_settemp, tempStr);
      }
      
      esp_task_wdt_reset();
      controlHVAC();
      publishRelayStates();
    } else {
      Serial.print("[REMOTE] Invalid target temp rejected: ");
      Serial.println(newTarget);
    }
  }
  
  // Handle remote mode changes
  else if (topicStr == TOPIC_REMOTE_MODE) {
    String newMode = message;
    newMode.toUpperCase();
    
    HVACMode oldMode = currentMode;
    
    if (newMode == "OFF") {
      currentMode = MODE_OFF;
    } 
    else if (newMode == "HEAT") {
      currentMode = MODE_HEAT;
    } 
    else if (newMode == "COOL") {
      currentMode = MODE_COOL;
    } 
    else if (newMode == "AUTO") {
      currentMode = MODE_AUTO;
    } 
    else if (newMode == "FAN") {
      currentMode = MODE_FAN;
    }
    else {
      Serial.print("[REMOTE] Invalid mode rejected: ");
      Serial.println(newMode);
      return;
    }
    
    Serial.print("[REMOTE] Mode updated to: ");
    Serial.println(newMode);
    
    // Update the UI dropdown to match
    if (ui_modeset != NULL) {
      uint16_t dropdownIndex = 0;
      switch(currentMode) {
        case MODE_OFF: dropdownIndex = 0; break;
        case MODE_HEAT: dropdownIndex = 1; break;
        case MODE_COOL: dropdownIndex = 2; break;
        case MODE_AUTO: dropdownIndex = 3; break;
        case MODE_FAN: dropdownIndex = 4; break;
      }
      lv_dropdown_set_selected(ui_modeset, dropdownIndex);
    }
    
    esp_task_wdt_reset();
    
    // If mode changed, run HVAC control
    if (oldMode != currentMode) {
      controlHVAC();
      publishRelayStates();
    }
  }
  
  // *** CRITICAL: Handle sensor temperature from NodeMCU ***
  else if (topicStr == TOPIC_SENSOR_TEMP) {
    float newTemp = message.toFloat();
    
    if (newTemp > 0.0 && newTemp < 150.0) {
      currentTemp = newTemp;
      
      Serial.print("[SENSOR] Temperature: ");
      Serial.print(currentTemp);
      Serial.println("°F");
      
      esp_task_wdt_reset();
      controlHVAC();
    } else {
      Serial.print("[SENSOR] Invalid temp rejected: ");
      Serial.println(newTemp);
    }
  }
  
  // Handle sensor humidity from NodeMCU
  else if (topicStr == TOPIC_SENSOR_HUMIDITY) {
    float newHumidity = message.toFloat();
    
    if (newHumidity >= 0.0 && newHumidity <= 100.0) {
      currentHumidity = newHumidity;
      
      Serial.print("[SENSOR] Humidity: ");
      Serial.print(currentHumidity);
      Serial.println("%");
    } else {
      Serial.print("[SENSOR] Invalid humidity rejected: ");
      Serial.println(newHumidity);
    }
  }
  
  // Handle sensor status updates
  else if (topicStr == TOPIC_SENSOR_STATUS) {
    Serial.println("[SENSOR] Status update received");
  }
  
  esp_task_wdt_reset();  // Feed at end
}

/*void readSensor() {
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);
  
  // Convert Celsius to Fahrenheit
  float tempF = temp.temperature * 9.0 / 5.0 + 32.0;
  float humidityPercent = humidity.relative_humidity;
  
  // Validate readings (SHT45 typical range: -40 to 125°C, 0-100% RH)
  if (tempF > -40.0 && tempF < 257.0 && humidityPercent >= 0.0 && humidityPercent <= 100.0) {
    currentTemp = tempF;
    currentHumidity = humidityPercent;
    
    // Debug output every 10 reads
    static int readCount = 0;
    if (++readCount >= 10) {
      Serial.print("[SHT45] Temp: ");
      Serial.print(currentTemp);
      Serial.print("°F, Humidity: ");
      Serial.print(currentHumidity);
      Serial.println("%");
      readCount = 0;
    }
  } else {
    Serial.println("[SHT45] Invalid reading detected!");
  }
}
*/
int readLightLevel() {
  // Read analog value from GT36516 light sensor
  // Take multiple samples for stability
  int total = 0;
  const int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    total += analogRead(LUX_SENSOR_PIN);
    delay(5);
  }
  
  int average = total / samples;
  
  // Debug output occasionally
  static int debugCount = 0;
  if (++debugCount >= 12) {  // Every minute (5s * 12 = 60s)
    Serial.print("[LIGHT] Sensor reading: ");
    Serial.print(average);
    Serial.print(" (Auto: ");
    Serial.print(autobrightness ? "ON" : "OFF");
    Serial.println(")");
    debugCount = 0;
  }
  
  return average;
}

void updateBrightness() {
  if (!autobrightness) return;  // Skip if manual mode
  
  int luxValue = readLightLevel();
  
  // Map lux reading to brightness with hysteresis
  if (luxValue < LUX_DARK_THRESHOLD) {
    // Dark environment - use minimum brightness
    targetBrightness = BRIGHTNESS_MIN;
  } 
  else if (luxValue > LUX_BRIGHT_THRESHOLD) {
    // Bright environment - use maximum brightness
    targetBrightness = BRIGHTNESS_MAX;
  }
  else {
    // Transition zone - map proportionally
    targetBrightness = map(luxValue, 
                          LUX_DARK_THRESHOLD, 
                          LUX_BRIGHT_THRESHOLD, 
                          BRIGHTNESS_MIN, 
                          BRIGHTNESS_MAX);
  }
  
  // Smooth transition - gradually change brightness
  if (currentBrightness != targetBrightness) {
    int diff = targetBrightness - currentBrightness;
    
    // Move 10% of the way toward target each update
    if (abs(diff) > 5) {
      currentBrightness += (diff > 0) ? 5 : -5;
    } else {
      currentBrightness = targetBrightness;
    }
    
    ledcWrite(TFT_BL_PIN, currentBrightness);
    
    // Update slider to reflect auto brightness change
    if (ui_brightoverride != NULL) {
      lv_slider_set_value(ui_brightoverride, currentBrightness, LV_ANIM_OFF);
    }
    
    Serial.print("[BACKLIGHT] Lux: ");
    Serial.print(luxValue);
    Serial.print(" -> Brightness: ");
    Serial.println(currentBrightness);
  }
}

void controlHVAC() {
  bool needHeat = false;
  bool needCool = false;
  bool needFan = false;
  bool needHumid = false;

  switch (currentMode) {
    case MODE_OFF:
      needHeat = false;
      needCool = false;
      needFan = false;
      needHumid = false;
      break;

    case MODE_HEAT:
      if (currentTemp < targetTemp - hysteresis) {
        needHeat = true;
        needFan = true;
        needHumid = true;
      } else if (currentTemp > targetTemp) {
        needHeat = false;
        needFan = false;
        needHumid = false;
      } else {
        return;
      }
      break;

    case MODE_COOL:
      if (currentTemp > targetTemp + hysteresis) {
        needCool = true;
        needFan = true;
        needHumid = true;
      } else if (currentTemp < targetTemp) {
        needCool = false;
        needFan = false;
        needHumid = false;
      } else {
        return;
      }
      break;

    case MODE_AUTO:
      if (currentTemp < targetTemp - hysteresis) {
        needHeat = true;
        needCool = false;
        needFan = true;
        needHumid = true;
      } else if (currentTemp > targetTemp + hysteresis) {
        needHeat = false;
        needCool = true;
        needFan = true;
        needHumid = true;
      } else {
        return;
      }
      break;

    case MODE_FAN:
      needHeat = false;
      needCool = false;
      needFan = true;
      needHumid = false;
      break;
  }

  if (needHeat != heatState) {
    heatState = needHeat;
    mqttClient.publish(TOPIC_HEAT, heatState ? "ON" : "OFF");
  }
  if (needCool != coolState) {
    coolState = needCool;
    mqttClient.publish(TOPIC_COOL, coolState ? "ON" : "OFF");
  }
  if (needFan != fanState) {
    fanState = needFan;
    mqttClient.publish(TOPIC_FAN, fanState ? "ON" : "OFF");
  }
  if (needHumid != humidifierState) {
    humidifierState = needHumid;
    mqttClient.publish(TOPIC_HUMID, humidifierState ? "ON" : "OFF");
  }
}

void publishRelayStates() {
  if (!mqttClient.connected()) return;
  
  // Publish complete status as JSON
  StaticJsonDocument<300> doc;
  doc["temp"] = currentTemp;
  doc["humidity"] = currentHumidity;
  doc["target"] = targetTemp;
  doc["heat"] = heatState;
  doc["cool"] = coolState;
  doc["fan"] = fanState;
  doc["humidifier"] = humidifierState;
  doc["mode"] = currentMode;
  doc["brightness"] = currentBrightness;
  doc["autobrightness"] = autobrightness;
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();  // WiFi signal strength in dBm
  doc["uptime"] = millis() / 1000;  // Uptime in seconds
  
  char buffer[300];
  serializeJson(doc, buffer);
  mqttClient.publish(TOPIC_STATUS, buffer);
  
  // Publish individual sensor values for easy monitoring
  char tempStr[16];
  sprintf(tempStr, "%.1f", currentTemp);
  mqttClient.publish(TOPIC_CURRENT_TEMP, tempStr);
  
  char targetStr[16];
  sprintf(targetStr, "%.1f", targetTemp);
  mqttClient.publish(TOPIC_TARGET_TEMP, targetStr);
  
  char humStr[16];
  sprintf(humStr, "%.1f", currentHumidity);
  mqttClient.publish(TOPIC_HUMIDITY, humStr);
  
  char brightStr[16];
  sprintf(brightStr, "%d", currentBrightness);
  mqttClient.publish(TOPIC_BRIGHTNESS, brightStr);
  
  // Publish mode as text
  const char* modeStr = "OFF";
  switch(currentMode) {
    case MODE_OFF: modeStr = "OFF"; break;
    case MODE_HEAT: modeStr = "HEAT"; break;
    case MODE_COOL: modeStr = "COOL"; break;
    case MODE_AUTO: modeStr = "AUTO"; break;
    case MODE_FAN: modeStr = "FAN"; break;
  }
  mqttClient.publish(TOPIC_MODE, modeStr);
}

void updateWeather() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = "http://api.openweathermap.org/data/2.5/weather?id=";
  url += cityId;
  url += "&appid=";
  url += apiKey;
  url += "&units=imperial";

  http.setTimeout(5000);  // 5 second timeout
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode > 0) {
    String payload = http.getString();
    
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      outsideTemp = doc["main"]["temp"];
      outsideHumidity = doc["main"]["humidity"];
      weatherDescription = doc["weather"][0]["description"].as<String>();
      Serial.println("Weather updated");
    }
  } else {
    Serial.print("Weather HTTP failed: ");
    Serial.println(httpCode);
  }
  http.end();
  
  // Yield to system after network operation
  yield();
}

void updateAllUI() {
  if (!uiReady) return;
  
  esp_task_wdt_reset();  // Feed before starting UI updates
  
  updateHomeScreen();
  esp_task_wdt_reset();
  
  updateTempScreen();
  esp_task_wdt_reset();
  
  updateOutsideScreen();
  esp_task_wdt_reset();
  
  updateSystemScreen();
  esp_task_wdt_reset();
}

void updateHomeScreen() {
  if (ui_timedate != NULL) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timeStr[64];
      strftime(timeStr, sizeof(timeStr), " %A, %b, %e, %Y \n              %I:%M %P", &timeinfo);
      lv_label_set_text(ui_timedate, timeStr);
    }
  }

  if (ui_dhttemp != NULL) {
    char tempStr[16];
    sprintf(tempStr, "%.1f°F", currentTemp);
    lv_label_set_text(ui_dhttemp, tempStr);
  }

  if (ui_dhthum != NULL) {
    char humStr[16];
    sprintf(humStr, "%.0f%%", currentHumidity);
    lv_label_set_text(ui_dhthum, humStr);
  }
}

void updateTempScreen() {
  if (ui_settemp != NULL) {
    char setTempStr[16];
    sprintf(setTempStr, "%.0f°F", targetTemp);
    lv_label_set_text(ui_settemp, setTempStr);
  }
}

void updateOutsideScreen() {
  if (ui_outsidetemp != NULL) {
    char outsideTempStr[16];
    sprintf(outsideTempStr, "%.1f°F", outsideTemp);
    lv_label_set_text(ui_outsidetemp, outsideTempStr);
  }

  if (ui_outsidehum != NULL) {
    char outsideHumStr[16];
    sprintf(outsideHumStr, "%.0f%%", outsideHumidity);
    lv_label_set_text(ui_outsidehum, outsideHumStr);
  }

  if (ui_outdescrip != NULL) {
    lv_label_set_text(ui_outdescrip, weatherDescription.c_str());
  }
}

void updateSystemScreen() {
  if (ui_getipprint != NULL) {
    lv_label_set_text(ui_getipprint, WiFi.localIP().toString().c_str());
  }

  if (ui_fnrly1val != NULL) {
    lv_label_set_text(ui_fnrly1val, fanState ? "ON" : "OFF");
  }
  if (ui_hetrly2val != NULL) {
    lv_label_set_text(ui_hetrly2val, heatState ? "ON" : "OFF");
  }
  if (ui_colrly3val != NULL) {
    lv_label_set_text(ui_colrly3val, coolState ? "ON" : "OFF");
  }
  if (ui_humrly4val != NULL) {
    lv_label_set_text(ui_humrly4val, humidifierState ? "ON" : "OFF");
  }

  if (ui_mqttstatus != NULL) {
    lv_label_set_text(ui_mqttstatus, mqttClient.connected() ? "Connected" : "Disconnected");
  }
}

void checkScreensaver() {
  // Don't activate screensaver if already active
  if (screensaverActive) return;
  
  // Check if timeout has elapsed
  if (millis() - lastTouchTime >= SCREENSAVER_TIMEOUT) {
    activateScreensaver();
  }
}

void activateScreensaver() {
  if (screensaverActive) return;
  
  screensaverActive = true;
  Serial.println("[SCREENSAVER] Activated");
  
  // Save current brightness
  brightnessBeforeScreensaver = currentBrightness;
  
  // Dim to screensaver brightness
  currentBrightness = SCREENSAVER_BRIGHTNESS;
  targetBrightness = SCREENSAVER_BRIGHTNESS;
  ledcWrite(TFT_BL_PIN, currentBrightness);
  
  // Create screensaver screen
  screensaver_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(screensaver_screen, lv_color_black(), 0);
  
  // Create time label (store reference globally)
  screensaver_time_label = lv_label_create(screensaver_screen);
  lv_obj_set_style_text_color(screensaver_time_label, lv_color_make(220, 220, 220), 0);
  lv_obj_set_style_text_font(screensaver_time_label, &lv_font_montserrat_48, 0);
  lv_obj_center(screensaver_time_label);
  
  // Set initial time
  updateScreensaverClock();
  
  lv_scr_load(screensaver_screen);
}

void updateScreensaverClock() {
  if (!screensaverActive || screensaver_time_label == NULL) return;
  
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeStr[32];
    strftime(timeStr, sizeof(timeStr), "%I:%M %P", &timeinfo);
    lv_label_set_text(screensaver_time_label, timeStr);
  }
}

void deactivateScreensaver() {
  if (!screensaverActive) return;
  
  screensaverActive = false;
  Serial.println("[SCREENSAVER] Deactivated");
  
  // Restore previous brightness
  currentBrightness = brightnessBeforeScreensaver;
  targetBrightness = brightnessBeforeScreensaver;
  ledcWrite(TFT_BL_PIN, currentBrightness);
  
  // Return to home screen (or last active screen)
  lv_scr_load(ui_Home);
  
  // Clean up screensaver objects
  if (screensaver_screen != NULL) {
    lv_obj_del(screensaver_screen);
    screensaver_screen = NULL;
    screensaver_time_label = NULL;
  }
  
  // Update slider to match restored brightness
  if (ui_brightoverride != NULL) {
    lv_slider_set_value(ui_brightoverride, currentBrightness, LV_ANIM_OFF);
  }
  
  // Reset the timeout
  lastTouchTime = millis();
}

void set_temp_val(lv_event_t * e) {
  lv_obj_t * arc = lv_event_get_target(e);
  int arcValue = lv_arc_get_value(arc);
  
  targetTemp = (float)arcValue;
  
  if (ui_settemp != NULL) {
    char tempStr[16];
    sprintf(tempStr, "%.0f°F", targetTemp);
    lv_label_set_text(ui_settemp, tempStr);
  }
  
  controlHVAC();
  
  Serial.print("Target temp: ");
  Serial.println(targetTemp);
}

void mode_changed(lv_event_t * e) {
  lv_obj_t * dropdown = lv_event_get_target(e);
  uint16_t selected = lv_dropdown_get_selected(dropdown);
  
  switch(selected) {
    case 0: currentMode = MODE_OFF; Serial.println("Mode: OFF"); break;
    case 1: currentMode = MODE_HEAT; Serial.println("Mode: HEAT"); break;
    case 2: currentMode = MODE_COOL; Serial.println("Mode: COOL"); break;
    case 3: currentMode = MODE_AUTO; Serial.println("Mode: AUTO"); break;
    case 4: currentMode = MODE_FAN; Serial.println("Mode: FAN"); break;
  }
  
  controlHVAC();
}

void brightness_override(lv_event_t * e) {
  lv_obj_t * slider = lv_event_get_target(e);
  int sliderValue = lv_slider_get_value(slider);
  
  // When user touches slider, disable auto brightness
  autobrightness = false;
  
  // Override automatic brightness
  currentBrightness = sliderValue;
  targetBrightness = sliderValue;
  ledcWrite(TFT_BL_PIN, currentBrightness);
  
  Serial.print("[MANUAL] Brightness override: ");
  Serial.print(currentBrightness);
  Serial.println(" (Auto mode disabled)");
}

// Optional: Function to re-enable auto brightness mode
void enable_auto_brightness(lv_event_t * e) {
  autobrightness = true;
  Serial.println("[AUTO] Brightness auto mode re-enabled");
  // Immediately update to current light conditions
  updateBrightness();
}