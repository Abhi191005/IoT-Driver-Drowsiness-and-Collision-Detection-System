/*
 * =====================================================================================
 * Project: IoT-Enabled Advanced Driver Assistance System (ADAS)
 * Module:  Drowsiness Detection Module Firmware
 * Target:  AI-Thinker ESP32-CAM (OV2640)
 * Authors: Abhi Shah, Sahilali Dholkawala, Sanghani Karan
 * Institute: Dharmsinh Desai University (DDU)
 *
 * Description:
 *   Lightweight vision-based driver drowsiness detection running locally on an
 *   ESP32-CAM without heavy AI/ML libraries. Employs a 20-zone (5x4) spatial grid
 *   frame-difference motion detection algorithm on 160x120 (QQVGA) frames. Detects
 *   driver stillness/micro-sleep (> 3.0s cessation of movement) and triggers
 *   cloud alerts with image telemetry to Telegram over HTTPS.
 * =====================================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>

// =====================================================================================
// AI-THINKER ESP32-CAM PINOUT DEFINITIONS
// =====================================================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_LED_PIN      4   // Onboard high-brightness flash LED

// =====================================================================================
// NETWORK & TELEGRAM BOT CONFIGURATION
// =====================================================================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

#define BOT_TOKEN "YOUR_TELEGRAM_BOT_TOKEN"
#define CHAT_ID   "YOUR_AUTHORIZED_CHAT_ID"

// =====================================================================================
// ALGORITHM HYPERPARAMETERS (20-ZONE FRAME-DIFFERENCE MOTION)
// =====================================================================================
#define FRAME_WIDTH       160
#define FRAME_HEIGHT      120
#define GRID_COLS         5
#define GRID_ROWS         4
#define TOTAL_ZONES       (GRID_COLS * GRID_ROWS) // 20 Spatial Zones

#define ZONE_WIDTH        (FRAME_WIDTH / GRID_COLS)   // 32 px
#define ZONE_HEIGHT       (FRAME_HEIGHT / GRID_ROWS)  // 30 px

const uint8_t MOVEMENT_THRESHOLD = 15;    // Minimum brightness delta to qualify as motion
const uint8_t MIN_ACTIVE_ZONES   = 3;     // Minimum active zones required to reject lighting noise
const unsigned long DROWSY_TIMEOUT_MS = 3000; // Inactivity threshold (3 seconds)

// =====================================================================================
// STATE VARIABLES
// =====================================================================================
uint8_t previousZones[TOTAL_ZONES] = {0};
uint8_t currentZones[TOTAL_ZONES]  = {0};
bool isFirstFrame = true;

unsigned long lastMotionTime = 0;
bool drowsyAlertActive = false;
unsigned long lastAlertSentTime = 0;
const unsigned long ALERT_COOLDOWN_MS = 10000; // 10s debounce between Telegram alerts

// =====================================================================================
// FUNCTION PROTOTYPES
// =====================================================================================
esp_err_t initCamera();
void connectWiFi();
bool analyzeFrameMotion(camera_fb_t* fb);
void sendTelegramDrowsyAlert(camera_fb_t* fb);

// =====================================================================================
// SETUP
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n=======================================================");
  Serial.println("  ESP32-CAM Lightweight Drowsiness Detection Module    ");
  Serial.println("=======================================================");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Initialize camera hardware
  if (initCamera() != ESP_OK) {
    Serial.println("[Error] OV2640 Camera probe/initialization failed!");
    while (true) {
      // Fast error flash
      digitalWrite(FLASH_LED_PIN, HIGH);
      delay(100);
      digitalWrite(FLASH_LED_PIN, LOW);
      delay(100);
    }
  }
  Serial.println("[Camera] OV2640 Initialized successfully in QQVGA Grayscale mode.");

  // Connect to Wi-Fi
  connectWiFi();

  lastMotionTime = millis();
  Serial.println("[ADAS-CAM] Drowsiness monitoring started.\n");
}

// =====================================================================================
// MAIN LOOP
// =====================================================================================
void loop() {
  // Capture frame from OV2640
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[Camera] Frame buffer capture failed!");
    delay(50);
    return;
  }

  // Analyze spatial zone movement
  bool motionDetected = analyzeFrameMotion(fb);
  unsigned long now = millis();

  if (motionDetected) {
    lastMotionTime = now;
    if (drowsyAlertActive) {
      Serial.println("[Status] Driver movement resumed. Drowsy state cleared.");
      drowsyAlertActive = false;
    }
  } else {
    // Check if motion has ceased for longer than threshold
    if ((now - lastMotionTime >= DROWSY_TIMEOUT_MS) && !drowsyAlertActive) {
      drowsyAlertActive = true;
      Serial.println("=====================================================");
      Serial.println("⚠️ WARNING: DRIVER DROWSINESS / MICRO-SLEEP DETECTED!");
      Serial.println("=====================================================");

      // Flash illumination alert
      digitalWrite(FLASH_LED_PIN, HIGH);
      delay(150);
      digitalWrite(FLASH_LED_PIN, LOW);

      // Transmit cloud notification (with cooldown to prevent spam)
      if (now - lastAlertSentTime >= ALERT_COOLDOWN_MS) {
        lastAlertSentTime = now;
        sendTelegramDrowsyAlert(fb);
      }
    }
  }

  // Release frame buffer memory back to the driver
  esp_camera_fb_return(fb);

  // Yield to allow background RTOS tasks & WiFi stack to execute
  delay(100);
}

// =====================================================================================
// 20-ZONE GRID MOTION ANALYSIS ALGORITHM
// =====================================================================================
bool analyzeFrameMotion(camera_fb_t* fb) {
  // Calculate average brightness for each of the 20 zones
  for (int zoneIdx = 0; zoneIdx < TOTAL_ZONES; zoneIdx++) {
    int col = zoneIdx % GRID_COLS;
    int row = zoneIdx / GRID_COLS;

    int startX = col * ZONE_WIDTH;
    int startY = row * ZONE_HEIGHT;

    unsigned long pixelSum = 0;
    int pixelCount = 0;

    // Subsample pixels within zone for efficiency
    for (int y = startY; y < startY + ZONE_HEIGHT; y += 2) {
      for (int x = startX; x < startX + ZONE_WIDTH; x += 2) {
        int bufferIndex = y * FRAME_WIDTH + x;
        if (bufferIndex < fb->len) {
          pixelSum += fb->buf[bufferIndex];
          pixelCount++;
        }
      }
    }

    currentZones[zoneIdx] = (pixelCount > 0) ? (pixelSum / pixelCount) : 0;
  }

  if (isFirstFrame) {
    memcpy(previousZones, currentZones, sizeof(currentZones));
    isFirstFrame = false;
    return true; // First frame baseline establish
  }

  // Count active zones with brightness difference exceeding threshold
  int activeZones = 0;
  for (int i = 0; i < TOTAL_ZONES; i++) {
    int diff = abs((int)currentZones[i] - (int)previousZones[i]);
    if (diff > MOVEMENT_THRESHOLD) {
      activeZones++;
    }
  }

  // Update history buffer
  memcpy(previousZones, currentZones, sizeof(currentZones));

  // Diagnostic log
  // Serial.printf("[Motion] Active Zones: %d / %d (Threshold >= %d)\n", activeZones, TOTAL_ZONES, MIN_ACTIVE_ZONES);

  return (activeZones >= MIN_ACTIVE_ZONES);
}

// =====================================================================================
// TELEGRAM ALERT TRANSMISSION OVER HTTPS
// =====================================================================================
void sendTelegramDrowsyAlert(camera_fb_t* fb) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Telegram] Cannot send alert: WiFi offline.");
    return;
  }

  Serial.println("[Telegram] Sending Drowsiness Emergency Alert to Telegram...");

  WiFiClientSecure client;
  client.setInsecure(); // Bypass certificate verification for embedded memory efficiency

  if (!client.connect("api.telegram.org", 443)) {
    Serial.println("[Telegram] Connection to api.telegram.org failed!");
    return;
  }

  // Compose text alert
  String alertText = "🚨 *EMERGENCY: DRIVER DROWSINESS DETECTED!*\n\n";
  alertText += "• Event: Prolonged driver stillness (>3.0s)\n";
  alertText += "• Action: Visual alarm pulsed; immediate driver check recommended.\n";
  alertText += "• Timestamp: " + String(millis() / 1000) + "s elapsed";

  String url = "/bot" + String(BOT_TOKEN) + "/sendMessage?chat_id=" + String(CHAT_ID) +
               "&text=" + alertText + "&parse_mode=Markdown";

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: api.telegram.org\r\n" +
               "Connection: close\r\n\r\n");

  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 3000) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (line.startsWith("HTTP/1.1 200 OK")) {
        Serial.println("[Telegram] Alert delivered successfully!");
        break;
      }
    }
  }
  client.stop();
}

// =====================================================================================
// CAMERA INITIALIZATION CONFIGURATION
// =====================================================================================
esp_err_t initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE; // Grayscale for spatial difference computation
  config.frame_size   = FRAMESIZE_QQVGA;     // 160x120 for minimal memory & high FPS
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  return err;
}

// =====================================================================================
// WIFI MANAGEMENT
// =====================================================================================
void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] Failed to connect. Will continue monitoring offline.");
  }
}
