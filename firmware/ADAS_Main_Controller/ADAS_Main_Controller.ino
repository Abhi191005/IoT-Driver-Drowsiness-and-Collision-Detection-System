/*
 * =====================================================================================
 * Project: IoT-Enabled Advanced Driver Assistance System (ADAS)
 * Module:  ADAS Main Controller Firmware
 * Target:  ESP32 DevKit V1
 * Authors: Abhi Shah, Sahilali Dholkawala, Sanghani Karan
 * Institute: Dharmsinh Desai University (DDU)
 *
 * Description:
 *   Real-time collision detection and warning system using an HC-SR04 ultrasonic
 *   sensor with a 10-sample circular moving-average filter and 4-zone proximity
 *   classification (Danger, Warning, Caution, Safe). Implements a non-blocking
 *   multitasking architecture with millis() separating real-time safety actuation
 *   (5 Hz) from background Telegram Bot IoT communication (0.2 Hz).
 * =====================================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

// =====================================================================================
// PIN DEFINITIONS & HARDWARE ASSIGNMENTS
// =====================================================================================
#define TRIG_PIN       5    // HC-SR04 Trigger Pin (Output, 10us HIGH pulse)
#define ECHO_PIN       18   // HC-SR04 Echo Pin (Input, via 3.3k + 3.3k Voltage Divider)
#define RED_LED_PIN    19   // Red Indicator LED (Danger / Warning Alert)
#define GREEN_LED_PIN  21   // Green Indicator LED (Caution / Safe Status)
#define BUZZER_PIN     22   // 5V Active Buzzer (Acoustic Audio Alert)

// =====================================================================================
// NETWORK & TELEGRAM BOT CONFIGURATION
// =====================================================================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Telegram credentials (obtained via @BotFather and @userinfobot)
#define BOT_TOKEN "YOUR_TELEGRAM_BOT_TOKEN"
#define CHAT_ID   "YOUR_AUTHORIZED_CHAT_ID"

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

// =====================================================================================
// PROXIMITY ZONE DEFINITIONS
// =====================================================================================
enum Zone {
  ZONE_DANGER,   // < 20 cm: Fast Red blink, fast buzzer beep, Telegram alert
  ZONE_WARNING,  // 20 - 50 cm: Slow Red blink, slow buzzer beep, Telegram alert
  ZONE_CAUTION,  // 50 - 100 cm: Green blink, silent buzzer
  ZONE_SAFE      // > 100 cm: Green solid ON, Red OFF, silent buzzer
};

// =====================================================================================
// MOVING AVERAGE FILTER CONFIGURATION
// =====================================================================================
#define NUM_READINGS 10
float readings[NUM_READINGS];
int readIndex = 0;
float runningTotal = 0.0;
bool filterPopulated = false;

// =====================================================================================
// TIMING CONSTANTS (NON-BLOCKING MULTITASKING)
// =====================================================================================
const unsigned long SENSOR_INTERVAL   = 200;   // 5 Hz polling rate for real-time safety
const unsigned long TELEGRAM_INTERVAL = 5000;  // 0.2 Hz polling rate for IoT background task

const unsigned long BLINK_DANGER_MS   = 150;   // Rapid hazard alert cycle
const unsigned long BLINK_WARNING_MS  = 500;   // Cautionary alert cycle
const unsigned long BLINK_CAUTION_MS  = 800;   // Safe proximity awareness cycle

// =====================================================================================
// SYSTEM STATE VARIABLES
// =====================================================================================
Zone currentZone = ZONE_SAFE;
Zone previousReportedZone = ZONE_SAFE;

bool systemArmed = true;         // Arm/Disarm remote control state
bool alertState = false;         // Output toggle state for blinking LEDs / Buzzer

unsigned long lastSensorTime   = 0;
unsigned long lastTelegramTime = 0;
unsigned long lastBlinkTime    = 0;

// Telemetry & Statistics
unsigned long dangerEventCount  = 0;
unsigned long warningEventCount = 0;

// =====================================================================================
// FUNCTION PROTOTYPES
// =====================================================================================
float measureRawDistance();
float getSmoothedDistance();
Zone classifyZone(float distance);
void updateHardwareOutputs(Zone zone);
void handleTelegramUpdates();
void processIncomingCommand(const telegramMessage& msg);
void sendZoneChangeNotification(Zone newZone, float distance);
void connectWiFi();

// =====================================================================================
// SETUP
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n=======================================================");
  Serial.println("  IoT-Enabled ADAS Main Controller Starting Up...     ");
  Serial.println("=======================================================");

  // Pin mode configuration
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize outputs to safe inactive state
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize circular moving-average buffer
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = 200.0; // Default baseline at SAFE distance
  }
  runningTotal = 200.0 * NUM_READINGS;

  // Configure SSL client: bypass strict root CA chain check for embedded reliability
  secured_client.setInsecure();

  // Connect to Wi-Fi network
  connectWiFi();

  // Startup verification pulse
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(500);
  digitalWrite(GREEN_LED_PIN, LOW);

  Serial.println("System Initialized. Entering Non-Blocking Scheduler Loop.\n");
}

// =====================================================================================
// MAIN LOOP: NON-BLOCKING DUAL-LAYER SCHEDULER
// =====================================================================================
void loop() {
  unsigned long currentMillis = millis();

  // -----------------------------------------------------------------------------------
  // LAYER 1: REAL-TIME SAFETY LAYER (Executes every 200 ms / 5 Hz)
  // -----------------------------------------------------------------------------------
  if (currentMillis - lastSensorTime >= SENSOR_INTERVAL) {
    lastSensorTime = currentMillis;

    float smoothedDistance = getSmoothedDistance();
    currentZone = classifyZone(smoothedDistance);

    // Track state metrics
    if (currentZone != previousReportedZone) {
      if (currentZone == ZONE_DANGER) dangerEventCount++;
      if (currentZone == ZONE_WARNING) warningEventCount++;
    }

    // Print diagnostic telemetry to UART
    Serial.printf("[ADAS] Dist: %6.1f cm | Zone: %-8s | Armed: %s\n",
                  smoothedDistance,
                  currentZone == ZONE_DANGER  ? "DANGER" :
                  currentZone == ZONE_WARNING ? "WARNING" :
                  currentZone == ZONE_CAUTION ? "CAUTION" : "SAFE",
                  systemArmed ? "YES" : "NO");
  }

  // Drive Hardware Actuators (Continuous Blink Timing)
  updateHardwareOutputs(currentZone);

  // -----------------------------------------------------------------------------------
  // LAYER 2: LOGGING & IOT BACKGROUND LAYER (Executes every 5000 ms / 0.2 Hz)
  // -----------------------------------------------------------------------------------
  if (currentMillis - lastTelegramTime >= TELEGRAM_INTERVAL) {
    lastTelegramTime = currentMillis;

    // Verify WiFi status; auto-reconnect if dropped
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Lost connection. Attempting background reconnect...");
      WiFi.reconnect();
    } else {
      // Send Telegram alert if proximity zone has transitioned
      if (currentZone != previousReportedZone) {
        sendZoneChangeNotification(currentZone, readings[readIndex]);
        previousReportedZone = currentZone;
      }

      // Check and execute incoming remote commands
      handleTelegramUpdates();
    }
  }
}

// =====================================================================================
// ULTRASONIC SENSOR ACQUISITION & FILTERING
// =====================================================================================

// Trigger HC-SR04 and compute distance using acoustic time-of-flight
float measureRawDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure round-trip echo pulse duration (30ms timeout = ~500cm max range)
  long durationUs = pulseIn(ECHO_PIN, HIGH, 30000);

  if (durationUs <= 0) {
    return -1.0; // Out-of-range or sensor timeout
  }

  // Distance (cm) = (duration_us * 0.034 cm/us) / 2
  float distance = (durationUs * 0.0343) / 2.0;

  if (distance < 2.0 || distance > 450.0) {
    return -1.0; // Reject out-of-spec physical bounds
  }

  return distance;
}

// 10-sample circular moving-average filter with outlier rejection
float getSmoothedDistance() {
  float newReading = measureRawDistance();

  // Outlier Handling: If reading is invalid, retain last known valid sample
  if (newReading < 0) {
    int prevIndex = (readIndex - 1 + NUM_READINGS) % NUM_READINGS;
    newReading = readings[prevIndex];
  }

  // Circular buffer update
  runningTotal -= readings[readIndex];
  readings[readIndex] = newReading;
  runningTotal += newReading;
  readIndex = (readIndex + 1) % NUM_READINGS;

  return runningTotal / (float)NUM_READINGS;
}

// =====================================================================================
// ZONE CLASSIFICATION STATE MACHINE
// =====================================================================================
Zone classifyZone(float distance) {
  if (distance < 20.0)  return ZONE_DANGER;
  if (distance < 50.0)  return ZONE_WARNING;
  if (distance < 100.0) return ZONE_CAUTION;
  return ZONE_SAFE;
}

// =====================================================================================
// HARDWARE OUTPUT ACTUATION (NON-BLOCKING)
// =====================================================================================
void updateHardwareOutputs(Zone zone) {
  unsigned long now = millis();

  switch (zone) {
    case ZONE_DANGER:
      // Fast blinking and beeping (150ms interval)
      if (now - lastBlinkTime >= BLINK_DANGER_MS) {
        lastBlinkTime = now;
        alertState = !alertState;
        digitalWrite(RED_LED_PIN, alertState ? HIGH : LOW);
        digitalWrite(BUZZER_PIN, (systemArmed && alertState) ? HIGH : LOW);
        digitalWrite(GREEN_LED_PIN, LOW);
      }
      break;

    case ZONE_WARNING:
      // Moderate warning blinking and beeping (500ms interval)
      if (now - lastBlinkTime >= BLINK_WARNING_MS) {
        lastBlinkTime = now;
        alertState = !alertState;
        digitalWrite(RED_LED_PIN, alertState ? HIGH : LOW);
        digitalWrite(BUZZER_PIN, (systemArmed && alertState) ? HIGH : LOW);
        digitalWrite(GREEN_LED_PIN, LOW);
      }
      break;

    case ZONE_CAUTION:
      // Slow awareness blink (800ms interval), buzzer silent
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      if (now - lastBlinkTime >= BLINK_CAUTION_MS) {
        lastBlinkTime = now;
        alertState = !alertState;
        digitalWrite(GREEN_LED_PIN, alertState ? HIGH : LOW);
      }
      break;

    case ZONE_SAFE:
    default:
      // Solid green, red and buzzer completely off
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, HIGH);
      break;
  }
}

// =====================================================================================
// TELEGRAM NOTIFICATIONS & COMMAND HANDLERS
// =====================================================================================

void sendZoneChangeNotification(Zone newZone, float distance) {
  String msg = "";
  switch (newZone) {
    case ZONE_DANGER:
      msg = "🚨 *COLLISION DANGER!*\nObstacle distance: " + String(distance, 1) + " cm\n*Action*: Immediate braking required!";
      break;
    case ZONE_WARNING:
      msg = "⚠️ *PROXIMITY WARNING*\nObstacle detected at " + String(distance, 1) + " cm.";
      break;
    case ZONE_CAUTION:
      msg = "ℹ️ *Caution Zone*\nForward path obstacle at " + String(distance, 1) + " cm.";
      break;
    case ZONE_SAFE:
      msg = "✅ *Path Clear*\nObstacle cleared (> 100 cm). Safe to proceed.";
      break;
  }

  if (msg.length() > 0) {
    bot.sendMessage(CHAT_ID, msg, "Markdown");
  }
}

void handleTelegramUpdates() {
  int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

  while (numNewMessages) {
    for (int i = 0; i < numNewMessages; i++) {
      telegramMessage& msg = bot.messages[i];

      // Validate security: reject messages not from authorized chat ID
      if (msg.chat_id != CHAT_ID) {
        bot.sendMessage(msg.chat_id, "⛔ Unauthorized Access Rejected.", "");
        continue;
      }

      processIncomingCommand(msg);
    }
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
}

void processIncomingCommand(const telegramMessage& msg) {
  String text = msg.text;
  String from = msg.from_name;

  Serial.println("[Telegram] Received command: " + text);

  if (text == "/start" || text == "/help") {
    String welcome = "🚗 *ADAS IoT Telemetry & Control*\n\n";
    welcome += "Available Commands:\n";
    welcome += "• `/status` - Current vehicle distance & zone\n";
    welcome += "• `/distance` - Trigger 3-sample burst distance check\n";
    welcome += "• `/arm` - Enable physical buzzer & red LED alerts\n";
    welcome += "• `/disarm` - Mute buzzer alerts\n";
    welcome += "• `/stats` - View session trip safety statistics\n";
    bot.sendMessage(msg.chat_id, welcome, "Markdown");
  }
  else if (text == "/status") {
    int prevIndex = (readIndex - 1 + NUM_READINGS) % NUM_READINGS;
    float currentDist = readings[prevIndex];
    unsigned long uptimeSec = millis() / 1000;

    String status = "📊 *ADAS Status Report*\n\n";
    status += "• *Proximity Zone*: " + String(currentZone == ZONE_DANGER ? "🚨 DANGER" :
                                                currentZone == ZONE_WARNING ? "⚠️ WARNING" :
                                                currentZone == ZONE_CAUTION ? "🟡 CAUTION" : "🟢 SAFE") + "\n";
    status += "• *Smoothed Distance*: " + String(currentDist, 1) + " cm\n";
    status += "• *Alarm System*: " + String(systemArmed ? "ARMED" : "DISARMED (Muted)") + "\n";
    status += "• *Free SRAM*: " + String(ESP.getFreeHeap() / 1024) + " KB\n";
    status += "• *Controller Uptime*: " + String(uptimeSec / 60) + "m " + String(uptimeSec % 60) + "s\n";
    bot.sendMessage(msg.chat_id, status, "Markdown");
  }
  else if (text == "/distance") {
    float sum = 0.0;
    int valid = 0;
    for (int k = 0; k < 3; k++) {
      float d = measureRawDistance();
      if (d > 0) { sum += d; valid++; }
      delay(30);
    }
    float avg = (valid > 0) ? (sum / valid) : -1.0;
    bot.sendMessage(msg.chat_id, "📏 *Active Distance Scan*: " + String(avg, 1) + " cm", "Markdown");
  }
  else if (text == "/arm") {
    systemArmed = true;
    bot.sendMessage(msg.chat_id, "🔔 *System ARMED*: Acoustic alerts and hazard LEDs are active.", "Markdown");
  }
  else if (text == "/disarm") {
    systemArmed = false;
    digitalWrite(BUZZER_PIN, LOW);
    bot.sendMessage(msg.chat_id, "🔕 *System DISARMED*: Buzzer alerts muted. Visual LEDs remain active.", "Markdown");
  }
  else if (text == "/stats") {
    String stats = "📈 *Trip Hazard Statistics*\n\n";
    stats += "• Danger Events Logged: " + String(dangerEventCount) + "\n";
    stats += "• Warning Events Logged: " + String(warningEventCount) + "\n";
    stats += "• Total Uptime: " + String(millis() / 60000) + " minutes\n";
    bot.sendMessage(msg.chat_id, stats, "Markdown");
  }
  else {
    bot.sendMessage(msg.chat_id, "❓ Unknown command. Type `/help` for options.", "Markdown");
  }
}

// =====================================================================================
// WIFI MANAGEMENT
// =====================================================================================
void connectWiFi() {
  Serial.print("[WiFi] Connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected successfully!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] Initial connection timeout. Will retry in background loop.");
  }
}
