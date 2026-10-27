/*
 * =====================================================================================
 * Project: IoT-Enabled Advanced Driver Assistance System (ADAS)
 * Module:  ADAS Main Controller Firmware (Full Version with Acoustic & Telemetry Alerts)
 * Target:  ESP32 DevKit V1
 * Authors: Abhi Shah, Sahilali Dholkawala, Sanghani Karan
 * Institute: Dharmsinh Desai University (DDU)
 *
 * Description:
 *   Real-time collision detection and warning system using an HC-SR04 ultrasonic
 *   sensor with a 10-sample moving-average filter and 4-tier proximity classification.
 *   Provides serial bar visualization, non-blocking acoustic/visual alerts, session
 *   analytics (min/max/event counts), and bidirectional Telegram Bot command & control.
 * =====================================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

// =====================================================================================
// WIFI & TELEGRAM CONFIGURATION
// =====================================================================================
const char* ssid     = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

#define BOT_TOKEN "YOUR_BOT_TOKEN"
#define CHAT_ID   "YOUR_CHAT_ID"

// =====================================================================================
// PIN ASSIGNMENTS
// =====================================================================================
#define TRIG_PIN    5    // Ultrasonic Trigger Pin
#define ECHO_PIN    18   // Ultrasonic Echo Pin (via 3.3k + 3.3k Voltage Divider)
#define RED_LED     4    // Red LED (Danger / Warning Indicator)
#define GREEN_LED   15   // Green LED (Safe / Caution Indicator)
#define BUZZER      13   // 5V Active Buzzer (Acoustic Alert)

// =====================================================================================
// DISTANCE THRESHOLDS (cm)
// =====================================================================================
#define DANGER_DIST  20
#define WARNING_DIST 50
#define SAFE_DIST    100

// =====================================================================================
// SYSTEM OBJECTS & TIMERS
// =====================================================================================
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

unsigned long lastTelegramCheck = 0;
unsigned long lastAlert         = 0;
unsigned long systemStartTime   = 0;
unsigned long lastBlink         = 0;

bool systemArmed      = true;
bool blinkState       = false;
String currentZone    = "CLEAR";
String lastAlertedZone= "CLEAR";

float currentDistance = 999;
float minDistance     = 999;
float maxDistance     = 0;

int dangerCount   = 0;
int warningCount  = 0;
int cautionCount  = 0;
int totalReadings = 0;

// 10-Sample Moving Average Buffer
#define NUM_SAMPLES 10
float samples[NUM_SAMPLES];
int sampleIndex = 0;

// =====================================================================================
// FUNCTION PROTOTYPES
// =====================================================================================
float singleReading();
float getSmoothed();
String getZone(float dist);
void controlOutputs();
void printBar(float distance);
void sendAlert(float distance);
void checkCommands();
String formatTime();
void connectWiFi();

// =====================================================================================
// SETUP
// =====================================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BUZZER, LOW);

    for (int i = 0; i < NUM_SAMPLES; i++) samples[i] = 999;

    Serial.println("\n=================================");
    Serial.println("  ADAS Safety System Starting... ");
    Serial.println("=================================");

    // Startup Hardware Self-Test (LED + Buzzer verification pulse)
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(200);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER, LOW);
    digitalWrite(GREEN_LED, HIGH);
    delay(200);

    connectWiFi();
    secured_client.setInsecure();

    systemStartTime = millis();

    String msg = "=================================\n";
    msg += "   ADAS SAFETY SYSTEM ONLINE\n";
    msg += "=================================\n\n";
    msg += "All modules initialized:\n";
    msg += "  Ultrasonic Sensor: ACTIVE\n";
    msg += "  Red LED (Danger): ACTIVE\n";
    msg += "  Green LED (Safe): ACTIVE\n";
    msg += "  Buzzer Alert: ACTIVE\n";
    msg += "  WiFi: CONNECTED\n";
    msg += "  Signal: " + String(WiFi.RSSI()) + " dBm\n";
    msg += "  IP: " + WiFi.localIP().toString() + "\n\n";
    msg += "Alert Zones:\n";
    msg += "  DANGER:  < 20cm (fast beep)\n";
    msg += "  WARNING: 20-50cm (slow beep)\n";
    msg += "  CAUTION: 50-100cm (watching)\n";
    msg += "  SAFE:    > 100cm (all clear)\n\n";
    msg += "Available Commands:\n";
    msg += "  /status   - Full system report\n";
    msg += "  /distance - Live distance reading\n";
    msg += "  /arm      - Activate all alerts\n";
    msg += "  /disarm   - Pause all alerts\n";
    msg += "  /stats    - Trip safety report\n";
    msg += "  /help     - Show all commands\n\n";
    msg += "System is monitoring. Drive safe!";
    bot.sendMessage(CHAT_ID, msg, "");

    Serial.println("Ready!");
}

// =====================================================================================
// MAIN LOOP
// =====================================================================================
void loop() {
    float raw = singleReading();
    samples[sampleIndex] = raw;
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;
    currentDistance = getSmoothed();

    // Track session min/max records
    totalReadings++;
    if (currentDistance < 400) {
        if (currentDistance < minDistance) minDistance = currentDistance;
        if (currentDistance > maxDistance) maxDistance = currentDistance;
    }

    String prevZone = currentZone;
    currentZone = getZone(currentDistance);

    if (systemArmed) {
        controlOutputs();
    }

    if (currentZone == "DANGER") dangerCount++;
    if (currentZone == "WARNING") warningCount++;
    if (currentZone == "CAUTION") cautionCount++;

    printBar(currentDistance);

    // Non-blocking 5-second IoT check loop
    if (millis() - lastTelegramCheck > 5000) {
        if (systemArmed && currentZone != lastAlertedZone) {
            sendAlert(currentDistance);
        }
        checkCommands();
        lastTelegramCheck = millis();
    }

    delay(200);
}

// =====================================================================================
// HARDWARE OUTPUT CONTROL
// =====================================================================================
void controlOutputs() {
    unsigned long now = millis();

    if (currentZone == "DANGER") {
        if (now - lastBlink > 150) {
            blinkState = !blinkState;
            digitalWrite(RED_LED, blinkState);
            digitalWrite(BUZZER, blinkState);
            lastBlink = now;
        }
        digitalWrite(GREEN_LED, LOW);
    }
    else if (currentZone == "WARNING") {
        if (now - lastBlink > 500) {
            blinkState = !blinkState;
            digitalWrite(RED_LED, blinkState);
            digitalWrite(BUZZER, blinkState);
            lastBlink = now;
        }
        digitalWrite(GREEN_LED, LOW);
    }
    else if (currentZone == "CAUTION") {
        digitalWrite(RED_LED, LOW);
        digitalWrite(BUZZER, LOW);
        if (now - lastBlink > 800) {
            blinkState = !blinkState;
            digitalWrite(GREEN_LED, blinkState);
            lastBlink = now;
        }
    }
    else {
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(BUZZER, LOW);
    }
}

// =====================================================================================
// ULTRASONIC SENSOR SAMPLING & FILTERING
// =====================================================================================
float singleReading() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long d = pulseIn(ECHO_PIN, HIGH, 25000);
    if (d == 0) return 999.0;

    float dist = (d * 0.034) / 2.0;
    if (dist < 2.0 || dist > 400.0) return 999.0;
    return dist;
}

float getSmoothed() {
    float sum = 0;
    int count = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        if (samples[i] > 0 && samples[i] < 500) {
            sum += samples[i];
            count++;
        }
    }
    return count == 0 ? 999.0 : sum / count;
}

String getZone(float dist) {
    if (dist > 400) return "CLEAR";
    if (dist > SAFE_DIST) return "SAFE";
    if (dist > WARNING_DIST) return "CAUTION";
    if (dist > DANGER_DIST) return "WARNING";
    return "DANGER";
}

// =====================================================================================
// ASCII REAL-TIME SERIAL BAR VISUALIZATION
// =====================================================================================
void printBar(float distance) {
    int f = 0;
    if (distance < 400) f = 30 - (int)((distance / 400.0) * 30);
    if (f < 0) f = 0;
    if (f > 30) f = 30;
    Serial.print("[");
    for (int i = 0; i < 30; i++) Serial.print(i < f ? "#" : "-");
    Serial.print("] ");
    if (distance > 400) Serial.print("--- cm");
    else {
        Serial.print(distance, 1);
        Serial.print(" cm");
    }
    Serial.print(" | ");
    Serial.print(currentZone);
    if (currentZone == "DANGER") Serial.print(" | BUZZER:FAST");
    if (currentZone == "WARNING") Serial.print(" | BUZZER:SLOW");
    Serial.println();
}

// =====================================================================================
// TELEGRAM ALERT DISPATCH
// =====================================================================================
void sendAlert(float distance) {
    if (millis() - lastAlert < 10000) return;
    String msg = "";

    if (currentZone == "DANGER") {
        msg = "===========================\n";
        msg += "  COLLISION DANGER ALERT!\n";
        msg += "===========================\n\n";
        msg += "Obstacle detected critically close!\n\n";
        msg += "  Distance: " + String(distance, 1) + " cm\n";
        msg += "  Zone: DANGER (< 20cm)\n";
        msg += "  Red LED: FAST BLINKING\n";
        msg += "  Buzzer: RAPID BEEPING\n\n";
        msg += "Action Required:\n";
        msg += "  APPLY BRAKES IMMEDIATELY!\n";
        msg += "  Collision risk is HIGH!\n\n";
        msg += "Time: " + formatTime() + " since start\n";
        msg += "Danger events so far: " + String(dangerCount);
    }
    else if (currentZone == "WARNING") {
        msg = "===========================\n";
        msg += "  PROXIMITY WARNING!\n";
        msg += "===========================\n\n";
        msg += "Obstacle approaching vehicle!\n\n";
        msg += "  Distance: " + String(distance, 1) + " cm\n";
        msg += "  Zone: WARNING (20-50cm)\n";
        msg += "  Red LED: SLOW BLINKING\n";
        msg += "  Buzzer: SLOW BEEPING\n\n";
        msg += "Action Required:\n";
        msg += "  Reduce speed immediately!\n";
        msg += "  Maintain safe distance.\n\n";
        msg += "Time: " + formatTime() + " since start\n";
        msg += "Warning events so far: " + String(warningCount);
    }
    else if (currentZone == "CAUTION") {
        msg = "===========================\n";
        msg += "  OBSTACLE DETECTED\n";
        msg += "===========================\n\n";
        msg += "Object detected in forward path.\n\n";
        msg += "  Distance: " + String(distance, 1) + " cm\n";
        msg += "  Zone: CAUTION (50-100cm)\n";
        msg += "  Green LED: BLINKING\n";
        msg += "  Buzzer: OFF\n\n";
        msg += "Status: Monitoring obstacle.\n";
        msg += "No immediate action required.\n";
        msg += "System will alert if closer.";
    }
    else {
        msg = "===========================\n";
        msg += "  ALL CLEAR\n";
        msg += "===========================\n\n";
        msg += "No obstacles in forward path.\n\n";
        msg += "  Distance: " + String(distance, 1) + " cm\n";
        msg += "  Zone: SAFE (> 100cm)\n";
        msg += "  Green LED: SOLID ON\n";
        msg += "  Buzzer: OFF\n\n";
        msg += "Status: Safe driving conditions.\n";
        msg += "System continues monitoring.";
    }

    if (msg.length() > 0) {
        bot.sendMessage(CHAT_ID, msg, "");
        lastAlertedZone = currentZone;
        lastAlert = millis();
    }
}

// =====================================================================================
// TELEGRAM COMMAND PROCESSING
// =====================================================================================
void checkCommands() {
    int n = bot.getUpdates(bot.last_message_received + 1);
    while (n) {
        for (int i = 0; i < n; i++) {
            String id = bot.messages[i].chat_id;
            String t = bot.messages[i].text;
            String name = bot.messages[i].from_name;

            if (id != CHAT_ID) {
                bot.sendMessage(id, "Access Denied!\nThis system is private.", "");
                continue;
            }

            if (t == "/status") {
                String s = "===========================\n";
                s += "  ADAS SYSTEM STATUS\n";
                s += "===========================\n\n";
                s += "System State: " + String(systemArmed ? "ARMED & ACTIVE" : "DISARMED") + "\n\n";
                s += "Current Readings:\n";
                s += "  Distance: " + String(currentDistance, 1) + " cm\n";
                s += "  Zone: " + currentZone + "\n";
                s += "  Red LED: ";
                if (currentZone == "DANGER") s += "FAST BLINK";
                else if (currentZone == "WARNING") s += "SLOW BLINK";
                else s += "OFF";
                s += "\n  Green LED: ";
                if (currentZone == "SAFE" || currentZone == "CLEAR") s += "SOLID ON";
                else if (currentZone == "CAUTION") s += "BLINKING";
                else s += "OFF";
                s += "\n  Buzzer: ";
                if (currentZone == "DANGER") s += "RAPID BEEP";
                else if (currentZone == "WARNING") s += "SLOW BEEP";
                else s += "SILENT";
                s += "\n\nNetwork:\n";
                s += "  WiFi Signal: " + String(WiFi.RSSI()) + " dBm\n";
                s += "  IP Address: " + WiFi.localIP().toString() + "\n";
                s += "\nSession Info:\n";
                s += "  Uptime: " + formatTime() + "\n";
                s += "  Total Readings: " + String(totalReadings) + "\n";
                s += "  Closest Object: " + String(minDistance, 1) + " cm\n";
                s += "  Farthest Object: " + String(maxDistance, 1) + " cm\n";
                s += "\nSafety Events:\n";
                s += "  Danger alerts: " + String(dangerCount) + "\n";
                s += "  Warning alerts: " + String(warningCount) + "\n";
                s += "  Caution alerts: " + String(cautionCount) + "\n";
                s += "\nMemory: " + String(ESP.getFreeHeap()) + " bytes free";
                bot.sendMessage(id, s, "");
            }
            else if (t == "/distance") {
                float d1 = singleReading();
                delay(100);
                float d2 = singleReading();
                delay(100);
                float d3 = singleReading();

                float avg = 999;
                int cnt = 0;
                float sum = 0;
                if (d1 < 500) { sum += d1; cnt++; }
                if (d2 < 500) { sum += d2; cnt++; }
                if (d3 < 500) { sum += d3; cnt++; }
                if (cnt > 0) avg = sum / cnt;

                String s = "===========================\n";
                s += "  LIVE DISTANCE READING\n";
                s += "===========================\n\n";
                s += "Three consecutive measurements:\n\n";
                s += "  Reading 1: " + String(d1, 1) + " cm\n";
                s += "  Reading 2: " + String(d2, 1) + " cm\n";
                s += "  Reading 3: " + String(d3, 1) + " cm\n\n";
                s += "  Average: " + String(avg, 1) + " cm\n";
                s += "  Zone: " + getZone(avg) + "\n\n";
                s += "Session records:\n";
                s += "  Closest: " + String(minDistance, 1) + " cm\n";
                s += "  Farthest: " + String(maxDistance, 1) + " cm";
                bot.sendMessage(id, s, "");
            }
            else if (t == "/arm") {
                systemArmed = true;
                lastAlertedZone = "NONE";
                String s = "===========================\n";
                s += "  SYSTEM ARMED\n";
                s += "===========================\n\n";
                s += "All safety modules activated:\n\n";
                s += "  Ultrasonic Sensor: MONITORING\n";
                s += "  Red LED Alert: ENABLED\n";
                s += "  Green LED Status: ENABLED\n";
                s += "  Buzzer Alert: ENABLED\n";
                s += "  Telegram Alerts: ENABLED\n\n";
                s += "System will alert on obstacles.\n";
                s += "Drive safe, " + name + "!";
                bot.sendMessage(id, s, "");
            }
            else if (t == "/disarm") {
                systemArmed = false;
                digitalWrite(RED_LED, LOW);
                digitalWrite(GREEN_LED, HIGH);
                digitalWrite(BUZZER, LOW);
                String s = "===========================\n";
                s += "  SYSTEM DISARMED\n";
                s += "===========================\n\n";
                s += "All alerts paused:\n\n";
                s += "  Buzzer: SILENCED\n";
                s += "  Red LED: OFF\n";
                s += "  Green LED: ON (standby)\n";
                s += "  Telegram Alerts: PAUSED\n\n";
                s += "Sensor still reading in background.\n";
                s += "Send /arm to reactivate.";
                bot.sendMessage(id, s, "");
            }
            else if (t == "/stats") {
                String s = "===========================\n";
                s += "  TRIP SAFETY REPORT\n";
                s += "===========================\n\n";
                s += "Session Duration: " + formatTime() + "\n\n";
                s += "Safety Events:\n";
                s += "  Danger (< 20cm): " + String(dangerCount) + " times\n";
                s += "  Warning (20-50cm): " + String(warningCount) + " times\n";
                s += "  Caution (50-100cm): " + String(cautionCount) + " times\n\n";
                s += "Distance Records:\n";
                s += "  Closest approach: " + String(minDistance, 1) + " cm\n";
                s += "  Farthest reading: " + String(maxDistance, 1) + " cm\n";
                s += "  Current distance: " + String(currentDistance, 1) + " cm\n\n";
                s += "System Health:\n";
                s += "  Total readings: " + String(totalReadings) + "\n";
                s += "  WiFi signal: " + String(WiFi.RSSI()) + " dBm\n";
                s += "  Free memory: " + String(ESP.getFreeHeap()) + " bytes\n";
                s += "  System: " + String(systemArmed ? "ARMED" : "DISARMED");
                bot.sendMessage(id, s, "");
            }
            else if (t == "/start" || t == "/help") {
                String s = "===========================\n";
                s += "  ADAS COMMAND CENTER\n";
                s += "===========================\n\n";
                s += "Available Commands:\n\n";
                s += "  /status\n";
                s += "    Full system status report\n";
                s += "    Shows distance, zone, LEDs,\n";
                s += "    buzzer, WiFi, memory usage\n\n";
                s += "  /distance\n";
                s += "    Takes 3 live measurements\n";
                s += "    Shows average and zone\n\n";
                s += "  /arm\n";
                s += "    Activate all safety alerts\n";
                s += "    LEDs + Buzzer + Telegram\n\n";
                s += "  /disarm\n";
                s += "    Pause all alerts\n";
                s += "    Sensor keeps reading\n\n";
                s += "  /stats\n";
                s += "    Complete trip safety report\n";
                s += "    Events, distances, health\n\n";
                s += "  /help\n";
                s += "    Show this command list";
                bot.sendMessage(id, s, "");
            }
            else {
                String s = "Unknown command: " + t + "\n\n";
                s += "Try /help for available commands.";
                bot.sendMessage(id, s, "");
            }
        }
        n = bot.getUpdates(bot.last_message_received + 1);
    }
}

// =====================================================================================
// UTILITY FUNCTIONS
// =====================================================================================
String formatTime() {
    unsigned long s = millis() / 1000;
    unsigned long m = s / 60;
    unsigned long h = m / 60;
    s %= 60;
    m %= 60;
    String r = "";
    if (h > 0) r += String(h) + "h ";
    if (m > 0) r += String(m) + "m ";
    r += String(s) + "s";
    return r;
}

void connectWiFi() {
    Serial.print("WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int a = 0;
    while (WiFi.status() != WL_CONNECTED && a < 30) {
        delay(500);
        Serial.print(".");
        a++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" OK!");
    } else {
        Serial.println(" FAIL!");
        ESP.restart();
    }
}
