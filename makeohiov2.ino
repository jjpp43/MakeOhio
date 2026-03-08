/*
 * BinIQ - Smart Sorting Bin
 * SunFounder ESP32 WROOM 32E + ESP32 Camera Extension + OV2640
 *
 * IMPORTANT HARDWARE NOTES
 * - Camera mapping below is for the SunFounder camera extension.
 * - microSD uses IO2, IO4, IO12, IO13, IO14, IO15.
 * - IO16/IO17 are occupied by PSRAM and are not broken out.
 * - IO33 should NOT be used to drive WS2812.
 * - IO0/IO2/IO5/IO12/IO15 are strapping pins; avoid them if possible.
 *
 * CURRENT BEST-EFFORT WIRING FOR TESTING
 * - IR_LEFT_PIN  -> IO13  (shared SD pin; okay only if SD not used)
 * - IR_RIGHT_PIN -> IO14  (shared SD pin; okay only if SD not used)
 * - BUZZER_PIN   -> IO4   (shared SD pin; okay only if SD not used)
 * - SERVO_PIN    -> IO2   (strapping pin; use cautiously, or disable servo)
 * - WS2812       -> DISABLED by default
 *
 * If uploads become flaky again, disconnect the servo and buzzer first.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "esp_camera.h"
#include "mbedtls/base64.h"
#include "esp_heap_caps.h"

#define USE_SERVO       1
#define USE_WS2812      1

#if USE_WS2812
  #include <Adafruit_NeoPixel.h>
#endif

// ─────────────────────────────────────────────
// DEBUG
// ─────────────────────────────────────────────
#define DEBUG 1
#if DEBUG
  #define DBG(tag, msg)        Serial.printf("[%-6s] %s\n", tag, msg)
  #define DBG_F(tag, fmt, ...) Serial.printf("[%-6s] " fmt "\n", tag, ##__VA_ARGS__)
#else
  #define DBG(tag, msg)
  #define DBG_F(tag, fmt, ...)
#endif

// ─────────────────────────────────────────────
// USER CONFIG
// ─────────────────────────────────────────────
const char* WIFI_SSID      = "Junna";
const char* WIFI_PASSWORD  = "11110000";
const char* GOOGLE_API_KEY = "AIzaSyCDHMwv6B9lKuu6_k-tdUXXVI6F8enB_vQ";

// ─────────────────────────────────────────────
// PIN DEFINITIONS
// BEST-EFFORT TEST CONFIG FOR SUNFOUNDER CAMERA EXTENSION
// ─────────────────────────────────────────────
#define IR_LEFT_PIN     13
#define IR_RIGHT_PIN    14
#define BUZZER_PIN      4

#if USE_SERVO
  #define SERVO_PIN     2
#endif

#if USE_WS2812
  #define LED_PIN       15   // not recommended on this board; disabled by default
  #define NUM_LEDS      8
#endif

// ─────────────────────────────────────────────
// CONSTANTS
// ─────────────────────────────────────────────
#define FEEDBACK_MS        2000
#define DETECTION_COOLDOWN 1200
#define SETTLE_DELAY_MS    500
#define API_COOLDOWN_MS    5000

#define SERVO_NEUTRAL   90
#define SERVO_RECYCLE   180
#define SERVO_WASTE     0

// ─────────────────────────────────────────────
// CAMERA PIN CONFIG — SunFounder ESP32 Camera Extension / OV2640
// ─────────────────────────────────────────────
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   33
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D0      5
#define CAM_PIN_D1      18
#define CAM_PIN_D2      19
#define CAM_PIN_D3      21
#define CAM_PIN_D4      36
#define CAM_PIN_D5      39
#define CAM_PIN_D6      34
#define CAM_PIN_D7      35
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// ─────────────────────────────────────────────
// RECYCLABLE KEYWORDS
// ─────────────────────────────────────────────
const char* RECYCLABLE_KEYWORDS[] = {
  "bottle", "plastic bottle", "can", "tin can", "aluminum",
  "cardboard", "paper", "newspaper", "magazine", "carton",
  "glass bottle", "jar", "container", "packaging",
  "plastic bag", "cup", "jug", "box"
};
const int NUM_RECYCLABLE_KEYWORDS = 18;

// ─────────────────────────────────────────────
// OBJECTS
// ─────────────────────────────────────────────
#if USE_WS2812
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
#endif

#if USE_SERVO
Servo sortServo;
#endif

// ─────────────────────────────────────────────
// GLOBAL STATE
// ─────────────────────────────────────────────
bool cameraReady = false;
unsigned long lastDetectionTime = 0;
unsigned long lastApiCallTime = 0;

// ─────────────────────────────────────────────
// FUNCTION PROTOTYPES
// ─────────────────────────────────────────────
bool   initCamera();
bool   testCameraCapture();
String captureAndEncodeImage();
bool   classifyWithVisionAPI(const String& base64Image, String& detectedLabel);
bool   isRecyclable(const String& label);
void   sortItem(bool placedOnRecycle, bool itemIsRecyclable);
void   giveFeedback(bool placedOnRecycle, bool isRecyclable);
void   setAllLEDs(uint32_t color);
void   playCorrectTone();
void   playIncorrectTone();
void   resetState();
void   waitForSensorsToClear();
void   logHeapStatus();

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n╔══════════════════════════════╗");
  Serial.println("║   BinIQ - Smart Sorting Bin  ║");
  Serial.println("║   SunFounder Camera Ext      ║");
  Serial.println("╚══════════════════════════════╝");

  DBG("BOOT", "Initializing pins...");
  pinMode(IR_LEFT_PIN, INPUT_PULLUP);
  pinMode(IR_RIGHT_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  DBG_F("BOOT", "IR Left  → IO%d", IR_LEFT_PIN);
  DBG_F("BOOT", "IR Right → IO%d", IR_RIGHT_PIN);
  DBG_F("BOOT", "Buzzer   → IO%d", BUZZER_PIN);

#if USE_WS2812
  DBG("BOOT", "Initializing WS2812 LEDs...");
  strip.begin();
  strip.setBrightness(80);
  strip.show();
  DBG_F("BOOT", "LED strip on IO%d", LED_PIN);
#else
  DBG("BOOT", "WS2812 disabled in code");
#endif

#if USE_SERVO
  DBG("BOOT", "Initializing servo...");
  sortServo.setPeriodHertz(50);
  sortServo.attach(SERVO_PIN, 500, 2400);
  sortServo.write(SERVO_NEUTRAL);
  delay(500);
  DBG_F("BOOT", "Servo on IO%d", SERVO_PIN);
#else
  DBG("BOOT", "Servo disabled in code");
#endif

  logHeapStatus();

  DBG("BOOT", "Initializing camera...");
  cameraReady = initCamera();

  if (!cameraReady) {
    DBG("ERROR", "Camera init FAILED");
    setAllLEDs(0);
  } else {
    DBG("BOOT", "Camera initialized ✓");
    if (testCameraCapture()) {
      DBG("BOOT", "Camera self-test passed ✓");
    } else {
      DBG("ERROR", "Camera self-test failed");
      cameraReady = false;
    }
  }

  DBG_F("BOOT", "Connecting to WiFi SSID: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    DBG_F("BOOT", "WiFi connected ✓ — IP: %s", WiFi.localIP().toString().c_str());
    DBG_F("BOOT", "RSSI: %d dBm", WiFi.RSSI());
  } else {
    DBG("ERROR", "WiFi connection failed");
  }

  DBG("BOOT", "Ready");
}

// ─────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────
void loop() {
  if (millis() - lastDetectionTime < DETECTION_COOLDOWN) {
    delay(30);
    return;
  }

  bool leftTriggered  = (digitalRead(IR_LEFT_PIN)  == LOW);
  bool rightTriggered = (digitalRead(IR_RIGHT_PIN) == LOW);

  if (!leftTriggered && !rightTriggered) {
    delay(50);
    return;
  }

  Serial.println("\n──────────────────────────────────────────");
  DBG_F("IR", "Initial → LEFT:%s RIGHT:%s",
        leftTriggered ? "TRIGGERED" : "idle",
        rightTriggered ? "TRIGGERED" : "idle");

  delay(SETTLE_DELAY_MS);

  leftTriggered  = (digitalRead(IR_LEFT_PIN)  == LOW);
  rightTriggered = (digitalRead(IR_RIGHT_PIN) == LOW);

  DBG_F("IR", "Settled → LEFT:%s RIGHT:%s",
        leftTriggered ? "TRIGGERED" : "idle",
        rightTriggered ? "TRIGGERED" : "idle");

  if (!leftTriggered && !rightTriggered) {
    DBG("IR", "No valid trigger after settle");
    return;
  }

  bool placedOnRecycle = leftTriggered;
  if (leftTriggered && rightTriggered) {
    DBG("IR", "Both sensors active — defaulting LEFT");
    placedOnRecycle = true;
  }

  DBG_F("IR", "Detected on %s",
        placedOnRecycle ? "LEFT/RECYCLING" : "RIGHT/WASTE");

  if (!cameraReady) {
    DBG("ERROR", "Camera not ready");
    playIncorrectTone();
    waitForSensorsToClear();
    lastDetectionTime = millis();
    return;
  }
DBG("CAM", "Waiting 700ms so item is fully in frame...");
delay(500);
  String base64Image = captureAndEncodeImage();
  if (base64Image.isEmpty()) {
    DBG("ERROR", "Image capture failed");
    playIncorrectTone();
    waitForSensorsToClear();
    lastDetectionTime = millis();
    return;
  }

  if (millis() - lastApiCallTime < API_COOLDOWN_MS) {
    DBG("API", "Cooldown active — skipping API call");
    DBG_F("API", "Wait %lu ms", API_COOLDOWN_MS - (millis() - lastApiCallTime));
    playIncorrectTone();
    waitForSensorsToClear();
    lastDetectionTime = millis();
    return;
  }

  DBG("API", "Sending image to Vision API...");
  String detectedLabel = "";
  bool classified = classifyWithVisionAPI(base64Image, detectedLabel);
  lastApiCallTime = millis();

  if (!classified) {
    DBG("ERROR", "Classification failed");
    playIncorrectTone();
    waitForSensorsToClear();
    lastDetectionTime = millis();
    return;
  }

  DBG_F("CLASS", "Top label: '%s'", detectedLabel.c_str());

  bool itemIsRecyclable = isRecyclable(detectedLabel);
  DBG_F("CLASS", "Final classification: %s",
        itemIsRecyclable ? "RECYCLABLE" : "WASTE");

  sortItem(placedOnRecycle, itemIsRecyclable);

  bool correct = (placedOnRecycle == itemIsRecyclable);
  DBG_F("RESULT", "Placed: %s | Should be: %s | %s",
        placedOnRecycle ? "RECYCLE" : "WASTE",
        itemIsRecyclable ? "RECYCLE" : "WASTE",
        correct ? "CORRECT" : "INCORRECT");

  giveFeedback(placedOnRecycle, itemIsRecyclable);

  delay(FEEDBACK_MS);
  resetState();
  waitForSensorsToClear();
  lastDetectionTime = millis();
}

// ─────────────────────────────────────────────
// CAMERA INIT
// ─────────────────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = CAM_PIN_D0;
  config.pin_d1       = CAM_PIN_D1;
  config.pin_d2       = CAM_PIN_D2;
  config.pin_d3       = CAM_PIN_D3;
  config.pin_d4       = CAM_PIN_D4;
  config.pin_d5       = CAM_PIN_D5;
  config.pin_d6       = CAM_PIN_D6;
  config.pin_d7       = CAM_PIN_D7;
  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_pclk     = CAM_PIN_PCLK;
  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
  DBG("CAM", "PSRAM found");
  config.frame_size   = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
} else {
  DBG("CAM", "PSRAM not found");
  config.frame_size   = FRAMESIZE_QQVGA;
  config.jpeg_quality = 15;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_DRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
}

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DBG_F("ERROR", "esp_camera_init failed: 0x%x", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    DBG("ERROR", "Camera sensor not detected");
    return false;
  }

  DBG_F("CAM", "Sensor detected, PID: 0x%x", s->id.PID);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);

  for (int i = 0; i < 2; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      DBG_F("CAM", "Warm-up frame %d OK (%d bytes)", i + 1, fb->len);
      esp_camera_fb_return(fb);
    } else {
      DBG_F("ERROR", "Warm-up frame %d failed", i + 1);
    }
    delay(100);
  }

  return true;
}

bool testCameraCapture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    DBG("ERROR", "Self-test capture failed");
    return false;
  }
  DBG_F("CAM", "Self-test OK (%d bytes)", fb->len);
  esp_camera_fb_return(fb);
  return true;
}

// ─────────────────────────────────────────────
// CAPTURE + BASE64
// ─────────────────────────────────────────────
String captureAndEncodeImage() {
  DBG("CAM", "Capturing frame...");
  camera_fb_t* fb = esp_camera_fb_get();

  if (!fb) {
    DBG("ERROR", "esp_camera_fb_get returned NULL");
    return "";
  }

  if (fb->len == 0 || fb->buf == nullptr) {
    DBG("ERROR", "Invalid frame buffer");
    esp_camera_fb_return(fb);
    return "";
  }

  size_t encodedLen = 0;
  size_t base64Len = ((fb->len + 2) / 3) * 4 + 1;
  unsigned char* base64Buf = (unsigned char*)malloc(base64Len);

  if (!base64Buf) {
    DBG("ERROR", "malloc failed for base64 buffer");
    esp_camera_fb_return(fb);
    return "";
  }

  int ret = mbedtls_base64_encode(base64Buf, base64Len, &encodedLen, fb->buf, fb->len);
  esp_camera_fb_return(fb);

  if (ret != 0) {
    DBG_F("ERROR", "base64 encode failed: %d", ret);
    free(base64Buf);
    return "";
  }

  base64Buf[encodedLen] = '\0';
  String encoded = String((char*)base64Buf);
  free(base64Buf);

  DBG_F("CAM", "Base64 length: %d", encoded.length());
  Serial.println("BEGIN_BASE64");
Serial.println(encoded);
Serial.println("END_BASE64");
  return encoded;
}

// ─────────────────────────────────────────────
// GOOGLE VISION
// ─────────────────────────────────────────────
bool classifyWithVisionAPI(const String& base64Image, String& detectedLabel) {
  if (WiFi.status() != WL_CONNECTED) {
    DBG("ERROR", "WiFi not connected");
    return false;
  }

  if (String(GOOGLE_API_KEY) == "YOUR_GOOGLE_VISION_API_KEY_HERE" ||
      String(GOOGLE_API_KEY).length() == 0) {
    DBG("ERROR", "GOOGLE_API_KEY missing");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://vision.googleapis.com/v1/images:annotate?key=" + String(GOOGLE_API_KEY);

  if (!http.begin(client, url)) {
    DBG("ERROR", "HTTP begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.setTimeout(15000);

  String requestBody = "{\"requests\":[{\"image\":{\"content\":\"";
  requestBody += base64Image;
  requestBody += "\"},\"features\":[{\"type\":\"LABEL_DETECTION\",\"maxResults\":5}]}]}";

  int httpCode = http.POST(requestBody);
  DBG_F("API", "HTTP code: %d", httpCode);

  if (httpCode != 200) {
    String errorResponse = http.getString();
    if (errorResponse.length() > 0) {
      DBG("ERROR", "Vision API response:");
      Serial.println(errorResponse.substring(0, 300));
    }
    http.end();
    return false;
  }

  String response = http.getString();
  http.end();

  DynamicJsonDocument doc(8192);
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    DBG_F("ERROR", "JSON parse failed: %s", error.c_str());
    return false;
  }

  JsonArray labels = doc["responses"][0]["labelAnnotations"].as<JsonArray>();
  if (labels.isNull() || labels.size() == 0) {
    DBG("ERROR", "No labels returned");
    return false;
  }

  detectedLabel = labels[0]["description"].as<String>();
  detectedLabel.toLowerCase();
  return true;
}

// ─────────────────────────────────────────────
// CLASSIFICATION
// ─────────────────────────────────────────────
bool isRecyclable(const String& label) {
  String lowerLabel = label;
  lowerLabel.toLowerCase();

  for (int i = 0; i < NUM_RECYCLABLE_KEYWORDS; i++) {
    if (lowerLabel.indexOf(RECYCLABLE_KEYWORDS[i]) >= 0) {
      DBG_F("CLASS", "Matched keyword: %s", RECYCLABLE_KEYWORDS[i]);
      return true;
    }
  }
  return false;
}

// ─────────────────────────────────────────────
// ACTUATION / FEEDBACK
// ─────────────────────────────────────────────
void sortItem(bool placedOnRecycle, bool itemIsRecyclable) {
#if USE_SERVO
  bool correct = (placedOnRecycle == itemIsRecyclable);

  // pin 13 = recycle side
  // pin 14 = waste side
  if (placedOnRecycle) {
    // User placed item on recycle side (pin 13)
    if (correct) {
      DBG_F("SERVO", "Placed on RECYCLE side and correct -> move to RECYCLE (%d°)", SERVO_RECYCLE);
      sortServo.write(SERVO_RECYCLE);
    } else {
      DBG_F("SERVO", "Placed on RECYCLE side but incorrect -> move to WASTE (%d°)", SERVO_WASTE);
      sortServo.write(SERVO_WASTE);
    }
  } else {
    // User placed item on waste side (pin 14)
    if (correct) {
      DBG_F("SERVO", "Placed on WASTE side and correct -> move to WASTE (%d°)", SERVO_WASTE);
      sortServo.write(SERVO_WASTE);
    } else {
      DBG_F("SERVO", "Placed on WASTE side but incorrect -> move to RECYCLE (%d°)", SERVO_RECYCLE);
      sortServo.write(SERVO_RECYCLE);
    }
  }

  delay(900);
#else
  DBG("SERVO", "Servo disabled");
#endif
}

void giveFeedback(bool placedOnRecycle, bool itemIsRecyclable) {
  bool correct = (placedOnRecycle == itemIsRecyclable);

#if USE_WS2812
  if (correct) {
    setAllLEDs(itemIsRecyclable ? strip.Color(0, 0, 255) : strip.Color(0, 255, 0));
  } else {
    setAllLEDs(strip.Color(255, 0, 0));
  }
#endif

  if (correct) playCorrectTone();
  else playIncorrectTone();
}

void setAllLEDs(uint32_t color) {
#if USE_WS2812
  for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, color);
  strip.show();
#else
  (void)color;
#endif
}

void playCorrectTone() {
  tone(BUZZER_PIN, 900, 80);
  delay(100);
  tone(BUZZER_PIN, 1200, 80);
  delay(100);
  noTone(BUZZER_PIN);
}

void playIncorrectTone() {
  tone(BUZZER_PIN, 600, 120);
  delay(150);
  noTone(BUZZER_PIN);
}

void resetState() {
  setAllLEDs(0);
  noTone(BUZZER_PIN);
#if USE_SERVO
  sortServo.write(SERVO_NEUTRAL);
  delay(500);
#endif
}

void waitForSensorsToClear() {
  unsigned long start = millis();
  while ((digitalRead(IR_LEFT_PIN) == LOW || digitalRead(IR_RIGHT_PIN) == LOW) &&
         millis() - start < 3000) {
    delay(20);
  }
}

void logHeapStatus() {
  DBG_F("BOOT", "Free heap: %u", ESP.getFreeHeap());
  DBG_F("BOOT", "Largest free block: %u", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  DBG_F("BOOT", "PSRAM found: %s", psramFound() ? "YES" : "NO");
}
