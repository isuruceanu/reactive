#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "comparator.h"

const char* ssid = "iWireless";
const char* password = "Guacamole4you";

Comparator* zeroCrossDetector;

WebServer server(80);

const int zeroCrossPin = 35;
const int outputPin = 27;
const int ledPin       = 2;
const int threshold = 0;  

bool ledState = LOW;
volatile unsigned long lastInterruptTime = 0; // For debounce

unsigned long timeoutMs = 20;
int debounceSamples = 10;

bool outputState = true;

const unsigned long interval = 500; // Blink every 500 ms
unsigned long previousMillis = 0;
int minVal = 0;
int maxVal = 0;

String zc_state = "No Crossing";


void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Power Factor</title></head><body>";
  html += "<h1>ESP32 Power Factor Monitor</h1>";
  html += "<p><strong>Min analog redings:</strong> ";
  html += String(minVal);
  html += "</p>";
  html += "<p><strong>Max analog redings:</strong> ";
  html += String(maxVal);
   html += "<p><strong>State 0-cross:</strong> ";
  html += zc_state;
  html += "</p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void setupWeb() {
  delay(100);

  // Connect to Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  // Start web server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

void onZeroCross(ZeroCrossType zc) {
  bool statePin = zc == ZC_RISING;
  zeroCrossDetector->start();
  digitalWrite(outputPin, statePin);
}


void setup() {
  pinMode(zeroCrossPin, INPUT_PULLDOWN);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, HIGH);  // Start LOW

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  zeroCrossDetector = new Comparator(zeroCrossPin, threshold, onZeroCross, timeoutMs);

  Serial.begin(115200);
  Serial.println("Zero-cross test started.");
  //setupWeb();
  zeroCrossDetector->start();
}

void loop() {
  
  // unsigned long currentMillis = millis();

  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;
  //   ledState = !ledState;
  //   digitalWrite(ledPin, ledState);
    
  // }
  zeroCrossDetector->loop();
  //server.handleClient();
}
