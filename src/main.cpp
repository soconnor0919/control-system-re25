#include <Arduino.h>
#include <Adafruit_AS7341.h>
#include <Wire.h>

Adafruit_AS7341 as7341;

int measurement = -1;
int baselineValue = 0;
bool reactionComplete = false;
unsigned long startTime = 0;
const int triggerPin = 2;
const int SIGNIFICANT_DECREASE = 5000;
bool triggerArmed = false;  // Add this to wait for pin to stabilize

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Wire.begin();
  delay(100);

  Serial.println("Looking for AS7341 sensor...");
  if (!as7341.begin()) {
    Serial.println("Could not find AS7341 sensor. Check wiring!");
    while (1) delay(10);
  }

  Serial.println("AS7341 sensor initialized!");

  // Configure sensor
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
  as7341.enableLED(false);
  
  pinMode(triggerPin, INPUT_PULLUP);
  
  // Wait for trigger pin to stabilize to LOW
  delay(100);
  if (digitalRead(triggerPin) == LOW) {
    triggerArmed = true;
    // Serial.println("Trigger armed and ready!");
  }
}

void loop() {
  delay(50);
  
  if (!as7341.readAllChannels()) {
    Serial.println("Error reading channels");
    return;
  }

  measurement = as7341.getChannel(AS7341_CHANNEL_630nm_F7);

  // Print trigger state periodically
  static unsigned long lastTriggerDebug = 0;
  if (millis() - lastTriggerDebug > 500) {
    // Serial.print("Trigger pin state: ");
    // Serial.println(digitalRead(triggerPin));
    lastTriggerDebug = millis();
  }

  // Start timer when trigger is released (goes from LOW to HIGH) and take baseline
  if (triggerArmed && !reactionComplete && digitalRead(triggerPin) == HIGH && startTime == 0) {
    baselineValue = measurement;
    startTime = millis();
    Serial.println("Reaction started!");
    Serial.print("Baseline value: ");
    Serial.println(baselineValue);
  }

  // Check for completion if reaction has started
  if (!reactionComplete && startTime > 0) {
    if (baselineValue - measurement >= SIGNIFICANT_DECREASE) {
      reactionComplete = true;
      float duration = (millis() - startTime) / 1000.0;
      Serial.print("Reaction complete! Duration: ");
      Serial.print(duration);
      Serial.println(" seconds");
    }
  }

  Serial.print('>');
  Serial.print("630nm:");
  Serial.println(measurement);
}