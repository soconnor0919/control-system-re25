/*
 * BUCKNELL CHEM-E-CAR TEAM PRESENTS:
 * 2025 Control System Code
 * 
 * This code is designed to run on an Arduino Uno R4 Minima microcontroller.
 * It is responsible for controlling the motor based off of reading 
 * spectrometer sensor values.
 * 
 * Copyright (C) 2025 Bucknell University
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * 
 * Author: Sean O'Connor (sso005@bucknell.edu)
 */

#include <Arduino.h>
#include <Adafruit_AS7341.h>
#include <Wire.h>
#include "constants.h"
#include "hardware.h"
#include "subsystems/LEDUtil.h"

// SYSTEM CONFIGURATION
bool DEBUG_MODE = true;
bool LED_ACTIVE = true;

// STATE VARIABLES
int measurement = -1;
int baselineValue = 0;
bool reactionComplete = false;
unsigned long startTime = 0;
bool triggerArmed = false;  // Add this to wait for pin to stabilize

void setup() {
  // Set pin modes
  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  // Initialize serial communication
  if (DEBUG_MODE) {
    Serial.begin(115200);
    while (!Serial) delay(10);
  }

  // Initialize LED strip
  if (LED_ACTIVE) {
    initLED();
    setLEDColor(Color::WHITE);
    if (DEBUG_MODE) {
      Serial.println("LEDs initialized.");
    }
  }

  if (DEBUG_MODE) {
    Serial.println("Looking for AS7341 sensor...");
  }

  if (!as7341.begin()) {
    if (DEBUG_MODE) {
      Serial.println("Could not find AS7341 sensor. Check wiring!");
    }
    while (1) delay(10);
  }

  if (DEBUG_MODE) {
    Serial.println("AS7341 sensor initialized!");
  }

  // Configure spectrometer sensor
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
  
  // Wait for trigger pin to stabilize to LOW
  delay(100);
  if (digitalRead(TRIGGER_PIN) == LOW) {
    triggerArmed = true;
    // Serial.println("Trigger armed and ready!");
  }
}

void loop() {
  delay(50);
  
  if (!as7341.readAllChannels()) {
    if (DEBUG_MODE) {
      Serial.println("Error reading channels");
    }
    return;
  }

  measurement = as7341.getChannel(AS7341_CHANNEL_630nm_F7);

  // Print trigger state periodically
  static unsigned long lastTriggerDebug = 0;
  if (millis() - lastTriggerDebug > 500) {
    // Serial.print("Trigger pin state: ");
    // Serial.println(digitalRead(TRIGGER_PIN));
    lastTriggerDebug = millis();
  }

  // Start timer when trigger is released (goes from LOW to HIGH) and take baseline
  if (triggerArmed && !reactionComplete && digitalRead(TRIGGER_PIN) == HIGH && startTime == 0) {
    baselineValue = measurement;
    startTime = millis();
    if (DEBUG_MODE) {
      Serial.println("Reaction started!");
      Serial.print("Baseline value: ");
      Serial.println(baselineValue);
    }
  }

  // Check for completion if reaction has started
  if (!reactionComplete && startTime > 0) {
    if (baselineValue - measurement >= SIGNIFICANT_DECREASE) {
      reactionComplete = true;
      float duration = (millis() - startTime) / 1000.0;
      if (DEBUG_MODE) {
        Serial.print("Reaction complete! Duration: ");
        Serial.print(duration);
        Serial.println(" seconds");
      }
    }
  }

  if (DEBUG_MODE) {
    Serial.print('>');
    Serial.print("630nm:");
    Serial.println(measurement);
  }
}