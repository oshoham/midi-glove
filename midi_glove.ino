/*
 * MIDI Glove (Right Hand)
 * 
 * Controller code for a Blueooth MIDI controller glove, made as a
 * final project for Tom Igoe's Fall 2017 Introduction to Phyiscal
 * Computing class at NYU ITP.
 * 
 * Written December 15, 2017
 * By Oren Shoham
 * 
 * 
 * Intended to run on an Adafruit Feather 32u4 Bluefruit LE
 * https://www.adafruit.com/product/2829
 * 
 * Bluetooth MIDI code adapted from Adafruit's Bluefruit LE nRF51 MIDI example:
 * https://github.com/adafruit/Adafruit_BluefruitLE_nRF51/blob/bdad61148ee3a4a1a9c84bc76fcf962a60ca3b11/examples/midi/midi.ino
 * 
 * Before you can run this code on your Feather, you may need to
 * upgrade its firmware to the latest version using Adafruit's
 * Bluefruit LE Connect app.
 * https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/software-resources
 * 
 * 
 * To connect to an iOS device:
 * 
 * 1) Download midimittr (https://itunes.apple.com/us/app/midimittr/id925495245?mt=8)
 * 2) Download a MIDI-compatible audio app, e.g.
 *     - iGrand Piano Free (https://itunes.apple.com/us/app/igrand-piano-free-for-ipad/id562914032?mt=8)
 *     - DrumsLive Lite (https://itunes.apple.com/us/app/drumslive-lite-touch-and-midi-drums/id600089094?mt=8)
 * 3) Enable Bluetooth on your device
 * 4) Turn on the glove and wait for it to enter pairing mode (the LEDs will start fading)
 * 5) Open the midimittr app and under Clients select "MIDI Glove Right Hand"
 * 6) Once you see the 'Connected' label, open your audio app
 * 7) Tap the fingers of the glove and you should see notes playing on the app
 *
 * To connect to a computer running OS X:
 *
 * 1) Open the Audio MIDI Setup application on your computer
 * 2) Enable Bluetooth on your computer
 * 3) Turn on the glove and wait for it to enter pairing mode (the LEDs will start fading)
 * 4) Click on the Bluetooth icon in Audio MIDI Setup
 * 5) Find "MIDI Glove Right Hand" in the list of devices and click "Conncect"
 * 6) Open a DAW like Ableton Live, Logic Pro, or GarageBand
 * 7) Use the glove like any other MIDI instrument
 */ 

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEMIDI.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);

const int thumbInputPin = A4;
const int indexInputPin = A3;
const int middleInputPin = A2;
const int ringInputPin = A1;
const int pinkyInputPin = A0;

const int thumbLEDPin = 5;
const int indexLEDPin = 6;
const int middleLEDPin = 9;
const int ringLEDPin = 10;
const int pinkyLEDPin = 11;

const int inputPins[5] = {thumbInputPin, indexInputPin, middleInputPin, ringInputPin, pinkyInputPin};
const int LEDPins[5] = {thumbLEDPin, indexLEDPin, middleLEDPin, ringLEDPin, pinkyLEDPin};

const int tapDebounceThreshold = 100;

bool isConnected = false;

bool fingerStates[5] = {false, false, false, false, false};
int thresholds[5] = {200, 200, 500, 500, 400};
int notes[5] = {60, 62, 64, 65, 67};
unsigned long lastTimePlayed[5] = {0, 0, 0, 0, 0};

const int ledFadeRate = 30;
unsigned long previousMillis = 0;
int ledFadeValue = 0;
bool fadingUp = true;

// error logging helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void connected(void) {
  isConnected = true;
  Serial.println(F("connected"));
  flashLEDs(4);
  delay(1000);
}

void disconnected(void) {
  Serial.println("disconnected");
  isConnected = false;
  ledFadeValue = 0;
  flashLEDs(4);
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2) {
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");

  Serial.println();
}

void flashLEDs(int times) {
  for (int i = 0; i < 5; i++) {
    analogWrite(LEDPins[i], 0);
  }
  delay(125);
  for (int l = 0; l < times; l++) {
    for (int j = 0; j < 5; j++) {
      analogWrite(LEDPins[j], 255);
    }
    delay(125);
    for (int k = 0; k < 5; k++) {
      analogWrite(LEDPins[k], 0);
    }
    delay(125);
  }
}

void logSensorValues() {
  Serial.print("thumb: ");
  Serial.print(analogRead(thumbInputPin));
  Serial.print(", index: ");
  Serial.print(analogRead(indexInputPin));
  Serial.print(", middle: ");
  Serial.print(analogRead(middleInputPin));
  Serial.print(", ring: ");
  Serial.print(analogRead(ringInputPin));
  Serial.print(", pinky: ");
  Serial.println(analogRead(pinkyInputPin));
}

void setup(void) {
  // flash the LEDs to show that the glove is on
  for (int i = 0; i < 5; i++) {
    pinMode(LEDPins[i], OUTPUT);
    analogWrite(LEDPins[i], 255);
    delay(125);
    analogWrite(LEDPins[i], 0);
    delay(125);
  }
  for (int j = 3; j >= 0; j--) {
    analogWrite(LEDPins[j], 255);
    delay(125);
    analogWrite(LEDPins[j], 0);
    delay(125);
  }

  Serial.begin(115200);

  // initialise the module
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring?"));
  }
  Serial.println(F("OK!"));

  if (FACTORYRESET_ENABLE)
  {
    // perform a factory reset to make sure everything is in a known state
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()) {
      error(F("Couldn't factory reset"));
    }
  }

  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  // print Bluefruit information
  ble.info();

  // change the device name to make it easier to find
  Serial.println(F("Setting device name to 'MIDI Glove Right Hand': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=MIDI Glove Right Hand" )) ) {
    error(F("Could not set device name?"));
  }

  // set BLE callbacks
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enable MIDI: "));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.print(F("Waiting for a connection..."));
}

void loop(void) {
  // interval for each scanning ~ 500ms (non blocking)
  ble.update(500);

  // uncomment the following line for debugging
//  logSensorValues();

  unsigned long currentMillis = millis();

  if (!isConnected) {
    // if we're not connected, fade the finger LEDs up and down
    // to show that we're waiting to pair
    if (currentMillis - previousMillis >= ledFadeRate) {
      previousMillis = currentMillis;
      for (int j = 0; j < 5; j++) {
        analogWrite(LEDPins[j], ledFadeValue);
      }
      
      if (fadingUp && ledFadeValue >= 255) {
        fadingUp = false;
      } else if (!fadingUp && ledFadeValue <= 0) {
        fadingUp = true;
      }
      
      if (fadingUp) {
        ledFadeValue += 5;
      } else {
        ledFadeValue -= 5;
      }
    }
    // bail on the loop function because we aren't connected
    return;
  }

  // for each finger sensor
  // if the sensor is over its threshold:
  //   turn on the corresponding LED
  //   and if the corresponding MIDI note wasn't already playing:
  //     play it
  // otherwise:
  //   turn off the corresponding LED
  //   and if the corresponding MIDI was playing:
  //     stop playing it
  for (int i = 0; i < 5; i++) {
    int value = analogRead(inputPins[i]);
    int note = notes[i];
    
    if (value > thresholds[i]) {
      int velocity = map(value, thresholds[i], 1023, 63, 127);
      if (!fingerStates[i] && currentMillis - lastTimePlayed[i] >= tapDebounceThreshold) {
        midi.send(0x90, notes[i], velocity);
        fingerStates[i] = true;
        lastTimePlayed[i] = currentMillis;
      }
      digitalWrite(LEDPins[i], HIGH);
    } else {
      if (fingerStates[i]) {
        midi.send(0x80, notes[i], 0x64);
        fingerStates[i] = false;
      }
      
      digitalWrite(LEDPins[i], LOW);
    }
  }
}

