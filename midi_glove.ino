/*
 * Based on Adafruit's Bluefruit LE nRF51 MIDI example:
 * https://github.com/adafruit/Adafruit_BluefruitLE_nRF51/blob/bdad61148ee3a4a1a9c84bc76fcf962a60ca3b11/examples/midi/midi.ino
 */ 

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEMIDI.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"

// This app was tested on iOS with the following apps:
//
// https://itunes.apple.com/us/app/midimittr/id925495245?mt=8
// https://itunes.apple.com/us/app/igrand-piano-free-for-ipad/id562914032?mt=8
//
// To test:
// - Run this sketch and open the Serial Monitor
// - Open the iGrand Piano Free app
// - Open the midimittr app on your phone and under Clients select "Adafruit Bluefruit LE"
// - When you see the 'Connected' label switch to the Routing panel
// - Set the Destination to 'iGrand Piano'
// - Switch to the iGrand Piano Free app and you should see notes playing one by one

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

bool isConnected = false;

bool fingerStates[5] = {false, false, false, false, false};
int thresholds[5] = {100, 200, 400, 500, 400};
int notes[5] = {60, 62, 64, 65, 67};

unsigned long initializedAt = 0;
unsigned long previousMillis = 0;
int ledFadeValue = 0;
bool fadingUp = true;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// callback
void connected(void)
{
  isConnected = true;
  Serial.println(F(" CONNECTED!"));
  flashLEDs(4);
  delay(1000);
}

void disconnected(void)
{
  Serial.println("disconnected");
  isConnected = false;
  ledFadeValue = 0;
  initializedAt = millis();
  flashLEDs(3);
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
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

void setup(void)
{
  /* Flash some lights to show that the glove is on */
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

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring?"));
  }
  Serial.println(F("OK!"));

  if (FACTORYRESET_ENABLE)
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()) {
      error(F("Couldn't factory reset"));
    }
  }

  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'MIDI Glove Right Hand': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=MIDI Glove Right Hand" )) ) {
    error(F("Could not set device name?"));
  }

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enable MIDI: "));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.print(F("Waiting for a connection..."));
  initializedAt = millis();
}

void loop(void) {
  // interval for each scanning ~ 500ms (non blocking)
  ble.update(500);

  unsigned long currentMillis = millis();

  // bail if not connected
  if (!isConnected) {
    if (currentMillis - previousMillis >= 50) {
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
    return;
  }

  Serial.print("A0: ");
  Serial.print(analogRead(A0));
  Serial.print(", A1: ");
  Serial.print(analogRead(A1));
  Serial.print(", A2: ");
  Serial.print(analogRead(A2));
  Serial.print(", A3: ");
  Serial.print(analogRead(A3));
  Serial.print(", A4: ");
  Serial.println(analogRead(A4));

  for (int i = 0; i < 5; i++) {
    int value = analogRead(inputPins[i]);
    int note = notes[i];
    if (value > thresholds[i]) {
      if (!fingerStates[i]) {
        midi.send(0x90, notes[i], map(value, thresholds[i], 1023, 63, 127));
        fingerStates[i] = true;
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

