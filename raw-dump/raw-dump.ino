// davis-rx — Davis Vantage Pro2 ISS receiver
// Copyright (C) 2026  davis-rx contributors
// Licensed under the GNU General Public License v3.0 or later.
// This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.

/*
 * RAW capture — what is ACTUALLY on the air?
 *
 * Instead of asking the radio to match Davis's 0xCB89 sync (which has so far
 * never fired), we sync on the PREAMBLE itself (0xAA 0xAA) and grab the bytes
 * that follow.  If Davis is transmitting normally we should see 0xCB 0x89
 * appear in the dump, followed by 8 payload bytes and 2 CRC bytes.
 *
 * This distinguishes, in one test:
 *   - wrong sync word           -> preamble syncs, but no CB 89 follows
 *   - whitening/scrambling      -> CB 89 absent, data looks random
 *   - wrong bitrate/deviation   -> nothing syncs at all
 *   - all correct               -> CB 89 plainly visible
 * Whitening is explicitly DISABLED: Davis uses DCFREE_OFF, and RadioLib does
 * not necessarily default it off.
 */
#include <Adafruit_TinyUSB.h>
#include <RadioLib.h>
#define LORA_NSS 42
#define LORA_DIO1 47
#define LORA_NRST 38
#define LORA_BUSY 46
#define LORA_SCK 43
#define LORA_MOSI 44
#define LORA_MISO 45
SPIClass SPI_LORA(NRF_SPIM2, LORA_MISO, LORA_SCK, LORA_MOSI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, SPI_LORA);

static const float FREQ_MHZ = 909.3819f;   // hop idx 24 + measured -25 kHz
static const uint8_t CAP_LEN = 20;         // grab well past where CB 89 should be
uint8_t PREAMBLE_SYNC[] = {0xAA, 0xAA};

volatile bool rx = false;
void onRx() { rx = true; }

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  for (int i = 4; i > 0; i--) { Serial.print(F("start in ")); Serial.println(i); Serial.flush(); delay(1000); }

  pinMode(WB_IO2, OUTPUT); digitalWrite(WB_IO2, HIGH); delay(300);
  SPI_LORA.begin();
  int st = radio.beginFSK(FREQ_MHZ, 19.2, 9.9, 78.2, 10, 16, 3.3, false);
  Serial.print(F("beginFSK -> ")); Serial.println(st);
  if (st != RADIOLIB_ERR_NONE) { while (true) delay(1000); }

  radio.setRxBoostedGainMode(true);
  radio.setDataShaping(RADIOLIB_SHAPING_0_5);
  Serial.print(F("setWhitening(false) -> ")); Serial.println(radio.setWhitening(false));
  Serial.print(F("sync on preamble 0xAAAA -> ")); Serial.println(radio.setSyncWord(PREAMBLE_SYNC, 2));
  Serial.print(F("fixedPacketLengthMode(20) -> ")); Serial.println(radio.fixedPacketLengthMode(CAP_LEN));
  radio.setCRC(0);
  radio.setPacketReceivedAction(onRx);
  Serial.print(F("startReceive -> ")); Serial.println(radio.startReceive());
  Serial.print(F("RAW capture on ")); Serial.print(FREQ_MHZ, 4); Serial.println(F(" MHz"));
  Serial.println(F("looking for CB 89 in the dumps below"));
}

uint32_t n = 0;
void loop() {
  static uint32_t last = 0;
  if (!rx) {
    if (millis() - last > 10000) { last = millis();
      Serial.print(F("  ... captures=")); Serial.println(n); }
    return;
  }
  rx = false;
  uint8_t b[CAP_LEN];
  int st = radio.readData(b, CAP_LEN);
  n++;
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
  if (st == RADIOLIB_ERR_NONE) {
    Serial.print(F("RAW ")); 
    for (uint8_t i = 0; i < CAP_LEN; i++) {
      if (b[i] < 0x10) Serial.print('0');
      Serial.print(b[i], HEX); Serial.print(' ');
    }
    Serial.print(F(" rssi=")); Serial.print(radio.getRSSI(), 0);
    // flag the Davis sync if it appears anywhere in the capture
    for (uint8_t i = 0; i + 1 < CAP_LEN; i++)
      if (b[i] == 0xCB && b[i+1] == 0x89) { Serial.print(F("  <<< CB89 at byte ")); Serial.print(i); }
    Serial.println();
  } else {
    Serial.print(F("readData err ")); Serial.println(st);
  }
  radio.startReceive();
}
