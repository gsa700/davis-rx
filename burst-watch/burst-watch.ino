// davis-rx — Davis Vantage Pro2 ISS receiver
// Copyright (C) 2026  davis-rx contributors
// Licensed under the GNU General Public License v3.0 or later.
// This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.

/*
 * Channel activity monitor — is the ISS energy actually arriving?
 *
 * Parks on ONE real Davis channel and samples instantaneous RSSI as fast as it
 * can, reporting every excursion above a threshold with its duration.  Decodes
 * nothing: this separates "is the signal here?" from "can my packet engine
 * lock onto it?", which packet-level debugging cannot do.
 *
 * Expectation on a real Davis channel: a burst every ~131 s (2.5625 s interval
 * x 51 channels), lasting order 10 ms, well above the noise floor.
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

static const float FREQ_MHZ  = 909.3819f;   // hop idx 24 + measured -25 kHz
static const float THRESH    = -95.0f;     // dBm; noise floor measured ~-117
static const float RX_BW     = 78.2f;

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  for (int i = 4; i > 0; i--) { Serial.print(F("start in ")); Serial.println(i); Serial.flush(); delay(1000); }

  pinMode(WB_IO2, OUTPUT); digitalWrite(WB_IO2, HIGH); delay(300);
  SPI_LORA.begin();
  int st = radio.beginFSK(FREQ_MHZ, 19.2, 9.9, RX_BW, 10, 16, 3.3, false);
  Serial.print(F("beginFSK -> ")); Serial.println(st);
  if (st != RADIOLIB_ERR_NONE) { while (true) delay(1000); }
  radio.setRxBoostedGainMode(true);      // max sensitivity
  radio.setCRC(0);
  radio.startReceive();
  Serial.print(F("watching ")); Serial.print(FREQ_MHZ, 4);
  Serial.print(F(" MHz for bursts above ")); Serial.print(THRESH, 0);
  Serial.println(F(" dBm"));
  Serial.println(F("expect ~1 burst every 131 s if this is a live Davis channel"));
}

uint32_t bursts = 0;
float noiseAcc = -117; 

void loop() {
  static uint32_t lastReport = 0, inBurstStart = 0;
  static float burstPeak = -200;
  static bool inBurst = false;

  float r = radio.getRSSI(false);
  noiseAcc = noiseAcc * 0.999f + r * 0.001f;   // slow noise-floor tracker

  if (r > THRESH) {
    if (!inBurst) { inBurst = true; inBurstStart = millis(); burstPeak = r; }
    if (r > burstPeak) burstPeak = r;
  } else if (inBurst) {
    uint32_t dur = millis() - inBurstStart;
    inBurst = false; bursts++;
    Serial.print(F("BURST  peak=")); Serial.print(burstPeak, 0);
    Serial.print(F(" dBm  dur=")); Serial.print(dur);
    Serial.print(F(" ms  t=")); Serial.print(millis() / 1000);
    Serial.print(F("s  total=")); Serial.println(bursts);
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
  }

  if (millis() - lastReport > 15000) {
    lastReport = millis();
    Serial.print(F("  noise floor ~")); Serial.print(noiseAcc, 0);
    Serial.print(F(" dBm, bursts so far ")); Serial.println(bursts);
  }
}
