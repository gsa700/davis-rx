// davis-rx — Davis Vantage Pro2 ISS receiver
// Copyright (C) 2026  davis-rx contributors
// Licensed under the GNU General Public License v3.0 or later.
// This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.

/*
 * Milestone 0 — RSSI sweep of the 902-928 MHz ISM band.
 *
 * Decodes nothing.  It parks briefly on each of 51 channel slots, samples RSSI,
 * and reports the peak seen per channel.  The Davis ISS transmits ~every 2.5 s
 * hopping across the band, so over a full sweep its bursts show up as elevated
 * peaks on real channels.
 *
 * Purpose: validate the FREQUENCY assumption independently of sync word, bit
 * rate, deviation and CRC — i.e. prove the transmitter is where we think it is
 * before blaming the decoder.
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

// sweep the whole US ISM band in 51 steps, independent of any hop table
static const float F_START = 902.0f;
static const float F_END   = 928.0f;
static const int   N_CHAN  = 51;
static const int   DWELL_MS = 120;   // per channel, per pass

float peak[N_CHAN];
float floorv[N_CHAN];

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  for (int i = 5; i > 0; i--) { Serial.print(F("scan starts in ")); Serial.println(i); Serial.flush(); delay(1000); }

  pinMode(WB_IO2, OUTPUT); digitalWrite(WB_IO2, HIGH); delay(300);
  SPI_LORA.begin();

  int st = radio.beginFSK(915.0, 19.2, 9.5, 467.0, 10, 16, 3.3, false);
  Serial.print(F("beginFSK -> ")); Serial.println(st);
  if (st != RADIOLIB_ERR_NONE) { Serial.println(F("STOP")); while (true) delay(1000); }
  radio.setCRC(0);

  for (int i = 0; i < N_CHAN; i++) { peak[i] = -200; floorv[i] = 200; }
  Serial.println(F("RX bw 467 kHz vs 520 kHz steps = ~90% coverage"));
  Serial.println(F("sweeping 902-928 MHz; watch for channels whose peak rises above the floor"));
}

int pass = 0;

void loop() {
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
  float step = (F_END - F_START) / (N_CHAN - 1);

  for (int i = 0; i < N_CHAN; i++) {
    float f = F_START + i * step;
    radio.standby();
    if (radio.setFrequency(f) != RADIOLIB_ERR_NONE) continue;
    radio.startReceive();
    uint32_t t0 = millis();
    while (millis() - t0 < DWELL_MS) {
      float r = radio.getRSSI(false);   // false = instantaneous, not last-packet
      if (r > peak[i])   peak[i] = r;
      if (r < floorv[i]) floorv[i] = r;
      delay(2);
    }
  }
  pass++;

  // peaks accumulate across passes (never reset), so real transmissions
  // permanently lift a channel above the noise ceiling.
  float best = -200; int besti = -1;
  for (int i = 0; i < N_CHAN; i++) if (peak[i] > best) { best = peak[i]; besti = i; }

  Serial.print(F("--- pass ")); Serial.print(pass);
  Serial.print(F("  strongest: ")); Serial.print(F_START + besti * step, 3);
  Serial.print(F(" MHz @ ")); Serial.print(best, 0); Serial.println(F(" dBm"));

  int hits = 0;
  for (int i = 0; i < N_CHAN; i++) {
    if (peak[i] > -100) {                       // well above the ~-110 noise ceiling
      hits++;
      Serial.print(F("   SIGNAL  ")); Serial.print(F_START + i * step, 3);
      Serial.print(F(" MHz  peak=")); Serial.print(peak[i], 0);
      Serial.print(F(" dBm  floor=")); Serial.println(floorv[i], 0);
    }
  }
  if (!hits) Serial.println(F("   (nothing above -100 dBm yet)"));
  Serial.flush();
}
