// davis-rx — Davis Vantage Pro2 ISS receiver
// Copyright (C) 2026  davis-rx contributors
// Licensed under the GNU General Public License v3.0 or later.
// This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.

/*
 * Milestone 1 — Davis Vantage Pro2 ISS receiver, park-on-one-channel.
 *
 * Hardware: RAK4631 (nRF52840 + SX1262).  The SX1262 is used in GFSK mode,
 * NOT LoRa — the Davis ISS transmits plain GFSK at ~19.2 kbps.
 *
 * This sketch does NOT hop.  It sits on a single US channel and prints every
 * packet it hears.  Because the ISS hops across 51 channels roughly every
 * 2.5 s, expect a packet about every ~2 minutes.  That is enough to prove the
 * radio config, sync word and CRC are right, which is the hard part.
 *
 * Radio parameters and the sync word come from the DavisRFM69 project.
 * If nothing decodes, see NOTES at the bottom for what to vary first.
 */
#include <Adafruit_TinyUSB.h>   // REQUIRED: makes Serial the USB CDC on this core
#include <RadioLib.h>

// RAK4631 SX1262 wiring.  The BSP does NOT export PIN_LORA_* macros, and the
// default `SPI` object is the WisBlock IO-slot bus (29/30/3) — NOT the radio.
// The SX1262 lives on its own SPI, so we instantiate one for it.
//   nRF52840 numbering: P1.xx == 32 + xx
#define LORA_NSS   42   // P1.10
#define LORA_DIO1  47   // P1.15
#define LORA_NRST  38   // P1.06
#define LORA_BUSY  46   // P1.14
#define LORA_SCK   43   // P1.11
#define LORA_MOSI  44   // P1.12
#define LORA_MISO  45   // P1.13

SPIClass SPI_LORA(NRF_SPIM2, LORA_MISO, LORA_SCK, LORA_MOSI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, SPI_LORA);

// ---- Davis US hop table — REAL values, decoded from the DavisRFM69 FRF
// register triplets (freq = FRF * 32e6/2^19).  Stored in HOP SEQUENCE order.
// NOTE: my earlier guess of "902.419 + n*501.9 kHz" was WRONG — it landed 37 kHz
// off the nearest real channel, outside a 58.6 kHz passband, hence zero packets.
static const float DAVIS_US_HOP[51] = {
  911.4138f, 902.3819f, 911.9152f, 922.9532f, 914.9266f,
  906.3959f, 925.9646f, 918.4384f, 908.9047f, 920.4454f,
  913.4204f, 903.8881f, 916.9335f, 924.4589f, 910.4099f,
  904.8906f, 915.9291f, 921.4484f, 907.3994f, 926.9677f,
  912.9191f, 903.3854f, 917.4344f, 923.4563f, 909.4069f,
  926.4664f, 905.8946f, 914.4243f, 919.4414f, 924.9602f,
  902.8845f, 910.9121f, 921.9497f, 915.4279f, 906.8981f,
  917.9357f, 927.4698f, 920.9471f, 908.4029f, 912.4174f,
  918.9401f, 904.3893f, 923.9572f, 916.4318f, 909.9091f,
  919.9436f, 905.3920f, 922.4519f, 907.9011f, 913.9226f,
  925.4624f
};
// Park on hop index 24 = 909.4069 MHz.
// NOT chosen by RSSI: hop idx 20 (912.9191) turned out to have a WIDE -55 dBm
// mesh signal sitting on it - 33 dB stronger than the ~-88 dBm Davis burst, so
// the receiver was permanently captured by it. Spectrum analyser peaks at
// 907.975/911.025/914.925/915.850/916.150/916.375 are the local strong signals;
// 909.4069 is the furthest Davis channel from all of them (1.4 MHz clear).
static const uint8_t PARK_CHANNEL = 24;
static const float FREQ_OFFSET_MHZ = -0.0330f;   // MEASURED from IQ: -33.1 kHz, std 0.6 kHz over 27 bursts

// ---- Davis link parameters ----
static const float BITRATE_KBPS   = 19.2f;
static const float DEVIATION_KHZ  = 9.9f;   // RF_FDEV*_9900 in DavisRFM69
static const float RX_BW_KHZ      = 156.2f;  // WIDE: the TX crystal drifts with temperature
                                             // (-25 kHz measured afternoon, -33.1 kHz at 18:35).
                                             // A working CRC makes false syncs harmless, so buy
                                             // capture range rather than chase the drift.
static const uint8_t PACKET_LEN   = 10;      // 8 payload + 2 CRC
static uint8_t DAVIS_SYNC[]       = {0xCB, 0x89};

volatile bool gotPacket = false;
void onDio1() { gotPacket = true; }

// Davis transmits every byte LSB-FIRST. The SX1262 (like the RFM69) shifts in
// MSB-first, so each received byte arrives bit-reversed and must be flipped
// before anything can be interpreted. Measured from IQ capture, 27/27 bursts.
static uint8_t revByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// CRC-16/CCITT, poly 0x1021, init 0x0000.
// Frame is [6 data][2 CRC][FF FF] -- NOT 8 data + 2 CRC. The CRC covers bytes
// 0-5 only and is compared against bytes 6-7; bytes 8-9 are repeater bytes and
// must be excluded. Running it over all 10 never returns 0 on a real packet.
uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 5000) { delay(10); }

  Serial.println();
  Serial.println(F("=== Davis ISS receiver — milestone 1 (park on one channel) ==="));

  // WisBlock: bring up the 3V3_S rail before touching the radio
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(300);

  SPI_LORA.begin();

  // MEASURED CORRECTION: a Rigol RSA3015 max-hold showed this ISS transmitting
  // 25 kHz LOW vs the DavisRFM69 table - confirmed on two channels (idx 24 at
  // -25.0 kHz, idx 31 at -25.2 kHz, exactly 3 channel spacings apart).
  // ~27 ppm of crystal error. Apply it globally.
  float freq = DAVIS_US_HOP[PARK_CHANNEL] + FREQ_OFFSET_MHZ;
  Serial.print(F("parking on channel ")); Serial.print(PARK_CHANNEL);
  Serial.print(F(" = ")); Serial.print(freq, 6); Serial.println(F(" MHz"));

  // GFSK mode.  tcxoVoltage 3.3 is REQUIRED on RAK4631 — its SX1262 runs a
  // TCXO powered from DIO3.  Omit it and the radio silently never receives.
  int st = radio.beginFSK(freq, BITRATE_KBPS, DEVIATION_KHZ, RX_BW_KHZ,
                          10 /*dBm, tx unused*/, 24 /*preamble detect bits: fewer false syncs*/,
                          3.3 /*tcxo*/, false /*DC-DC*/);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print(F("beginFSK FAILED: ")); Serial.println(st);
    while (true) delay(1000);
  }

  // Davis: 2-byte sync, fixed 10-byte packet, no hardware CRC (we check in
  // software), no whitening, no Manchester.
  if (radio.setSyncWord(DAVIS_SYNC, sizeof(DAVIS_SYNC)) != RADIOLIB_ERR_NONE)
    Serial.println(F("setSyncWord failed"));
  if (radio.fixedPacketLengthMode(PACKET_LEN) != RADIOLIB_ERR_NONE)
    Serial.println(F("fixedPacketLengthMode failed"));
  radio.setCRC(0);
  radio.setDataShaping(RADIOLIB_SHAPING_0_5);   // GFSK BT=0.5

  radio.setPacketReceivedAction(onDio1);
  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print(F("startReceive FAILED: ")); Serial.println(st);
    while (true) delay(1000);
  }
  Serial.println(F("listening… (expect ~1 packet every 2 min at 1-in-51 odds)"));
}

uint32_t heard = 0, good = 0;

void loop() {
  // heartbeat: slow blink = alive and listening
  static uint32_t bl = 0;
  if (millis() - bl > 1000) { bl = millis(); digitalWrite(LED_GREEN, !digitalRead(LED_GREEN)); }

  if (!gotPacket) {
    static uint32_t last = 0;
    if (millis() - last > 5000) {
      last = millis();
      Serial.print(F("  … still listening, heard=")); Serial.print(heard);
      Serial.print(F(" good=")); Serial.println(good);
    }
    return;
  }
  gotPacket = false;

  uint8_t buf[PACKET_LEN];
  int st = radio.readData(buf, PACKET_LEN);
  heard++;

  if (st == RADIOLIB_ERR_NONE) {
    uint8_t d[PACKET_LEN];
    for (uint8_t i = 0; i < PACKET_LEN; i++) d[i] = revByte(buf[i]);

    uint16_t calc = crc16_ccitt(d, 6);              // CRC covers bytes 0-5
    uint16_t rx   = ((uint16_t)d[6] << 8) | d[7];   // CRC is in bytes 6-7
    bool ok = (calc == rx);
    if (ok) good++;

    Serial.print(ok ? F("[CRC OK ] ") : F("[CRC BAD] "));
    for (uint8_t i = 0; i < PACKET_LEN; i++) {
      if (d[i] < 0x10) Serial.print('0');
      Serial.print(d[i], HEX); Serial.print(' ');
    }
    Serial.print(F(" msg=0x")); Serial.print(d[0] >> 4, HEX);
    Serial.print(F(" id=")); Serial.print(d[0] & 0x07);
    Serial.print(F(" rssi=")); Serial.print(radio.getRSSI(), 0);
    if (d[8] == 0xFF && d[9] == 0xFF) Serial.print(F(" [direct]"));
    if (!ok) { Serial.print(F("  calc=0x")); Serial.print(calc, HEX);
               Serial.print(F(" rx=0x")); Serial.print(rx, HEX); }
    Serial.println();
  } else {
    Serial.print(F("readData error: ")); Serial.println(st);
  }
  radio.startReceive();
}

/* NOTES — if nothing decodes, vary in this order:
 *   1. PARK_CHANNEL — try several; a neighbour may be quieter.
 *   2. RX_BW_KHZ — try 50.0 or 78.2.
 *   3. DAVIS_SYNC — some builds expect only 0xCB, or a 4-byte sync.
 *   4. Drop the sync word entirely and dump raw bytes to see if anything
 *      resembling a preamble (0xAA…) is arriving at all.
 */
