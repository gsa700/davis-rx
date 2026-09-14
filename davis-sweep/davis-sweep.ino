// davis-rx — Davis Vantage Pro2 ISS receiver
// Copyright (C) 2026  davis-rx contributors
// Licensed under the GNU General Public License v3.0 or later.
// This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.

/*
 * davis-sweep — measure the ISS transmitter's ACTUAL center frequency by
 * sweeping the receiver across it, and log it against temperature.
 *
 * WHY THIS EXISTS
 *   The receive rate collapses when the ISS gets cold: 23.4 pkt/min at 62 F,
 *   ~10 at 53 F, recovering every morning with the sun.  RSSI of the packets
 *   that DO arrive is unchanged, CRC-bad count rises ~50x, and the hop follower
 *   never loses lock.  Strong signal + failing CRC = bursts landing outside the
 *   capture window, i.e. the transmitter is drifting off frequency as it cools.
 *   It already sits -33 kHz off the nominal hop table at room temperature.
 *
 *   We cannot simply ask the radio how far off it is:  SX126x has NO documented
 *   frequency-error register in GFSK.  RadioLib exposes getFrequencyError(), but
 *   its first line is `if(modem != PACKET_TYPE_LORA) return 0.0;` — it returns
 *   zero for us, always.  (The RFM69 that every open-source Davis receiver uses
 *   has hardware AFC and FEI registers.  The SX126x dropped them.  That is the
 *   whole reason this sketch has to exist.)
 *
 *   So we measure it the honest way: step our OWN center frequency across a
 *   range of offsets and record the CRC-valid yield at each one.  The peak of
 *   that curve is where the transmitter actually is, measured through the real
 *   receive chain — filter shape, sync detector and CRC included.  That is a
 *   better number than a spectrum-analyzer reading, because it is the quantity
 *   that actually decides whether we get a packet.
 *
 * THE INTERLEAVE — why this does not cost us the station
 *   A naive sweep parks on a probe offset for a while.  Probe far enough off and
 *   packets stop, the miss streak runs up, and the follower drops back to
 *   ACQUIRE — we would lose lock exactly when we are trying to measure.
 *
 *   Instead we ALTERNATE, slot by slot:
 *     even slots -> BASELINE offset (-33 kHz, known good) — keeps lock, keeps
 *                   the data flowing to wxrx, re-anchors the clock
 *     odd  slots -> PROBE offset — the measurement, allowed to fail freely
 *   The miss streak resets on every baseline packet, so a probe that hears
 *   nothing can never trigger a resync.  Production cost is one thing only:
 *   field freshness goes from ~2.5 s to ~5 s.  Nothing downstream cares — the
 *   dashboard's own stale threshold is 15 minutes.
 *
 * OUTPUT
 *   Packet lines are byte-compatible with davis-hop (wxrx.py's regex stops at
 *   rssi=), with ch= and probe= appended.  Sweep results come out on their own
 *   SWEEP / SWEEPDONE lines, which wxrx ignores.
 *
 * KEEP ../davis-hop AS PRODUCTION.  This is a measurement instrument, not a
 * replacement.  Same rule as milestone 1: the known-good sketch stays.
 */
#include <Adafruit_TinyUSB.h>   // REQUIRED: makes Serial the USB CDC on this core
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>     // RAK1906 = BME680 on the WisBlock I2C bus

// RAK4631 SX1262 wiring — the BSP does NOT export PIN_LORA_*, and the default
// `SPI` object is the WisBlock IO-slot bus, NOT the radio. P1.xx == 32 + xx.
#define LORA_NSS   42
#define LORA_DIO1  47
#define LORA_NRST  38
#define LORA_BUSY  46
#define LORA_SCK   43
#define LORA_MOSI  44
#define LORA_MISO  45

SPIClass SPI_LORA(NRF_SPIM2, LORA_MISO, LORA_SCK, LORA_MOSI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, SPI_LORA);

// ---- RAK1906 (BME680) in the WisBlock sensor slot ----
// Fitted 2026-09-13.  The station reading we actually want from it is PRESSURE,
// for the APRS beacon's `b` field.
//
// TEMPERATURE AND HUMIDITY ARE RACK AMBIENT, NOT BASEMENT AND NOT SELF-HEATING.
// His correction, 2026-09-13: the case sits on TOP of the rack, dead center, and
// ~80 F is genuinely what the air inside the rack is doing — the basement itself
// is much cooler.  The rack has a THERMOSTATICALLY CONTROLLED FAN, so expect a
// sawtooth, and treat this as a rack thermal sensor rather than a room one.
// Published as wxrx_env_* and never to be confused with the Davis outdoor sensors.
//
// GAS HEATER IS DELIBERATELY OFF.  It self-heats the die by several degrees,
// which would corrupt the one reading we care least about being wrong; and a VOC
// proxy taken inside a sealed enclosure measures the enclosure, not the room.
// Turn it on only if the sensor is ever moved into open air.
Adafruit_BME680 bme;
static bool     envOk   = false;
static uint8_t  envAddr = 0;
static uint32_t lastEnvMs = 0;
static const uint32_t ENV_PERIOD_MS = 30000;

// Real US hop table, in HOP SEQUENCE order — do not sort it.
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
static const uint8_t NUM_CHANNELS = 51;

// Reacquire on hop idx 24 = 909.4069 MHz — furthest from every strong local
// signal.  NOT chosen by RSSI: idx 20 carries a -55 dBm mesh signal.
static const uint8_t PARK_CHANNEL = 24;

// The established correction, measured from IQ: -33.1 kHz, std 0.6 kHz, 27
// bursts, at room temperature.  Probes are expressed as deltas FROM here, so a
// sweep result of 0 kHz means "still where it was in August".
static const float BASE_OFFSET_MHZ = -0.0330f;

// ---- the sweep ----
// +/-20 kHz in 2 kHz steps.  The span is set by what we are hunting: the rate
// falls off a cliff somewhere below 62 F, and 156 kHz of rxBw should not care
// about a few kHz, so if the cause is center-frequency drift the shift must be
// big — tens of kHz.  2 kHz resolution is finer than we need to find a peak and
// still cheap.  Widen SWEEP_MIN/MAX if a pass comes back with the peak pinned
// at an end, which would mean the transmitter has walked outside the window.
static const int16_t SWEEP_MIN_KHZ  = -20;
static const int16_t SWEEP_MAX_KHZ  =  20;
static const int16_t SWEEP_STEP_KHZ =   2;
static const uint8_t NUM_PROBES = ((SWEEP_MAX_KHZ - SWEEP_MIN_KHZ) / SWEEP_STEP_KHZ) + 1;  // 21

// Probe slots per point.  10 probe slots = 10 baseline slots interleaved =
// 20 * 2.5625 s = ~51 s per point, so a full 21-point pass takes ~18 min.
// Several passes per night, each tagged with the temperature at the time —
// which is exactly the drift-vs-temperature curve we are after.
static const uint8_t SLOTS_PER_PROBE = 10;

// ---- Davis link parameters (unchanged from davis-hop) ----
static const float   BITRATE_KBPS  = 19.2f;
static const float   DEVIATION_KHZ = 9.9f;    // ISS side; the 4.8 in DavisRFM69 is the console's RX
static const float   RX_BW_KHZ     = 156.2f;  // left WIDE on purpose — do not change two things at once
static const uint8_t PACKET_LEN    = 10;
static uint8_t DAVIS_SYNC[]        = {0xCB, 0x89};

// ---- Hop timing ----
static const uint32_t INTERVAL_US   = 2562500UL;   // (41 + 0) / 16 s, station ID 1
static const uint32_t MISS_GUARD_US = 150000UL;    // 150 ms
static const uint16_t MAX_MISSES    = 20;          // ~51 s of silence

enum State { ACQUIRE, TRACK };
static State    state     = ACQUIRE;
static uint8_t  curIdx    = PARK_CHANNEL;
static uint32_t nextDueUs = 0;
static uint16_t misses    = 0;

static uint32_t heard = 0, good = 0, bad = 0, slots = 0, missTotal = 0, resyncs = 0;

// ---- sweep state ----
static uint8_t  probeIdx   = 0;      // which offset we are testing
static uint8_t  probeSlots = 0;      // probe slots used at this offset
static bool     onProbe    = false;  // is the CURRENT slot a probe slot?
static uint16_t pGood = 0, pBad = 0, pMiss = 0;
static int32_t  pRssiSum = 0;
static uint16_t pass = 0;
// best-so-far within this pass
static uint8_t  bestIdx = 0;  static uint16_t bestGood = 0;

static int16_t probeKhz(uint8_t i) { return SWEEP_MIN_KHZ + (int16_t)i * SWEEP_STEP_KHZ; }

volatile bool     gotPacket = false;
volatile uint32_t packetUs  = 0;
void onDio1() { packetUs = micros(); gotPacket = true; }

static uint8_t revByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// CRC-16/CCITT, poly 0x1021, init 0x0000, over bytes 0-5 ONLY.
uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// Retune and re-arm.  calibrate=false: image-reject calibration is per BAND and
// 902-928 is one band, already covered by beginFSK at startup.
static void hopTo(uint8_t idx, bool probe) {
  float off = BASE_OFFSET_MHZ;
  if (probe) off += (float)probeKhz(probeIdx) / 1000.0f;
  radio.standby();
  radio.setFrequency(DAVIS_US_HOP[idx] + off, false);
  radio.startReceive();
  gotPacket = false;     // discard anything latched during the retune
}

// Close out the current probe point, report it, and advance.
static void finishProbe() {
  Serial.print(F("SWEEP pass="));   Serial.print(pass);
  Serial.print(F(" khz="));         Serial.print(probeKhz(probeIdx));
  Serial.print(F(" abs_khz="));     Serial.print(BASE_OFFSET_MHZ * 1000.0f + probeKhz(probeIdx), 1);
  Serial.print(F(" good="));        Serial.print(pGood);
  Serial.print(F(" bad="));         Serial.print(pBad);
  Serial.print(F(" missed="));      Serial.print(pMiss);
  Serial.print(F(" rssi="));        Serial.print(pGood ? (pRssiSum / (int32_t)pGood) : 0);
  // Receiver-side die temperature.  The ISS is the thing we expect to drift,
  // but until this is logged that is an ASSUMPTION — if our own TCXO is moving
  // too, the sweep peak would shift without the transmitter going anywhere.
  // Absolute value runs hot from self-heating; the CHANGE is what matters.
  Serial.print(F(" rxtempc="));     Serial.println(readCPUTemperature(), 1);

  if (pGood > bestGood) { bestGood = pGood; bestIdx = probeIdx; }

  probeIdx++;
  if (probeIdx >= NUM_PROBES) {
    Serial.print(F("SWEEPDONE pass="));   Serial.print(pass);
    Serial.print(F(" best_khz="));        Serial.print(probeKhz(bestIdx));
    Serial.print(F(" best_abs_khz="));    Serial.print(BASE_OFFSET_MHZ * 1000.0f + probeKhz(bestIdx), 1);
    Serial.print(F(" best_good="));       Serial.print(bestGood);
    Serial.print(F(" of="));              Serial.print(SLOTS_PER_PROBE);
    Serial.print(F(" rxtempc="));         Serial.print(readCPUTemperature(), 1);
    // A peak pinned at either end means the transmitter has walked outside the
    // window and the real center is further out than we looked.
    if (bestIdx == 0 || bestIdx == NUM_PROBES - 1) Serial.print(F(" PEG=1"));
    Serial.println();
    pass++; probeIdx = 0; bestGood = 0; bestIdx = 0;
  }
  probeSlots = 0; pGood = 0; pBad = 0; pMiss = 0; pRssiSum = 0;
}

// Read the RAK1906 and emit one ENV line.
//
// TIMING IS THE WHOLE POINT OF WHERE THIS IS CALLED FROM.  performReading() is a
// blocking forced-mode conversion of ~100 ms, and a Davis burst is ~5 ms long in
// a slot that repeats every 2.5625 s.  Block at the wrong moment and we simply
// miss packets — the exact failure we have spent two weeks measuring.  So this is
// only ever called immediately AFTER a hop, when the next burst is ~2.4 s away,
// never from the top of loop().
static void readEnv() {
  if (!envOk) return;
  if (millis() - lastEnvMs < ENV_PERIOD_MS) return;
  lastEnvMs = millis();
  if (!bme.performReading()) { Serial.println(F("ENV read-failed=1")); return; }
  Serial.print(F("ENV t_c="));    Serial.print(bme.temperature, 2);
  Serial.print(F(" rh="));        Serial.print(bme.humidity, 2);
  Serial.print(F(" p_hpa="));     Serial.print(bme.pressure / 100.0f, 2);
  Serial.println();
}

static void goAcquire(const __FlashStringHelper *why) {
  state   = ACQUIRE;
  curIdx  = PARK_CHANNEL;
  misses  = 0;
  resyncs++;
  onProbe = false;       // never probe while hunting — we need every packet
  Serial.print(F("STATUS state=ACQUIRE reason=")); Serial.print(why);
  Serial.print(F(" ch=")); Serial.print(curIdx);
  Serial.print(F(" resyncs=")); Serial.println(resyncs);
  hopTo(curIdx, false);
}

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 5000) { delay(10); }

  Serial.println();
  Serial.println(F("=== Davis ISS — FREQUENCY SWEEP (davis-sweep) ==="));
  Serial.print(F("sweep ")); Serial.print(SWEEP_MIN_KHZ);
  Serial.print(F(" to "));   Serial.print(SWEEP_MAX_KHZ);
  Serial.print(F(" kHz step ")); Serial.print(SWEEP_STEP_KHZ);
  Serial.print(F(" -> "));   Serial.print(NUM_PROBES);
  Serial.print(F(" points, ")); Serial.print(SLOTS_PER_PROBE);
  Serial.println(F(" probe slots each (~18 min/pass)"));

  pinMode(WB_IO2, OUTPUT);          // WisBlock: 3V3_S rail up before the radio
  digitalWrite(WB_IO2, HIGH);       // ...and before the RAK1906: same rail
  delay(300);
  SPI_LORA.begin();

  // RAK1906.  Address is 0x76 on the RAK module, but try 0x77 too rather than
  // silently running without a sensor.  A missing sensor is NOT fatal: this
  // sketch's job is the Davis link, and it must still do that job alone.
  Wire.begin();
  static const uint8_t ENV_ADDRS[2] = {0x76, 0x77};
  for (uint8_t i = 0; i < 2 && !envOk; i++) {
    if (bme.begin(ENV_ADDRS[i])) { envOk = true; envAddr = ENV_ADDRS[i]; }
  }
  if (envOk) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0, 0);          // off — see the note at the declaration
    Serial.print(F("RAK1906 (BME680) found at 0x")); Serial.println(envAddr, HEX);
  } else {
    Serial.println(F("RAK1906 (BME680) NOT found — continuing without it"));
  }

  float freq = DAVIS_US_HOP[PARK_CHANNEL] + BASE_OFFSET_MHZ;

  // tcxoVoltage 3.3 is REQUIRED — the RAK4631's SX1262 runs a TCXO fed from
  // DIO3.  Omit it and the radio inits cleanly and then never receives.
  int st = radio.beginFSK(freq, BITRATE_KBPS, DEVIATION_KHZ, RX_BW_KHZ,
                          10 /*dBm, tx unused*/, 24 /*preamble detect bits*/,
                          3.3 /*tcxo*/, false /*DC-DC*/);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print(F("beginFSK FAILED: ")); Serial.println(st);
    while (true) delay(1000);
  }
  if (radio.setSyncWord(DAVIS_SYNC, sizeof(DAVIS_SYNC)) != RADIOLIB_ERR_NONE)
    Serial.println(F("setSyncWord failed"));
  if (radio.fixedPacketLengthMode(PACKET_LEN) != RADIOLIB_ERR_NONE)
    Serial.println(F("fixedPacketLengthMode failed"));
  radio.setCRC(0);                                // checked in software
  radio.setDataShaping(RADIOLIB_SHAPING_0_5);     // GFSK BT=0.5
  radio.setPacketReceivedAction(onDio1);

  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print(F("startReceive FAILED: ")); Serial.println(st);
    while (true) delay(1000);
  }
  Serial.print(F("ACQUIRE on ch ")); Serial.print(PARK_CHANNEL);
  Serial.print(F(" = ")); Serial.print(freq, 6);
  Serial.println(F(" MHz (sweeping starts once locked)"));
}

// Move to the next slot: advance the channel, flip baseline/probe, retune.
static void advanceSlot() {
  curIdx = (curIdx + 1) % NUM_CHANNELS;
  // Probe only while tracking, and only on alternate slots.  Baseline slots are
  // what hold the lock and feed wxrx, so they are never sacrificed.
  onProbe = (state == TRACK) ? !onProbe : false;
  hopTo(curIdx, onProbe);
  readEnv();                     // safe here: just retuned, ~2.4 s to the next burst
}

void loop() {
  // LED: slow blink = hunting, fast = locked and sweeping.
  static uint32_t bl = 0;
  uint32_t period = (state == TRACK) ? 200 : 1000;
  if (millis() - bl > period) { bl = millis(); digitalWrite(LED_GREEN, !digitalRead(LED_GREEN)); }

  static uint32_t lastStat = 0;
  if (millis() - lastStat > 15000) {
    lastStat = millis();
    Serial.print(F("STATUS state=")); Serial.print(state == TRACK ? F("TRACK") : F("ACQUIRE"));
    Serial.print(F(" ch="));       Serial.print(curIdx);
    Serial.print(F(" heard="));    Serial.print(heard);
    Serial.print(F(" good="));     Serial.print(good);
    Serial.print(F(" bad="));      Serial.print(bad);
    Serial.print(F(" slots="));    Serial.print(slots);
    Serial.print(F(" missed="));   Serial.print(missTotal);
    Serial.print(F(" streak="));   Serial.print(misses);
    Serial.print(F(" resyncs="));  Serial.print(resyncs);
    Serial.print(F(" sweeppass=")); Serial.print(pass);
    Serial.print(F(" probekhz=")); Serial.println(probeKhz(probeIdx));
  }

  // While ACQUIRE-ing there is no hop to hang the env read off, so do it here.
  // Without this the barometer freezes for as long as the Davis link is down -
  // which defeats the point: an APRS weather beacon can still publish pressure even when
  // the ISS fields have gone to dots, because the two have independent failure
  // modes. Parked, a packet is only due every ~131 s, so a 100 ms read is free.
  if (state == ACQUIRE) readEnv();

  // ---- slot expiry: nothing arrived in time ----
  if (!gotPacket && state == TRACK &&
      (int32_t)(micros() - (nextDueUs + MISS_GUARD_US)) >= 0) {
    slots++; missTotal++;
    if (onProbe) {
      // A probe miss is DATA, not a fault — it is the whole point of probing an
      // offset the transmitter may not be on.  It must not count toward the
      // resync streak, or a wide sweep would tear down our own lock.
      pMiss++; probeSlots++;
    } else {
      misses++;
    }
    if (misses >= MAX_MISSES) {
      goAcquire(F("lost-sync"));
      return;
    }
    // We still know the schedule, so keep walking it.
    nextDueUs += INTERVAL_US;
    if (onProbe && probeSlots >= SLOTS_PER_PROBE) finishProbe();
    advanceSlot();
    return;
  }

  if (!gotPacket) return;
  gotPacket = false;

  uint8_t buf[PACKET_LEN];
  int st = radio.readData(buf, PACKET_LEN);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print(F("readData error: ")); Serial.println(st);
    radio.startReceive();
    return;
  }
  heard++;

  uint8_t d[PACKET_LEN];
  for (uint8_t i = 0; i < PACKET_LEN; i++) d[i] = revByte(buf[i]);

  uint16_t calc = crc16_ccitt(d, 6);
  uint16_t rx   = ((uint16_t)d[6] << 8) | d[7];
  bool ok = (calc == rx);
  int   rssi = (int)radio.getRSSI();

  // Byte-compatible with davis-hop: wxrx.py's regex stops at rssi=, so ch= and
  // probe= are additive and safe.
  Serial.print(ok ? F("[CRC OK ] ") : F("[CRC BAD] "));
  for (uint8_t i = 0; i < PACKET_LEN; i++) {
    if (d[i] < 0x10) Serial.print('0');
    Serial.print(d[i], HEX); Serial.print(' ');
  }
  Serial.print(F(" msg=0x")); Serial.print(d[0] >> 4, HEX);
  Serial.print(F(" id="));    Serial.print(d[0] & 0x07);
  Serial.print(F(" rssi="));  Serial.print(rssi);
  Serial.print(F(" ch="));    Serial.print(curIdx);
  Serial.print(F(" probe=")); Serial.print(onProbe ? probeKhz(probeIdx) : 0);
  if (d[8] == 0xFF && d[9] == 0xFF) Serial.print(F(" [direct]"));
  if (!ok) { Serial.print(F("  calc=0x")); Serial.print(calc, HEX);
             Serial.print(F(" rx=0x")); Serial.print(rx, HEX); }
  Serial.println();

  if (!ok) {
    // A bad CRC is NOT a timing reference — it may be a false sync on noise.
    // Stay in the slot and keep listening; the real packet may still arrive.
    bad++;
    if (onProbe) pBad++;
    radio.startReceive();
    return;
  }

  good++;
  slots++;
  misses = 0;      // ANY good packet clears the streak, probe or baseline

  if (onProbe) { pGood++; pRssiSum += rssi; probeSlots++; }

  if (state == ACQUIRE) {
    state = TRACK;
    Serial.print(F("STATUS state=TRACK acquired-on-ch=")); Serial.println(curIdx);
  }

  // Re-anchor on every good packet — the clock is derived from the transmitter,
  // never free-run, so drift cannot accumulate.  A probe packet is just as valid
  // a timing reference as a baseline one; the offset does not move arrival time.
  nextDueUs = packetUs + INTERVAL_US;
  if (onProbe && probeSlots >= SLOTS_PER_PROBE) finishProbe();
  advanceSlot();
}
