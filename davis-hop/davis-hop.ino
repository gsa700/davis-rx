// davis-rx — Davis Vantage Pro2 ISS receiver
// Copyright (C) 2026  davis-rx contributors
// Licensed under the GNU General Public License v3.0 or later.
// This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.

/*
 * Milestone 2 — Davis Vantage Pro2 ISS receiver, HOP FOLLOWER.
 *
 * Milestone 1 (../davis-rx) parks on one of 51 channels and therefore hears
 * 1 packet in 51 — about one every 131 s.  This sketch follows the hop
 * sequence and gets one every 2.5625 s: a 51x improvement in packet rate.
 *
 * KEEP ../davis-rx AROUND. It is the known-good fallback: if this sketch ever
 * looks broken, the first question is always "is the transmitter even on?"
 * (the ISS ran ~19 h/day for two years on a dead battery), and park mode is the
 * simplest way to answer it.
 *
 * HOW THE HOP WORKS
 *   DAVIS_US_HOP[] is stored in HOP SEQUENCE order, so the transmitter simply
 *   walks it: idx -> (idx+1) % 51, one step per transmission, every 2.5625 s
 *   for station ID 1 (interval = (41 + id_field) / 16 s, id_field = 0).
 *   So ONE valid packet tells us both where we are in the sequence and when the
 *   next one is due.  Everything after that is dead reckoning.
 *
 * STATE MACHINE — deliberately park-and-reacquire, not free-run-forever.
 *   ACQUIRE : sit on PARK_CHANNEL until a CRC-valid packet arrives.  Costs up
 *             to ~2 min, and is the ONLY way to re-establish phase after the
 *             transmitter has been away (its schedule restarts arbitrarily).
 *   TRACK   : hop with the transmitter, re-anchoring the clock on every good
 *             packet so drift can never accumulate.
 *   A miss does NOT drop us out of TRACK — we know the schedule, so we advance
 *   the channel anyway and keep listening.  Only MAX_MISSES consecutive misses
 *   (~51 s of silence) sends us back to ACQUIRE.  That is what stops the
 *   receiver wedging in a stale hop-sync state through the nightly ISS gap.
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

// Real US hop table, decoded from the DavisRFM69 FRF register triplets
// (freq = FRF * 32e6 / 2^19).  In HOP SEQUENCE order — do not sort it.
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
static const uint8_t  NUM_CHANNELS = 51;

// Reacquire on hop idx 24 = 909.4069 MHz.  NOT chosen by RSSI — that mistake
// cost an hour on idx 20, which carries a wide -55 dBm mesh signal ~33 dB
// louder than the Davis.  909.4069 is the Davis channel furthest (1.4 MHz)
// from every strong local signal the spectrum analyzer found.
static const uint8_t PARK_CHANNEL = 24;
// The transmitter's center frequency MOVES WITH TEMPERATURE, so this is a
// starting point, not a constant.  -33.1 kHz was measured from IQ in August
// (std 0.6 kHz, 27 bursts); by 52 F on 2026-09-14 the ISS had walked to about
// -26.7 kHz and the fixed -33 kHz offset was down to 2 good packets in 10 while
// -27 kHz was still getting 10 of 10.  The link is not weak -- it is mistuned.
// Hence the AFC loop below.
static const int32_t NOMINAL_OFFSET_HZ = -33000;
static int32_t offsetHz = NOMINAL_OFFSET_HZ;    // live, tracked

// ---- AFC: balanced-edge tracking -------------------------------------------
//
// The SX1262 has NO FSK AFC or FEI registers -- getFrequencyError() returns 0
// outside LoRa -- so the error cannot be read off the radio.  It has to be
// inferred from what we can actually observe: whether packets decode.
//
// The measured response is a FLAT-TOPPED PLATEAU about 8 kHz wide (10/10 good
// from -31 to -23 kHz on pass 37) with rolloff beyond.  Two consequences:
//   - fine tuning is pointless; we only need to stay near the middle, and
//   - a center-seeking search sees no gradient at all while inside the plateau.
// So we do not hunt for a peak.  We sit on the baseline and probe the two
// SHOULDERS, +/- AFC_PROBE_HZ out where the rolloff actually is.  Whichever
// shoulder hears more, that is the direction the transmitter has moved.
//
// Probe slots ALTERNATE with baseline slots, so every other slot is on the
// known-good offset and re-anchors the clock.  A probe that hears nothing
// therefore cannot cost us lock -- the same interleave davis-sweep proved.
static const bool     AFC_ENABLED        = true;
static const uint32_t AFC_PERIOD_MS      = 1200000UL;  // 20 min between cycles
static const int32_t  AFC_PROBE_HZ       = 6000;       // shoulder distance
static const uint8_t  AFC_SLOTS_PER_SIDE = 6;          // probe slots each side
static const int32_t  AFC_STEP_HZ        = 1500;       // correction per cycle
static const uint8_t  AFC_MARGIN         = 3;          // packets of difference to act
// Hard rails.  A pathological run of probes must never be able to walk the
// receiver off into empty spectrum and strand it there.  Wide on purpose: the
// loop only ever moves on POSITIVE evidence (packets heard on a shoulder), and
// holds when both shoulders are silent, so it cannot walk itself into noise.
// The upper rail was -5 kHz, chosen from autumn data, which was too tight --
// see the winter note on the acquire scan below.
static const int32_t  AFC_MIN_HZ         = -60000;
static const int32_t  AFC_MAX_HZ         =  10000;

enum AfcPhase { AFC_IDLE, AFC_LOW, AFC_HIGH };
static AfcPhase afcPhase   = AFC_IDLE;
static bool     curIsProbe = false;
static uint8_t  afcSlots   = 0;
static uint8_t  afcGoodLow = 0, afcGoodHigh = 0;
static uint32_t lastAfcMs  = 0;
static uint32_t afcCycles  = 0, afcMoves = 0;

// ---- ACQUIRE rescue scan ----------------------------------------------------
//
// The AFC above only runs in TRACK, which is useless if drift has already got
// bad enough that we cannot acquire at all -- exactly the state a cold morning
// could leave us in after an overnight ISS gap.  So if ACQUIRE goes quiet for
// long enough, walk the offset across the plausible band until something
// decodes.  This is also why the tracked offset is deliberately NOT persisted
// to flash: the receiver can always find its way home from the nominal value,
// and a stale or corrupt saved offset would be a way to fail to.
//
// WINTER RANGE (widened 2026-09-14, BEFORE the cold rather than after).  The
// transmitter moves LESS negative as it cools, and the measured slope is not
// constant -- three separate windows gave -0.305, -0.146 and -0.417 kHz/F,
// because an AT-cut crystal's tempco is cubic.  Projecting from ~50 F down to
// -10 F therefore lands anywhere between about -18 kHz and -2 kHz.  The old
// window (-48k..-14k) covered only the shallow end: on a cold morning the
// receiver could have gone deaf and then swept a range the transmitter had
// already walked out of -- the exact failure this scan exists to prevent,
// reappearing at a different temperature.  So the scan now spans -55k..+5k,
// which also allows for the offset crossing zero if it keeps climbing.
// Cost is worst-case acquisition time, ~11 min instead of ~6, and that only
// applies when we are already deaf.
static const uint32_t ACQ_SCAN_AFTER_MS = 180000UL;  // 3 min of silence
static const uint32_t ACQ_DWELL_MS      = 21000UL;   // ~8 slots per step
static const int32_t  ACQ_SCAN_MIN_HZ   = -55000;
static const int32_t  ACQ_SCAN_MAX_HZ   =   5000;
static const int32_t  ACQ_SCAN_STEP_HZ  = 2000;
static uint32_t acquireSinceMs = 0, lastAcqStepMs = 0;
static bool     acqScanning    = false;

// ---- Davis link parameters ----
static const float   BITRATE_KBPS  = 19.2f;
static const float   DEVIATION_KHZ = 9.9f;    // RF_FDEV*_9900 (ISS side; *_4800 is console RX)
static const float   RX_BW_KHZ     = 156.2f;  // WIDE on purpose: the TX crystal drifts with supply
                                              // and temperature. Deliberately UNCHANGED from
                                              // milestone 1 — narrowing it to recover ~3 dB is a
                                              // separate experiment, worth doing only after the new
                                              // 18650 supply proves the drift has stopped. Do not
                                              // change two variables at once.
static const uint8_t PACKET_LEN    = 10;
static uint8_t DAVIS_SYNC[]        = {0xCB, 0x89};

// ---- Hop timing ----
// Station ID 1 -> id field 0 -> interval = (41 + 0) / 16 s = 2.5625 s exactly.
// Confirmed by IQ: inter-burst spacing landed on exact multiples of this.
static const uint32_t INTERVAL_US   = 2562500UL;
// How late a packet may be before the slot counts as missed.  Covers TX jitter
// and our own clock; the nRF52 runs a 32.768 kHz crystal so drift over one
// 2.5 s slot is microseconds, not milliseconds.
static const uint32_t MISS_GUARD_US = 150000UL;   // 150 ms
// ~51 s of unbroken silence.  Long enough to ride out a deep fade, short enough
// that we are not dead-reckoning into a transmitter that has powered down.
static const uint16_t MAX_MISSES    = 20;

enum State { ACQUIRE, TRACK };
static State    state       = ACQUIRE;
static uint8_t  curIdx      = PARK_CHANNEL;
static uint32_t nextDueUs   = 0;
static uint16_t misses      = 0;

static uint32_t heard = 0, good = 0, bad = 0, slots = 0, missTotal = 0, resyncs = 0;

volatile bool     gotPacket = false;
volatile uint32_t packetUs  = 0;
void onDio1() { packetUs = micros(); gotPacket = true; }

// Davis transmits every byte LSB-FIRST; the SX1262 shifts in MSB-first, so every
// received byte arrives bit-reversed.  Measured from IQ, 27/27 bursts.
static uint8_t revByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// CRC-16/CCITT, poly 0x1021, init 0x0000, over bytes 0-5 ONLY.
// Frame is [6 data][2 CRC][FF FF]; bytes 8-9 are repeater bytes.  Running the
// CRC over all 10 can never return 0 on a real packet.
uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// Retune and re-arm RX.  calibrate=false: the SX1262 calibrates its image reject
// per BAND, and 902-928 is one band already covered by the calibration beginFSK
// did at startup.  Re-calibrating on every hop would add delay 51x per cycle for
// nothing.
static void hopTo(uint8_t idx, bool probe = false) {
  int32_t off = offsetHz;
  if (probe) off += (afcPhase == AFC_LOW) ? -AFC_PROBE_HZ : AFC_PROBE_HZ;
  radio.standby();
  radio.setFrequency(DAVIS_US_HOP[idx] + (float)off / 1000000.0f, false);
  radio.startReceive();
  gotPacket = false;     // discard anything latched during the retune
}

// Close out an AFC cycle and move the baseline if the shoulders disagree.
static void afcDecide() {
  const int32_t before = offsetHz;
  const __FlashStringHelper *act;
  if (afcGoodLow == 0 && afcGoodHigh == 0) {
    // Silence on both shoulders is not evidence of direction -- it usually
    // means the ISS itself is quiet.  Chasing it would walk us off the
    // transmitter while it is not even there.  Hold and wait.
    act = F("dead");
  } else if (afcGoodHigh >= afcGoodLow + AFC_MARGIN) {
    offsetHz += AFC_STEP_HZ; act = F("up");
  } else if (afcGoodLow >= afcGoodHigh + AFC_MARGIN) {
    offsetHz -= AFC_STEP_HZ; act = F("down");
  } else {
    act = F("hold");      // balanced: we are near the middle of the plateau
  }
  if (offsetHz > AFC_MAX_HZ) offsetHz = AFC_MAX_HZ;
  if (offsetHz < AFC_MIN_HZ) offsetHz = AFC_MIN_HZ;
  if (offsetHz != before) afcMoves++;

  Serial.print(F("AFC cycle="));      Serial.print(afcCycles);
  Serial.print(F(" low="));           Serial.print(afcGoodLow);
  Serial.print(F(" high="));          Serial.print(afcGoodHigh);
  Serial.print(F(" of="));            Serial.print(AFC_SLOTS_PER_SIDE);
  Serial.print(F(" action="));        Serial.print(act);
  Serial.print(F(" offset_hz="));     Serial.print(offsetHz);
  Serial.print(F(" was_hz="));        Serial.print(before);
  Serial.print(F(" moves="));         Serial.println(afcMoves);
}

// Attribute a resolved slot. Only probe slots feed the AFC.
static void afcRecord(bool decoded) {
  if (!curIsProbe) return;
  if (decoded) { if (afcPhase == AFC_LOW) afcGoodLow++; else afcGoodHigh++; }
  if (++afcSlots >= AFC_SLOTS_PER_SIDE) {
    afcSlots = 0;
    if (afcPhase == AFC_LOW) { afcPhase = AFC_HIGH; }
    else { afcDecide(); afcPhase = AFC_IDLE; lastAfcMs = millis(); }
  }
}

// Decide whether the slot we are about to hop into is a probe slot.
static bool afcAdvance() {
  if (!AFC_ENABLED || state != TRACK) { curIsProbe = false; return false; }
  if (afcPhase == AFC_IDLE) {
    if (millis() - lastAfcMs < AFC_PERIOD_MS) { curIsProbe = false; return false; }
    afcPhase = AFC_LOW; afcSlots = 0; afcGoodLow = 0; afcGoodHigh = 0;
    afcCycles++; curIsProbe = true; return true;
  }
  curIsProbe = !curIsProbe;     // strict alternation with baseline slots
  return curIsProbe;
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
  Serial.print(F("STATUS state=ACQUIRE reason=")); Serial.print(why);
  Serial.print(F(" ch=")); Serial.print(curIdx);
  Serial.print(F(" resyncs=")); Serial.println(resyncs);
  // Abandon any AFC cycle in flight: its half-collected counts describe a link
  // we have since lost, and acting on them would move the baseline on evidence
  // gathered from a transmitter we can no longer hear.
  afcPhase = AFC_IDLE; curIsProbe = false; afcSlots = 0;
  acquireSinceMs = millis(); acqScanning = false;
  hopTo(curIdx);
}

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 5000) { delay(10); }

  Serial.println();
  Serial.println(F("=== Davis ISS receiver — milestone 2 (hop follower) ==="));

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

  float freq = DAVIS_US_HOP[PARK_CHANNEL] + (float)offsetHz / 1000000.0f;

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
  Serial.println(F(" MHz (up to ~2 min for first packet, then tracking)"));
}

void loop() {
  // LED: slow blink = hunting, fast = locked and tracking.
  static uint32_t bl = 0;
  uint32_t period = (state == TRACK) ? 200 : 1000;
  if (millis() - bl > period) { bl = millis(); digitalWrite(LED_GREEN, !digitalRead(LED_GREEN)); }

  // ---- periodic status, machine-readable and ignored by wxrx's regex ----
  static uint32_t lastStat = 0;
  if (millis() - lastStat > 15000) {
    lastStat = millis();
    Serial.print(F("STATUS state=")); Serial.print(state == TRACK ? F("TRACK") : F("ACQUIRE"));
    Serial.print(F(" ch="));      Serial.print(curIdx);
    Serial.print(F(" heard="));   Serial.print(heard);
    Serial.print(F(" good="));    Serial.print(good);
    Serial.print(F(" bad="));     Serial.print(bad);
    Serial.print(F(" slots="));   Serial.print(slots);
    Serial.print(F(" missed="));  Serial.print(missTotal);
    Serial.print(F(" streak="));  Serial.print(misses);
    Serial.print(F(" resyncs=")); Serial.print(resyncs);
    Serial.print(F(" offset_hz=")); Serial.print(offsetHz);
    Serial.print(F(" afc_cycles=")); Serial.print(afcCycles);
    Serial.print(F(" afc_moves=")); Serial.println(afcMoves);
  }

  // While ACQUIRE-ing there is no hop to hang the env read off, so do it here.
  // Without this the barometer freezes for as long as the Davis link is down -
  // which defeats the point: an APRS weather beacon can still publish pressure even when
  // the ISS fields have gone to dots, because the two have independent failure
  // modes. Parked, a packet is only due every ~131 s, so a 100 ms read is free.
  if (state == ACQUIRE) {
    readEnv();
    // Rescue scan: drift can outrun the fixed offset far enough that we cannot
    // acquire at all.  Walk the plausible band until something decodes.
    if (millis() - acquireSinceMs > ACQ_SCAN_AFTER_MS &&
        millis() - lastAcqStepMs  > ACQ_DWELL_MS) {
      lastAcqStepMs = millis();
      if (!acqScanning) { acqScanning = true; offsetHz = ACQ_SCAN_MIN_HZ; }
      else {
        offsetHz += ACQ_SCAN_STEP_HZ;
        if (offsetHz > ACQ_SCAN_MAX_HZ) offsetHz = ACQ_SCAN_MIN_HZ;
      }
      Serial.print(F("AFC acquire-scan offset_hz=")); Serial.println(offsetHz);
      hopTo(curIdx);
    }
  }

  // ---- slot expiry: nothing arrived in time ----
  if (!gotPacket && state == TRACK &&
      (int32_t)(micros() - (nextDueUs + MISS_GUARD_US)) >= 0) {
    // A probe slot is deliberately parked off-frequency, so its miss says
    // nothing about the health of the link.  Counting it would both inflate the
    // miss streak toward a needless resync and understate the true receive rate.
    // Probe slots are therefore accounted ONLY in the AFC counters.
    if (curIsProbe) {
      afcRecord(false);
    } else {
      misses++; missTotal++; slots++;
    }
    if (misses >= MAX_MISSES) {
      goAcquire(F("lost-sync"));
    } else {
      // We still know the schedule, so keep walking it — a miss is not a loss
      // of lock.  Anchor advances by exactly one interval.
      nextDueUs += INTERVAL_US;
      curIdx = (curIdx + 1) % NUM_CHANNELS;
      bool probe = afcAdvance();
      hopTo(curIdx, probe);
      readEnv();                 // safe here: just retuned, ~2.4 s to the next burst
    }
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

  // Line format is byte-compatible with milestone 1 — wxrx.py's regex stops at
  // rssi=, so the trailing ch=/[direct] fields are additive and safe.
  Serial.print(ok ? F("[CRC OK ] ") : F("[CRC BAD] "));
  for (uint8_t i = 0; i < PACKET_LEN; i++) {
    if (d[i] < 0x10) Serial.print('0');
    Serial.print(d[i], HEX); Serial.print(' ');
  }
  Serial.print(F(" msg=0x")); Serial.print(d[0] >> 4, HEX);
  Serial.print(F(" id="));    Serial.print(d[0] & 0x07);
  Serial.print(F(" rssi="));  Serial.print(radio.getRSSI(), 0);
  Serial.print(F(" ch="));    Serial.print(curIdx);
  if (d[8] == 0xFF && d[9] == 0xFF) Serial.print(F(" [direct]"));
  if (!ok) { Serial.print(F("  calc=0x")); Serial.print(calc, HEX);
             Serial.print(F(" rx=0x")); Serial.print(rx, HEX); }
  Serial.println();

  if (!ok) {
    // A bad CRC is NOT a timing reference — it may be a false sync on noise.
    // Stay on this channel and keep listening; the real packet may still be
    // inside the slot, and if it is not, the miss timeout will move us on.
    bad++;
    radio.startReceive();
    return;
  }

  // A packet decoded on a probe slot is real weather data and is printed like
  // any other, but it belongs to the AFC measurement, not to the link stats.
  if (curIsProbe) {
    afcRecord(true);
  } else {
    good++; slots++; misses = 0;
  }

  if (state == ACQUIRE) {
    state = TRACK;
    acqScanning = false;
    lastAfcMs = millis();     // settle before the first cycle
    Serial.print(F("STATUS state=TRACK acquired-on-ch=")); Serial.print(curIdx);
    Serial.print(F(" offset_hz=")); Serial.println(offsetHz);
  }

  // Re-anchor on every good packet: the clock is derived from the transmitter,
  // never free-run, so drift cannot accumulate across a session.
  nextDueUs = packetUs + INTERVAL_US;
  curIdx    = (curIdx + 1) % NUM_CHANNELS;
  bool probe = afcAdvance();
  hopTo(curIdx, probe);
  readEnv();                     // safe here: just retuned, ~2.4 s to the next burst
}
