// davis-rx — Davis Vantage Pro2 ISS receiver
// Copyright (C) 2026  davis-rx contributors
// Licensed under the GNU General Public License v3.0 or later.
// This program comes with ABSOLUTELY NO WARRANTY. See LICENSE for details.

/*
 * davis-rf69 — Davis Vantage Pro2 ISS receiver on an RFM69, with HARDWARE AFC.
 *
 * Target: Adafruit Feather M0 RFM69HCW 900 MHz (product 3176).
 * Build:  arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m0
 *
 * WHY THIS EXISTS
 *   ../davis-hop works, but its receive rate collapses when the ISS gets cold:
 *   23.4 pkt/min above ~62 F, ~10 at 53 F, recovering every morning. The packets
 *   that do arrive are at full strength (r(RSSI,rate) = -0.15 measured over a
 *   week), while CRC failures climb ~50x and hop lock is never lost. That is a
 *   transmitter walking off frequency as it cools, not a link-budget problem.
 *   The ISS already sits 33 kHz off the nominal hop table at room temperature.
 *
 *   The SX1262 cannot deal with that and cannot even measure it: Semtech dropped
 *   the FSK AFC and FEI registers. RadioLib's SX126x::getFrequencyError() opens
 *   with `if(modem != PACKET_TYPE_LORA) return 0.0;` — it returns zero for us,
 *   always. That gap is the only reason ../davis-sweep had to be written.
 *
 *   The RFM69 (SX1231) HAS those registers, which is why every working
 *   open-source Davis receiver uses this chip, and almost certainly why the
 *   Davis console has coped with a 33 kHz offset for years without complaint.
 *
 * WHAT THIS BUYS, CONCRETELY
 *   1. AFC re-centres the radio on EVERY packet's preamble, before the payload
 *      and before the CRC. No convergence period, no feedback loop, nothing to
 *      tune. That is the fix.
 *   2. FEI gives a per-packet frequency-error number. That is the instrument:
 *      the drift-vs-temperature curve builds itself at ~23 points per MINUTE
 *      instead of one point per 51 s of sweeping.
 *
 * DELIBERATELY UNCHANGED FROM davis-hop
 *   The hop table, the 2.5625 s timing, the ACQUIRE/TRACK state machine, the
 *   CRC, the byte reversal, and — most importantly — THE SERIAL LINE FORMAT.
 *   wxrx.py parses this sketch with no changes at all, so this can run as a
 *   SECOND wxrx instance alongside the RAK for a true side-by-side comparison:
 *       wxrx      :8000  <- RAK4631 / SX1262   (production, untouched)
 *       wxrx-rf69 :8001  <- Feather M0 / RFM69 (this)
 *   Prometheus separates them by the `instance` label. Nothing is at risk.
 *
 * TWO THINGS TO VERIFY ON FIRST POWER-UP — see BRING-UP NOTES at the bottom.
 */
#include <RadioLib.h>

// Adafruit Feather M0 RFM69HCW, from the official pinout diagram.
#define RFM69_CS    8
#define RFM69_IRQ   3
#define RFM69_RST   4

// Keep our OWN Module pointer as well as handing it to RF69. RadioLib makes
// RF69::getMod() protected and RF69::mod private, so this is the only way to
// reach the AFC/FEI registers from a sketch — and reaching them is the entire
// point of moving to this chip. Module's SPI accessors are public.
Module *rfmMod = new Module(RFM69_CS, RFM69_IRQ, RFM69_RST);
RF69 radio = rfmMod;

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
// signal. NOT chosen by RSSI: idx 20 carries a -55 dBm mesh signal.
static const uint8_t PARK_CHANNEL = 24;

// Measured from IQ capture of HIS transmitter: -33.1 kHz, std 0.6 kHz, 27
// bursts, at room temperature.
//
// KEEP THIS EVEN THOUGH AFC EXISTS. AFC has a limited pull-in range (roughly
// the receive bandwidth), so it corrects the last few kHz — it is not a licence
// to start 33 kHz away and hope. Land close, let AFC clean up the rest.
static const float FREQ_OFFSET_MHZ = -0.0330f;

// ---- Davis link parameters (same measurements as davis-hop) ----
static const float   BITRATE_KBPS  = 19.2f;
static const float   DEVIATION_KHZ = 9.9f;    // ISS side; 4.8 is the console's RX setting
static const float   RX_BW_KHZ     = 156.2f;  // wide on purpose; also sets the AFC pull-in range
static const uint8_t PACKET_LEN    = 10;
static uint8_t DAVIS_SYNC[]        = {0xCB, 0x89};

// AFC low-beta mode is intended for modulation index < 2, and ours is
//   h = 2 * 9.23 kHz / 19.231 kbps = 0.96
// so on paper we want it ON. It is OFF by default here because DavisRFM69's
// author documented not getting it working ("TODO: Should use LOWBETA_ON, but
// having trouble getting it working") and plain AFC is what actually ships in
// their working receiver. If you enable it, the DAGC companion register MUST
// change with it — that pairing is very likely what they were missing, and this
// sketch handles it. Change one thing at a time and measure.
#define USE_AFC_LOW_BETA 0

// ---- Hop timing (identical to davis-hop) ----
static const uint32_t INTERVAL_US   = 2562500UL;   // (41 + 0) / 16 s, station ID 1
static const uint32_t MISS_GUARD_US = 150000UL;    // 150 ms
static const uint16_t MAX_MISSES    = 20;          // ~51 s of silence

// FEI/AFC registers are in units of the synthesiser step: FXOSC / 2^19.
static const float F_STEP_HZ = 32000000.0f / 524288.0f;   // 61.035 Hz

enum State { ACQUIRE, TRACK };
static State    state     = ACQUIRE;
static uint8_t  curIdx    = PARK_CHANNEL;
static uint32_t nextDueUs = 0;
static uint16_t misses    = 0;

static uint32_t heard = 0, good = 0, bad = 0, slots = 0, missTotal = 0, resyncs = 0;
// Counts packets that failed CRC bit-reversed but PASSED un-reversed. Should
// stay 0 forever; if it climbs, see BRING-UP NOTE 1.
static uint32_t rawOrderHits = 0;

// Rolling FEI summary between STATUS lines. Per-packet fei= is the raw datum and
// is what gets logged, but during bring-up a mean and a spread are far easier to
// read than a scrolling column of numbers -- and the spread is the number that
// says whether this chip really is the better instrument.
static int32_t feiSum = 0, feiMin = 0, feiMax = 0;
static uint16_t feiN = 0;
static void feiNote(int32_t hz) {
  if (feiN == 0) { feiMin = feiMax = hz; }
  else { if (hz < feiMin) feiMin = hz; if (hz > feiMax) feiMax = hz; }
  feiSum += hz; feiN++;
}

volatile bool     gotPacket = false;
volatile uint32_t packetUs  = 0;
void onIrq() { packetUs = micros(); gotPacket = true; }

// Davis transmits every byte LSB-FIRST; the radio shifts in MSB-first, so every
// received byte arrives bit-reversed.
static uint8_t revByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// CRC-16/CCITT, poly 0x1021, init 0x0000, over bytes 0-5 ONLY.
// Frame is [6 data][2 CRC][FF FF]; bytes 8-9 are repeater bytes. Running the
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

// Read a signed 16-bit AFC or FEI register pair and convert to Hz.
// AFC = the correction the radio APPLIED. FEI = the error it still MEASURES.
// Together they say where the transmitter actually is and whether AFC caught it.
static int32_t readErrHz(uint16_t regMsb) {
  int16_t raw = ((int16_t)rfmMod->SPIreadRegister(regMsb) << 8)
              | rfmMod->SPIreadRegister(regMsb + 1);
  return (int32_t)(raw * F_STEP_HZ);
}

static void hopTo(uint8_t idx) {
  radio.standby();
  radio.setFrequency(DAVIS_US_HOP[idx] + FREQ_OFFSET_MHZ);
  radio.startReceive();
  gotPacket = false;     // discard anything latched during the retune
}

static void goAcquire(const char *why) {
  state   = ACQUIRE;
  curIdx  = PARK_CHANNEL;
  misses  = 0;
  resyncs++;
  Serial.print(F("STATUS state=ACQUIRE reason=")); Serial.print(why);
  Serial.print(F(" ch=")); Serial.print(curIdx);
  Serial.print(F(" resyncs=")); Serial.println(resyncs);
  hopTo(curIdx);
}

// Everything AFC. Done AFTER begin(), because begin() writes these registers.
static void enableAfc() {
  Module *m = rfmMod;
  // AfcAutoOn: run AFC automatically every time RX starts, i.e. on every
  // packet's preamble. AfcAutoClear: discard the previous correction first, so
  // a packet is never corrected using the last packet's error.
  m->SPIwriteRegister(RADIOLIB_RF69_REG_AFC_FEI,
                      RADIOLIB_RF69_AFC_AUTOCLEAR_ON | RADIOLIB_RF69_AFC_AUTO_ON);
#if USE_AFC_LOW_BETA
  m->SPIwriteRegister(RADIOLIB_RF69_REG_AFC_CTRL, RADIOLIB_RF69_AFC_LOW_BETA_ON);
  m->SPIwriteRegister(RADIOLIB_RF69_REG_TEST_DAGC, RADIOLIB_RF69_CONTINUOUS_DAGC_LOW_BETA_ON);
#else
  m->SPIwriteRegister(RADIOLIB_RF69_REG_AFC_CTRL, RADIOLIB_RF69_AFC_LOW_BETA_OFF);
  m->SPIwriteRegister(RADIOLIB_RF69_REG_TEST_DAGC, RADIOLIB_RF69_CONTINUOUS_DAGC_LOW_BETA_OFF);
#endif
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 5000) { delay(10); }

  Serial.println();
  Serial.println(F("=== Davis ISS receiver — RFM69 with hardware AFC ==="));

  float freq = DAVIS_US_HOP[PARK_CHANNEL] + FREQ_OFFSET_MHZ;
  int st = radio.begin(freq, BITRATE_KBPS, DEVIATION_KHZ, RX_BW_KHZ,
                       13 /*dBm, TX unused*/, 16 /*preamble bits*/);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print(F("begin FAILED: ")); Serial.println(st);
    while (true) delay(1000);
  }

  // maxErrBits = 0: the sync word must match exactly. Davis's 0xCB89 is only 16
  // bits, so tolerating errors here invites false syncs on noise, and a false
  // sync that survives to a CRC check wastes a whole slot.
  st = radio.setSyncWord(DAVIS_SYNC, sizeof(DAVIS_SYNC), 0);
  if (st != RADIOLIB_ERR_NONE) { Serial.print(F("setSyncWord failed: ")); Serial.println(st); }
  st = radio.fixedPacketLengthMode(PACKET_LEN);
  if (st != RADIOLIB_ERR_NONE) { Serial.print(F("fixedPacketLengthMode failed: ")); Serial.println(st); }
  st = radio.setCrcFiltering(false);              // checked in software
  if (st != RADIOLIB_ERR_NONE) { Serial.print(F("setCrcFiltering failed: ")); Serial.println(st); }
  st = radio.setDataShaping(RADIOLIB_SHAPING_0_5);  // GFSK BT=0.5
  if (st != RADIOLIB_ERR_NONE) { Serial.print(F("setDataShaping failed: ")); Serial.println(st); }

  enableAfc();
  Serial.print(F("AFC enabled, low-beta=")); Serial.println(USE_AFC_LOW_BETA ? "ON" : "OFF");

  radio.setPacketReceivedAction(onIrq);
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
  if (millis() - bl > period) { bl = millis(); digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }

  // ---- periodic status: same keys as davis-hop, so wxrx needs no changes ----
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
    Serial.print(F(" raworder=")); Serial.print(rawOrderHits);
    // SIGN CONVENTION IS UNVERIFIED until first light. The datasheet has
    // AfcValue = -FeiValue, but rather than guess we emit both and settle it
    // against ground truth: the transmitter is known to sit near -27 kHz at
    // room temperature, so whichever of fei/afc lands there with the right sign
    // is the one to trust. Fix this comment once it is known.
    if (feiN) {
      Serial.print(F(" fei_mean=")); Serial.print(feiSum / (int32_t)feiN);
      Serial.print(F(" fei_min="));  Serial.print(feiMin);
      Serial.print(F(" fei_max="));  Serial.print(feiMax);
      Serial.print(F(" fei_n="));    Serial.print(feiN);
      feiSum = 0; feiN = 0;
    }
    Serial.println();
  }

  // ---- slot expiry: nothing arrived in time ----
  if (!gotPacket && state == TRACK &&
      (int32_t)(micros() - (nextDueUs + MISS_GUARD_US)) >= 0) {
    misses++; missTotal++; slots++;
    if (misses >= MAX_MISSES) {
      goAcquire("lost-sync");
    } else {
      // We still know the schedule, so keep walking it — a miss is not a loss
      // of lock. Anchor advances by exactly one interval.
      nextDueUs += INTERVAL_US;
      curIdx = (curIdx + 1) % NUM_CHANNELS;
      hopTo(curIdx);
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

  // Read AFC/FEI straight after the packet, BEFORE any retune clears them.
  int32_t afcHz = readErrHz(RADIOLIB_RF69_REG_AFC_MSB);
  int32_t feiHz = readErrHz(RADIOLIB_RF69_REG_FEI_MSB);
  int     rssi  = (int)radio.getRSSI();

  uint8_t d[PACKET_LEN];
  for (uint8_t i = 0; i < PACKET_LEN; i++) d[i] = revByte(buf[i]);

  uint16_t calc = crc16_ccitt(d, 6);
  uint16_t rx   = ((uint16_t)d[6] << 8) | d[7];
  bool ok = (calc == rx);

  // Self-diagnosis for the one real bring-up unknown: if the reversed bytes fail
  // but the RAW bytes pass, this radio de-serialises the other way round and
  // revByte() must be dropped. Cheap to check, and it turns a baffling
  // everything-fails-CRC session into one obvious log line.
  if (!ok) {
    uint16_t rawCalc = crc16_ccitt(buf, 6);
    uint16_t rawRx   = ((uint16_t)buf[6] << 8) | buf[7];
    if (rawCalc == rawRx) {
      rawOrderHits++;
      Serial.println(F("*** CRC passes on RAW bytes, not reversed — drop revByte(). See BRING-UP NOTE 1."));
    }
  }

  // Byte-compatible with davis-hop: wxrx.py's regex stops at rssi=, so ch=,
  // fei= and afc= are additive and safe.
  Serial.print(ok ? F("[CRC OK ] ") : F("[CRC BAD] "));
  for (uint8_t i = 0; i < PACKET_LEN; i++) {
    if (d[i] < 0x10) Serial.print('0');
    Serial.print(d[i], HEX); Serial.print(' ');
  }
  Serial.print(F(" msg=0x")); Serial.print(d[0] >> 4, HEX);
  Serial.print(F(" id="));    Serial.print(d[0] & 0x07);
  Serial.print(F(" rssi="));  Serial.print(rssi);
  Serial.print(F(" ch="));    Serial.print(curIdx);
  Serial.print(F(" fei="));   Serial.print(feiHz);
  Serial.print(F(" afc="));   Serial.print(afcHz);
  if (d[8] == 0xFF && d[9] == 0xFF) Serial.print(F(" [direct]"));
  if (ok) feiNote(feiHz);
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

  good++;
  slots++;
  misses = 0;

  if (state == ACQUIRE) {
    state = TRACK;
    Serial.print(F("STATUS state=TRACK acquired-on-ch=")); Serial.println(curIdx);
  }

  // Re-anchor on every good packet: the clock is derived from the transmitter,
  // never free-run, so drift cannot accumulate across a session.
  nextDueUs = packetUs + INTERVAL_US;
  curIdx    = (curIdx + 1) % NUM_CHANNELS;
  hopTo(curIdx);
}

/*
 * BRING-UP NOTES — the two things that are genuinely unknown until hardware runs
 *
 * 1. BIT ORDER. The SX1262 handed us bit-reversed bytes, so revByte() is needed
 *    there. Both radios de-serialise MSB-first, so the same should apply here —
 *    but "should" is doing work in that sentence. The sketch checks itself: if
 *    CRC fails reversed and passes raw, it prints a loud line and counts it in
 *    STATUS raworder=. If that counter climbs, delete the revByte() loop.
 *
 * 2. SYNC WORD BYTE ORDER. We use 0xCB 0x89, measured from IQ. If NOTHING is
 *    ever heard — heard=0 after several minutes on the park channel while the
 *    RAK is receiving fine — the sync word is the first suspect. Try, in order:
 *      {0xD3, 0x91}  (each byte bit-reversed)
 *      {0x91, 0xD3}  (bit-reversed AND byte-swapped)
 *    heard=0 is the signature of a sync mismatch; heard>0 with every CRC failing
 *    is the signature of note 1. They are distinguishable, which is the point.
 *
 * WHAT SUCCESS LOOKS LIKE
 *   Side by side with the RAK through one cold night. The RAK's rate falls with
 *   temperature and this one does not. And fei= tells us, per packet, exactly
 *   how far the transmitter has walked — the measurement davis-sweep spends
 *   18 minutes per pass approximating.
 */
