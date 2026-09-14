# davis-rx — a Davis Vantage Pro2 ISS receiver for the RAK4631 (SX1262)

Receives and decodes the 900 MHz frequency-hopping telemetry from a Davis
Instruments Integrated Sensor Suite, on a RAK4631 WisBlock (Nordic nRF52840 +
Semtech SX1262), and serves it as a web page and Prometheus metrics.

Most prior work in this space uses the RFM69 / SX1231. This does not, and that
turns out to matter a great deal — see **Frequency tracking** below.

## What is new here

Two findings that were not previously documented anywhere I could find:

1. **The ISS transmitter's centre frequency moves several kHz with temperature**,
   and on a cold night it will walk far enough to break a receiver tuned to a
   fixed offset — while the signal stays strong the whole time.
2. **A way to track it on a radio with no frequency-error detector**, which is
   every SX126x. The RFM69 has hardware AFC; the SX1262 deliberately does not
   expose one outside LoRa.

There is also an open question about ISS message type `0x3`, with data, at the
bottom.

## The cold drift

The transmitter was measured from IQ captures in August at **33.1 kHz low**
(std 0.6 kHz over 27 bursts) at roughly 22 °C. That number was then treated as a
constant, which was wrong.

By 11 °C the same transmitter had walked to about **-26.7 kHz**. Measured across
three separate windows, the slope was:

| window | slope |
|---|---|
| 22.0 → 12.9 °C | -0.55 kHz/°C |
| 15.9 → 11.2 °C | -0.26 kHz/°C |
| 9.7 → 13.7 °C  | -0.75 kHz/°C |

Those disagree by a factor of three because an AT-cut crystal's temperature
coefficient is **cubic, not linear**. Do not extrapolate a slope measured over a
mild autumn night into winter — that is the mistake this whole exercise exists
to correct.

**The symptom is easy to misread.** As the transmitter drifts off your fixed
offset you see:

- RSSI unchanged and strong,
- CRC failures climbing,
- lock eventually lost,
- full recovery when it warms up again.

That looks like a marginal link getting worse in the cold. It is not. It is
**discriminator mistuning** — the same thing you hear when you are off-frequency
on an FM signal: full quieting on the meter, distorted audio.

Measured at 11 °C, with the transmitter centred near -26.7 kHz:

| receiver offset | good packets |
|---|---|
| -33 kHz *(the "correct" fixed value)* | **2 / 10** |
| -31 to -23 kHz | **10 / 10** |
| -21 kHz | 5 / 10 |

### Why a wider filter does not fix it

The obvious reaction is to open the IF bandwidth. It does not help, and the
reason is worth understanding.

The link is 19.2 kbps 2-GFSK with **±9.9 kHz deviation**, BT=0.5, so occupied
bandwidth by Carson's rule is about **39 kHz**. This receiver runs a 156.2 kHz
filter — four times wider — and the signal is comfortably inside it even when
badly off-tuned. That is exactly why RSSI stays high.

But the **demodulator** compares instantaneous frequency against *where you
tuned*, not where the signal is. The two tones sit at ±9.9 kHz with the decision
threshold between them, so every kHz of tuning error comes straight out of the
decision margin:

| tuning error | mark | space | margin |
|---|---|---|---|
| 0 kHz | +9.9 | -9.9 | 9.9 kHz |
| 5 kHz | +14.9 | -4.9 | 4.9 kHz |
| 9.9 kHz | +19.8 | **0.0** | none |

**The tolerance is set by the deviation, not by the filter width.** The measured
plateau (flat to about ±4 kHz, rolling off by ±6) matches the ±Δf/2 rule of
thumb closely.

## Frequency tracking (the AFC)

`RadioLib`'s `SX126x::getFrequencyError()` returns 0.0 outside LoRa — the chip
has no FSK frequency-error registers at all. So the error cannot be read; it has
to be inferred from whether packets decode.

**This rules out peak-hunting.** The response is a flat-topped plateau about
8 kHz wide, so a centre-seeking search sees no gradient whatsoever while it is
inside the plateau, which is exactly where it normally sits.

So `davis-hop` does not look for a peak. It sits on the baseline and probes the
two **shoulders** at ±6 kHz, out where the rolloff actually is, and steps
1.5 kHz toward whichever shoulder hears more.

Key properties, all deliberate:

- **Probe slots alternate with baseline slots.** Every other slot is on the
  known-good offset and re-anchors the clock, so a probe that hears nothing
  cannot cost you lock.
- **Probe slots are accounted only in the AFC counters.** Counting a
  deliberately mistuned slot as a miss would inflate the miss streak toward a
  needless resync and understate the true receive rate.
- **Silence on both shoulders means hold, not search.** It usually means the ISS
  is quiet; chasing it would walk the receiver off a transmitter that is not
  transmitting.
- **The offset is not persisted.** An ACQUIRE rescue scan walks -55 kHz to
  +5 kHz after three minutes of silence, so the receiver can always find its way
  home from nominal. A stale or corrupt saved offset would be a way to fail to.

`coldwatch/afc-sim.py` simulates the loop against the measured plateau — seven
scenarios covering convergence, drift five times faster than anything observed,
two hours of transmitter silence, and rail behaviour under a pathological run.
Worth running before changing any of the constants.

**Measured result:** 14 hours with **zero resyncs**, 99.84 % slot yield over a
settled three-hour window (6 missed slots of 3844), CRC-bad 0.28 %.

## Hardware

- **RAK4631** WisBlock Core (nRF52840 + SX1262) on a WisBlock Base.
- Optional **RAK1906** (BME680) in the sensor slot for local pressure. The gas
  heater is deliberately left off, and the sensor is read only immediately after
  a hop — a blocking 100 ms conversion at the wrong moment costs packets.
- 900 MHz antenna. Siting matters more than you expect; `rssi-scan` exists
  because a strong local signal on an adjacent hop channel wasted an hour.

`davis-rf69/` is an RFM69 port for an Adafruit Feather M0 RFM69HCW
(CS=8, IRQ=3, RST=4). It compiles but **had not been run on hardware at the time
of writing** — the SX1262 proved sufficient once the AFC existed. Hardware is now
on hand to test it, and the interesting comparison is not sensitivity but
frequency handling: the RFM69 has a **hardware AFC and reports frequency error
directly**, so it should produce a far cleaner drift measurement than the
software loop's ±1.5 kHz limit cycle can. The two may end up complementary — the
SX1262 as the receiver, the RFM69 as the instrument.

## Protocol notes

Enough to write your own receiver:

- **51-channel hop sequence**, US band. Table in `davis-hop.ino`, derived from
  the DavisRFM69 FRF triplets (`freq = FRF * 32e6 / 2^19`).
- **Slot interval** = `(41 + id) / 16` seconds. Station ID 1 (id field 0) gives
  exactly **2.5625 s**, confirmed against IQ burst spacing.
- **19.2 kbps 2-GFSK, ±9.9 kHz deviation, BT=0.5.**
- **Every byte is transmitted LSB-first.** The SX1262 shifts in MSB-first, so
  every received byte arrives bit-reversed. This cost a lot of time; it is not
  optional.
- **Packet is 10 bytes:** 6 data, 2 CRC, then `FF FF` (repeater bytes).
- **CRC-16/CCITT**, poly `0x1021`, init `0x0000`, over **bytes 0-5 only**.
  Running it over all 10 can never return 0 on a real packet.
- **Byte 0** = message type in the high nibble, station id in the low 3 bits.
- **Sensor value** is 10-bit: `v = (byte3 << 2) | (byte4 >> 6)`.

### Message types

| type | meaning |
|---|---|
| `0x2` | supercapacitor voltage |
| `0x3` | **unidentified — see below** |
| `0x4` | UV index |
| `0x5` | rain rate |
| `0x6` | solar radiation |
| `0x7` | solar panel voltage |
| `0x8` | temperature |
| `0x9` | 10-minute wind gust |
| `0xA` | humidity |
| `0xE` | rain bucket tips |

Wind speed and direction ride in bytes 1 and 2 of *every* packet regardless of
type.

**`1023` means different things on different channels.** On the voltage channels
it is simply the top of the ADC range (about 3.41 V, if the `/300` scaling is
right — that factor is inherited from prior work and is **not independently
verified here**). On UV and solar radiation, which are not voltages, a pinned
1023 means no sensor is fitted.

### Message type 0x3 — open question, with data

`0x3` is the one type the reverse-engineering community never pinned down. It is
**not** an absent sensor. Logged alongside two channels whose sensors are
genuinely not fitted:

| type | samples | distinct values | range |
|---|---|---|---|
| `0x4` UV *(not fitted)* | 18 | **1** | 1023 only |
| `0x6` solar rad *(not fitted)* | 17 | **1** | 1023 only |
| **`0x3`** | 6 | **5** | **996 – 1023** |

Absent channels are frozen solid. `0x3` gave five distinct values in six
packets, drifting by single counts — the behaviour of a live ADC channel, not an
empty slot, and not a counter or flag field either.

On the `/300` scaling that is **3.32 – 3.41 V**, occasionally clipping at full
scale. A regulated supply rail under a bursty load fits: the ISS is transmitting
at the moment it samples.

Caveats: six samples, and this particular station is externally powered with no
battery fitted, which may not be representative. `service/msgtype-log.py` logs
raw payloads for this and the two control channels if you want to add data.

## What is in here

| path | |
|---|---|
| `davis-hop/` | **the receiver.** Hop follower with the AFC. Start here. |
| `davis-sweep/` | measurement sketch — sweeps the offset to map the plateau |
| `davis-rx/` | single-channel receiver, the simplest starting point |
| `davis-rf69/` | RFM69 port, compiles, **never run on hardware** |
| `raw-dump/`, `rssi-scan/`, `burst-watch/` | bring-up and RF-siting diagnostics |
| `service/wxrx.py` | parses the serial stream, serves a web page + Prometheus metrics |
| `service/msgtype-log.py` | logs raw message-type payloads (for the 0x3 question) |
| `coldwatch/afc-sim.py` | simulates the AFC loop against the measured plateau |
| `coldwatch/sweep-curve.py` | turns sweep output into a drift curve |
| `tools/fei-log.py` | logs the RFM69's per-packet frequency error to CSV |
| `tools/flash-rak` | DFU flashing, handling the 1200-baud touch properly |

`coldwatch/cold-night-2026-09-13.csv` is the raw overnight measurement behind
the drift finding.

**Use the centroid, not the best-scoring offset**, when reducing sweep data. The
plateau is flat-topped, so "best" is a tie-break artifact that jumps around.
`sweep-curve.py` does this correctly.

## How this was worked out

The frequency plan is the fun part, and it is mostly arithmetic once you know
where to look.

### The hop table

DavisRFM69 carries the US channel list as **FRF triplets** — the three register
bytes an RFM69 loads into its synthesiser. Those convert straight to megahertz:

```
freq_MHz = FRF * 32e6 / 2**19 / 1e6        # 32 MHz reference, 19-bit fraction
```

Run that over the triplets in hop-sequence order and you get the 51 channels,
spanning roughly 902–927 MHz. They are **not** in ascending frequency order —
the sequence itself is the hop pattern, so index order is what matters. The
table is in `davis-hop.ino`.

### The slot interval falls out of the station ID

```
interval = (41 + id) / 16   seconds
```

Station ID 1 puts 0 in the id field, giving exactly **2.5625 s**. That was later
confirmed independently from IQ: inter-burst spacing landed on exact integer
multiples of 2.5625 s as the transmitter hopped in and out of the capture
window. Two derivations agreeing is worth a lot when everything else is failing.

So a receiver only needs to find the transmitter **once**, then dead-reckon:
advance one table index every 2.5625 s and re-anchor the clock on each good
packet. That is all `davis-hop` does.

### The part that took two weeks

The receiver could *hear* the transmitter and could not decode a single packet.
`burst-watch` measured a **7 ms burst at −84 dBm** — exactly right for 10 bytes
plus preamble and sync at 19.2 kbps — and fifteen minutes of packet RX on that
frequency yielded **zero packets** where seven were expected.

The decisive experiment was to stop syncing on Davis's `CB 89` and sync on the
**bare `AA AA` preamble** instead, whitening explicitly off. Still nothing, in a
band known to be busy. That eliminated the sync word and proved the fault was
upstream: the demodulator was not recovering a clean bit stream at all.

Which left bitrate, deviation, shaping or bit polarity — and no way to tell them
apart from inside the receiver. So: capture raw IQ with an SDR and measure.

### What the IQ said

An 8-minute, 10 MHz capture centred on 906 MHz, FM-demodulated with
`np.angle(x[1:] * np.conj(x[:-1]))`:

| quantity | assumed | **measured** |
|---|---|---|
| symbol rate | 19.2 kbps | **19.231 kbps** (27 bursts, 19.19–19.25) |
| deviation | 9.9 or 4.8 kHz? | **9.23 kHz** → the 9.9 family, 4.8 eliminated |
| `CB 89` sync | ? | **present, normal polarity, 0 bit errors in 27/27** |

Every assumption was right. The bugs were somewhere nobody had been looking.

### The two actual bugs

**1. The transmitter was 33 kHz low, not 25.** A spectrum-analyser reading had
put it at −25.0 kHz, so the receiver was tuned there. The IQ said **−33.1 kHz,
std 0.6 kHz across 27 bursts on ten different channels.** With a narrow filter
that residual 8 kHz parks part of a ~38 kHz signal outside the passband.

*(This is also the first hint of the drift described above — the −25 and −33
readings were taken weeks apart at different temperatures, and neither was
wrong.)*

**2. Davis transmits every byte LSB-first.** The over-the-air bit pattern
`AA AA CB 89` is exactly as configured — the sync word was never the problem.
But the ten bytes *after* sync arrive bit-reversed on any MSB-first radio, which
is both the SX1262 and the RFM69.

And the frame is not what it looks like:

```
[4x AA preamble][CB 89 sync][6 data][CRC-16 over those 6][FF FF]
```

**CRC-16/CCITT over bytes 0–5 only.** Bytes 8–9 are repeater bytes and are
`FF FF` on every direct packet. Running the CRC over all ten can never return
zero on a real packet — so a perfectly working receiver still looks broken.
Reverse each byte through a 256-entry table; it is simpler than running a
reflected CRC.

### Burst anatomy

7.99 ms total: **128 bits of frame (6.67 ms) followed by ~1.3 ms of unmodulated
carrier tail.** Worth knowing when setting RX timeouts — the tail is not data
and the packet is complete before the energy stops.

### Two mistakes worth inheriting

**Do not pick a listening channel from an RSSI sweep.** There are two 900 MHz
mesh repeaters in a tree nearby, plus local mesh nodes, putting −44 to −55 dBm
signals across the band. An RSSI sweep chose 912.9191 MHz — which has a mesh
signal roughly **33 dB louder than the Davis** parked on it. An hour went into
tuning a receiver that was staring into a floodlight. **Only a valid CRC or a
spectrum measurement identifies a transmitter.** RSSI cannot tell a weather
station from a repeater.

This is why `PARK_CHANNEL` is hop index 24 (909.4069 MHz): not the strongest
channel, but the one furthest — 1.4 MHz — from every strong local signal the
spectrum analyser found.

**Loose settings manufacture success.** Widening RX bandwidth to 117 kHz and
dropping preamble detection to 16 bits produced a satisfying stream of "packets"
at −116 dBm with random station IDs and uniformly bad CRCs. It briefly looked
like reception was working. It was false syncs on noise. **Once the CRC is
correct it becomes a true accept/reject signal**, and that ambiguity disappears
for good — which is why getting the CRC right matters more than it first seems.

## RAK4631 + RadioLib gotchas

Hours were lost to these, and none of them are obvious:

- **`beginFSK()` must be passed `tcxoVoltage = 3.3`.** Otherwise the radio
  initialises without error and simply never receives anything.
- **The default `SPI` object is the WisBlock IO-slot bus, not the radio.** The
  SX1262 lives on its own `SPIClass` on `NRF_SPIM2`.
- **The BSP exports no `PIN_LORA_*` macros.** Pins are NSS 42, DIO1 47,
  NRST 38, BUSY 46, SCK 43, MOSI 44, MISO 45.
- **`Serial` is only USB CDC if the sketch includes `<Adafruit_TinyUSB.h>`.**
- **Call `setWhitening(false)` explicitly.** Davis uses DCFREE_OFF and RadioLib
  does not clearly default it off. (Davis is not encrypted; whitening is the
  lookalike that makes people think it is.)
- Turn hardware CRC off — the frame's CRC does not cover what the radio assumes.

## Prior art

This builds directly on years of other people's reverse engineering:

- [dekay/DavisRFM69](https://github.com/dekay/DavisRFM69) — the hop table, packet
  format and much of the message-type mapping.
- [madscientistlabs](http://madscientistlabs.blogspot.com/) — the original
  Davis protocol write-ups.
- The [wxforum.net](https://www.wxforum.net/) community.

The ISS transmits with a TI CC1020; the Vantage Pro2 console receives with a
CC1021, which has **hardware AFC**. That is almost certainly how Davis has been
silently compensating for this drift for twenty years without ever reporting it.

## Licence

GPL-3.0. See `LICENSE`.
