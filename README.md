# YM2149F Polyphonic Synth - Turbo Sound x 3

An Arduino Pro Micro-based polyphonic synthesizer using three Yamaha YM2149F sound chips, complete with MIDI input, per-channel pitch bend, vibrato, expression, portamento and noise-based drum channel, and LED indicators.

## Recent Stability Fixes and Updates (v1.44)

### Critical Bugs Fixed

#### 1. Array Bounds Violation (MIDI Channels 11-16)
**Problem:** MIDI files with active channels 11-16 caused wild behavior, stuck notes, and memory corruption.
- When channels 11-16 sent note messages, the code accessed `midiToChip[10-15]`
- The `midiToChip` array only has 9 elements (indices 0-8)
- Out-of-bounds array access read garbage memory
- Random chip values caused invalid YM writes and state corruption
- Behavior degraded over time as corruption accumulated

**Fix:** Added channel validation to prevent out-of-bounds access:
- `noteOn()`: Returns early for channels >= 9
- `noteOff()`: Returns early for channels >= 9
- `pitchBend()`: Returns early for channels >= 9

#### 2. Division by Zero in Laser Mode
**Problem:** Laser mode caused stuck glitchy notes that wouldn't stop, even with MIDI Stop.
- Invalid period values propagated through the voice state
- Caused unstoppable glitchy "portamento/laser" effect

**Fix:** Changed laser calculation to prevent division by zero:
- Added laserAmt safety check
- Added period clamping

#### 3. Portamento Speed Scaling
**Problem:** Portamento speed was inverted (CC5: 0=fast, 127=slow instead of 0=slow, 127=fast).

**Fix:** Applied quadratic curve to CC5 mapping

#### 4. Note Range Validation
**Problem:** Out-of-bounds MIDI notes caused invalid period calculations and glitches.

**Fix:** Added note clamping to valid range

### Enhancements

#### MIDI Real-Time Message Handling
Added proper support for MIDI Stop/Start/Continue (0xFC/0xFA/0xFB):
- **MIDI Stop (0xFC)**: Calls `allNotesOffPanic()` and `resetAllControllers()` for all channels
- **MIDI Start (0xFA)**: Resets all controllers for clean playback start
- **MIDI Continue (0xFB)**: Resets all controllers

#### Controller Reset Functions
- `resetAllControllers(ch)`: Resets all CC values to defaults for a channel
- `allNotesOffPanic()`: Emergency stop for all voices on all chips
- `allNotesOffChannel(ch)`: Stop all notes on a specific channel

#### CC121 Support (Reset All Controllers)
Implements standard MIDI CC121 behavior to reset stuck controller states.

#### Improved Note-Off Handling
Changed `noteOff()` to release **ALL** matching notes instead of just the first one:
- Prevents stuck notes when duplicate note-ons occur
- Removed `break` statements in voice-scanning loops

#### YM2149 Class Integration
Replaced slow `digitalWrite()` calls with optimized class methods

## Features

- **3× YM2149F Chips**: Provides 9 independent tone voices (3 per chip).  
- **MIDI Input**: Supports USB-MIDI (via MIDIUSB library) and TRS-serial MIDI (31250 baud).  
- **Per-Channel Voice Routing**: Maps MIDI channels 1–9 to specific chip voices:  
  - Channels 1–3 → Chip 0 voices A/B/C  
  - Channels 4–6 → Chip 1 voices A/B/C  
  - Channels 7–9 → Chip 2 voices A/B/C  
- **Pitch Bend & Vibrato**: Standard pitch bend (±2 semitones) and CC1 mod-wheel vibrato.
- **Other Standard CC**: Velocity, Expression, Portamento, Sweep, Envelope
- **Drum Noise Channels**: (If enabled in Code: Disabled by Default) MIDI channels 10 trigger noise percussion on chip 2.  
- **LED Indicators**: LEDs light when each YM chip is active, with inverted logic (LOW = on).  
- **74HC138 Decoder**: Selects among the three chips via A/B/C lines.
- **Serial playback support**: Compatible player is a work in progress..
- **Digidrums**: Work in progress..

## Configuration Parameters

### Velocity Curve Settings

The YM2149F has only 16 volume levels (0-15), which can make soft MIDI notes inaudible. These parameters allow you to customize how MIDI velocity (0-127) maps to YM volume.

#### `VELOCITY_GAMMA` (default: `0.4f`)
- **Range:** 0.1 - 1.0
- **Purpose:** Gamma curve exponent for velocity response
- **Lower values** (0.2-0.4): Boost soft notes significantly, compress loud notes
- **Higher values** (0.6-1.0): More linear response, preserves velocity dynamics
- **Recommended:** 0.4 for general use, 0.2-0.3 for very sensitive response

#### `VELOCITY_MIN` (default: `3`)
- **Range:** 0-15
- **Purpose:** Minimum YM volume for softest MIDI velocity (0)
- **0:** Softest notes are silent (maximum dynamic range)
- **3-4:** Softest notes are audible but quiet (recommended)
- **8-10:** All notes are fairly loud (compressed dynamics)
- **15:** All notes at maximum volume (no velocity response)

#### `VELOCITY_MAX` (default: `15`)
- **Range:** 1-15
- **Purpose:** Maximum YM volume for loudest MIDI velocity (127)
- **15:** Full volume range (recommended)
- **10-14:** Limit maximum loudness
- **Note:** Must be ≥ VELOCITY_MIN

**Example configurations:**

```cpp
// Balanced response (default)
#define VELOCITY_GAMMA  0.4f
#define VELOCITY_MIN    3
#define VELOCITY_MAX    15

// Very sensitive for quiet playing
#define VELOCITY_GAMMA  0.2f
#define VELOCITY_MIN    5
#define VELOCITY_MAX    15

// Compressed (all notes fairly loud)
#define VELOCITY_GAMMA  0.5f
#define VELOCITY_MIN    10
#define VELOCITY_MAX    15

// Maximum volume (ignore velocity)
#define VELOCITY_GAMMA  0.4f
#define VELOCITY_MIN    15
#define VELOCITY_MAX    15
```

### Volume Control Settings

#### `EXPRESSION_AMOUNT` (default: `0.3f`)
- **Range:** 0.0-1.0
- **Purpose:** Controls how much CC7 (Channel Volume) and CC11 (Expression) affect volume
- **0.0:** Bypass expression entirely (velocity only controls volume)
- **0.3-0.5:** Reduced expression effect (dynamic range compression)
- **1.0:** Full expression control (standard MIDI behavior)
- **Use case:** Reduce to 0.0-0.3 if MIDI files have very low CC7/CC11 values causing inaudible notes

#### `USE_CC4_ENVELOPE` (default: `1`)
- **Range:** 0 or 1
- **Purpose:** Enable/disable CC4 volume envelope shaping
- **1:** Enable CC4 volume envelopes (standard)
- **0:** Disable CC4 processing (bypass envelope)
- **Use case:** Disable if CC4 envelopes cause unwanted volume changes

**Example configurations:**

```cpp
// Standard MIDI behavior (default)
#define EXPRESSION_AMOUNT 1.0f
#define USE_CC4_ENVELOPE  1

// Bypass all MIDI volume control (velocity only)
#define EXPRESSION_AMOUNT 0.0f
#define USE_CC4_ENVELOPE  0

// Compressed dynamics (limit CC7/CC11 effect)
#define EXPRESSION_AMOUNT 0.3f
#define USE_CC4_ENVELOPE  1
```

### Noise Channel

#### `ENABLE_NOISE_CHANNEL` (default: `0`)
- **Range:** 0 or 1
- **Purpose:** Enable MIDI channel 10 (percussion) via noise generator
- **0:** Disabled (noise code excluded from compilation)
- **1:** Enabled (channel 10 uses chip 2 noise generator)
- **Note:** When enabled, chip 2 voice C is dedicated to noise and unavailable for tone

---

## Hardware

<h1 align="center">
    <img width="55%" src="ym2149f-tsx3.png">
</h1>

### Components

- Arduino Pro Micro (ATmega32U4, 5V)  
- 3 × YM2149F sound chips  
- 74HC138 3-to-8 decoder for chip select  
- Logic gates (74LS08, 74LS04)
- 8 × digital lines for data bus (D0–D7)  
- Control lines: BC1, BDIR, RESET, clock output  
- Audio mixing: resistors + capacitor per channel  
- 3 × LEDs for chip activity  
- 3.5 mm stereo jack for audio output  

### Pin Connections

| Arduino Pin | Function       | YM2149F / 74HC138                    |
|-------------|----------------|--------------------------------------|
| 2–9         | D0–D7          | Data bus D0–D7                       |
| 10          | BC1            | YM2149F BC1                          |
| 20          | BDIR           | YM2149F BDIR                         |
| A3, A1, A0  | SEL A, B, C    | 74HC138 A, B, C inputs               |
| A2          | Enable (high)  | 74HC138 enable                       |
| 9           | Clock out      | YM2149F CLK                          |
| 15 (RESET)  | nRESET         | YM2149F RESET                        |
| LEDs        | 16, 14, 15     | Chip 0, 1, 2 indicator (LOW = on)    |

## Software Setup

1. **Install Arduino IDE** (v1.8 or later). 
2. **Add MIDIUSB library** via Library Manager.
3. **Download or clone** this repo.  
4. **Load `ym2149f-turbo-sound-x3.ino`** in Arduino IDE.  
5. **Select Board “Arduino Pro Micro”** select the COM port, and upload (ctrl + u).  

## Usage

- Connect USB-MIDI to a PC or MAC or use a TRS-MIDI cable.  
- Send notes on channels 1–9 to play polyphonic voices.  
- Use pitch bend wheel for ±2 semitone bends.  
- CC 1 – Mod Wheel → scales vibrato depth
- CC 4 – Volume Env Shape: 0=OFF, 1–63=Ramp up, 64–127=Ramp down
- CC 5 – Portamento Time → glide speed (0.005–0.5)
- CC 7 – Channel Volume → alternate expression control
- CC 9 – Pitch Sweep Amount → max semitone sweep (0–2 semis)
- CC 10 – Pitch Sweep Envelope → < 64 = attack, ≥ 64 = release shape
- CC 11 – Expression → per-channel volume scaling
- CC 64 – Sustain On/Off → ≥ 64 enables sustain (send All Notes Off or set to 0 to stop)
- CC 65 – Portamento On/Off → ≥ 64 enables glide
- CC 68 – Laser-Jump On/Off → ≥ 64 one-shot “laser” jump on note-on with Portamento on CC64 and Portamento Time CC5.
- CC 69 – Laser-Jump Amount → 0.0–1.0 blend toward zero for the laser effect
- CC 70 – 0-42: Semi-poly (chip-based, channels share 3 voices per chip)
- CC 70 – 43-84: Full poly (global pool, any channel can use any voice)
- CC 70 – 85-127: Mono (1:1 channel-to-voice, no stealing)
- CC 76 – Vibrato Rate → 0–10 Hz LFO speed
- CC 77 – Vibrato Depth → 0–2 semitone LFO range
- CC 85 – Vibrato Delay → delay before LFO engages
- CC 120/123 – All Sound / All Notes Off → channel reset
- Noise - Send on MIDI channel 10 (Chip 3 / YM2). *If enabled in Code: Disabled by Default
- LEDs will flash when each chip is active.

## Possible Future Features

- Sostenuto Pedal (CC 66)
Like sustain, but only for notes held at the moment you press the pedal.
- Soft Pedal (CC 67)
Map it to globally reduce volume or tweak envelope curves for a “muffled” effect.
- Filter Cutoff (CC 74)
Routing fine‐tuned volume curves or toggling between “bright” and “dark” lookup tables.
- Resonance / Emphasis (CC 71)
Emphasize higher harmonics by briefly boosting volume or switching to an “overdrive” lookup.
- Release Time (CC 72)
How quickly the volume envelope decays to zero after key-off.
- Arpeggio Pattern Select (e.g. CC 18 or CC 19)
Quickly switch between up/down, random, chord-hold, or custom progressions on the fly.
- Arpeggio Speed / Gate (alternate CC)
- Noise Level (CC 12 or CC 15)
Control the volume of the noise channel (for percussion or wind-like effects).
- Voice Balance (CC 13)
Pan or fade between the three YM chips (e.g. chip 0 left, chip 1 center, chip 2 right) for a simple stereo spread.
- Aftertouch / Channel Pressure
Use poly- or channel-aftertouch to add real-time pitch bend, filter simulation, or envelope modulation keyed to how hard you press.

## Hardware
https://hobbychop.com - YM2149F Turbo Sound x3

## Contributors
Special thanks to Ben Baker for his support and code contributions: https://baker76.com, https://github.com/benbaker76

Ym2149Synth by trash80 - YM2149F project on which some CC code is based. https://github.com/trash80/Ym2149Synth
